#pragma once
// Minimal fixed-size thread pool with a parallelFor helper.
//
// Design notes:
//  - Workers are created once and live for the pool's lifetime; per-step task
//    submission is lock-guarded but cheap relative to solver work.
//  - parallelFor chunks the index range to amortize dispatch overhead and give
//    each worker cache-coherent slices.
//  - Determinism: callers must make each chunk write only to its own slots
//    (results indexed by input position, never appended under a lock in
//    completion order).

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <atomic>

namespace mc {

class JobSystem {
public:
    explicit JobSystem(unsigned threadCount = std::thread::hardware_concurrency()) {
        if (threadCount < 1) threadCount = 1;
        for (unsigned i = 0; i < threadCount; ++i) {
            workers_.emplace_back([this] { workerLoop(); });
        }
    }

    ~JobSystem() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& w : workers_) w.join();
    }

    unsigned threadCount() const { return static_cast<unsigned>(workers_.size()); }

    // Runs fn(begin, end) over [0, n) split into chunks across the pool.
    // Blocks until every chunk has finished.
    void parallelFor(int n, int minChunk, const std::function<void(int, int)>& fn) {
        if (n <= 0) return;
        int chunk = std::max(minChunk, (n + static_cast<int>(workers_.size()) - 1) /
                                           static_cast<int>(workers_.size()));
        int numChunks = (n + chunk - 1) / chunk;
        if (numChunks <= 1) { fn(0, n); return; }

        // remaining/doneMutex/doneCv live on this stack frame, so the decrement
        // must happen under doneMutex: otherwise the waiter can see 0, return and
        // destroy them while the last worker is still about to lock the mutex.
        int remaining = numChunks;
        std::mutex doneMutex;
        std::condition_variable doneCv;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (int c = 0; c < numChunks; ++c) {
                int begin = c * chunk, end = std::min(n, begin + chunk);
                tasks_.push([&, begin, end] {
                    fn(begin, end);
                    std::lock_guard<std::mutex> dl(doneMutex);
                    if (--remaining == 0) doneCv.notify_one();
                });
            }
        }
        cv_.notify_all();
        std::unique_lock<std::mutex> dl(doneMutex);
        doneCv.wait(dl, [&] { return remaining == 0; });
    }

private:
    void workerLoop() {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
};

} // namespace mc
