// A pool of threads for the contact solver. The caller and the workers run the same job, and
// meet at a barrier between the colors of the constraint graph; the workers spin while a job
// runs, and sleep between jobs.

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sys/sysctl.h>

class WorkerPool
{
public:
    // The threads to use by default: the performance cores but one, so that a thread never waits
    // at a barrier for a slow core, and one core stays free for the rest of the app and the
    // system. With every core taken, a preempted thread stalls all the others.
    static int defaultThreads()
    {
        int cores = 0;
        size_t size = sizeof(cores);
        if (sysctlbyname("hw.perflevel0.logicalcpu", &cores, &size, nullptr, 0) != 0 || cores < 1)
            cores = (int)std::thread::hardware_concurrency();
        return std::max(1, cores - 1);
    }

    // threads counts the caller; one means no workers
    explicit WorkerPool(int threads) : count(std::max(1, threads)), arrived(0), generation(0), jobGeneration(0), quit(false)
    {
        for (int i = 1; i < count; i++)
            workers.emplace_back([this, i] { work(i); });
    }

    ~WorkerPool()
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            quit = true;
        }
        wake.notify_all();
        for (std::thread &worker : workers)
            worker.join();
    }

    int threads() const { return count; }

    // Runs job(thread, threads) on the caller (thread 0) and on every worker, and returns once all
    // have finished. Jobs can call barrier(), each the same number of times.
    void run(const std::function<void(int, int)> &job)
    {
        if (count == 1)
        {
            job(0, 1);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            current = &job;
            finished = 0;
            jobGeneration++;
        }
        wake.notify_all();
        job(0, count);
        for (int spins = 0; finished.load(std::memory_order_acquire) < count - 1; spins++)
            spin(spins);
    }

    // Waits until every thread of the job has arrived
    void barrier()
    {
        if (count == 1)
            return;
        unsigned gen = generation.load(std::memory_order_acquire);
        if (arrived.fetch_add(1, std::memory_order_acq_rel) == count - 1)
        {
            arrived.store(0, std::memory_order_relaxed);
            generation.store(gen + 1, std::memory_order_release);
        }
        else
        {
            for (int spins = 0; generation.load(std::memory_order_acquire) == gen; spins++)
                spin(spins);
        }
    }

private:
    int count;
    std::vector<std::thread> workers;
    std::mutex mutex;
    std::condition_variable wake;
    const std::function<void(int, int)> *current = nullptr;
    std::atomic<int> finished{0};
    std::atomic<int> arrived;
    std::atomic<unsigned> generation;
    unsigned jobGeneration;
    bool quit;

    // Busy for a while, then gives the core up, in case the thread being waited for was preempted
    static void spin(int spins)
    {
        if (spins < 4096)
        {
#if defined(__aarch64__)
            __asm__ volatile("yield");
#endif
        }
        else
            std::this_thread::yield();
    }

    void work(int thread)
    {
        pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
        unsigned seen = 0;
        while (true)
        {
            const std::function<void(int, int)> *job;
            {
                std::unique_lock<std::mutex> lock(mutex);
                wake.wait(lock, [&] { return quit || jobGeneration != seen; });
                if (quit)
                    return;
                seen = jobGeneration;
                job = current;
            }
            (*job)(thread, count);
            finished.fetch_add(1, std::memory_order_acq_rel);
        }
    }
};
