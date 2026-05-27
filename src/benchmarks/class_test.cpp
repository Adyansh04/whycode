#include <sys/resource.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

struct CPUUsage
{
    double user_time, system_time, total_time;
};
static CPUUsage get_cpu_usage()
{
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    CPUUsage cpu;
    cpu.user_time   = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
    cpu.system_time = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    cpu.total_time  = cpu.user_time + cpu.system_time;
    return cpu;
}

// Simple micro DoNotOptimize to avoid dead-code elimination
template <class T>
inline void do_not_optimize_away(const T& value)
{
    asm volatile("" : : "r,m"(value) : "memory");
}

class Example
{
public:
    Example()
      : s_(0)
      , x_(1.0f)
      , y_(2.0f)
    {}
    explicit Example(int seed)
      : s_(seed)
      , x_(seed * 0.5f + 1.0f)
      , y_(seed * 0.25f + 2.0f)
    {}

    // Small non-trivial method
    void tick() noexcept
    {
        // Some work that can’t be trivially optimized out
        s_ = (s_ * 1664525 + 1013904223);
        x_ = std::fmaf(x_, 1.000123f, 0.000987f);
        y_ = std::fmaf(y_, 0.999877f, 0.001013f);
        if ((s_ & 0x3FF) == 0)
        {
            // lightweight branch to keep realistic behavior
            x_ += 0.0001f;
        }
    }

    // Returns a small computed value
    float compute(float in) const noexcept
    {
        float t = std::sin(in * 0.001f + x_) + std::cos(in * 0.0007f + y_);
        return t * 0.5f + 0.25f;
    }

    // Mixed method with input/output ref
    void mix(float in, float& acc) noexcept
    {
        acc = std::fmaf(in, 1.00001f, acc);
        tick();
    }

    int   state() const noexcept { return s_; }
    float x() const noexcept { return x_; }
    float y() const noexcept { return y_; }

private:
    int   s_;
    float x_;
    float y_;
};

struct BenchResult
{
    std::string name;
    double      ms;
    double      cpu_percent;
    double      rel_speedup;  // baseline / this
    int64_t     guard;        // combine outputs to avoid DCE
};

template <typename F>
BenchResult bench(const std::string& name, F&& fn, int iters)
{
    CPUUsage s_cpu = get_cpu_usage();
    auto     t0    = std::chrono::high_resolution_clock::now();

    int64_t guard = 0;
    for (int i = 0; i < iters; ++i)
    {
        guard ^= fn();
    }

    auto     t1    = std::chrono::high_resolution_clock::now();
    CPUUsage e_cpu = get_cpu_usage();
    double   ms  = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    double   cpu = (e_cpu.total_time - s_cpu.total_time) * 100.0 / (ms / 1000.0);
    do_not_optimize_away(guard);
    return { name, ms, cpu, 1.0, guard };
}

int main()
{
    const int outer_loops     = 1000;  // number of repetitions of each micro-test
    const int inner_calls     = 2000;  // method calls inside each repetition
    const int construct_loops = 50000;

    std::cout << "=== unique_ptr vs direct object call overhead benchmark ===\n";
    std::cout << "outer_loops=" << outer_loops << " inner_calls=" << inner_calls
              << " construct_loops=" << construct_loops << "\n\n";

    std::vector<BenchResult> results;

    // 1) Repeated trivial method call: tick()
    // Direct
    results.push_back(bench(
        "direct.tick()",
        [&]() -> int64_t {
            Example obj(123);
            int64_t acc = 0;
            for (int i = 0; i < inner_calls; ++i)
            {
                obj.tick();
                acc += obj.state();
            }
            return acc;
        },
        outer_loops));

    // unique_ptr
    results.push_back(bench(
        "unique_ptr.tick()",
        [&]() -> int64_t {
            auto    pobj = std::make_unique<Example>(123);
            int64_t acc  = 0;
            for (int i = 0; i < inner_calls; ++i)
            {
                pobj->tick();
                acc += pobj->state();
            }
            return acc;
        },
        outer_loops));

    // 2) compute() accumulation path
    // Direct
    results.push_back(bench(
        "direct.compute()",
        [&]() -> int64_t {
            Example obj(7);
            double  sum = 0.0;
            for (int i = 0; i < inner_calls; ++i)
            {
                sum += obj.compute(float(i));
            }
            // hash to int64_t
            return static_cast<int64_t>(sum * 1e6) ^ obj.state();
        },
        outer_loops));

    // unique_ptr
    results.push_back(bench(
        "unique_ptr.compute()",
        [&]() -> int64_t {
            auto   pobj = std::make_unique<Example>(7);
            double sum  = 0.0;
            for (int i = 0; i < inner_calls; ++i)
            {
                sum += pobj->compute(float(i));
            }
            return static_cast<int64_t>(sum * 1e6) ^ pobj->state();
        },
        outer_loops));

    // 3) Mixed loop: mix() + compute()
    // Direct
    results.push_back(bench(
        "direct.mix+compute",
        [&]() -> int64_t {
            Example obj(999);
            float   acc = 0.0f;
            int64_t g   = 0;
            for (int i = 0; i < inner_calls; ++i)
            {
                obj.mix(float(i), acc);
                g ^= int64_t(obj.compute(acc) * 1e6);
            }
            return g ^ obj.state();
        },
        outer_loops));

    // unique_ptr
    results.push_back(bench(
        "unique_ptr.mix+compute",
        [&]() -> int64_t {
            auto    pobj = std::make_unique<Example>(999);
            float   acc  = 0.0f;
            int64_t g    = 0;
            for (int i = 0; i < inner_calls; ++i)
            {
                pobj->mix(float(i), acc);
                g ^= int64_t(pobj->compute(acc) * 1e6);
            }
            return g ^ pobj->state();
        },
        outer_loops));

    // 4) Construction/destruction overhead
    // Direct
    results.push_back(bench(
        "direct.construct/destroy",
        [&]() -> int64_t {
            // construct on the stack repeatedly (RVO not applicable here)
            int64_t g = 0;
            for (int i = 0; i < construct_loops; ++i)
            {
                Example obj(i);
                g ^= obj.state();
            }
            return g;
        },
        1));  // one timing batch

    // unique_ptr
    results.push_back(bench(
        "unique_ptr.construct/destroy",
        [&]() -> int64_t {
            int64_t g = 0;
            for (int i = 0; i < construct_loops; ++i)
            {
                auto p = std::make_unique<Example>(i);
                g ^= p->state();
            }
            return g;
        },
        1));

    // Compute relative speedups versus the corresponding direct versions per category
    // We will take direct.* as baselines when applicable.
    auto find = [&](const std::string& name) -> int {
        for (size_t i = 0; i < results.size(); ++i)
        {
            if (results[i].name == name)
                return int(i);
        }
        return -1;
    };

    auto baseline_idx = find("direct.tick()");
    if (baseline_idx >= 0)
    {
        double base = results[baseline_idx].ms;
        int    u    = find("unique_ptr.tick()");
        if (u >= 0)
            results[u].rel_speedup = base / results[u].ms;
    }
    baseline_idx = find("direct.compute()");
    if (baseline_idx >= 0)
    {
        double base = results[baseline_idx].ms;
        int    u    = find("unique_ptr.compute()");
        if (u >= 0)
            results[u].rel_speedup = base / results[u].ms;
    }
    baseline_idx = find("direct.mix+compute");
    if (baseline_idx >= 0)
    {
        double base = results[baseline_idx].ms;
        int    u    = find("unique_ptr.mix+compute");
        if (u >= 0)
            results[u].rel_speedup = base / results[u].ms;
    }
    // construction baseline
    baseline_idx = find("direct.construct/destroy");
    if (baseline_idx >= 0)
    {
        double base = results[baseline_idx].ms;
        int    u    = find("unique_ptr.construct/destroy");
        if (u >= 0)
            results[u].rel_speedup = base / results[u].ms;
    }

    // Print results
    std::cout << std::left << std::setw(28) << "Test" << std::setw(12) << "Time (ms)"
              << std::setw(10) << "CPU %" << std::setw(12) << "RelSpeed"
              << "Guard\n";
    std::cout << std::string(70, '-') << "\n";
    for (const auto& r : results)
    {
        std::cout << std::left << std::setw(28) << r.name << std::setw(12) << std::fixed
                  << std::setprecision(3) << r.ms << std::setw(10) << std::setprecision(1)
                  << r.cpu_percent << std::setw(12) << std::setprecision(3) << r.rel_speedup
                  << r.guard << "\n";
    }

    std::cout << "\nNotes:\n";
    std::cout << "- For method calls, unique_ptr<T>->f() compiles to the same code as obj.f() "
                 "after inlining; overhead mainly comes from heap allocation in construction and "
                 "potential cache effects.\n";
    std::cout << "- The construct/destroy test isolates heap allocation overhead of "
                 "std::make_unique vs. stack construction.\n";
    std::cout << "- Run with -O3 -march=native -DNDEBUG for realistic results. Consider taskset to "
                 "pin CPU, and run multiple times.\n";
    std::cout << "- For perf: perf stat -e cycles,instructions,cache-references,cache-misses "
                 "./unique_vs_direct_bench\n";
    return 0;
}
