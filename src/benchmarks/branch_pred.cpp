#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define a simple structure to hold the data, mimicking the original code.
struct AnalysisResult
{
    std::vector<int> smooth;
    int              num_points = 0;
};

// --- Function with branch ---
void process_with_branch(
    AnalysisResult& analysis_data, const int id_samples, const float segmentWidth,
    volatile float& sx, volatile float& sy)
{
    sx                              = 0.0f;
    sy                              = 0.0f;
    analysis_data.num_points        = 0;
    const float two_pi_over_segment = 2.0f * M_PI / segmentWidth;
    for (int a = 1; a < id_samples; a++)
    {
        if (analysis_data.smooth[a] != analysis_data.smooth[a - 1])
        {
            sx += cos(two_pi_over_segment * a);
            sy += sin(two_pi_over_segment * a);
            analysis_data.num_points++;
        }
    }
}

// --- Branch-free function ---
void process_branch_free(
    AnalysisResult& analysis_data, const int id_samples, const float segmentWidth,
    volatile float& sx, volatile float& sy)
{
    sx                              = 0.0f;
    sy                              = 0.0f;
    analysis_data.num_points        = 0;
    const float two_pi_over_segment = 2.0f * M_PI / segmentWidth;
    for (int a = 1; a < id_samples; a++)
    {
        // Branch-free version of the if statement
        int is_edge = (analysis_data.smooth[a] != analysis_data.smooth[a - 1]);
        sx += is_edge * cos(two_pi_over_segment * a);
        sy += is_edge * sin(two_pi_over_segment * a);
        analysis_data.num_points += is_edge;
    }
}

// --- Benchmarking utility ---
void run_benchmark(
    const std::string&                                                                 name,
    std::function<void(AnalysisResult&, int, float, volatile float&, volatile float&)> func,
    AnalysisResult& data, const int id_samples, const float segmentWidth, int iterations)
{
    volatile float sx = 0.0f, sy = 0.0f;

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i)
    {
        func(data, id_samples, segmentWidth, sx, sy);
    }
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "Function: " << name << std::endl;
    std::cout << "Total time for " << iterations << " iterations: " << duration.count() << " ms"
              << std::endl;
    std::cout << "Average time per iteration: " << duration.count() / iterations << " ms"
              << std::endl;
    std::cout << "Result: num_points=" << data.num_points << ", sx=" << sx << ", sy=" << sy
              << std::endl;
    std::cout << "----------------------------------------" << std::endl;
}

// --- Data generation ---
void generate_data(AnalysisResult& data, int size, double change_probability)
{
    data.smooth.resize(size);
    if (size == 0)
        return;

    std::random_device               rd;
    std::mt19937                     gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    data.smooth[0]  = 0;
    int current_val = 0;
    for (int i = 1; i < size; ++i)
    {
        if (dis(gen) < change_probability)
        {
            current_val++;
        }
        data.smooth[i] = current_val;
    }
}

int main()
{
    const int   id_samples   = 720;
    const float segmentWidth = 360.0f;
    const int   iterations   = 640 * 480;

    AnalysisResult data;

    std::cout << "======== BENCHMARKING WITH LOW BRANCH PREDICTION MISS RATE (10% change) ========"
              << std::endl;
    generate_data(data, id_samples, 0.10);
    run_benchmark("With Branch", process_with_branch, data, id_samples, segmentWidth, iterations);
    run_benchmark("Branch-Free", process_branch_free, data, id_samples, segmentWidth, iterations);

    std::cout
        << "\n======== BENCHMARKING WITH HIGH BRANCH PREDICTION MISS RATE (50% change) ========"
        << std::endl;
    generate_data(data, id_samples, 0.50);
    run_benchmark("With Branch", process_with_branch, data, id_samples, segmentWidth, iterations);
    run_benchmark("Branch-Free", process_branch_free, data, id_samples, segmentWidth, iterations);

    std::cout
        << "\n======== BENCHMARKING WITH VERY LOW BRANCH PREDICTION MISS RATE (1% change) ========"
        << std::endl;
    generate_data(data, id_samples, 0.01);
    run_benchmark("With Branch", process_with_branch, data, id_samples, segmentWidth, iterations);
    run_benchmark("Branch-Free", process_branch_free, data, id_samples, segmentWidth, iterations);

    std::cout
        << "\n======== BENCHMARKING WITH VERY LOW BRANCH PREDICTION MISS RATE (0% change) ========"
        << std::endl;
    generate_data(data, id_samples, 0.0);
    run_benchmark("With Branch", process_with_branch, data, id_samples, segmentWidth, iterations);
    run_benchmark("Branch-Free", process_branch_free, data, id_samples, segmentWidth, iterations);

    return 0;
}