#include <mppi_local_planner/tools/noise_generator.hpp>

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

void NoiseGenerator::initialize(
    mppi::models::OptimizerSettings & settings, bool is_holonomic,
    const std::string & name, ros::NodeHandle* nh)
{
    settings_ = settings;
    is_holonomic_ = is_holonomic;
    active_ = true;

    // TODO: get param
    regenerate_noises_ = false;

    if (regenerate_noises_) {
        noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
    } else {
        generateNoisedControls();
    }
}

void NoiseGenerator::shutdown()
{
    active_ = false;
    ready_ = true;
    noise_cond_.notify_all();
    if (noise_thread_.joinable()) {
        noise_thread_.join();
    }
}

void NoiseGenerator::generateNextNoises()
{
    // Trigger the thread to run in parallel to this iteration
    // to generate the next iteration's noises (if applicable).
    {
        std::unique_lock<std::mutex> guard(noise_lock_);
        ready_ = true;
    }
    noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
    models::State & state,
    const models::ControlSequence & control_sequence)
{
    std::unique_lock<std::mutex> guard(noise_lock_);

    xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
    xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
    xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
    settings_ = settings;
    is_holonomic_ = is_holonomic;

    // Recompute the noises on reset, initialization, and fallback
    {
        std::unique_lock<std::mutex> guard(noise_lock_);
        xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        ready_ = true;
    }

    if (regenerate_noises_) {
        noise_cond_.notify_all();
    } else {
        generateNoisedControls();
    }
}

void NoiseGenerator::noiseThread()
{
    do {
        std::unique_lock<std::mutex> guard(noise_lock_);
        noise_cond_.wait(guard, [this]() {return ready_;});
        ready_ = false;
        generateNoisedControls();
    } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
    auto & s = settings_;

    xt::noalias(noises_vx_) = xt::random::randn<float>(
        {s.batch_size, s.time_steps}, 0.0f,
        s.sampling_std.vx);
    xt::noalias(noises_wz_) = xt::random::randn<float>(
        {s.batch_size, s.time_steps}, 0.0f,
        s.sampling_std.wz);
    if (is_holonomic_) {
        xt::noalias(noises_vy_) = xt::random::randn<float>(
            {s.batch_size, s.time_steps}, 0.0f,
            s.sampling_std.vy);
    }
}

}  // namespace mppi
