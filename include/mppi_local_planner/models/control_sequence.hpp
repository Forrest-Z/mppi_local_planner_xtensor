#ifndef MPPI_LOCAL_PLANNER_MODELS_CONTROL_SEQUENCE_HPP
#define MPPI_LOCAL_PLANNER_MODELS_CONTROL_SEQUENCE_HPP

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Control
 * @brief A set of controls
 */
struct Control
{
    float vx, vy, wz;
};

/**
 * @struct mppi::models::ControlSequence
 * @brief A control sequence over time (e.g. trajectory)
 */
struct ControlSequence
{
    xt::xtensor<float, 1> vx;
    xt::xtensor<float, 1> vy;
    xt::xtensor<float, 1> wz;

    void reset(unsigned int time_steps)
    {
        vx = xt::zeros<float>({time_steps});
        vy = xt::zeros<float>({time_steps});
        wz = xt::zeros<float>({time_steps});
    }
};

}  // namespace mppi::models

#endif // MPPI_LOCAL_PLANNER_MODELS_CONTROL_SEQUENCE_HPP
