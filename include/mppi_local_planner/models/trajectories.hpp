#ifndef MPPI_LOCAL_PLANNER_MODELS_TRAJECTORIES_HPP
#define MPPI_LOCAL_PLANNER_MODELS_TRAJECTORIES_HPP

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

namespace mppi::models
{

/**
 * @class mppi::models::Trajectories
 * @brief Candidate Trajectories
 */
struct Trajectories
{
    xt::xtensor<float, 2> x;
    xt::xtensor<float, 2> y;
    xt::xtensor<float, 2> yaws;

    /**
    * @brief Reset state data
    */
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        x = xt::zeros<float>({batch_size, time_steps});
        y = xt::zeros<float>({batch_size, time_steps});
        yaws = xt::zeros<float>({batch_size, time_steps});
    }
};

}  // namespace mppi::models

#endif // MPPI_LOCAL_PLANNER_MODELS_TRAJECTORIES_HPP
