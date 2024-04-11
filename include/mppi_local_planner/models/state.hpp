#ifndef MPPI_LOCAL_PLANNER_MODELS_STATE_HPP
#define MPPI_LOCAL_PLANNER_MODELS_STATE_HPP

#include <xtensor/xtensor.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief State information: velocities, controls, poses, speed
 */
struct State
{
    xt::xtensor<float, 2> vx;
    xt::xtensor<float, 2> vy;
    xt::xtensor<float, 2> wz;

    xt::xtensor<float, 2> cvx;
    xt::xtensor<float, 2> cvy;
    xt::xtensor<float, 2> cwz;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist speed;

    /**
    * @brief Reset state data
    */
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        vx = xt::zeros<float>({batch_size, time_steps});
        vy = xt::zeros<float>({batch_size, time_steps});
        wz = xt::zeros<float>({batch_size, time_steps});

        cvx = xt::zeros<float>({batch_size, time_steps});
        cvy = xt::zeros<float>({batch_size, time_steps});
        cwz = xt::zeros<float>({batch_size, time_steps});
    }
};
}  // namespace mppi::models

#endif // MPPI_LOCAL_PLANNER_MODELS_STATE_HPP
