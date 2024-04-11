#include <mppi_local_planner/critics/prefer_forward_critic.hpp>

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
    // TODO: get param
    power_ = 1;
    weight_ = 5.0;
    threshold_to_consider_ = 0.5;

    ROS_INFO("prefer forward critic plugin initialized");
    ROS_INFO(
         "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(CriticData & data)
{
    using xt::evaluation_strategy::immediate;
    if (!enabled_ ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        return;
    }

    auto backward_motion = xt::maximum(-data.state.vx, 0);
    data.costs += xt::pow(
        xt::sum(
            std::move(
                backward_motion) * data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::PreferForwardCritic,
    mppi::critics::CriticFunction)
