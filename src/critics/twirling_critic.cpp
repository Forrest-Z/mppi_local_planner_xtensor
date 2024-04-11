#include <mppi_local_planner/critics/twirling_critic.hpp>

namespace mppi::critics
{

void TwirlingCritic::initialize()
{
    // TODO: get param
    power_ = 1;
    weight_ = 10.0;

    ROS_INFO("twirling critic plugin initialized");
    ROS_INFO(
         "TwirlingCritic instantiated with %d power and %f weight.", power_, weight_);
}

void TwirlingCritic::score(CriticData & data)
{
    using xt::evaluation_strategy::immediate;
    if (!enabled_ ||
        utils::withinPositionGoalTolerance(data.pose_tolerance, data.state.pose.pose, data.path))
    {
        return;
    }

    const auto wz = xt::abs(data.state.wz);
    data.costs += xt::pow(xt::mean(wz, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::TwirlingCritic,
    mppi::critics::CriticFunction)
