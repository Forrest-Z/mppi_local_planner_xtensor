#include <mppi_local_planner/critics/goal_critic.hpp>

namespace mppi::critics
{

using xt::evaluation_strategy::immediate;

void GoalCritic::initialize()
{
    // TODO: get param from ros node handle

    power_ = 1;
    weight_ = 5.0;
    threshold_to_consider_ = 1.4;

    ROS_INFO("goal critic critic plugin initialized");
    ROS_INFO(
         "GoalCritic instantiated with %d power and %f weight.",
        power_, weight_);
}

void GoalCritic::score(CriticData & data)
{
    if (!enabled_ || !utils::withinPositionGoalTolerance(
                         threshold_to_consider_, data.state.pose.pose, data.path))
    {
        return;
    }

    const auto goal_idx = data.path.x.shape(0) - 1;

    const auto goal_x = data.path.x(goal_idx);
    const auto goal_y = data.path.y(goal_idx);

    const auto traj_x = xt::view(data.trajectories.x, xt::all(), xt::all());
    const auto traj_y = xt::view(data.trajectories.y, xt::all(), xt::all());

    auto dists = xt::sqrt(
        xt::pow(traj_x - goal_x, 2) +
        xt::pow(traj_y - goal_y, 2));

    data.costs += xt::pow(xt::mean(dists, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
