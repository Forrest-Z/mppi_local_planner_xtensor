#include <mppi_local_planner/critics/goal_angle_critic.hpp>

namespace mppi::critics
{

void GoalAngleCritic::initialize()
{
    // TODO: get param from ros node handle
    power_ = 1;
    weight_ = 3.0;
    threshold_to_consider_ = 0.5;

    ROS_INFO("goal angle critic plugin initialized");
    ROS_INFO(
        "GoalAngleCritic instantiated with %d power, %f weight, and %f "
        "angular threshold.",
        power_, weight_, threshold_to_consider_);
}

void GoalAngleCritic::score(CriticData & data)
{
    if (!enabled_ || !utils::withinPositionGoalTolerance(
                         threshold_to_consider_, data.state.pose.pose, data.path))
    {
        return;
    }

    const auto goal_idx = data.path.x.shape(0) - 1;
    const float goal_yaw = data.path.yaws(goal_idx);

    data.costs += xt::pow(
        xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
            weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::GoalAngleCritic,
    mppi::critics::CriticFunction)
