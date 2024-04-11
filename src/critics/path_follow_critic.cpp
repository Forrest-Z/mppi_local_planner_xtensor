#include <mppi_local_planner/critics/path_follow_critic.hpp>

#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

namespace mppi::critics
{

void PathFollowCritic::initialize()
{
    // TODO: get param
    threshold_to_consider_ = 1.4;
    offset_from_furthest_ = 6;
    power_ = 1;
    weight_ = 5.0;

    ROS_INFO("path follow critic plugin initialized");
}

void PathFollowCritic::score(CriticData & data)
{
    if (!enabled_ || data.path.x.shape(0) < 2 ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        return;
    }

    utils::setPathFurthestPointIfNotSet(data);
    utils::setPathCostsIfNotSet(data, costmap_ros_);
    const size_t path_size = data.path.x.shape(0) - 1;

    auto offseted_idx = std::min(
        *data.furthest_reached_path_point + offset_from_furthest_, path_size);

    // Drive to the first valid path point, in case of dynamic obstacles on path
    // we want to drive past it, not through it
    bool valid = false;
    while (!valid && offseted_idx < path_size - 1) {
        valid = (*data.path_pts_valid)[offseted_idx];
        if (!valid) {
            offseted_idx++;
        }
    }

    const auto path_x = data.path.x(offseted_idx);
    const auto path_y = data.path.y(offseted_idx);

    const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
    const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

    auto dists = xt::sqrt(
        xt::pow(last_x - path_x, 2) +
        xt::pow(last_y - path_y, 2));

    data.costs += xt::pow(weight_ * std::move(dists), power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::PathFollowCritic,
    mppi::critics::CriticFunction)
