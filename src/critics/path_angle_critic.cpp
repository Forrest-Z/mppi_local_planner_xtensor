#include <mppi_local_planner/critics/path_angle_critic.hpp>

#include <math.h>

namespace mppi::critics
{

using xt::evaluation_strategy::immediate;

void PathAngleCritic::initialize()
{
    // TODO: get param
    float vx_min = -0.35;

    if (fabs(vx_min) < 1e-6) {  // zero
        reversing_allowed_ = false;
    } else if (vx_min < 0.0) {   // reversing possible
        reversing_allowed_ = true;
    }

    // TODO: get param
    offset_from_furthest_ = 4;
    power_ = 1;
    weight_ = 2.2;
    threshold_to_consider_ = 0.5;
    max_angle_to_furthest_ = 0.785398;

    // TODO: set mode
    int mode = 0;

    mode_ = static_cast<PathAngleMode>(mode);
    if (!reversing_allowed_ && mode_ == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
        mode_ = PathAngleMode::FORWARD_PREFERENCE;
        ROS_WARN(
            "Path angle mode set to no directional preference, but controller's settings "
            "don't allow for reversing! Setting mode to forward preference.");
    }

    ROS_INFO("path angle critic plugin initialized");
    ROS_INFO(
        "PathAngleCritic instantiated with %d power and %f weight. Mode set to: %s",
        power_, weight_, modeToStr(mode_).c_str());
}

void PathAngleCritic::score(CriticData & data)
{
    if (!enabled_ ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        return;
    }

    utils::setPathFurthestPointIfNotSet(data);
    auto offseted_idx = std::min(
        *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

    const float goal_x = xt::view(data.path.x, offseted_idx);
    const float goal_y = xt::view(data.path.y, offseted_idx);
    const float goal_yaw = xt::view(data.path.yaws, offseted_idx);
    const geometry_msgs::Pose & pose = data.state.pose.pose;

    switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
        if (utils::posePointAngle(pose, goal_x, goal_y, true) < max_angle_to_furthest_) {
            return;
        }
        break;
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
        if (utils::posePointAngle(pose, goal_x, goal_y, false) < max_angle_to_furthest_) {
            return;
        }
        break;
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
        if (utils::posePointAngle(pose, goal_x, goal_y, goal_yaw) < max_angle_to_furthest_) {
            return;
        }
        break;
    default:
        break;
//        throw something
    }

    auto yaws_between_points = xt::atan2(
        goal_y - data.trajectories.y,
        goal_x - data.trajectories.x);

    auto yaws =
        xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

    switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
    {
        data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
        return;
    }
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
    {
        const auto yaws_between_points_corrected = xt::where(
            yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
        const auto corrected_yaws = xt::abs(
            utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
        data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
        return;
    }
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
    {
        const auto yaws_between_points_corrected = xt::where(
            xt::abs(utils::shortest_angular_distance(yaws_between_points, goal_yaw)) < M_PI_2,
            yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
        const auto corrected_yaws = xt::abs(
            utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
        data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
        return;
    }
    }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::PathAngleCritic,
    mppi::critics::CriticFunction)
