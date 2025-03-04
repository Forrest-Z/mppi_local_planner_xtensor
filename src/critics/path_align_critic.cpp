#include <mppi_local_planner/critics/path_align_critic.hpp>

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void PathAlignCritic::initialize()
{
    // TODO: get param
    power_ = 1;
    weight_ = 10.0;
    max_path_occupancy_ratio_ = 0.07;
    offset_from_furthest_ = 20;
    trajectory_point_step_ = 4;
    threshold_to_consider_ = 0.5;
    use_path_orientations_ = false;

    ROS_INFO("path align critic plugin initialized");
    ROS_INFO(
        "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
        power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{
    // Don't apply close to goal, let the goal critics take over
    if (!enabled_ ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        return;
    }

    // Don't apply when first getting bearing w.r.t. the path
    utils::setPathFurthestPointIfNotSet(data);
    const size_t path_segments_count = *data.furthest_reached_path_point;  // up to furthest only
    if (path_segments_count < offset_from_furthest_) {
        return;
    }

    // Don't apply when dynamic obstacles are blocking significant proportions of the local path
    utils::setPathCostsIfNotSet(data, costmap_ros_);
    const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
    unsigned int invalid_ctr = 0;
    const float range = *data.furthest_reached_path_point - closest_initial_path_point;
    for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
        if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
        if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
            return;
        }
    }

    const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
    const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points
    const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));  // path points

    const size_t batch_size = data.trajectories.x.shape(0);
    const size_t time_steps = data.trajectories.x.shape(1);
    auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

    // Find integrated distance in the path
    std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
    float dx = 0.0f, dy = 0.0f;
    for (unsigned int i = 1; i != path_segments_count; i++) {
        dx = P_x(i) - P_x(i - 1);
        dy = P_y(i) - P_y(i - 1);
        float curr_dist = sqrtf(dx * dx + dy * dy);
        path_integrated_distances[i] = path_integrated_distances[i - 1] + curr_dist;
    }

    float traj_integrated_distance = 0.0f;
    float summed_path_dist = 0.0f, dyaw = 0.0f;
    float num_samples = 0.0f;
    float Tx = 0.0f, Ty = 0.0f;
    size_t path_pt = 0;
    for (size_t t = 0; t < batch_size; ++t) {
        traj_integrated_distance = 0.0f;
        summed_path_dist = 0.0f;
        num_samples = 0.0f;
        const auto T_x = xt::view(data.trajectories.x, t, xt::all());
        const auto T_y = xt::view(data.trajectories.y, t, xt::all());
        for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
            Tx = T_x(p);
            Ty = T_y(p);
            dx = Tx - T_x(p - trajectory_point_step_);
            dy = Ty - T_y(p - trajectory_point_step_);
            traj_integrated_distance += sqrtf(dx * dx + dy * dy);
            path_pt = utils::findClosestPathPt(
                path_integrated_distances, traj_integrated_distance, path_pt);

            // The nearest path point to align to needs to be not in collision, else
            // let the obstacle critic take over in this region due to dynamic obstacles
            if ((*data.path_pts_valid)[path_pt]) {
                dx = P_x(path_pt) - Tx;
                dy = P_y(path_pt) - Ty;
                num_samples += 1.0f;
                if (use_path_orientations_) {
                    const auto T_yaw = xt::view(data.trajectories.yaws, t, xt::all());
                    dyaw = angles::shortest_angular_distance(P_yaw(path_pt), T_yaw(p));
                    summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
                } else {
                    summed_path_dist += sqrtf(dx * dx + dy * dy);
                }
            }
        }
        if (num_samples > 0) {
            cost[t] = summed_path_dist / num_samples;
        } else {
            cost[t] = 0.0f;
        }
    }

    data.costs += xt::pow(std::move(cost) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::PathAlignCritic,
    mppi::critics::CriticFunction)
