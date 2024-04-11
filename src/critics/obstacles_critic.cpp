#include <cmath>
#include <mppi_local_planner/critics/obstacles_critic.hpp>
#include <costmap_2d/inflation_layer.h>
namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
    // TODO: get param from ros node handle
    consider_footprint_ = false;
    power_ = 1;
    repulsion_weight_ = 1.5;
    critical_weight_ = 20.0;
    collision_cost_ = 100000.0;
    collision_margin_distance_ = 0.10;
    near_goal_distance_ = 0.5;
    inflation_layer_name_ = "";

    // TODO: collision check with robot footprint
    possible_collision_cost_ = findCircumscribedCost(costmap_ros_);

    if (possible_collision_cost_ < 1.0f) {
        ROS_ERROR(
            "Inflation layer either not found or inflation is not set sufficiently for "
            "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
            " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
            "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
            " for full instructions. This will substantially impact run-time performance.");
    }

    ROS_INFO("onstacls critic plugin initialized");

    ROS_INFO(
        "ObstaclesCritic instantiated with %d power and %f / %f weights. "
        "Critic will collision check based on %s cost.",
        power_, critical_weight_, repulsion_weight_, consider_footprint_ ?
                            "footprint" : "circular");
}

float ObstaclesCritic::findCircumscribedCost(
    costmap_2d::Costmap2DROS* costmap)
{
    double result = -1.0;
    const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
    if (static_cast<float>(circum_radius) == circumscribed_radius_) {
        // early return if footprint size is unchanged
        return circumscribed_cost_;
    }

    // check if the costmap has an inflation layer
//    const auto inflation_layer = costmap_2d::InflationLayer::getInflationLayer(
//        costmap,
//        inflation_layer_name_);
//    if (inflation_layer != nullptr) {
//        const double resolution = costmap->getCostmap()->getResolution();
//        result = inflation_layer->computeCost(circum_radius / resolution);
//        inflation_scale_factor_ = static_cast<float>(inflation_layer->getCostScalingFactor());
//        inflation_radius_ = static_cast<float>(inflation_layer->getInflationRadius());
//    } else {
//        RCLCPP_WARN(
//            logger_,
//            "No inflation layer found in costmap configuration. "
//            "If this is an SE2-collision checking plugin, it cannot use costmap potential "
//            "field to speed up collision checking by only checking the full footprint "
//            "when robot is within possibly-inscribed radius of an obstacle. This may "
//            "significantly slow down planning times and not avoid anything but absolute collisions!");
//    }

    circumscribed_radius_ = static_cast<float>(circum_radius);
    circumscribed_cost_ = static_cast<float>(result);

    return circumscribed_cost_;
}

float ObstaclesCritic::distanceToObstacle(const CollisionCost & cost)
{
    const float scale_factor = inflation_scale_factor_;
    const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;

    // If not footprint collision checking, the cost is using the center point cost and
    // needs the radius subtracted to obtain the closest distance to the object
    if (!cost.using_footprint) {
        dist_to_obj -= min_radius;
    }

    return dist_to_obj;
}

void ObstaclesCritic::score(CriticData & data)
{
    using xt::evaluation_strategy::immediate;
    if (!enabled_) {
        return;
    }

    if (consider_footprint_) {
        // footprint may have changed since initialization if user has dynamic footprints
        possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
    }

    // If near the goal, don't apply the preferential term since the goal is near obstacles
    bool near_goal = false;
    if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
        near_goal = true;
    }

    auto && raw_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
    raw_cost.fill(0.0f);
    auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
    repulsive_cost.fill(0.0f);

    const size_t traj_len = data.trajectories.x.shape(1);
    bool all_trajectories_collide = true;
    for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
        bool trajectory_collide = false;
        float traj_cost = 0.0f;
        const auto & traj = data.trajectories;
        CollisionCost pose_cost;

        for (size_t j = 0; j < traj_len; j++) {
            pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));
            if (pose_cost.cost < 1.0f) {continue;}  // In free space

            if (inCollision(pose_cost.cost)) {
                trajectory_collide = true;
                break;
            }

            // Cannot process repulsion if inflation layer does not exist
            if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
                continue;
            }

            const float dist_to_obj = distanceToObstacle(pose_cost);

            // Let near-collision trajectory points be punished severely
            if (dist_to_obj < collision_margin_distance_) {
                traj_cost += (collision_margin_distance_ - dist_to_obj);
            }

            // Generally prefer trajectories further from obstacles
            if (!near_goal) {
                repulsive_cost[i] += inflation_radius_ - dist_to_obj;
            }
        }

        if (!trajectory_collide) {all_trajectories_collide = false;}
        raw_cost[i] = trajectory_collide ? collision_cost_ : traj_cost;
    }

    // Normalize repulsive cost by trajectory length & lowest score to not overweight importance
    // This is a preferential cost, not collision cost, to be tuned relative to desired behaviors
    auto && repulsive_cost_normalized =
        (repulsive_cost - xt::amin(repulsive_cost, immediate)) / traj_len;

    data.costs += xt::pow(
        (critical_weight_ * raw_cost) +
            (repulsion_weight_ * repulsive_cost_normalized),
        power_);
    data.fail_flag = all_trajectories_collide;
}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */
bool ObstaclesCritic::inCollision(float cost) const
{
    bool is_tracking_unknown =
        costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

    switch (static_cast<unsigned char>(cost)) {
        using namespace costmap_2d; // NOLINT
    case (LETHAL_OBSTACLE):
        return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
        return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
        return is_tracking_unknown ? false : true;
    }

    return false;
}

CollisionCost ObstaclesCritic::costAtPose(float x, float y, float theta)
{
    CollisionCost collision_cost;
    float & cost = collision_cost.cost;
    collision_cost.using_footprint = false;
    unsigned int x_i, y_i;
    if (!costmap_->worldToMap(x, y, x_i, y_i)) {
        cost = costmap_2d::NO_INFORMATION;
        return collision_cost;
    }
    cost = costmap_->getCost(x_i, y_i);

    if (consider_footprint_ &&
        (cost >= possible_collision_cost_ || possible_collision_cost_ < 1.0f))
    {
        // TODO: get footprint cost
//        cost = static_cast<float>(collision_checker_.footprintCostAtPose(
//            x, y, theta, costmap_ros_->getRobotFootprint()));
        collision_cost.using_footprint = true;
    }

    return collision_cost;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::ObstaclesCritic,
    mppi::critics::CriticFunction)

