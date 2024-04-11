#ifndef MPPI_LOCAL_PLANNER_TOOLS_MPPI_CONFIG_HPP
#define MPPI_LOCAL_PLANNER_TOOLS_MPPI_CONFIG_HPP

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <mutex>


namespace mppi
{

/**
 * @class MPPIConfig
 * @brief Config class for the mppi_local_planner and its components.
 */
class MPPIConfig
{
public:

    std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
    std::string map_frame; //!< Global planning frame

    struct Controller
    {
        std::string plugin;
        double controller_frequency;
        int time_steps;
        double model_dt;
        int batch_size;
        double vx_std;
        double vy_std;
        double wz_std;
        double vx_max;
        double vx_min;
        double vy_max;
        double wz_max;
        int iteration_count;
        double prune_distance;
        double transform_tolerance;
        double temperature;
        double gamma;
        std::string motion_model;
        bool visualize;
    } controller;

    struct TrajectoryVisualizer
    {
        int trajectory_step;
        int time_step;
    } trajectory_visualizer;

    struct AckermannConstraints
    {
        double min_turning_r;
    } ackermann_constraints;

    struct ConstraintCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
    } constraint_critic;

    struct GoalCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double threshold_to_consider;
    } goal_critic;

    struct GoalAngleCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double threhold_to_consider;
    } goal_angle_critic;

    struct PreferForwardCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double threhold_to_consider;
    } prefer_forward_critic;

//    struct ObstaclesCritic
//    {
//        bool enabled;
//        double cost_power;
//        double repulsion_weight;
//        double critical_weight;
//        double consider_footprint;
//        double collision_cost;
//        double collsion_margin_distance;
//        double near_goal_distance;
//    } obstalce_critic;

    struct CostCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double critical_cost;
        bool cosider_footprint;
        double collision_cost;
        double near_goal_distance;
    } cost_critic;

    struct PathAlignCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double max_path_occupancy_ratio;
        int trajectory_point_step;
        double threshold_to_consider;
        double offset_from_furthest;
        bool use_path_orientations;
    } path_align_critic;

    struct PathFollowCritic
    {
        bool enabled;
        double cost_powet;
        double cost_weight;
        double offset_from_furthest;
        double threhold_to_consider;
    } path_follow_critic;

    struct PathAngleCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double offset_from_furthest;
        double threshold_to_consider;
        double max_angle_to_furthest;
        bool forward_preference;
    } path_angle_critic;

//    struct TwirlingCritic
//    {
//        bool enabled;
//        double twirling_cost_power;
//        double twirling_cost_weight;
//    };

    //! Obstacle related parameters
    struct Obstacles
    {
        double min_obstacle_dist; //!< Minimum desired separation from obstacles
        double inflation_dist; //!< buffer zone around obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
        double dynamic_obstacle_inflation_dist; //!< Buffer zone around predicted locations of dynamic obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
        bool include_dynamic_obstacles; //!< Specify whether the movement of dynamic obstacles should be predicted by a constant velocity model (this also effects homotopy class planning); If false, all obstacles are considered to be static.
        bool include_costmap_obstacles; //!< Specify whether the obstacles in the costmap should be taken into account directly
        double costmap_obstacles_behind_robot_dist; //!< Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters)
        int obstacle_poses_affected; //!< The obstacle position is attached to the closest pose on the trajectory to reduce computational effort, but take a number of neighbors into account as well
        bool legacy_obstacle_association; //!< If true, the old association strategy is used (for each obstacle, find the nearest TEB pose), otherwise the new one (for each teb pose, find only "relevant" obstacles).
        double obstacle_association_force_inclusion_factor; //!< The non-legacy obstacle association technique tries to connect only relevant obstacles with the discretized trajectory during optimization, all obstacles within a specifed distance are forced to be included (as a multiple of min_obstacle_dist), e.g. choose 2.0 in order to consider obstacles within a radius of 2.0*min_obstacle_dist.
        double obstacle_association_cutoff_factor; //!< See obstacle_association_force_inclusion_factor, but beyond a multiple of [value]*min_obstacle_dist all obstacles are ignored during optimization. obstacle_association_force_inclusion_factor is processed first.
        std::string costmap_converter_plugin; //!< Define a plugin name of the costmap_converter package (costmap cells are converted to points/lines/polygons)
        bool costmap_converter_spin_thread; //!< If \c true, the costmap converter invokes its callback queue in a different thread
        int costmap_converter_rate; //!< The rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate)
        double obstacle_proximity_ratio_max_vel; //!< Ratio of the maximum velocities used as an upper bound when reducing the speed due to the proximity to a static obstacles
        double obstacle_proximity_lower_bound; //!< Distance to a static obstacle for which the velocity should be lower
        double obstacle_proximity_upper_bound; //!< Distance to a static obstacle for which the velocity should be higher
    } obstacles; //!< Obstacle related parameters


    MPPIConfig()
    {

        odom_topic = "odom";
        map_frame = "map";

        // controller
        controller.plugin = "mppi_local_planner::MPPILocalPlannerROS";

        controller.controller_frequency = 30.0;

        controller.time_steps = 56;

        controller.model_dt = 0.05;

        controller.batch_size = 2000;

        controller.vx_std = 0.2;

        controller.vy_std = 0.2;

        controller.wz_std = 0.4;



        // Obstacles
        obstacles.min_obstacle_dist = 0.5;
        obstacles.inflation_dist = 0.6;
        obstacles.dynamic_obstacle_inflation_dist = 0.6;
        obstacles.include_dynamic_obstacles = true;
        obstacles.include_costmap_obstacles = true;
        obstacles.costmap_obstacles_behind_robot_dist = 1.5;
        obstacles.obstacle_poses_affected = 25;
        obstacles.legacy_obstacle_association = false;
        obstacles.obstacle_association_force_inclusion_factor = 1.5;
        obstacles.obstacle_association_cutoff_factor = 5;
        obstacles.costmap_converter_plugin = "costmap_converter::CostmapToPolygonsDBSMCCH";
        obstacles.costmap_converter_spin_thread = true;
        obstacles.costmap_converter_rate = 5;
        obstacles.obstacle_proximity_ratio_max_vel = 1;
        obstacles.obstacle_proximity_lower_bound = 0;
        obstacles.obstacle_proximity_upper_bound = 0.5;

    }

    /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local ros::NodeHandle
   */
    void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

    /**
   * @brief Return the internal config mutex
   */
    std::mutex& configMutex() {return config_mutex_;}

private:
    std::mutex config_mutex_; //!< Mutex for config accesses and changes

};


} // namespace mppi

#endif // MPPI_LOCAL_PLANNER_TOOLS_MPPI_CONFIG_HPP
