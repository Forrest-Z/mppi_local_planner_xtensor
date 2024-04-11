#include <mppi_local_planner/mppi_local_planner_ros.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mppi::MPPILocalPlannerROS, nav_core::BaseLocalPlanner)

namespace mppi
{

MPPILocalPlannerROS::MPPILocalPlannerROS():
    costmap_ros_(NULL)
    , costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
    , initialized_(false)
    , goal_reached_(false)
    , visualize_(true)
{

}

MPPILocalPlannerROS::~MPPILocalPlannerROS()
{

}

void MPPILocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    // check if the plugin is already initialized
    if(!initialized_)
    {
        name_ = name;

        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle nh("~/" + name);

        // get parameters of MPPIConfig via the nodehandle and override the default config
        //cfg_.loadRosParamFromNodeHandle(nh);

        // reserve some memory for obstacles
        obstacles_.reserve(500);


        // init other variables
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

        costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

        global_frame_ = costmap_ros_->getGlobalFrameID();
        cfg_.map_frame = global_frame_; // TODO
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

        //Initialize a costmap to polygon converter
        if (!cfg_.obstacles.costmap_converter_plugin.empty())
        {
            try
            {
                costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
                std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
                // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
                boost::replace_all(converter_name, "::", "/");
                costmap_converter_->setOdomTopic(cfg_.odom_topic);
                costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
                costmap_converter_->setCostmap2D(costmap_);

                costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_,
                                                cfg_.obstacles.costmap_converter_spin_thread);
                ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");
            }
            catch(pluginlib::PluginlibException& ex)
            {
                ROS_WARN("The specified costmap converter plugin cannot be loaded. "
                   "All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
                costmap_converter_.reset();
            }
        }
        else
        {
            ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
        }


        // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
        footprint_spec_ = costmap_ros_->getRobotFootprint();

        // init the odom helper to receive the robot's velocity from odom messages
        odom_helper_.setOdomTopic(cfg_.odom_topic);

        // optimizer initialize
        optimizer_.initialize(&nh, name_, costmap_ros_);

        // path_handler initialize
        path_hander_.initialize(&nh, name_, costmap_ros_, tf_);

        // trajectory_visualizer initialize
        trajectory_visualizer_.on_configure(&nh, name_, costmap_ros_->getGlobalFrameID());


        // set initialized flag
        initialized_ = true;

        ROS_DEBUG("mppi_local_planner plugin initialized.");
    }
    else
    {
        ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
    }
}

void MPPILocalPlannerROS::reset()
{
    optimizer_.reset();
}

void MPPILocalPlannerROS::cleanup()
{
    optimizer_.shutdown();
    trajectory_visualizer_.on_cleanup();
    ROS_INFO("Cleaned up MPPI local planner");
}

void MPPILocalPlannerROS::activeVisual()
{
    trajectory_visualizer_.on_activate();
    ROS_INFO("Actived MPPI local planner");
}

void MPPILocalPlannerROS::deactiveVisual()
{
    trajectory_visualizer_.on_deactivate();
    ROS_INFO("Deactived MPPI local planner");
}

void MPPILocalPlannerROS::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
    optimizer_.setSpeedLimit(speed_limit, percentage);
}

bool MPPILocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
{
    if (orig_global_plan.empty() || !initialized_)
    {
        return false;
    }
    nav_msgs::Path path;
    path.poses = orig_global_plan;
    path.header.frame_id = orig_global_plan.back().header.frame_id;
    path.header.stamp = orig_global_plan.back().header.stamp;
    path_hander_.setPath(path);
    return true;
}

void MPPILocalPlannerROS::visualize(nav_msgs::Path transformed_plan)
{
    trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(),
                               "Candidate Trajectories");
    trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(),
                               "Optimal Trajectory");
    trajectory_visualizer_.visualize(std::move(transformed_plan));
}

bool MPPILocalPlannerROS::isGoalReached()
{
    if (goal_reached_)
    {
        ROS_INFO("GOAL Reached!");
        cleanup();
        return true;
    }
    return false;
}

bool MPPILocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    if (!initialized_)
    {
        ROS_ERROR("Please initialize before computing velocity");
        return false;
    }
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    geometry_msgs::Twist robot_speed;
    geometry_msgs::PoseStamped robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);
    robot_speed.linear.x = robot_vel_tf.pose.position.x;
    robot_speed.linear.y = robot_vel_tf.pose.position.y;
    robot_speed.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

    setSpeedLimit(95.0, true);
    cmd_vel = computeVelocityCommands(robot_pose, robot_speed);

    return true;
}

geometry_msgs::Twist MPPILocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped &robot_pose,
                                                                  const geometry_msgs::Twist &robot_speed)
{
    // loop execution time test
    auto start = std::chrono::system_clock::now();

    // TODO: lock params




    // init velocity command
    geometry_msgs::TwistStamped cmd;
    static uint32_t seq = 0;
    cmd.header.seq = seq++;
    cmd.header.frame_id = robot_base_frame_;
    cmd.header.stamp = ros::Time::now();
    cmd.twist.linear.x = 0;
    cmd.twist.linear.y = 0;
    cmd.twist.angular.z = 0;
    geometry_msgs::TwistStamped zero_cmd = cmd;

    // set goal reached flag
    goal_reached_ = false;

    // set retry attemp flag
    optimizer_.retry_attemp_failed_ = false;
    // transform global plan
    nav_msgs::Path transformed_plan = path_hander_.transformPath(robot_pose);

    // check if global goal is reached
    double dx = transformed_plan.poses.back().pose.position.x; - robot_pose.pose.position.x;
    double dy = transformed_plan.poses.back().pose.position.y; - robot_pose.pose.position.y;
    double delta_orient = normalize_theta(tf2::getYaw(transformed_plan.poses.back().pose.orientation)
                                          - tf2::getYaw(robot_pose.pose.orientation));
    if (fabs(std::sqrt(dx*dx+dy*dy)) < 0.03
        && fabs(delta_orient) < 0.05)
    {
        goal_reached_ = true;
        ROS_INFO("-[MPPI]: Robot arrived at goal");
        return zero_cmd.twist;
    }

    costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

    cmd = optimizer_.evalControl(robot_pose, robot_speed, transformed_plan);

    // address attemped failed
    if (optimizer_.retry_attemp_failed_)
    {
        ROS_INFO("-[MPPI]: Retry attemped failed, set velocity to zero");
        return zero_cmd.twist;
    }

    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                    (end - start).count();
    ROS_INFO("-[MPPI]: Control loop execution time: %ld [ms]", duration);

    if (visualize_)
    {
        visualize(std::move(transformed_plan));
    }

    return cmd.twist;
}


} //end namespace mppi




