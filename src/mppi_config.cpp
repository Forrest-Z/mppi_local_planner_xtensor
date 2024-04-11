#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi
{

void TebConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{
    nh.param("odom_topic", odom_topic, odom_topic);
    nh.param("map_frame", map_frame, map_frame);

    // Trajectory
//    nh.param("teb_autosize", trajectory.teb_autosize, trajectory.teb_autosize);

}

} // namespace mppi
