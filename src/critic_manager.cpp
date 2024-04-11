#include <mppi_local_planner/critic_manager.hpp>

namespace mppi
{

void CriticManager::on_configure(
    ros::NodeHandle* nh, const std::string & name,
    costmap_2d::Costmap2DROS* costmap_ros)
{
    nh_ = nh;
    costmap_ros_ = costmap_ros;
    name_ = name;

    getParams();
    loadCritics();
}

void CriticManager::getParams()
{
    // TODO: get critic plugin names (vector<string>)
    critic_names_ = {"ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
       "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"};
    //getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
    if (!loader_) {
        loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
            "mppi_local_planner", "mppi::critics::CriticFunction");
    }

    critics_.clear();
    for (auto name : critic_names_) {
        std::string fullname = getFullName(name);
        auto instance = std::unique_ptr<critics::CriticFunction>(
            loader_->createUnmanagedInstance(fullname));
        critics_.push_back(std::move(instance));
        critics_.back()->on_configure(
            nh_, name_, name_ + "." + name, costmap_ros_);
        ROS_INFO("Critic loaded : %s", fullname.c_str());
    }
}

std::string CriticManager::getFullName(const std::string & name)
{
//    return "mppi::critics::" + name;
    return "mppi/critics/" + name;
}

void CriticManager::evalTrajectoriesScores(
    CriticData & data) const
{
    for (size_t q = 0; q < critics_.size(); q++) {
        if (data.fail_flag) {
            break;
        }
        critics_[q]->score(data);
    }
}

}  // namespace mppi
