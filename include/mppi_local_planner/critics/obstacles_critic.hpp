#ifndef MPPI_LOCAL_PLANNER_CRITICS_OBSTACLES_CRITIC_HPP
#define MPPI_LOCAL_PLANNER_CRITICS_OBSTACLES_CRITIC_HPP

#include <memory>
#include <string>

#include <costmap_2d/inflation_layer.h>
#include <mppi_local_planner/critic_function.hpp>
#include <mppi_local_planner/models/state.hpp>
#include <mppi_local_planner/tools/utils.hpp>

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for avoiding obstacles, allowing it to deviate off
 * the planned path. This is important to tune in tandem with PathAlign to make a balance
 * between path-tracking and dynamic obstacle avoidance capabilities as desirable for a
 * particular application
 */
class ObstaclesCritic : public CriticFunction
{
public:
    /**
    * @brief Initialize critic
    */
    void initialize() override;

    /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
    void score(CriticData & data) override;

protected:
    /**
    * @brief Checks if cost represents a collision
    * @param cost Costmap cost
    * @return bool if in collision
    */
    inline bool inCollision(float cost) const;

    /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return Collision information at pose
    */
    inline CollisionCost costAtPose(float x, float y, float theta);

    /**
    * @brief Distance to obstacle from cost
    * @param cost Costmap cost
    * @return float Distance to the obstacle represented by cost
    */
    inline float distanceToObstacle(const CollisionCost & cost);

    /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
    float findCircumscribedCost(costmap_2d::Costmap2DROS* costmap);

protected:
//    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
//        collision_checker_{nullptr};

    bool consider_footprint_{true};
    float collision_cost_{0};
    float inflation_scale_factor_{0}, inflation_radius_{0};

    float possible_collision_cost_;
    float collision_margin_distance_;
    float near_goal_distance_;
    float circumscribed_cost_{0}, circumscribed_radius_{0};

    unsigned int power_{0};
    float repulsion_weight_, critical_weight_{0};
    std::string inflation_layer_name_;
};

}  // namespace mppi::critics

#endif // MPPI_LOCAL_PLANNER_CRITICS_OBSTACLES_CRITIC_HPP
