#include <mppi_local_planner/optimizer.hpp>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>


namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void Optimizer::initialize(
    ros::NodeHandle* nh, const std::string & name,
    costmap_2d::Costmap2DROS* costmap_ros)
{
    nh_ = nh;
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    retry_attemp_failed_ = false;

    getParams();

    critic_manager_.on_configure(nh_, name_, costmap_ros_);
    noise_generator_.initialize(settings_, isHolonomic(), name_, nh_);

    reset();
}

void Optimizer::shutdown()
{
    noise_generator_.shutdown();
}

void Optimizer::getParams()
{
    std::string motion_model_name;

    auto & s = settings_;
    // TODO: get param
    s.model_dt = 0.05f;
    s.time_steps = 56; // 56
    s.batch_size = 1200; // 1000
    s.iteration_count = 1;
    s.temperature = 0.3f;
    s.gamma = 0.015f;
    s.base_constraints.vx_max = 1.0;
    s.base_constraints.vx_min = -0.35;
    s.base_constraints.vy = 0.5; // vy_max
    s.base_constraints.wz = 2.0; // wz_max, 1.9
    s.sampling_std.vx = 0.2; // vx_std, 0.2
    s.sampling_std.vy = 0.2; // vy_std
    s.sampling_std.wz = 0.45; // wz_std, 0.4
    s.retry_attempt_limit = 1;

    s.constraints = s.base_constraints;

    motion_model_name = "DiffDrive";
    setMotionModel(motion_model_name);

    reset();

    double controller_frequency;
    // TODO: get controller frequency
    controller_frequency = 20;
    setOffset(controller_frequency);
}

void Optimizer::setOffset(double controller_frequency)
{
    const double controller_period = 1.0 / controller_frequency;
    constexpr double eps = 1e-6;

    if ((controller_period + eps) < settings_.model_dt) {
        ROS_WARN(
            "Controller period is less then model dt, consider setting it equal");
    } else if (abs(controller_period - settings_.model_dt) < eps) {
        ROS_INFO(
            "Controller period is equal to model dt. Control sequence "
            "shifting is ON");
        settings_.shift_control_sequence = true;
    } else {
        ROS_ERROR(
            "Controller period more then model dt, set it equal to model dt");
    }
}

void Optimizer::reset()
{
    state_.reset(settings_.batch_size, settings_.time_steps);
    control_sequence_.reset(settings_.time_steps);
    control_history_[0] = {0.0, 0.0, 0.0};
    control_history_[1] = {0.0, 0.0, 0.0};
    control_history_[2] = {0.0, 0.0, 0.0};
    control_history_[3] = {0.0, 0.0, 0.0};

    settings_.constraints = settings_.base_constraints;

    costs_ = xt::zeros<float>({settings_.batch_size});
    generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

    noise_generator_.reset(settings_, isHolonomic());
    ROS_INFO("Optimizer reset");
}

geometry_msgs::TwistStamped Optimizer::evalControl(
    const geometry_msgs::PoseStamped & robot_pose,
    const geometry_msgs::Twist & robot_speed,
    const nav_msgs::Path & plan)
{
    prepare(robot_pose, robot_speed, plan);

    do {
        optimize();
    } while (fallback(critics_data_.fail_flag));

    utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
    auto control = getControlFromSequenceAsTwist(plan.header.stamp);

    if (settings_.shift_control_sequence) {
        shiftControlSequence();
    }

    return control;
}

void Optimizer::optimize()
{
    for (size_t i = 0; i < settings_.iteration_count; ++i) {
        generateNoisedTrajectories();
        critic_manager_.evalTrajectoriesScores(critics_data_);
        updateControlSequence();
    }
}

bool Optimizer::fallback(bool fail)
{
    static size_t counter = 0;

    if (!fail) {
        counter = 0;
        retry_attemp_failed_ = false;
        return false;
    }

    reset();

    if (++counter > settings_.retry_attempt_limit) {
        counter = 0;
        retry_attemp_failed_ = true;
        ROS_ERROR("-[MPPI]: Optimizer fail to compute path");
        //throw nav2_core::NoValidControl("Optimizer fail to compute path");
    }

    return true;
}

void Optimizer::prepare(
    const geometry_msgs::PoseStamped & robot_pose,
    const geometry_msgs::Twist & robot_speed,
    const nav_msgs::Path & plan)
{
    state_.pose = robot_pose;
    state_.speed = robot_speed;
    path_ = utils::toTensor(plan);
    costs_.fill(0);

    // TODO: adjust goal tolerance
    goal_tolerance_ = 0.05;

    critics_data_.fail_flag = false;
    // TODO: goal check prepare
    critics_data_.motion_model = motion_model_;
    critics_data_.furthest_reached_path_point.reset();
    critics_data_.path_pts_valid.reset();
}

void Optimizer::shiftControlSequence()
{
    using namespace xt::placeholders;  // NOLINT
    control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
    control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


    xt::view(control_sequence_.vx, -1) =
        xt::view(control_sequence_.vx, -2);

    xt::view(control_sequence_.wz, -1) =
        xt::view(control_sequence_.wz, -2);


    if (isHolonomic()) {
        control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
        xt::view(control_sequence_.vy, -1) =
            xt::view(control_sequence_.vy, -2);
    }
}

void Optimizer::generateNoisedTrajectories()
{
    noise_generator_.setNoisedControls(state_, control_sequence_);
    noise_generator_.generateNextNoises();
    updateStateVelocities(state_);
    integrateStateVelocities(generated_trajectories_, state_);
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyControlSequenceConstraints()
{
    auto & s = settings_;

    if (isHolonomic()) {
        control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
    }

    control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
    control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

    motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(
    models::State & state) const
{
    updateInitialStateVelocities(state);
    propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
    models::State & state) const
{
    xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
    xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

    if (isHolonomic()) {
        xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
    }
}

void Optimizer::propagateStateVelocitiesFromInitials(
    models::State & state) const
{
    motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
    xt::xtensor<float, 2> & trajectory,
    const xt::xtensor<float, 2> & sequence) const
{
    float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

    const auto vx = xt::view(sequence, xt::all(), 0);
    const auto vy = xt::view(sequence, xt::all(), 2);
    const auto wz = xt::view(sequence, xt::all(), 1);

    auto traj_x = xt::view(trajectory, xt::all(), 0);
    auto traj_y = xt::view(trajectory, xt::all(), 1);
    auto traj_yaws = xt::view(trajectory, xt::all(), 2);

    xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

    auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
    auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

    const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

    xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
    xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
    xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
    xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

    auto && dx = xt::eval(vx * yaw_cos);
    auto && dy = xt::eval(vx * yaw_sin);

    if (isHolonomic()) {
        dx = dx - vy * yaw_sin;
        dy = dy + vy * yaw_cos;
    }

    xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
    xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
}

void Optimizer::integrateStateVelocities(
    models::Trajectories & trajectories,
    const models::State & state) const
{
    const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

    xt::noalias(trajectories.yaws) =
        xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw;

    const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

    auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
    auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
    xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
    xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
    xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
    xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

    auto && dx = xt::eval(state.vx * yaw_cos);
    auto && dy = xt::eval(state.vx * yaw_sin);

    if (isHolonomic()) {
        dx = dx - state.vy * yaw_sin;
        dy = dy + state.vy * yaw_cos;
    }

    xt::noalias(trajectories.x) = state.pose.pose.position.x +
                                  xt::cumsum(dx * settings_.model_dt, 1);
    xt::noalias(trajectories.y) = state.pose.pose.position.y +
                                  xt::cumsum(dy * settings_.model_dt, 1);
}

xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
    auto && sequence =
        xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
    auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

    xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
    xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

    if (isHolonomic()) {
        xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
    }

    integrateStateVelocities(trajectories, sequence);
    return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
    auto & s = settings_;
    auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
    auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
    xt::noalias(costs_) +=
        s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
                                                   xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
    xt::noalias(costs_) +=
        s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
                                                   xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

    if (isHolonomic()) {
        auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
        xt::noalias(costs_) +=
            s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
                                                       xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
                                                       1, immediate);
    }

    auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
    auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
    auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
    auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

    xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
    xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
    if (isHolonomic()) {
        xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
    }

    applyControlSequenceConstraints();
}

geometry_msgs::TwistStamped Optimizer::getControlFromSequenceAsTwist(
    const ros::Time & stamp)
{
    unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

    auto vx = control_sequence_.vx(offset);
    auto wz = control_sequence_.wz(offset);

    if (isHolonomic()) {
        auto vy = control_sequence_.vy(offset);
        return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
    }

    return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

void Optimizer::setMotionModel(const std::string & model)
{
    if (model == "DiffDrive") {
        motion_model_ = std::make_shared<DiffDriveMotionModel>();
    } else if (model == "Omni") {
        motion_model_ = std::make_shared<OmniMotionModel>();
    } else if (model == "Ackermann") {
        motion_model_ = std::make_shared<AckermannMotionModel>(nh_);
    } else {
//        throw nav2_core::ControllerException(
//            std::string(
//                "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
//                                   "or Ackermann"));
        ROS_ERROR("Model is not valid! Valid options are DiffDrive, Omni, "
                                   "or Ackermann");
    }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
    auto & s = settings_;
    // TODO: no speed limit in costmap?
    double costmap_no_speed_limit  = 1.0;
    if (speed_limit == costmap_no_speed_limit) {
        s.constraints.vx_max = s.base_constraints.vx_max;
        s.constraints.vx_min = s.base_constraints.vx_min;
        s.constraints.vy = s.base_constraints.vy;
        s.constraints.wz = s.base_constraints.wz;
    }
    else {
        if (percentage) {
            // Speed limit is expressed in % from maximum speed of robot
            double ratio = speed_limit / 100.0;
            s.constraints.vx_max = s.base_constraints.vx_max * ratio;
            s.constraints.vx_min = s.base_constraints.vx_min * ratio;
            s.constraints.vy = s.base_constraints.vy * ratio;
            s.constraints.wz = s.base_constraints.wz * ratio;
        }
        else {
            // Speed limit is expressed in absolute value
            double ratio = speed_limit / s.base_constraints.vx_max;
            s.constraints.vx_max = s.base_constraints.vx_max * ratio;
            s.constraints.vx_min = s.base_constraints.vx_min * ratio;
            s.constraints.vy = s.base_constraints.vy * ratio;
            s.constraints.wz = s.base_constraints.wz * ratio;
        }
    }
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
    return generated_trajectories_;
}

}  // namespace mppi
