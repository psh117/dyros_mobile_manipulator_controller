#include <dyros_mobile_manipulator_controllers/hqp_wholebody_controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <franka/robot_state.h>

namespace dyros_mobile_manipulator_controllers
{

bool HQPWholeBodyController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  std::string dyros_controllers_path = ros::package::getPath("dyros_mobile_manipulator_controllers");
  input_file_.open(dyros_controllers_path + "/command/input.txt");
  reflex_client_ = node_handle.serviceClient<franka_control::SetForceTorqueCollisionBehavior>("/franka");

  husky_base_contrl_pub_.init(node_handle,"/cmd_vel", 4);
	std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void HQPWholeBodyController::starting(const ros::Time& time) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();
  husky_cmd_.setZero();
  start_time_ = time;
}


void HQPWholeBodyController::update(const ros::Time& time, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();
  desired_force_torque(2) = desired_mass_ * -9.81;
  tau_ext = - gravity;//tau_measured - gravity - tau_ext_initial_;
  tau_d << jacobian.transpose() * desired_force_torque;
  tau_error_ = tau_error_ + period.toSec() * (tau_d - tau_ext);
  // FF + PI control (PI gains are initially all 0)
  //tau_cmd = gravity;//tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;
  //tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

  input_file_ >> tau_cmd(0) >> tau_cmd(1) >> tau_cmd(2) >> tau_cmd(3) >> tau_cmd(4) >> tau_cmd(5) >> tau_cmd(6) >>
                husky_cmd_(0) >> husky_cmd_(1);


  if(input_file_.eof())
  {
    tau_cmd.setZero();
    husky_cmd_.setZero();
  }
  husky_cmd_(0) = sin((time - start_time_).toSec() * 2 * M_PI / 4) * 0.1;
  //husky_cmd_(1) = sin((time - start_time_).toSec() * 2 * M_PI / 8);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }
  if (rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    ROS_INFO_STREAM("husky :" << husky_cmd_.transpose());
  }

  // HUSKY CONTROl
  if (husky_base_control_trigger_()) {
    if(husky_base_contrl_pub_.trylock())
    {
      husky_base_contrl_pub_.msg_.linear.x = husky_cmd_(0);
      husky_base_contrl_pub_.msg_.angular.z = husky_cmd_(1);
      husky_base_contrl_pub_.unlockAndPublish();
    }
  }
  // Update signals changed online through dynamic reconfigure
  desired_mass_ = filter_gain_ * target_mass_ + (1 - filter_gain_) * desired_mass_;
  k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
  k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
}

Eigen::Matrix<double, 7, 1> HQPWholeBodyController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

} // namespace dyros_mobile_manipulator_controllers



PLUGINLIB_EXPORT_CLASS(dyros_mobile_manipulator_controllers::HQPWholeBodyController,
                       controller_interface::ControllerBase)
