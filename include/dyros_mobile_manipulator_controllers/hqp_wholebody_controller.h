#pragma once

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <thread>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>

#include <franka_control/SetFullCollisionBehavior.h>
#include <franka_control/SetForceTorqueCollisionBehavior.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/trigger_rate.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>

//////////////////////////////////////////////////

#include "controller/Inverse-dynamics.h"

// for tasks
#include "tasks/task-com.h"
#include "tasks/task-operational.h"
#include "tasks/task-joint-posture.h"
#include "tasks/task-joint-bounds.h"
//#include "tasks/task-mobile.h"
#include "tasks/task-singularity.h"
#include "tasks/task-transition.h"

// for trajectories 
#include "trajectories/trajectory-operationalspace.h"
#include "trajectories/trajectory-jointspace.h"
#include "trajectories/trajectory-base.h"
// for solver

#include "solvers/solver-HQP-factory.hxx"
#include "solvers/solver-utils.h"
//#include "solvers/solver-HQP-eiquadprog.h"
#include "solvers/solver-HQP-qpoases.h"
#include "utils/container.h"

#include <string>
#include <vector>
#include <iomanip>



namespace dyros_mobile_manipulator_controllers {

class HQPWholeBodyController : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
							   hardware_interface::EffortJointInterface,
      //              hardware_interface::PositionJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private: 

  realtime_tools::RealtimePublisher<geometry_msgs::Twist> husky_base_contrl_pub_;
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
	  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
	  const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;


  double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f);
  Eigen::Matrix<double, 7, 1> dq_filtered_;
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 7, 1> tau_error_;
  Eigen::Matrix<double, 7, 1> desired_q_;
    Eigen::Matrix<double, 7, 1> joint_acc;
  Eigen::Matrix<double, 7, 1> cubic_q_;
  Eigen::Matrix<double, 7, 1> init_q_;
  Eigen::Matrix<double, 7, 1> q_error;
  Eigen::Matrix<double, 7, 1> tau_cmd;
  Eigen::Matrix<double, 7, 7> k_p_;
  Eigen::Matrix<double, 7, 7> k_d_;
  Eigen::Matrix<double, 7, 1> joint_accel;
  Eigen::Matrix<double, 7, 7> mass_fake;
  Eigen::Vector2d husky_cmd_;

  Transform3d init_T;
  Transform3d goal_T;

  static constexpr double kDeltaTauMax{1.0};
  int cnt_;
  double control_time_;
  double start_time_;
  bool init_flag;
 
 
  ros::ServiceClient reflex_client_;

  franka_hw::TriggerRate rate_trigger_{10};
  franka_hw::TriggerRate husky_base_control_trigger_{100};

	// HQP
  HQP::trajectories::TrajectorySample sampleJoint;
  HQP::trajectories::TrajectorySample s;
  HQP::trajectories::TrajectorySample samplePosture2;


  std::ifstream input_file_;


  bool is_calculation_done_{false};
  std::thread mode_chagne_thread_;
  std::thread async_calculation_thread_;
  std::mutex calculation_mutex_;
  bool quit_all_proc_{false};
  void modeChangeReaderProc();
  void asyncCalculationProc();
};

}  // namespace dyros_mobile_manipulator_controllers
