#include <dyros_mobile_manipulator_controllers/hqp_wholebody_controller.h>
#include <cmath>
#include <memory>
#include <fstream>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <franka/robot_state.h>

#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/resource.h>



FILE *singularity_avoidance;
FILE *hqp_joint_acc;
FILE *hqp_joint_pos;
FILE *hqp_joint_tor;
HQP::robot::RobotModel * robot_;
HQP::InverseDynamics * invdyn_, *invdyn2_, *invdyn_total_;
HQP::tasks::TaskJointPosture * jointTask, * jointCTRLTask1, *jointCTRLTask2;
HQP::tasks::TaskOperationalSpace * moveTask, * move2Task;
HQP::tasks::TaskJointLimit * jointLimitTask, * jointLimit_Acc;
//HQP::contact::Contact3dPoint * contactTask;
HQP::tasks::TaskSingularityAvoidance * singularTask;

HQP::trajectories::TrajectoryJointCubic * trajPosture;
HQP::trajectories::TrajectoryJointConstant * trajPostureConstant, *trajPostureConstant2;
HQP::trajectories::TrajectoryOperationCubic * trajEECubic;
HQP::trajectories::TrajectoryOperationConstant * trajEEConstant;
HQP::trajectories::TrajectoryOperationCircle * trajEECircle;
HQP::tasks::TaskJointLimitTransition * jointlimitTransition;

HQP::solver::SolverHQPBase * solver_;
HQP::solver::SolverHQPBase * solver_2;

VectorXd q_lb(dof); // mobile 2 + robot 7
VectorXd q_ub(dof); // mobile 2 + robot 7
double Hz_ = 1000.0;
int na;
int nv;
int nq;
bool mode_change = false;
bool HQP_flag = false;
int ctrl_mode = 0;
bool flag = false;

#define time_check
#ifdef time_check
#include <time.h>
#include <stdint.h>
#include <stdlib.h>

#define BILLION 1000000000L

int localpid(void) {
 static int a[9] = { 0 };
 return a[0];
}
uint64_t diff;
struct timespec start_, end_;
int i;

#endif

bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

namespace dyros_mobile_manipulator_controllers
{
using namespace HQP;
using namespace std;
double HQPWholeBodyController::Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
	{
		double rx_t;
		if (rT<rT_0)
		{
			rx_t = rx_0;
		}
//   if (cnt_ ==0)
// {

// 	robot_ = new HQP::ro
		else if (rT >= rT_0 && rT<rT_f)
		{
			rx_t = rx_0 + rx_dot_0 * (rT - rT_0)
				+ (3 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2 * rx_dot_0 / ((rT_f - rT_0) * (rT_f - rT_0)) - rx_dot_f / ((rT_f - rT_0) * (rT_f - rT_0)))*(rT - rT_0)*(rT - rT_0)
				+ (-2 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)))*(rT - rT_0)*(rT - rT_0)*(rT - rT_0);
		}
		else
		{
			rx_t = rx_f;
		}
		return (rx_t);
	}
bool HQPWholeBodyController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{



 //////////////////// HQP initial setting //////////////////////////////////

  std::string dyros_controllers_path = ros::package::getPath("dyros_mobile_manipulator_controllers");
  std::string file_name = dyros_controllers_path + "/command/hqp_joint_pos_obs.txt";
  //cout << dyros_controllers_path << endl;
  singularity_avoidance = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/singularity_position_qtol.txt","w");
  hqp_joint_acc = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/hqp_joint_acc.txt","w");
  hqp_joint_pos = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/hqp_joint_pos.txt","w");
  hqp_joint_tor = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/hqp_joint_tor.txt","w");

  ROS_INFO("Openning file ... %s ", file_name.c_str());
  input_file_.open(file_name);
  
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

  ///////////// joint position handle ///////////////
  // auto* position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
  // if (position_joint_interface_ == nullptr) {
  //   ROS_ERROR(
  //       "JointPositionExampleController: Error getting position joint interface from hardware!");
  //   return false;
  // }

  // position_joint_handles_.resize(7);
  // for (size_t i = 0; i < 7; ++i) {
  //   try {
  //     position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
  //   } catch (const hardware_interface::HardwareInterfaceException& e) {
  //     ROS_ERROR_STREAM(
  //         "JointPositionExampleController: Exception getting joint handles: " << e.what());
  //     return false;
  //   }
  // }
 

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
  desired_q_.setZero();
  cubic_q_.setZero();
  tau_cmd.setZero();
  start_time_ = 0.0;
  for (size_t i = 0; i < 7; i++) {
    init_q_(i) = robot_state.q[i];
  }
  cnt_ = 0;
  init_flag = true;

  k_p_.setIdentity();
  k_d_.setIdentity();
  joint_acc.setZero();
  mass_fake.setZero();
  
  
  ////////////// HQP Task setting ////////////////////

  robot_ = new HQP::robot::RobotModel(0); // 0: Manipulator, 1: Mobile Manipulaotr, 2: humanoid
	na = robot_->na();	
	nv = robot_->nv();		


	invdyn_ = new HQP::InverseDynamics(*robot_);
	invdyn2_ = new HQP::InverseDynamics(*robot_);
	invdyn_total_ = new HQP::InverseDynamics(*robot_);
	// Level 0 : Joint Velocity Limit for Mobile + Manipulator 
	q_lb = -1000.0 * VectorXd(dof).setOnes();
	q_ub = -1.0*q_lb;

	// q_lb(0) = -10.0 * M_PI / 180.0;
	// q_ub(0) = - q_lb(0);

	double kp_jointlimit = 100.0, w_jointlimit = 1.00;

	jointLimit_Acc = new tasks::TaskJointLimit("joint_acc_limit_task", *robot_);
	jointLimit_Acc->Kp(kp_jointlimit*VectorXd::Ones(robot_->nv()));
	jointLimit_Acc->Kd(2.0*jointLimit_Acc->Kp().cwiseSqrt());
	jointLimit_Acc->setJointLimit(q_lb, q_ub);
 
 // q_ub(0) = 10.0/180.0*M_PI;


	jointLimitTask = new tasks::TaskJointLimit("joint_limit_task", *robot_);
	jointLimitTask->Kp(kp_jointlimit*VectorXd::Ones(robot_->nv()));
	jointLimitTask->Kd(2.0*jointLimitTask->Kp().cwiseSqrt());
	jointLimitTask->setJointLimit(q_lb, q_ub);

	jointTask = new tasks::TaskJointPosture("joint_control_task", *robot_);
  double kp_posture = 400.0, w_posture = 1.00;
	jointTask->Kp(kp_posture*VectorXd::Ones(robot_->nv()));
	jointTask->Kd(1.0*jointTask->Kp().cwiseSqrt());

	moveTask = new tasks::TaskOperationalSpace("op_control_task", *robot_, 7);
  double kp_move = 460.0, w_move = 1.0;
	VectorXd a = VectorXd::Ones(6);
  a.tail(3) *= 1.0;
	moveTask->Kp(kp_move*a);
	moveTask->Kd(1.0*moveTask->Kp().cwiseSqrt());
	moveTask->setSingular(false);

	move2Task = new tasks::TaskOperationalSpace("end_effector_task2", *robot_, 7);
	move2Task->Kp(kp_move*a);
	move2Task->Kd(1.0*move2Task->Kp().cwiseSqrt());
	move2Task->setSingular(true);

	sampleJoint.resize(robot_->nv());
	s.resize(12, 6);


	// Task#2 Joint Ctrl for Initial Posture
	jointCTRLTask2 = new tasks::TaskJointPosture("joint_control_task2", *robot_);
	jointCTRLTask2->Kp(kp_posture*VectorXd::Ones(robot_->nv()));
	jointCTRLTask2->Kd(1.0*jointCTRLTask2->Kp().cwiseSqrt());
	//invdyn_->addJointPostureTask(*jointCTRLTask2, 0.0, 2, 0.0);

	jointlimitTransition = new tasks::TaskJointLimitTransition("joint_transition", *robot_);
	jointlimitTransition->Kp(400.0);
	jointlimitTransition->Kd(pow(400.0, 0.5));
	jointlimitTransition->setJointLimit(-10.0/180*M_PI, 10.0/180.0*M_PI, 0);



  solver_ = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_QPOASES, "solver-eiquadprog");
  solver_2 = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_QPOASES, "solver-eiquadprog");

	solver_->resize(invdyn_->nVar(), invdyn_->nEq(), invdyn_->nIn(), invdyn_->nBound());
	solver_2->resize(invdyn_->nVar(), invdyn_->nEq(), invdyn_->nIn(), invdyn_->nBound());

  // invdyn_ = new HQP::InverseDynamics(*robot_);
  // invdyn_->addJointLimitTask(*jointLimitTask, 1.0, 0, 0.0);
  // invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
  // invdyn_->addOperationalTask(*move2Task, w_move, 1, 0.0);
  // invdyn_->addJointPostureTask(*jointTask, 1.0, 2, 0.0); //weight, level, duration



}


void HQPWholeBodyController::update(const ros::Time& time, const ros::Duration& period) {

  if (_kbhit())
  {
    int key;
    key = getchar();
    key = tolower(key);
    switch (key)
    {
    case 'h': // for home position joint ctrl
      ctrl_mode = 1;
      mode_change = true;
      HQP_flag = true;
      break;
    case 'i': // for init position joint ctrl
      ctrl_mode = 2;
      mode_change = true;
      HQP_flag = true;
      cout <<"111" << endl;
      break; 
    case 's': // for singularity task
      ctrl_mode = 3;
      mode_change = true;
      HQP_flag = true;
      break;
    case 'd':
      ctrl_mode = 4;
      mode_change = true; 
      break;
    case 'j':
      ctrl_mode = 5;
      mode_change = true;
      break; 
    default:
      mode_change = true;
      ctrl_mode = 0;
      break;
    }
  }

  franka::RobotState robot_state = state_handle_->getRobotState();

  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 49> massmatrix_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_matrix(massmatrix_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> non_linear(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> joint_pos(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> joint_vel(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
//  Eigen::Matrix3d orientation(transform.linear());
  // for(int i=0;i<dof;i++){
  //   mass_fake(i) = mass_matrix(i,i);
  // }
  control_time_ = cnt_/Hz_;
	robot_->getUpdateKinematics(joint_pos, dq_filtered_);

 ///////////////// Init & Goal Configuration Statement /////////////////////
  // if(init_flag){
  // init_T = robot_->getTransformation(7);
  // goal_T = init_T;
  // goal_T.translation()(2) += 0.3;

  // // trajEECubic = new trajectories::TrajectoryOperationCubic("op_traj");
  // // trajEECubic->setInitSample(init_T);
  // // trajEECubic->setGoalSample(goal_T);
  // // trajEECubic->setDuration(15.0);
  // // trajEECubic->setStartTime(start_time_);
  // // trajEECubic->setReference(goal_T);

  // // trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
  // // trajPosture->setInitSample(init_q_);
  // // trajPosture->setGoalSample(desired_q_);
  // // trajPosture->setDuration(15.0);
  // // trajPosture->setStartTime(start_time_);
  // // trajPosture->setReference(desired_q_);

  // trajEECircle = new trajectories::TrajectoryOperationCircle("opcircle_traj");
  // trajEECircle->setInitSample(init_T);
  // trajEECircle->setRadius(0.2);
  // trajEECircle->setDuration(60.0);
  // trajEECircle->setStartTime(start_time_);
  // trajEECircle->setReference(goal_T);

  // trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
  // trajPosture->setInitSample(init_q_);
  // trajPosture->setGoalSample(desired_q_);
  // trajPosture->setDuration(15.0);
  // trajPosture->setStartTime(start_time_);
  // trajPosture->setReference(desired_q_);
  // init_flag = false;
  // }

  if (ctrl_mode == 0){

    if (mode_change)
    {
      cout << "Gravity compensation" << endl;
      mode_change = false;
    }
    tau_cmd = non_linear;
  }
  else if (ctrl_mode == 1)
  {
    if (mode_change)
    {
      cout << "Home position (joint control using HQP)" << endl;
      cout << "This mode has joint limit avoidance" << endl;

      // Level 0 : Joint Velocity Limit for Mobile + Manipulator
      //	invdyn_->resizeHqpData();
      invdyn_ = new HQP::InverseDynamics(*robot_);
      invdyn_->addJointLimitTask(*jointLimitTask, 1.0, 0, 0.0);
      invdyn_->addJointPostureTask(*jointTask, 1.0, 1, 0.0); //weight, level, duration

      init_q_ = joint_pos;
      start_time_ = control_time_;
      // combined task
      desired_q_.setZero();
      desired_q_(0) = 0.0/180.0*M_PI;
      desired_q_(1) = 30.0/180.0*M_PI;
      desired_q_(3) = -120.0/180.0*M_PI;
      desired_q_(5) = 150.0/180.0*M_PI;
      
      // desired_q_.setZero();
      // desired_q_(1) = -90.0/180.0*M_PI;
      // desired_q_(3) = -90.0/180.0*M_PI;
      // desired_q_(5) = 90.0/180.0*M_PI;
      // desired_q_(6) = -45.0/180.0*M_PI; 
      //  desired_q_.setZero();
      //  desired_q_(1) = 30.0/180.0*M_PI;
      //  desired_q_(3) = -120.0/180.0*M_PI;
      //  desired_q_(5) = 150.0/180.0*M_PI;

    
      trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
      trajPosture->setInitSample(init_q_);
      trajPosture->setGoalSample(desired_q_);
      trajPosture->setDuration(10.0);
      trajPosture->setStartTime(control_time_);
      trajPosture->setReference(desired_q_);
  
      mode_change = false;
    }
    trajPosture->setCurrentTime(control_time_);
    sampleJoint = trajPosture->computeNext();
    jointTask->setReference(sampleJoint);

    const solver::HQPData &HQPData = invdyn_->computeProblemData(control_time_, joint_pos, dq_filtered_,true);
    // if (HQP_flag)      desired_q_.setZero();
    //   desired_q_(0) = 0.0/180.0*M_PI;
    //   desired_q_(1) = 30.0/180.0*M_PI;
    //   desired_q_(3) = -120.0/180.0*M_PI;
    //   desired_q_(5) = 150.0/180.0*M_PI;

    // {
    //   cout << solver::HQPDataToString(HQPData, true) << endl;
    //   HQP_flag = false;
    // }

    const solver::HQPOutput &sol = solver_->solve(HQPData);

    const VectorXd &joint_acc = invdyn_->getActuatorForces(sol);
    tau_cmd =  mass_matrix*( joint_acc ) + non_linear;
    //cout << tau_cmd.transpose() << endl;
  }
  else if(ctrl_mode == 3){
    if(mode_change){
      double w_move = 1.0;
      invdyn_ = new HQP::InverseDynamics(*robot_);
      invdyn_->addJointLimitTask(*jointLimitTask, 1.0, 0, 0.0);
      invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
      invdyn_->addOperationalTask(*move2Task, w_move, 1, 0.0);
      invdyn_->addJointPostureTask(*jointTask, 1.0, 2, 0.0); //weight, level, duration

      init_q_ = joint_pos;
      init_T = robot_->getTransformation(7);
      //goal_T = init_T;
      //goal_T.translation()(2) += 0.3;
      desired_q_.setZero();
      desired_q_(1) = -70.0/180.0*M_PI;
      desired_q_(3) = -160.0/180.0*M_PI;
      desired_q_(5) = 90.0/180.0*M_PI;
  


      trajEECircle = new trajectories::TrajectoryOperationCircle("opcircle_traj");
      trajEECircle->setInitSample(init_T);
      trajEECircle->setRadius(0.25); // good case : 0.22
      trajEECircle->setDuration(200.0);
      trajEECircle->setStartTime(control_time_);
      trajEECircle->setReference(goal_T);

      trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
      trajPosture->setInitSample(init_q_);
      trajPosture->setGoalSample(desired_q_);
      trajPosture->setDuration(15.0);
      trajPosture->setStartTime(control_time_);
      trajPosture->setReference(desired_q_);
      k_p_ = 2.0*k_p_;
      k_p_(6,6) = 2.0;
      k_d_ = sqrt(10.0)*k_d_;
      mode_change = false;
    }
    trajEECircle->setCurrentTime(control_time_);
    s = trajEECircle->computeNext();
    moveTask->setReference(s);
    move2Task->setReference(s);

    trajPosture->setCurrentTime(control_time_);
    sampleJoint = trajPosture->computeNext();
    jointTask->setReference(sampleJoint);

    const solver::HQPData &HQPData = invdyn_->computeProblemData(control_time_, joint_pos, dq_filtered_,true);
    if (HQP_flag)
    {
      cout << solver::HQPDataToString(HQPData, true) << endl;
      HQP_flag = false;
    }
    const solver::HQPOutput &sol = solver_->solve(HQPData);
    VectorXd joint_acc = invdyn_->getActuatorForces(sol);
    fprintf(hqp_joint_acc, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", joint_acc(0), joint_acc(1), joint_acc(2), joint_acc(3), joint_acc(4), joint_acc(5), joint_acc(6));
    joint_acc(4) = 3.0*joint_acc(4);
    joint_acc(5) = 3.0*joint_acc(5);
    joint_acc(6) = 3.0*joint_acc(6);
    tau_cmd = mass_matrix * ( joint_acc )  + non_linear ;
    
    //tau_cmd = mass_fake*joint_acc;
    fprintf(singularity_avoidance,"%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t \n",s.pos.head(3)(0),s.pos.head(3)(1),s.pos.head(3)(2),position(0),position(1),position(2));
    fprintf(hqp_joint_pos, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", joint_pos(0), joint_pos(1), joint_pos(2), joint_pos(3), joint_pos(4), joint_pos(5), joint_pos(6));

  }
  else if (ctrl_mode == 4){
    if(mode_change){
      start_time_ = control_time_;
      init_q_ = joint_pos;
      desired_q_.setZero();
      desired_q_(1) = 30.0 / 180.0 * M_PI;
      desired_q_(3) = -120.0 / 180.0 * M_PI;
      desired_q_(5) = 150.0 / 180.0 * M_PI;

      mode_change = false;
    }


    for(int i=0;i<dof;i++){
      cubic_q_(i) = Cubic(control_time_ , start_time_, start_time_ + 5.0, init_q_(i), 0.0, desired_q_(i), 0.0);
    }

    tau_cmd =  (k_p_*(cubic_q_ - joint_pos) - k_d_ * joint_vel) + non_linear;

  }
  else if(ctrl_mode == 5){

// #ifdef time_check
// clock_gettime(CLOCK_MONOTONIC, &start_); /* mark start time */
// #endif	
//     ros::Duration d1;
//     ros::Duration d2;
//     ros::Duration d3;
    if(mode_change){

      double w_move = 1.0;
     	invdyn_->addOperationalTask(*moveTask, 0.0, 1, 0.0);
     	invdyn_->addJointPostureTask(*jointCTRLTask2, 0.0, 2, 0.0);
	 
      goal_T = robot_->getTransformation(7);
      goal_T.translation()(1) += 0.4;
     // cout << "T_des" << Tdes.translation().transpose() << endl;
      trajEECubic = new trajectories::TrajectoryOperationCubic("op_cubic");
      trajEECubic->setInitSample(robot_->getTransformation(7));
      trajEECubic->setGoalSample(goal_T);
      trajEECubic->setStartTime(control_time_);
      trajEECubic->setDuration(5.0);
      trajEECubic->setReference(goal_T);

      // for joint control
      desired_q_.setZero();
      desired_q_(1) = 30.0 / 180.0 * M_PI;
      desired_q_(3) = -120.0 / 180.0 * M_PI;
      desired_q_(5) = 150.0 / 180.0 * M_PI;

	    trajPostureConstant2 = new trajectories::TrajectoryJointConstant("joint_traj_initial");
			trajPostureConstant2->setReference(desired_q_);					


      mode_change = false;

    }

		trajEECubic->setCurrentTime(control_time_);
		s = trajEECubic->computeNext();
		moveTask->setReference(s);

    //trajPosture->setCurrentTime(control_time_);
    samplePosture2 = trajPostureConstant2->computeNext();
    jointCTRLTask2->setReference(samplePosture2);


			if (joint_pos(0) > 5.0/180.0*M_PI && !flag) {
					invdyn_->addJointLimitTransitionTask(*jointlimitTransition, 0.0, 0, 0.0);
					flag = true;
			}
			if (joint_pos(0) > 5.0/180.0*M_PI) {
					const solver::HQPData & HQPData = invdyn_->computeProblemData(control_time_, joint_pos, dq_filtered_,false);
					const solver::HQPOutput & sol = solver_->solve(HQPData);
				   joint_acc = invdyn_->getActuatorForces(sol);
			}
			else {
					const solver::HQPData & HQPData = invdyn_->computeProblemData(control_time_, joint_pos, dq_filtered_,false);
					const solver::HQPOutput & sol = solver_->solve(HQPData);
				 joint_acc = invdyn_->getActuatorForces(sol);
		  }
    tau_cmd =  mass_matrix*( joint_acc ) + non_linear;


  }
  else{

    if (mode_change)
    {
      cout << "Gravity compensation" << endl;
      mode_change = false;
    }
    tau_cmd = non_linear;

  }


  ///////////// Joint Velocity - Low Pass Filter //////////////////
  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_(i) = (1 - alpha) * dq_filtered_(i) + alpha * robot_state.dq[i];
  }

  Transform3d a =  robot_->getTransformation(7);
  q_error = desired_q_ - joint_pos;


/////////////// HQP solving & desired torque ////////////////////////////
  //  const solver::HQPData &HQPData = invdyn_->computeProblemData(control_time_, joint_pos, dq_filtered_);
  //  const solver::HQPOutput &sol = solver_->solve(HQPData);
  //  const VectorXd &joint_acc = invdyn_->getActuatorForces(sol);


  //  if(control_time_ == 15.0 || control_time_ == 20.0){
  //    ROSinput_file_INFO_STREAM("end_ee1" << (goal_T.translation()- a.translation()).transpose());
  //  }

  // if (cninput_file__/1000.0 > 2.0){
  //  inputinput_file_file_ >> tau_cmd(0) >> tau_cmd(1) >> tau_cmd(2) >> tau_cmd(3) >> tau_cmd(4) >> tau_cmd(5) >> tau_cmd(6) ;
  //   posiinput_file_ion_joint_handles_[i].setCommand(tau_cmd(i));
  // }
  // else{
  // for (size_t i = 0; i < 7; ++i) {
  // //  joint_handles_[i].setCommand(tau_cmd(i));
  // position_joint_handles_[i].setCommand(init_q_(i));
  // }
  //   }
//   desired_q_.setZero();
//   desired_q_(0) = 0.0/180.0*M_PI;
//   desired_q_(1) = 30.0/180.0*M_PI;
//   desired_q_(3) = -120.0/180.0*M_PI;
//   desired_q_(5) = 150.0/180.0*M_PI;
//   desired_q_(6) = -45.0/180.0*M_PI;

// for(int i=0;i<7;i++)
//  cubic_q_(i) = Cubic(cnt_, 0, 5000, init_q_(i), 0.0, desired_q_(i),0.0);

  input_file_ >> desired_q_(0) >> desired_q_(1) >> desired_q_(2) >> desired_q_(3) >> desired_q_(4) >> desired_q_(5) >> desired_q_(6) ;
  MatrixXd Gain;
  Gain.resize(7,7);
  Gain.setIdentity();
  Gain = 300.0*Gain;
  Gain(5,5) = 500.0;
  Gain(6,6) = 500.0;

  //tau_cmd = mass_matrix * (Gain* (desired_q_ -joint_pos) - 20.0*joint_vel) + non_linear;
  //fprintf(hqp_joint_tor, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", tau_cmd(0), tau_cmd(1), tau_cmd(2), tau_cmd(3), tau_cmd(4), tau_cmd(5), tau_cmd(6));

  for(int i=0;i<7;i++){
    joint_handles_[i].setCommand(tau_cmd(i));
  }

  // if(input_file_.eof())
  // {
  //   tau_cmd = joint_pos;
  // }

   if (rate_trigger_())
   {
   // ROS_INFO("--------------------------------------------------");
 //   ROS_INFO_STREAM("init_q : " << init_q_.transpose());
     // ROS_INFO_STREAM("cubic_q : " << cubic_q_.transpose() );
     // ROS_INFO_STREAM("current q :" << joint_pos.transpose());
  //   ROS_INFO_STREAM("q error :" << q_error.transpose());
     //cout << mass_matrix << endl;
   //  ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    // ROS_INFO_STREAM("husky :" << husky_cmd_.transpose());
     //ROS_INFO_STREAM("end_ee1" << (goal_T.translation()- init_T.translation()).transpose());
  //  ROS_INFO_STREAM("end_ee2" << (s.pos.head(3) - position).transpose());
   // ROS_INFO_STREAM("joint acc :" << joint_accel.transpose());
  }
 // tau_cmd.setZero();
  //husky_cmd_(0) = sin((time - start_time_).toSec() * 2 * M_PI / 4) * 0.1;
  //husky_cmd_(1) = sin((time - start_time_).toSec() * 2 * M_PI / 8);
 // cout << "1" << tau_cmd.transpose() <<endl;
 // cout << "2" <<joint_pos.transpose() <<endl;
 // cout << init_q_.transpose() << endl;
  //position_joint_handles_[i].setCommand(cubic_q_(i));

    husky_cmd_.setZero();

  // HUSKY CONTROl
  if (husky_base_control_trigger_()) {
    if(husky_base_contrl_pub_.trylock())
    {
      husky_base_contrl_pub_.msg_.linear.x = husky_cmd_(0);
      husky_base_contrl_pub_.msg_.angular.z = husky_cmd_(1);
      husky_base_contrl_pub_.unlockAndPublish();
    }
  }

  cnt_++;


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
