#include "robot/robot_model.h"
#include <vector>

using namespace std;

HQP::robot::RobotModel::RobotModel(int robottype) {
	model_ = new Model();
	model_->gravity = Eigen::Vector3d(0., 0., -9.81);

	m_robot_type_ = robottype;
	if (m_robot_type_ == 0) {
		m_nq_ = dof;
		m_nv_ = dof;
		m_na_ = dof;
	}
	else if (m_robot_type_ == 1) {
		m_nq_ = dof + 2;
		m_nv_ = dof + 2;
		m_na_ = dof;
	}
	else if (m_robot_type_ == 2) {
		m_nq_ = dof + 6;
		m_nv_ = dof + 6;
		m_na_ = dof;
	}

	q_rbdl_.resize(m_na_);
	qdot_rbdl_.resize(m_na_);
	q_rbdl_.setZero();
	qdot_rbdl_.setZero();

	qddot_rbdl_.resize(m_na_);
	qddot_rbdl_.setZero();

	m_NLE_.resize(m_na_ ); // 
	m_NLE_.setZero();
	m_Mass_mat_.resize(m_na_, m_na_ ); // 
	m_Mass_mat_.setZero();
	m_J_.resize(6, m_na_); // 
	m_J_.setZero();

	m_Ori_.resize(3, 3);
	m_pos_.setZero();
	m_Ori_.setZero();
	m_Trans_.linear().setZero();
	m_Trans_.translation().setZero();

	tau_.resize(m_na_);
	tau_.setZero();

	q_real_.resize(m_nv_);
	q_real_.setZero();
	qdot_real_.resize(m_nv_);
	qdot_real_.setZero();

	m_selection_.resize(m_na_, m_na_);
	m_selection_.setZero();
	m_selection_dot_.resize(m_na_, m_na_);
	m_selection_dot_.setZero();
	m_Mass_virtual_mat_.resize(m_na_,m_na_);
	m_Mass_virtual_mat_.setZero();
	setRobot();
}
HQP::robot::RobotModel::~RobotModel() {

}

void HQP::robot::RobotModel::setRobot() {
    mass_[0] = 6.98;
    mass_[1] = 1.0;
    mass_[2] = 5.23;
    mass_[3] = 6.99;
    mass_[4] = 3.3;
    mass_[5] = 2.59;
    mass_[6] = 1.0;


    inertia_[0] = Vector3d(0.02854672, 0.02411810, 0.01684034);
    inertia_[1] = Vector3d(0.00262692, 0.00281948, 0.00214297);
    inertia_[2] = Vector3d(0.04197161, 0.00856546, 0.04186745);
    inertia_[3] = Vector3d(0.04906429, 0.03081099, 0.02803779);
    inertia_[4] = Vector3d(0.00935279, 0.00485657, 0.00838836);
    inertia_[5] = Vector3d(0.00684717, 0.00659219, 0.00323356);
    inertia_[6] = Vector3d(0.00200000, 0.00200000, 0.00200000);
	
	axis_[0] = 1.0*Eigen::Vector3d::UnitZ();
	axis_[1] = 1.0*Eigen::Vector3d::UnitY();
	axis_[2] = 1.0*Eigen::Vector3d::UnitZ();
	axis_[3] = -1.0*Eigen::Vector3d::UnitY();
	axis_[4] = 1.0*Eigen::Vector3d::UnitZ();
	axis_[5] = -1.0*Eigen::Vector3d::UnitY();
	axis_[6] = -1.0*Eigen::Vector3d::UnitZ();

	joint_position_global_[0] = Eigen::Vector3d(0.0, 0.0, 0.333);
	joint_position_global_[1] = Eigen::Vector3d(0.0, 0.0, 0.333);
	joint_position_global_[2] = Eigen::Vector3d(0.0, 0.0, 0.649);
	joint_position_global_[3] = Eigen::Vector3d(0.0825, 0.0, 0.649);
	joint_position_global_[4] = Eigen::Vector3d(0.0, 0.0, 1.033);
	joint_position_global_[5] = Eigen::Vector3d(0.0, 0.0, 1.033);
	joint_position_global_[6] = Eigen::Vector3d(0.088, 0.0, 1.033);
	
	joint_position_local_[0] = joint_position_global_[0];

	for (int i=1; i < dof; i++)
		joint_position_local_[i] = joint_position_global_[i] - joint_position_global_[i-1];

	com_position_[0] = Vector3d(-0.00006, 0.04592, 0.20411);
	com_position_[1] = Vector3d(0.00007, 0.12869, 0.24008);
	com_position_[2] = Vector3d(0.00043, 0.1205, 0.38421);
	com_position_[3] = Vector3d(-0.00008, 0.05518, 0.59866);
	com_position_[4] = Vector3d(0.0, 0.01606, 0.74571);
	com_position_[5] = Vector3d(0.00002, 0.11355, 0.91399);
	com_position_[6] = Vector3d(0.088, 0.0, 0.926);

	for (int i = 0; i < dof; i++)
		com_position_[i] -=  joint_position_global_[i];

	for (int i = 0; i < dof; i++) {
		body_[i] = Body(mass_[i], com_position_[i], inertia_[i]);
		joint_[i] = Joint(JointTypeRevolute, axis_[i]);

		if (i == 0)
			body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	}
}
void HQP::robot::RobotModel::Jacobian(const int & frame_id) { //?
	MatrixXd J_temp(6, m_na_);
	J_temp.setZero();
	if (frame_id == 0) {
		CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id ], -1.0*joint_position_local_[frame_id], J_temp, true);
	}
	else {
		CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], J_temp, true);
	}
	for (int i = 0; i < 2; i++) {
		m_J_.block(i * 3, 0, 3, m_na_) = J_temp.block(3 - i * 3, 0, 3, m_na_);
	}
}
void HQP::robot::RobotModel::Position(const int & frame_id) { // for mobile
	m_pos_ = CalcBodyToBaseCoordinates(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], true);
}
void HQP::robot::RobotModel::Orientation(const int & frame_id) { // for mobile
	m_Ori_ = CalcBodyWorldOrientation(*model_, q_rbdl_, body_id_[frame_id - 1], true).transpose();
}
void HQP::robot::RobotModel::Transformation(const int & frame_id) { // for mobile
	Position(frame_id);
	Orientation(frame_id);
	m_Trans_.linear() = m_Ori_;
	m_Trans_.translation() = m_pos_;
}
void HQP::robot::RobotModel::NLEtorque() { // for mobile
	VectorXd NLE_virtual(m_na_);
	InverseDynamics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_, tau_);
	m_NLE_ = tau_;
}
void HQP::robot::RobotModel::MassMatrix() { // for mobile
	CompositeRigidBodyAlgorithm(*model_, q_rbdl_, m_Mass_virtual_mat_, true);
	m_Mass_mat_ =m_Mass_virtual_mat_;
}
void HQP::robot::RobotModel::Manipulability(const VectorXd &  q) {
	Jacobian(7);
	m_manipulability_[0] = sqrt( (m_J_* m_J_.transpose()).determinant());
}
void HQP::robot::RobotModel::ManipulabilityJacobian() {	
	m_J_manipulability_[0].resize(dof / 2);
	VectorXd q = q_rbdl_;
	Manipulability(q);
	const double mani_0 = m_manipulability_[0];
	const double h = 0.0000001;
	for (int i = 0; i < dof; i++) {
		q(i) += h;
		Manipulability(q);
		m_J_manipulability_[0](i) = (m_manipulability_[0] - mani_0) / h;
		q = q_rbdl_;
	}
}
void HQP::robot::RobotModel::getUpdateKinematics(const VectorXd & q, const VectorXd & qdot) { // for mobile
	q_rbdl_ = q;
	qdot_rbdl_ = qdot;	
	qddot_rbdl_.setZero();
	UpdateKinematics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_);	
}

void HQP::robot::RobotModel::PointVelocity(const int & frame_id) {
	p_dot_ = CalcPointVelocity6D(*model_, q_rbdl_, qdot_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1]);
	m_p_dot_.angular() = p_dot_.head<3>();
	m_p_dot_.linear() = p_dot_.tail<3>();
}

