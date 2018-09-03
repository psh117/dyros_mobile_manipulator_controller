#include "tasks/task-operational.h"
#include "utils/utils.h"
FILE *singular;
int cnt;
double h_factor_prev;
double h_factor_LPF;
double singular_val_QR;
using namespace std;
namespace HQP
{
  namespace tasks
  {
    using namespace constraint;
    using namespace trajectories;

	TaskOperationalSpace::TaskOperationalSpace(const std::string & name, RobotModel & robot, const Index & frameid):
      TaskMotion(name, robot),
      m_frame_id(frameid),
      m_constraint(name, 6, robot.nv()),
      m_ref(12, 6)
    {
      m_v_ref.setZero();
      m_a_ref.setZero();
      m_M_ref.setIdentity();
      m_wMl.setIdentity();
      m_p_error_vec.setZero(6);
      m_v_error_vec.setZero(6);
      m_p.resize(12);
      m_v.resize(6);
      m_p_ref.resize(12);
      m_v_ref_vec.resize(6);
      m_Kp.setZero(6);
      m_Kd.setZero(6);
      m_a_des.setZero(6);
      m_J.setZero(6, robot.nv());
	  m_singular = false;
        singular = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/singularity_value.txt","w");
      cnt =0;
    }

    int TaskOperationalSpace::dim() const
    {
      return 6;
    }

    const VectorXd & TaskOperationalSpace::Kp() const { return m_Kp; }

    const VectorXd & TaskOperationalSpace::Kd() const { return m_Kd; }

    void TaskOperationalSpace::Kp(Cref_vectorXd Kp)
    {
      assert(Kp.size()==6);
      m_Kp = Kp;
    }

    void TaskOperationalSpace::Kd(Cref_vectorXd Kd)
    {
      assert(Kd.size()==6);
      m_Kd = Kd;
    }
		
    void TaskOperationalSpace::setReference(TrajectorySample & ref)
    {
      m_ref = ref;
	  m_M_ref.translation() = ref.pos.head<3>();
	  typedef Eigen::Matrix<double, 3, 3> Matrix3;
	  m_M_ref.linear()= Eigen::Map<const Matrix3>(&ref.pos(3), 3, 3);

      m_v_ref = MotionVector<double>(ref.vel);
      m_a_ref = MotionVector<double>(ref.acc);
    }

    const TrajectorySample & TaskOperationalSpace::getReference() const
    {
      return m_ref;
    }

    const VectorXd & TaskOperationalSpace::position_error() const
    {
      return m_p_error_vec;
    }

    const VectorXd & TaskOperationalSpace::velocity_error() const
    {
      return m_v_error_vec;
    }

    const VectorXd & TaskOperationalSpace::position() const
    {
      return m_p;
    }

    const VectorXd & TaskOperationalSpace::velocity() const
    {
      return m_v;
    }

    const VectorXd & TaskOperationalSpace::position_ref() const
    {
      return m_p_ref;
    }

    const VectorXd & TaskOperationalSpace::velocity_ref() const
    {
      return m_v_ref_vec;
    }

    const VectorXd & TaskOperationalSpace::getDesiredAcceleration() const
    {
      return m_a_des;
    }

	VectorXd TaskOperationalSpace::getAcceleration(Cref_vectorXd dv) const
    {
      return m_constraint.matrix()*dv + m_drift.vector();
    }

    //Index TaskOperationalSpace::frame_id() const
    //{
    //  return m_frame_id;
    //}

    const ConstraintBase & TaskOperationalSpace::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskOperationalSpace::compute(const double t, Cref_vectorXd q, Cref_vectorXd v)
    {
	
		Transform3d oMi;
		MotionVector<double> v_frame;

		m_robot.getUpdateKinematics(q, v);
		oMi = m_robot.getTransformation(m_frame_id);

		v_frame = m_robot.getPointVeloecity(m_frame_id);
		m_drift.setZero(); // check acc
		m_wMl.linear() = oMi.linear();

		Transform3d b;
		b = m_M_ref.inverse() * oMi;
		m_p_error = log6(b);

		m_v_error = v_frame - m_v_ref; //check actInv
		m_v_error = actinv(oMi, m_v_error.vector());
		m_p_error_vec = m_p_error.vector();
		m_v_error_vec = m_v_error.vector();

		m_p_ref.head<3>() = m_M_ref.translation();
		typedef Eigen::Matrix<double, 9, 1> Vector9;
		m_p_ref.tail<9>() = Eigen::Map<const Vector9>(&m_M_ref.rotation()(0), 9);

		m_v_ref_vec = m_v_ref.vector();
		m_p.head<3>() = oMi.translation();
		m_p.tail<9>() = Eigen::Map<const Vector9>(&oMi.rotation()(0), 9);

		m_v = v_frame.vector();

		m_a_des = -m_Kp.cwiseProduct(m_p_error.vector())
			- m_Kd.cwiseProduct(m_v_error.vector())
			+ m_a_ref.actInv(m_wMl).vector();

		m_J = m_robot.getJacobian(m_frame_id);

		for (int i = 0; i < m_J.cols(); i++) {
			m_J.middleCols(i, 1) = actinv(oMi, m_J.middleCols(i, 1)).vector();
		} // world jacobian to local jacobian
		

		Eigen::FullPivHouseholderQR<MatrixXd> qr(m_J);
		MatrixXd U, Z, T;

		U = qr.matrixQ();
		VectorXd m_a_des_new;
		MatrixXd m_J_new;

		m_J_new = m_J;
		// Z = qr.matrixR().topLeftCorner(6, dof).triangularView<Upper>();
		// T = qr.colsPermutation();

		if (!m_singular) {
     // cout << "1" <<endl;
			m_constraint.resize(5, m_robot.nv());
			m_a_des_new = U.topLeftCorner(6, 5).transpose() * m_a_des;
			m_J_new = U.topLeftCorner(6, 5).transpose() * m_J;

			m_constraint.setMatrix(m_J_new);
			m_constraint.setVector(m_a_des_new);
			return m_constraint;
		}
		else {
    //  cout <<"2" << endl;
			m_J_new.resize(1, dof);
			m_J_new = U.topRightCorner(6, 1).transpose() * m_J;
			m_constraint.resize(1, m_robot.nv());
    
			double m_singular_value = sqrt((m_J * m_J.transpose()).determinant());
			m_a_des_new = h_factor(m_singular_value, 0.02, 0.005) * U.topRightCorner(6, 1).transpose() * m_a_des;

      // //////////// LPF/////////////////
      double cutoff = 20.0; // Hz
      double RC = 1.0 / (cutoff * 2.0 * M_PI);
      double dt = 0.001;
      double alpha = dt / (RC + dt);
      h_factor_LPF = alpha * h_factor(m_singular_value, 0.02, 0.005) + (1 - alpha) * h_factor_prev;
      h_factor_prev = h_factor_LPF;

      //          singular_val_QR = abs(Z(5,5));
      m_a_des_new = h_factor_LPF * U.topRightCorner(6, 1).transpose() * m_a_des;

      //cout << h_factor(m_singular_value,  0.05, 0.005) <<"  " <<  m_singular_value << endl;
      m_constraint.setMatrix(m_J_new);
      m_constraint.setVector(m_a_des_new);

      if(cnt % 100 == 0 ){
          cout <<  h_factor(m_singular_value, 0.02, 0.005) <<"  " <<  m_singular_value << endl;
      }

      cnt++;
       fprintf(singular,"%lf\t %lf\t \n",m_singular_value,h_factor(m_singular_value, 0.02, 0.005));


			return m_constraint;
		}
    }    
  }
}
