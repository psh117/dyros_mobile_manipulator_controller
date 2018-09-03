#include "tasks/task-transition.h"
#include "utils/utils.h"
namespace HQP
{
	namespace tasks
	{
		using namespace constraint;
		using namespace trajectories;

		TaskJointLimitTransition::TaskJointLimitTransition(const std::string & name, RobotModel & robot)
			: TaskBase(name, robot), m_constraint(name, 1, robot.nv()) {
			m_robot = robot;
			m_velocity_limit_ = 2000.0;
			m_constraint.setLowerBound(-m_velocity_limit_ * VectorXd(1).setOnes());
			m_constraint.setUpperBound(m_velocity_limit_ * VectorXd(1).setOnes());
			m_Kp = 0.0; 
			m_Kd = 0.0;
		
			m_buffer = 5.0 * M_PI / 180.0;
			m_old_sol.resize(robot.nv());
			m_old_sol.setZero();
			transition_flag = false;
		}

		double TaskJointLimitTransition::getactivation(const double & lb, const double & ub, const double & alpha, const double & q) {
			double h;
			if (q > lb + alpha && q < ub - alpha)
				h = 0;
			else if (lb < q < lb + alpha)
				h = 0.5;
			else if (q < lb)
				h = 1;
			else if (q > ub - alpha)
				h = h_factor(pow(q, 1), pow(ub, 1), pow(ub - alpha, 1));
				//h = 1;
			else if (q > ub)
				h = 1;

			return h;
		}
		const ConstraintBase & TaskJointLimitTransition::compute(const double t, Cref_vectorXd q, Cref_vectorXd v) {
			if (!transition_flag) {
				if (q(m_index) < m_q_lbound + m_buffer) {
					m_constraint.lowerBound()(0) = m_Kp * ((m_q_lbound + m_buffer) - q(m_index)) - m_Kd * v(m_index);
					m_constraint.upperBound()(0) = m_velocity_limit_;
					if (m_constraint.lowerBound()(0) > m_velocity_limit_)
						m_constraint.lowerBound()(0) = m_velocity_limit_;
				}
				else if (q(m_index) > m_q_ubound - m_buffer) {
					m_constraint.upperBound()(0) = (m_Kp * ((m_q_ubound - m_buffer) - q(m_index)) - m_Kd * v(m_index));
					m_constraint.lowerBound()(0) = m_velocity_limit_;// m_constraint.upperBound()(i); // 

					if (m_constraint.upperBound()(0) <  m_constraint.lowerBound()(0))
						m_constraint.lowerBound()(0) = m_constraint.upperBound()(0);
				}

			}
			else {
				double h = getactivation(m_q_lbound, m_q_ubound, m_buffer, q(m_index));
				if (q(m_index) < m_q_lbound + m_buffer) {
					m_constraint.lowerBound()(0) = m_Kp * ((m_q_lbound + m_buffer) - q(m_index)) - m_Kd * v(m_index);
					m_constraint.upperBound()(0) = m_velocity_limit_;

					if (m_constraint.lowerBound()(0) > m_velocity_limit_)
						m_constraint.lowerBound()(0) = m_velocity_limit_;
					//	m_constraint.upperBound()(0) = m_velocity_limit_;
				}
				else if (q(m_index) > m_q_ubound - m_buffer) {
					m_constraint.upperBound()(0) = h * (m_Kp * ((m_q_ubound - m_buffer) - q(m_index)) - m_Kd * v(m_index)) + (1 - h) * m_old_sol(m_index);
					m_constraint.lowerBound()(0) = -pow(h, 5) * m_velocity_limit_ + (1 - pow(h, 5)) * m_old_sol(m_index);// m_constraint.upperBound()(i); // 

					if (m_constraint.upperBound()(0) < m_constraint.lowerBound()(0))
						m_constraint.lowerBound()(0) = m_constraint.upperBound()(0);
				}
				else {
					m_constraint.upperBound()(0) = h * m_velocity_limit_ + (1 - h) * m_old_sol(m_index);
					m_constraint.lowerBound()(0) = -h * m_velocity_limit_ + (1 - h) * m_old_sol(m_index);
				}
			}
			MatrixXd A(1, m_robot.nv());
			A.setZero();
			A(m_index) = 1.0;
			m_constraint.setMatrix(A);

			return m_constraint;
			
			}
		const ConstraintBase & TaskJointLimitTransition::getConstraint() const {
			return m_constraint;
		}
		int TaskJointLimitTransition::dim() const
		{
			return m_robot.nv();
		}
		void TaskJointLimitTransition::setJointLimit(const double & q_low, const double & q_high, const int & index) {
			m_q_lbound = q_low;
			m_q_ubound = q_high;
			m_index = index;
		}
		const double & TaskJointLimitTransition::Kp() { return m_Kp; }

		const double & TaskJointLimitTransition::Kd() { return m_Kd; }

		void TaskJointLimitTransition::Kp(const double & Kp)
		{
			m_Kp = Kp;
		}

		void TaskJointLimitTransition::Kd(const double & Kd)
		{
			m_Kd = Kd;
		}
	}
}
