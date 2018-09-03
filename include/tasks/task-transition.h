#ifndef __HQP__TASK__TRANSITION__
#define __HQP__TASK__TRANSITION__

#include "robot/robot_model.h"
#include "task-base.h"
#include "task-motion.h"
#include "trajectory-base.h"
#include "constraint/constraint-inequality.h"

namespace HQP
{
	namespace tasks
	{
		class TaskJointLimitTransition : public TaskBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			typedef constraint::ConstraintInequality ConstraintInequality;
			typedef robot::RobotModel RobotModel;
			typedef constraint::ConstraintBase ConstraintBase;

			TaskJointLimitTransition(const std::string & name, RobotModel & robot);

			const ConstraintBase & compute(const double t, Cref_vectorXd q, Cref_vectorXd v);
			const ConstraintBase & getConstraint() const;
			const bool & getTransitionState() { return m_transition; };

			void setJointLimit(const double & q_low, const double & q_high, const int & index);

			const double & Kp();
			const double & Kd();
			void Kp(const double & Kp);
			void Kd(const double &  Kp);
			void setTransitionState(const bool & transition) { m_transition = transition; }
			void setPreviousSol(const VectorXd & old_sol) { m_old_sol = old_sol; }
			double getactivation(const double & lb, const double & ub, const double & alpha, const double & q);

			int dim() const;
		protected:
			double m_Kp;
			double m_Kd;
			VectorXd m_p_error, m_v_error;
			VectorXd m_p, m_v;
			VectorXd m_a_des;
			VectorXi m_activeAxes;
			VectorXd m_old_sol;

			double m_buffer;
			double m_q_lbound, m_q_ubound;
			ConstraintInequality m_constraint;
			double m_velocity_limit_;
			int m_index;
			bool transition_flag;

		};
	}
}

#endif // __HQP__TASK__JOINT_BOUND__
