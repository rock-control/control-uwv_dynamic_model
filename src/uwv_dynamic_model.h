/***************************************************************************/
/*  Dynamic model for a underwater vehicle         	                   */
/*                                                                         */
/* FILE --- uwv_dynamic_model.h		                                   */
/*                                                                         */
/* PURPOSE --- Header file for a dynamic model of an	                   */
/*             underwater vehicle. Based on T.I.Fossen & Giovanni Indiveri */
/*                                                                         */
/*  Sankaranarayanan Natarajan                                             */
/*  sankar.natarajan@dfki.de                                               */
/*  DFKI - BREMEN 2011                                                     */
/***************************************************************************/
#ifndef _UWV_DYNAMIC_MODEL_H_
#define _UWV_DYNAMIC_MODEL_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "RK4Integrator.hpp"	// need this header for simulation - it solves a system of n first order differential equations.
#include "uwv_dataTypes.h"

#define sq(x) (x)*(x)
#define PI 3.141592653589793
	

#define DEBUG


/** \brief Equation of motion of an Underwater Vehicle in Matrix form
*/
namespace underwaterVehicle
{
	class DynamicModel : public RK4_SIM
	{			
		
		public:
		    	/********************* Data functions **************************************/
			//DynamicModel(float samp_time = 0.005, int _simulation_per_cycle = 10,
			//		   float _initial_time = 0.0, float *_initial_state = NULL);
			DynamicModel(double samp_time = 0.005, int _simulation_per_cycle = 10,
					   double _initial_time = 0.0, double *_initial_state = NULL, int _plant_order=12, int _ctrl_order=5);
		    	~ DynamicModel();
			// initialise the model
			void init_param(underwaterVehicle::Parameters _param);
			// functions to get the dagon status
			Eigen::Vector3d getPosition();			
			Eigen::Vector3d getLinearVelocity();
			Eigen::Vector3d getAcceleration();
			Eigen::Quaterniond getOrientation_in_Quat();
			Eigen::Vector3d getOrientation_in_Euler();
			Eigen::Vector3d getAngularVelocity();
			double Simulationtime();		
			// calculating the model parameters 	
			void inertia_matrix(const Eigen::Matrix<double,6,1> &velocity,Eigen::Matrix<double,6,6>& mass_matrix);
			void gravity_buoyancy(const Eigen::Vector3d &eulerang, Eigen::Matrix<double,6,1>& gravitybuoyancy);
			void gravity_buoyancy(const Eigen::Quaternion<double> q, Eigen::Matrix<double,6,1>& gravitybuoyancy);
			void hydrodynamic_damping(const Eigen::Matrix<double,6,1> &velocity,Eigen::Matrix<double,6,6>& damping_matrix);
			//void thruster_ForceTorque(const Eigen::MatrixXd &thruster_control_matrix, const Eigen::MatrixXd &input_thrust, Eigen::MatrixXd &thrust);
			void thruster_ForceTorque(Eigen::MatrixXd &thruster_control_matrix, const Eigen::MatrixXd &input_thrust, Eigen::MatrixXd &thrust);
			// setting the input for the model
			void setPWMLevels(ThrusterMapping thrusters);
			void setRPMLevels(ThrusterMapping thrusters);
			void setThrust(std::vector<double> inputthrust);
			void pwm_to_dc(const ThrusterMapping thrusters, std::vector<double> & dc_volt);
			// simulating the model
			//void DERIV(const double t, const double *x, const double *u, double *xdot);
			void DERIV(const double t, double *x, const double *u, double *xdot);
		    	// mathematical functions		    	
			Eigen::Quaternion<double> Euler_to_Quaternion(const Eigen::Vector3d &euler_angles);
			Eigen::Vector3d Quaternion_to_Euler(Eigen::Quaternion<double> quaterion_angles);
			void rot_BI_euler(const Eigen::Vector3d &eulerang, Eigen::Matrix3d& rot_BI);
			void rot_IB_euler(const Eigen::Matrix3d &rot_BI, Eigen::Matrix3d &rot_IB);
			void rot_IB_euler(const Eigen::Vector3d &eulerang, Eigen::Matrix3d& rot_IB);
			void jacobianMatrix_euler(const Eigen::Vector3d &eulerang, Eigen::Matrix3d& jacob_kin_e);
			void inverseJacobianMatrix_euler(const Eigen::Vector3d &eulerang, Eigen::Matrix3d& inv_jacob_kin_e);
			void compute_inv_jacob_e_RIB(const Eigen::Matrix3d & rot_IB, const Eigen::Matrix3d & inv_jacob_kin_e, Eigen::Matrix<double , 6, 6 > & inv_jacob_e_RIB);
			void compute_invTrans_jacob_e_RIB(const Eigen::Matrix3d & rot_IB, const Eigen::Matrix3d & inv_jacob_kin_e, Eigen::Matrix<double , 6, 6 > & invTrans_jacob_e_RIB);
			void setPosition(base::Vector3d v);
			void setLinearVelocity(base::Vector3d v);
			void setSamplingtime(double dt);
		private:
			// variables used for simulation			
			Eigen::MatrixXd input_thrust;				// input thrust 						
			Eigen::Matrix<double,DOF,1> velocity;  			// This Vector velocity represent the linear and angular velocity of the body-fixed frame
			Eigen::Vector3d linear_velocity;			// vehicle linear velocity
			Eigen::Vector3d angular_velocity;			// vehicle angular velocity
										// velocity(0)=u; velocity(1)=v; velocity(2)=w; velocity(3)=p; velocity(4)=q; velocity(5)=r 
			Eigen::Matrix<double,DOF,1> acceleration;		// this vector acceleration represent the linear and angular acceleration of the body-fixed frame
			Eigen::Vector3d linear_acceleration;			// vehicle linear acceleration
			Eigen::Vector3d angular_acceleration;			// vehicle angular acceleration
			
			Eigen::Vector3d position;				// vehicle position
			Eigen::Vector3d orientation_euler;			// vehicle orienation in eulers angles
			Eigen::Quaterniond orientation_quaternion;		// vehicle orienation in quaternion angles		
			underwaterVehicle::Parameters param;			// parameter object						
			std::vector<double> dc_volt;		    		// direct current values used for modeling the thrusters which doesnt have speed values
			
			int number_of_thrusters;				// number of thrusters
			double negative_constant;
			Eigen::Matrix3d rot_BI;
			Eigen::Matrix3d rot_IB;
			Eigen::Matrix3d jacob_kin_e;
			Eigen::Matrix3d inv_jacob_kin_e;		
			Eigen::Matrix<double,6,6> inv_jacob_e_RIB;
			Eigen::Matrix<double,6,6> invTrans_jacob_e_RIB;			
			Eigen::Matrix3d zero3x3;

			/************** UWV variables ***************************/			
			/* bff - Body Fixed Frame */
			/* eff - Earth Fixed Frame */
			Eigen::Matrix<double,DOF,DOF> mass_matrix_bff;		// mass Matrix  w.r.t body-fixed frame
			Eigen::Matrix<double,DOF,DOF> mass_matrix_eff;		// mass Matrix  w.r.t earth-fixed frame
			Eigen::Matrix<double,DOF,DOF> Inv_massMatrix;		// inverse of mass Matrix  needed in simulation 
			Eigen::Matrix<double,DOF,DOF> damping_matrix_bff;	// damping Effect Matrix w.r.t body-fixed frame
			Eigen::Matrix<double,DOF,DOF> damping_matrix_eff;	// damping Effect Matrix w.r.t earth-fixed frame		    					    	
			Eigen::MatrixXd thruster_control_matrix;		// thruster Control Matrix(TCM)		
			Eigen::Matrix<double,DOF,1> gravitybuoyancy_bff;	// gravity and buoyancy Matrix w.r.t body-fixed frame
			Eigen::Matrix<double,DOF,1> gravitybuoyancy_eff;	// gravity and buoyancy Matrix w.r.t earth-fixed frame
			Eigen::MatrixXd thrust_bff;				// force and torque w.r.t body-fixed frame
			Eigen::MatrixXd thrust_eff;				// force and torque w.r.t earth-fixed frame
			/************** UWV physical parameters *****************/			
			double uwv_weight;   					// total gravity force magnitude
			double uwv_buoyancy;	   				// total buoyancy force magnitude

			Eigen::MatrixXd temp_thrust;	
				
			
	};
};
#endif
