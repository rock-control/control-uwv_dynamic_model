/***************************************************************************/
/*  Data types for an underwater vehicle 	                           */
/*                                                                         */
/* FILE --- uwv_dataTypes.h		                                   */
/*                                                                         */
/* PURPOSE --- Header file for a data types used in modeling a 		   */
/*             underwater vehicle. 					   */
/*                                                                         */
/*  Sankaranarayanan Natarajan                                             */
/*  sankar.natarajan@dfki.de                                               */
/*  DFKI - BREMEN 2011                                                     */
/***************************************************************************/
#ifndef _UWV_DATATYPES_H_
#define _UWV_DATATYPES_H_

#include "base/Eigen.hpp"
#include <vector>

namespace underwaterVehicle
{
	/**
	 * Structure used to define if the specific parameter is for positive or negative movement
	 */
	struct Direction
	{
		double positive;
		double negative;
	};

	/**
	 * Structure that contains all the necessary information for simulating the motion model
	 */
	struct Parameters
	{
		/**
		 * Number of controllable inputs
		 */
		int ctrl_order;

		/**
		 * Sampling time used in the simulation (in seconds)
		 */
		double samplingtime;

		/**
		 * Number of RK4 iterations per sampling interval
		 */
		int sim_per_cycle;

		/**
		 * Intertia matrices for positive and negative speeds
		 */
		base::Matrix6d massMatrix;
		base::Matrix6d massMatrixNeg;

		/**
		 * Coriolis matrices for positive and negative speeds
		 */
		base::Matrix6d coriolisMatrix;
		base::Matrix6d coriolisMatrixNeg;

		/**
		 * Linear damping matrices for positive and negative speeds
		 */
		base::Matrix6d linDampMatrix;
		base::Matrix6d linDampMatrixNeg;

		/**
		 * Quadratic damping matrices for positive and negative speeds
		 */
		base::Matrix6d quadDampMatrix;
		base::Matrix6d quadDampMatrixNeg;

		/**
		 * Thrust configuration matrix
		 */
		base::MatrixXd thruster_control_matrix;

		/**
		 * Distance from the origin of the body-fixed frame to the center of buoyancy
		 */
		base::Vector3d distance_body2centerofbuoyancy;

		/**
		 * Distance from the origin of the body-fixed frame to the center of grabity
		 */
		base::Vector3d distance_body2centerofgravity;

		/**
		 * Total mass of the vehicle
		 */
		double uwv_mass;

		/**
		 * Total volume of the vehicle
		 */
		double uwv_volume;

		/**
		 * Variable used to assume that the weight of the vehicle is equal to its buoyancy force
		 */
		bool uwv_float;

		/**
		 * Density of the water in kg/m3
		 */
		double waterDensity;

		/*
		 * Acceleration of gravity
		 */
		double gravity;

		/**
		 * Variable used to convert the PWM signal into its corresponding DC Voltage
		 */
		std::vector<double> thrusterVoltage;

		/**
		 * Independent coefficient used to convert the thruster voltage into efforts
		 */
		std::vector<Direction> thruster_coefficients_pwm;

		/**
		 * Linear coefficient used to convert the thruster voltage into efforts
		 */
		std::vector<Direction> linear_thruster_coefficients_pwm;
		
		/**
		 * Quadratic coefficient used to convert the thruster voltage into efforts
		 */
		std::vector<Direction> square_thruster_coefficients_pwm;
		
		/**
		 * Coefficients used to convert the thruster rotational speed into efforts
		 */
		std::vector<Direction> thruster_coefficient_rpm;

		/**
		 * Initial condition used for simulation
		 */
		double initial_condition[12];

		Parameters():
			ctrl_order(4),
			samplingtime(0.1),
			sim_per_cycle(10),
			massMatrix(Eigen::MatrixXd::Zero(6,6)),
			massMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			coriolisMatrix(Eigen::MatrixXd::Zero(6,6)),
			coriolisMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			linDampMatrix(Eigen::MatrixXd::Zero(6,6)),
			linDampMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			quadDampMatrix(Eigen::MatrixXd::Zero(6,6)),
			quadDampMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			thruster_control_matrix(Eigen::MatrixXd::Zero(6,1)),
			distance_body2centerofbuoyancy(Eigen::VectorXd::Zero(3)),
			distance_body2centerofgravity(Eigen::VectorXd::Zero(3)),
			uwv_mass(0),
			uwv_volume(0),
			uwv_float(false),
			waterDensity(998.2),
			gravity(9.81),
			thrusterVoltage(0)
		{
			thruster_coefficients_pwm.resize(1);
			linear_thruster_coefficients_pwm.resize(1);
			square_thruster_coefficients_pwm.resize(1);
			thruster_coefficient_rpm.resize(1);

			thruster_coefficients_pwm[0].positive = 0;
			thruster_coefficients_pwm[0].negative = 0;
			linear_thruster_coefficients_pwm[0].positive = 0;
			linear_thruster_coefficients_pwm[0].negative = 0;
			square_thruster_coefficients_pwm[0].positive = 0;
			square_thruster_coefficients_pwm[0].negative = 0;
			thruster_coefficient_rpm[0].positive = 0;
			thruster_coefficient_rpm[0].negative = 0;

			for(int i = 0; i < 12; i++)
				initial_condition[i] = 0;
		};
	};								    	
	
};
#endif
