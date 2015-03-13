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

namespace uwv_dynamic_model
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
		uint controlOrder;

		/**
		 * Sampling time used in the simulation (in seconds)
		 */
		double samplingTime;

		/**
		 * Number of RK4 iterations per sampling interval
		 */
		uint simPerCycle;

		/**
		 * Intertia matrices for positive and negative speeds
		 */
		base::Matrix6d inertiaMatrixPos;
		base::Matrix6d inertiaMatrixNeg;

		/**
		 * Coriolis matrices for positive and negative speeds
		 */
		base::Matrix6d coriolisMatrixPos;
		base::Matrix6d coriolisMatrixNeg;

		/**
		 * Added Mass matrices for positive and negative speeds
		 */
		base::Matrix6d AddedMassMatrixPos;
		base::Matrix6d AddedMassMatrixNeg;

		/**
		 * Linear damping matrices for positive and negative speeds
		 */
		base::Matrix6d linDampMatrixPos;
		base::Matrix6d linDampMatrixNeg;

		/**
		 * Quadratic damping matrices for positive and negative speeds
		 */
		base::Matrix6d quadDampMatrixPos;
		base::Matrix6d quadDampMatrixNeg;

		/**
		 * Lift coefficients (Yuv, Zuw, Muw, Nuv)
		 */
		base::Vector4d LiftCoefficients;

		/**
		 * Thrust configuration matrix
		 */
		base::MatrixXd thrustConfigMatrix;

		/**
		 * Distance from the origin of the body-fixed frame to the center of buoyancy
		 */
		base::Vector3d centerOfBuoyancy;

		/**
		 * Distance from the origin of the body-fixed frame to the center of grabity
		 */
		base::Vector3d centerOfGravity;

		/**
		 * Total mass of the vehicle
		 */
		double uwvMass;

		/**
		 * Total volume of the vehicle
		 */
		double uwvVolume;

		/**
		 * Variable used to assume that the weight of the vehicle is equal to its buoyancy force
		 */
		bool uwvFloat;

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
		double thrusterVoltage;

		/**
		 * Independent coefficient used to convert the thruster voltage into efforts
		 */
		Direction thrusterCoeffPWM;

		/**
		 * Linear coefficient used to convert the thruster voltage into efforts
		 */
		Direction linThrusterCoeffPWM;
		
		/**
		 * Quadratic coefficient used to convert the thruster voltage into efforts
		 */
		Direction quadThrusterCoeffPWM;
		
		/**
		 * Coefficients used to convert the thruster rotational speed into efforts
		 */
		Direction thrusterCoeffRPM;

		/**
		 * Initial condition used for simulation
		 */
		base::VectorXd initialStates;

		Parameters():
			controlOrder(4),
			samplingTime(0.1),
			simPerCycle(10),
			inertiaMatrixPos(Eigen::MatrixXd::Zero(6,6)),
			inertiaMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			coriolisMatrixPos(Eigen::MatrixXd::Zero(6,6)),
			coriolisMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
                        AddedMassMatrixPos(Eigen::MatrixXd::Zero(6,6)),
			AddedMassMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			linDampMatrixPos(Eigen::MatrixXd::Zero(6,6)),
			linDampMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			quadDampMatrixPos(Eigen::MatrixXd::Zero(6,6)),
			quadDampMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			thrustConfigMatrix(Eigen::MatrixXd::Zero(6,1)),
			centerOfBuoyancy(Eigen::VectorXd::Zero(3)),
			centerOfGravity(Eigen::VectorXd::Zero(3)),
			LiftCoefficients(Eigen::VectorXd::Zero(4)),
			uwvMass(0),
			uwvVolume(0),
			uwvFloat(false),
			waterDensity(998.2),
			gravity(9.81),
			thrusterVoltage(0),
			initialStates(Eigen::VectorXd::Zero(12))
		{
			thrusterCoeffPWM.positive = 0;
			thrusterCoeffPWM.negative = 0;
			linThrusterCoeffPWM.positive = 0;
			linThrusterCoeffPWM.negative = 0;
			quadThrusterCoeffPWM.positive = 0;
			quadThrusterCoeffPWM.negative = 0;
			thrusterCoeffRPM.positive = 0;
			thrusterCoeffRPM.negative = 0;
		};
	};								    	
	
};
#endif
