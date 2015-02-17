/***************************************************************************/
/*  Dynamic model for an underwater vehicle	                               */
/*                                                                         */
/* FILE --- uwv_dynamic_model.cpp	                                       */
/*                                                                         */
/* PURPOSE --- Source file for a Dynamic model of an 	                   */
/*             underwater vehicle. Based on T.I.Fossen & Giovanni Indiveri */
/*                                                                         */
/*  Sankaranarayanan Natarajan                                             */
/*  sankar.natarajan@dfki.de                                               */
/*  DFKI - BREMEN 2011                                                     */
/***************************************************************************/

//TODO check if initialStates has size equal to 12
#include "uwv_dynamic_model.hpp"
#include <base/Logging.hpp>

namespace uwv_dynamic_model
{
	DynamicModel::DynamicModel(uint controlOrder, uint systemOrder,
							   double samplingTime, uint simPerCycle,
							   double initialTime)
		: RK4_SIM(controlOrder, systemOrder, (samplingTime/(double)simPerCycle))
	{
		// Error flags. The errorModelInit will be unset when the model is initialized
		errorModelInit = true;
		errorConstruction = false;
		errorControlInput = false;
		errorInitialization = false;
		errorStatus = false;

		// Checks the arguments provided to the constructor and then initialize the
		// members of the class
		if(checkConstruction(samplingTime, simPerCycle, initialTime))
		{
			gSystemOrder = systemOrder;
			gControlOrder = controlOrder;
			gCurrentTime = initialTime;

			// States variables
			Eigen::VectorXd statesInit = Eigen::VectorXd::Zero(12);
			updateStates(statesInit);

			gLinearAcceleration = Eigen::VectorXd::Zero(3);
			gAngularAcceleration = Eigen::VectorXd::Zero(3);

			gEfforts = Eigen::VectorXd::Zero(6);

			// Model parameters
			setInertiaMatrix(Eigen::MatrixXd::Zero(6,6));
			setCoriolisMatrix(Eigen::MatrixXd::Zero(6,6));
			setLinDampingMatrix(Eigen::MatrixXd::Zero(6,6));
			setQuadDampingMatrix(Eigen::MatrixXd::Zero(6,6));
			setThrustConfigMatrix(Eigen::MatrixXd::Zero(6,1));

			// Thrusters' coefficients
			gThrusterCoeffPWM.positive = 0;
			gThrusterCoeffPWM.negative = 0;
			gLinThrusterCoeffPWM.positive = 0;
			gLinThrusterCoeffPWM.negative = 0;
			gQuadThrusterCoeffPWM.positive = 0;
			gQuadThrusterCoeffPWM.negative = 0;
			gThrusterCoeffRPM.positive = 0;
			gThrusterCoeffRPM.negative = 0;

			// Restoring forces' variables
			gWeight = 0;
			gBuoyancy = 0;
		}
		// If there is an error with the contructor's argument, the
		// errorConstruction flag is set
		else
			errorConstruction = true;

	}

	void DynamicModel::initParameters(const uwv_dynamic_model::Parameters &uwvParameters)
	{
		// Unset the errorModelInit because the model is being initialized
		errorModelInit = false;

		gUWVParameters = uwvParameters;

		// System states
		Eigen::VectorXd statesInit = uwvParameters.initialStates;
		updateStates(statesInit);
		gEfforts	=	Eigen::VectorXd::Zero(gControlOrder);
		gCurrentTime	=	uwvParameters.initialTime;

		// Model parameters
		setInertiaMatrix(uwvParameters.inertiaMatrixPos, uwvParameters.inertiaMatrixNeg);
		setCoriolisMatrix(uwvParameters.coriolisMatrixPos, uwvParameters.coriolisMatrixNeg);
		setLinDampingMatrix(uwvParameters.linDampMatrixPos, uwvParameters.linDampMatrixPos);
		setQuadDampingMatrix(uwvParameters.quadDampMatrixPos, uwvParameters.quadDampMatrixNeg);
		setThrustConfigMatrix(uwvParameters.thrustConfigMatrix);

		// Checks if the positive inertia, positive linear damping and thrust
		// configuration matrices were set
		checkPositiveMatrices();

		// Checking if the negative matrices were set. If not, they will
		// receive the respective positive matrix value
		checkNegativeMatrices(gInertiaMatrixNeg, gInertiaMatrixPos);
		checkNegativeMatrices(gCoriolisMatrixNeg, gCoriolisMatrixPos);
		checkNegativeMatrices(gLinDampMatrixNeg, gLinDampMatrixPos);
		checkNegativeMatrices(gQuadDampMatrixNeg, gQuadDampMatrixPos);

		gThrusterCoeffPWM = uwvParameters.thrusterCoeffPWM;
		gLinThrusterCoeffPWM = uwvParameters.linThrusterCoeffPWM;
		gQuadThrusterCoeffPWM = uwvParameters.quadThrusterCoeffPWM;
		gThrusterCoeffRPM = uwvParameters.thrusterCoeffRPM;

		// Restoring forces
		gWeight			= 	uwvParameters.uwvMass		* uwvParameters.gravity;
		gBuoyancy		= 	uwvParameters.waterDensity  * uwvParameters.gravity *
				            uwvParameters.uwvVolume;

		// Checks if the inertia matrices are invertible
		if (!checkInitialization(gInertiaMatrixPos, gInertiaMatrixNeg))
			errorInitialization = true;
	}

	void DynamicModel::sendPWMCommands(const base::samples::Joints &controlInput)
	{
		 // If the flag errorStatus is set, this function won't be executed.
		 // This garantee that the error message will be printed only once
		if(!errorStatus)
		{
			// Checks if there is any error flag activated
			if(checkErrors())
			{
				// Checks if the control input is valid
				if(checkControlInput(controlInput, "raw"))
				{
					//Calculates the forces and moments generated by the thrusters

					Eigen::VectorXd thrustersForce 	=	Eigen::VectorXd::Zero(6);
					Eigen::VectorXd dcVoltage 		=	Eigen::VectorXd::Zero(6);
					pwmToDC(dcVoltage, controlInput);
					dcToThrustForce(thrustersForce, dcVoltage);
					thrustForceToEffort(gEfforts, thrustersForce);
				}
				else
				{
					for (int i = 0; i < gControlOrder; i++)
						gEfforts[i] = 0.0;
					errorControlInput = true;
				}

				// Performs iterations to calculate the new system's states
				for (int i=0; i < gUWVParameters.simPerCycle ; i++)
					calcStates(gSystemStates, gCurrentTime, gEfforts);

				// Updates the new system's states
				updateStates(gSystemStates);
			}
			else
				errorStatus = true;
		}
	}

	void DynamicModel::sendRPMCommands(const base::samples::Joints &controlInput)
	{
		// If the flag errorStatus is set, this function won't be executed.
		// This garantee that the error message will be printed only once
		if(!errorStatus)
		{
			// Checks if there is any error flag activated
			if(checkErrors())
			{
				// Checks if the control input is valid
				if(checkControlInput(controlInput, "speed"))
				{
					/**
					 *Calculates the forces and moments generated by the thrusters
					 */

					Eigen::VectorXd thrustersForce = Eigen::VectorXd::Zero(gControlOrder);
					rpmToThrustForce(thrustersForce, controlInput);
					thrustForceToEffort(gEfforts, thrustersForce);
				}
				else
				{
					for (int i = 0; i < gControlOrder; i++)
						gEfforts[i] = 0.0;
					errorControlInput = true;
				}

				// Performs iterations to calculate the new system's states
				for (int ii=0; ii < gUWVParameters.simPerCycle; ii++)
					calcStates(gSystemStates, gCurrentTime, gEfforts);

				// Updates the new system's states
				updateStates(gSystemStates);
			}
			else
				errorStatus = true;
		}
	}

	void DynamicModel::sendEffortCommands(const base::samples::Joints &controlInput)
	{
		/**
		 * If the flag errorStatus is set, this function won't be executed.
		 * This garantee that the error message will be printed only once
		 */
		if(!errorStatus)
		{
			// Checks if there is any error flag activated
			if(checkErrors())
			{
				// Checks if the control input is valid
				if(checkControlInput(controlInput, "effort"))
				{
					for (int i = 0; i < gSystemOrder; i++)
						gEfforts[i] = controlInput[i].effort;
				}
				else
				{
					for (int i = 0; i < gSystemOrder; i++)
						gEfforts[i] = 0.0;
					errorControlInput = true;
				}

				// Performs iterations to calculate the new system's states
				for (int ii=0; ii < gUWVParameters.simPerCycle; ii++)
					calcStates(gSystemStates, gCurrentTime, gEfforts);

				// Updates the new system's states
				updateStates(gSystemStates);
			}
			else
				errorStatus = true;
		}
	}

	void DynamicModel::calcAcceleration(Eigen::VectorXd &velocityAndAcceleration,
			const Eigen::VectorXd &velocity,
			const Eigen::VectorXd &controlInput)
	{
		/**
		 * velocityAndAcceleration:
		 *
		 * [0] = u		[6]  = u_dot	(SURGE)
		 * [1] = v		[7]  = v_dot	(SWAY)
		 * [2] = w		[8]  = w_dot	(HEAVE)
		 * [3] = p		[9]  = p_dot	(ROLL)
		 * [4] = q		[10] = q_dot	(PITCH)
		 * [5] = r		[11] = r_dot	(YAW)
		 *
		 */

		// Forces and Moments vectors
		Eigen::VectorXd coriolisEffect		=	Eigen::VectorXd::Zero(6);
		Eigen::VectorXd linDamping			=	Eigen::VectorXd::Zero(6);
		Eigen::VectorXd quadDamping			=	Eigen::VectorXd::Zero(6);
		Eigen::VectorXd gravityBuoyancy		=	Eigen::VectorXd::Zero(6);
		Eigen::VectorXd acceleration		=	Eigen::VectorXd::Zero(6);
		base::Matrix6d invInertiaMatrix = Eigen::MatrixXd::Zero(6,6);

		// Calculating the efforts for each one of the hydrodynamics effects
		calcInvInertiaMatrix(invInertiaMatrix, velocity);
		calcCoriolisEffect(coriolisEffect, velocity);
		calcLinDamping(linDamping, velocity);
		calcQuadDamping(quadDamping, velocity);
		calcGravityBuoyancy(gravityBuoyancy, gEulerOrientation);

		// Calculating the acceleration based on all the hydrodynamics effects
		acceleration  = invInertiaMatrix * ( gEfforts - coriolisEffect -
				        linDamping - quadDamping - gravityBuoyancy );

		// Updating the RK4 vector with the velocity and acceleration values
		for (int i = 0; i < 6; i++)
		{
			velocityAndAcceleration[i] = velocity[i];
			velocityAndAcceleration[i+6] = acceleration[i];
		}

		// Updating global acceleration variables
		for(int i = 0; i < 3; i++)
		{
			gLinearAcceleration[i]  = acceleration[i];
			gAngularAcceleration[i] = acceleration[i+3];
		}

	}

	void DynamicModel::setInertiaMatrix(const base::Matrix6d &inertiaMatrixPos,
						  	  	  	    const base::Matrix6d &inertiaMatrixNeg)
	{
		gInertiaMatrixPos = inertiaMatrixPos;
		gInertiaMatrixNeg = inertiaMatrixNeg;

		checkNegativeMatrices(gInertiaMatrixNeg, gInertiaMatrixPos);
	}

	void DynamicModel::setCoriolisMatrix(const base::Matrix6d &coriolisMatrixPos,
			const base::Matrix6d &coriolisMatrixNeg)
	{
		gCoriolisMatrixPos = coriolisMatrixPos;
		gCoriolisMatrixNeg = coriolisMatrixNeg;

		checkNegativeMatrices(gCoriolisMatrixNeg, gCoriolisMatrixPos);
	}

	void DynamicModel::setLinDampingMatrix(const base::Matrix6d &linDampingMatrixPos,
			const base::Matrix6d &linDampingMatrixNeg)
	{
		gLinDampMatrixPos = linDampingMatrixPos;
		gLinDampMatrixNeg = linDampingMatrixNeg;

		checkNegativeMatrices(gLinDampMatrixNeg, gLinDampMatrixPos);
	}

	void DynamicModel::setQuadDampingMatrix(const base::Matrix6d &quadDampingMatrixPos,
			const base::Matrix6d &quadDampingMatrixNeg)
	{
		gQuadDampMatrixPos = quadDampingMatrixPos;
		gQuadDampMatrixNeg = quadDampingMatrixNeg;

		checkNegativeMatrices(gQuadDampMatrixNeg, gQuadDampMatrixPos);
	}

	void DynamicModel::setThrustConfigMatrix(const Eigen::MatrixXd &thrustConfigMatrix)
	{
		if (thrustConfigMatrix.rows() == 6)
			gThrustConfigMatrix = thrustConfigMatrix;
	}

	void DynamicModel::setPosition(const Eigen::Vector3d &position)
	{
		gPosition = position;
	}

	void DynamicModel::setEulerOrientation(const Eigen::Vector3d &eulerOrientation)
	{
		gEulerOrientation = eulerOrientation;
		Eigen::Quaterniond quatOrientation;
		eulerToQuaternion(quatOrientation, eulerOrientation);
		gQuatOrientation = quatOrientation;
	}

	void DynamicModel::setQuatOrientation(const Eigen::Quaterniond &quatOrientation)
	{
		gQuatOrientation = quatOrientation;
		Eigen::Vector3d eulerOrientation;
		eulerOrientation = base::getEuler(quatOrientation);
		gEulerOrientation = eulerOrientation;
	}

	void DynamicModel::setLinearVelocity(const Eigen::Vector3d &linearVelocity)
	{
		gLinearVelocity = linearVelocity;
	}

	void DynamicModel::setAngularVelocity(const Eigen::Vector3d &angularVelocity)
	{
		gAngularVelocity = angularVelocity;
	}

	void DynamicModel::resetAll()
	{
		Eigen::Vector3d resetVector3d = Eigen::VectorXd::Zero(3);
		Eigen::Quaterniond resetQuaterniond;
		eulerToQuaternion(resetQuaterniond, resetVector3d);

		setPosition(resetVector3d);
		setEulerOrientation(resetVector3d);
		setQuatOrientation(resetQuaterniond);
		setLinearVelocity(resetVector3d);
		setAngularVelocity(resetVector3d);
	}

	void DynamicModel::getUWVParameters(uwv_dynamic_model::Parameters &uwvParameters)
	{
		uwvParameters = gUWVParameters;
	}

	void DynamicModel::getPosition(base::Position &position)
	{
		position = gPosition;
	}

	void DynamicModel::getEulerOrientation(base::Vector3d &eulerOrientation)
	{
		eulerOrientation = gEulerOrientation;
	}

	void DynamicModel::getQuatOrienration(base::Quaterniond &quatOrientation)
	{
		quatOrientation = gQuatOrientation;
	}

	void DynamicModel::getLinearVelocity(base::Vector3d &linearVelocity)
	{
		linearVelocity = gLinearVelocity;
	}

	void DynamicModel::getAngularVelocity(base::Vector3d &angularVelocity)
	{
		angularVelocity = gAngularVelocity;
	}

	void DynamicModel::getLinearAcceleration(base::Vector3d &linearAcceleration)
	{
		linearAcceleration = gLinearAcceleration;
	}

	void DynamicModel::getAngularAcceleration(base::Vector3d &angularAcceleration)
	{
		angularAcceleration = gAngularAcceleration;
	}

	void DynamicModel::getSimulationTime(double &simulationTime)
	{		
		simulationTime = gCurrentTime;
	}

	void DynamicModel::eulerToQuaternion(Eigen::Quaterniond &quaternion,
									     const Eigen::Vector3d &eulerAngles)
	{

		quaternion.w() = ( cos(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) +
						 ( sin(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
		quaternion.x() = ( sin(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) -
						 ( cos(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
		quaternion.y() = ( cos(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) +
						 ( sin(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
		quaternion.z() = ( cos(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*sin(eulerAngles(2)/2) ) -
						 ( sin(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*cos(eulerAngles(2)/2) );
	}

	void DynamicModel::calcInvInertiaMatrix(base::Matrix6d &invInertiaMatrix,
			 	 	 	 	 	 	 	    const Eigen::VectorXd &velocity)
	{	
		Eigen::MatrixXd inertiaMatrix = Eigen::MatrixXd::Zero(6,6);
		for(int i = 0; i < 6; i++)
		{
			if(velocity(i) > -0.001)
				inertiaMatrix.block(0 , i , 6, 1) = gInertiaMatrixPos.block(0 , i , 6, 1);
			else
				inertiaMatrix.block(0 , i , 6, 1) = gInertiaMatrixNeg.block(0 , i , 6, 1);
		}

		invInertiaMatrix = inertiaMatrix.inverse();
	}

	void DynamicModel::calcCoriolisEffect(Eigen::VectorXd &coriolisEffect,
										  const Eigen::VectorXd &velocity)
	{
		Eigen::MatrixXd coriolisMatrix = Eigen::MatrixXd::Zero(6,6);

		for(int i = 0; i < 6; i++)
		{
			if(velocity(i) > -0.001)
				coriolisMatrix.block(0 , i , 6, 1) = gCoriolisMatrixPos.block(0 , i , 6, 1);
			else
				coriolisMatrix.block(0 , i , 6, 1) = gCoriolisMatrixNeg.block(0 , i , 6, 1);
		}
		coriolisEffect = coriolisMatrix*velocity;
	}

	void DynamicModel::calcLinDamping(Eigen::VectorXd &linDamping,
									  const Eigen::VectorXd &velocity)
	{
		Eigen::MatrixXd linDampMatrix = Eigen::MatrixXd::Zero(6,6);

		for(int i = 0; i < 6; i++)
		{

			if(velocity(i) > -0.001)
				linDampMatrix.block(0 , i , 6, 1) = gLinDampMatrixPos.block(0, i , 6, 1);
			else
				linDampMatrix.block(0 , i , 6, 1) = gLinDampMatrixNeg.block(0 , i , 6, 1);

		}
		linDamping = linDampMatrix*velocity;
	}

	void DynamicModel::calcQuadDamping(Eigen::VectorXd &quadDamping,
									   const Eigen::VectorXd &velocity)
	{
		Eigen::MatrixXd quadDampMatrix = Eigen::MatrixXd::Zero(6,6);
		Eigen::VectorXd absoluteVelocity = Eigen::VectorXd::Zero(6);

		for(int i = 0; i < 6; i++)
		{
			if(velocity(i) > -0.001)
				quadDampMatrix.block(0 , i , 6, 1) = gQuadDampMatrixPos.block(0, i , 6, 1);
			else
				quadDampMatrix.block(0 , i , 6, 1) = gQuadDampMatrixNeg.block(0 , i , 6, 1);

			absoluteVelocity[i] = fabs(velocity[i]);
		}

		quadDamping = quadDampMatrix*absoluteVelocity.asDiagonal()*velocity;
	}

	void DynamicModel::calcGravityBuoyancy(Eigen::VectorXd &gravitybuoyancy,
										   const Eigen::Vector3d &eulerOrientation)
	{
		float e1 = eulerOrientation(0);
		float e2 = eulerOrientation(1);
		float e3 = eulerOrientation(2);
		float xg = gUWVParameters.centerOfGravity(0);
		float yg = gUWVParameters.centerOfGravity(1);
		float zg = gUWVParameters.centerOfGravity(2);
		float xb = gUWVParameters.centerOfBuoyancy(0);
		float yb = gUWVParameters.centerOfBuoyancy(1);
		float zb = gUWVParameters.centerOfBuoyancy(2);

		if (gUWVParameters.uwvFloat == true)
			gWeight = gBuoyancy;

		gravitybuoyancy(0) 	= 	(gWeight - gBuoyancy) * sin(e2);
		gravitybuoyancy(1) 	=  -(gWeight - gBuoyancy) * (cos(e2)*sin(e1));
		gravitybuoyancy(2) 	=  -(gWeight - gBuoyancy) * (cos(e2)*cos(e1));
		gravitybuoyancy(3) 	=  -((yg*gWeight - yb*gWeight)*cos(e2)*cos(e1)) +
								((zg*gWeight - zb*gBuoyancy) * cos(e2)*sin(e1));
		gravitybuoyancy(4) 	=   ((zg*gWeight - zb*gWeight)*sin(e2)) +
								((xg*gWeight - xb*gBuoyancy)*cos(e2)*cos(e1));
		gravitybuoyancy(5) 	=  -((xg*gWeight - xb*gWeight)*cos(e2)*sin(e1)) -
								((yg*gWeight - yb*gBuoyancy)* sin(e2));

	}

	void DynamicModel::calcGravityBuoyancy(Eigen::VectorXd &gravitybuoyancy,
										   const Eigen::Quaterniond &quatOrientation)
	{
		float e1 = quatOrientation.x();
		float e2 = quatOrientation.y();
		float e3 = quatOrientation.z();
		float e4 = quatOrientation.w();
		float xg = gUWVParameters.centerOfGravity(0);
		float yg = gUWVParameters.centerOfGravity(1);
		float zg = gUWVParameters.centerOfGravity(2);
		float xb = gUWVParameters.centerOfBuoyancy(0);
		float yb = gUWVParameters.centerOfBuoyancy(1);
		float zb = gUWVParameters.centerOfBuoyancy(2);

		if (gUWVParameters.uwvFloat == true)
			gWeight = gBuoyancy;

		gravitybuoyancy(0) 	=	(2*((e4*e2)-(e1*e3)))*(gWeight-gBuoyancy);
		gravitybuoyancy(1) 	=  -(2*((e4*e1)+(e2*e3)))*(gWeight-gBuoyancy);
		gravitybuoyancy(2) 	=   (-pow(e4,2)+pow(e1,2)+pow(e2,2)-pow(e3,2))*
								(gWeight-gBuoyancy);
		gravitybuoyancy(3) 	= 	((-pow(e4,2)+pow(e1,2)+pow(e2,2)-pow(e3,2))*
								((yg*gWeight)-(yb*gBuoyancy)))+(2*((e4*e1)+(e2*e3))*
								((zg*gWeight)-(zb*gBuoyancy)));
		gravitybuoyancy(4) 	=  -((-pow(e4,2)+pow(e1,2)+pow(e2,2)-pow(e3,2))*
								((xg*gWeight)-(xb*gBuoyancy)))+(2*((e4*e2)-(e1*e3))*
								((zg*gWeight)-(zb*gBuoyancy)));
		gravitybuoyancy(5) 	=  -(2*((e4*e1)+(e2*e3))*((xg*gWeight)-(xb*gBuoyancy)))-
								(2*((e4*e2)-(e1*e3))*((yg*gWeight)-(yb*gBuoyancy)));
	}

	void DynamicModel::pwmToDC(Eigen::VectorXd &dcVolt,
							   const base::samples::Joints &controlInput)
	{	
		for ( int i = 0; i < gControlOrder; i ++)
			dcVolt[i] = gUWVParameters.thrusterVoltage * controlInput[i].raw;
	}			

	void DynamicModel::dcToThrustForce(Eigen::VectorXd &thrustForces,
									   const Eigen::VectorXd &dcVolt)
	{
		if(checkPWMCoefficients())
		{
			for ( int i = 0; i < gControlOrder; i ++)
			{
				float factor = 0.01;

				if ((dcVolt[i] >= -factor) && ( dcVolt[i] <= factor ))
					thrustForces[i] = 0.0;
				else
				{
					double thruster_coefficient;
					if(dcVolt[i] > 0)
						thruster_coefficient = gThrusterCoeffPWM.positive
						+ gLinThrusterCoeffPWM.positive*dcVolt[i]
					    + gQuadThrusterCoeffPWM.positive*dcVolt[i]*fabs(dcVolt[i]);
					else
						thruster_coefficient = gThrusterCoeffPWM.negative
						+ gLinThrusterCoeffPWM.negative*dcVolt[i]
						+ gQuadThrusterCoeffPWM.negative*dcVolt[i]*fabs(dcVolt[i]);

					thrustForces[i] = thruster_coefficient*fabs(dcVolt[i])*dcVolt[i];
				}
			}
		}
	}
	
	void DynamicModel::rpmToThrustForce(Eigen::VectorXd &thrustForces,
									    const base::samples::Joints &controlInput)
	{
		if(checkRPMCoefficients())
		{
			for (int i = 0; i < gControlOrder; i++)
			{
				if(controlInput[i].speed > -0.001)
					thrustForces[i] = gUWVParameters.thrusterCoeffRPM.positive *
					(fabs(controlInput[i].speed) * controlInput[i].speed);
				else
					thrustForces[i] = gUWVParameters.thrusterCoeffRPM.negative *
					(fabs(controlInput[i].speed) * controlInput[i].speed);
			}
		}
	}

	void DynamicModel::thrustForceToEffort(Eigen::VectorXd &forcesAndMoments,
										   const Eigen::VectorXd &thrustInput)
	{
		forcesAndMoments = gThrustConfigMatrix*thrustInput;
	}
	void DynamicModel::updateStates(Eigen::VectorXd &newSystemStates)
	{
		base::Vector3d position 		= newSystemStates.head(3);
		base::Vector3d eulerOrientation = newSystemStates.segment(3,3);
		base::Vector3d linearVelocity 	= newSystemStates.segment(6,3);
		base::Vector3d angularVelocity 	= newSystemStates.segment(9,3);

		Eigen::Quaterniond quatOrientation;
		eulerToQuaternion(quatOrientation, eulerOrientation);

		setPosition(position);
		setEulerOrientation(eulerOrientation);
		setLinearVelocity(linearVelocity);
		setAngularVelocity(angularVelocity);

		gSystemStates = newSystemStates;
	}

	bool DynamicModel::checkConstruction(double &samplingTime, uint &simPerCycle,
			   	   	   	   	   	   	   	 double &initialTime)
	{
		std::string textElement;
		std::string textComparison;
		bool checkError = false;

		if (samplingTime <= 0)
		{
			textElement = "sampling time";
			textComparison = "greater than";
			checkError = true;
		}
		else if (simPerCycle == 0)
		{
			textElement = "number of simulations per cycle";
			textComparison = "greater than";
			checkError = true;
		}
		else if (initialTime < 0)
		{
			textElement = "initial time";
			textComparison = "greater or equal to";
			checkError = true;
		}

		if(checkError)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
					  " The %s should be %s to zero.\x1b[0m\n\n"
					  , textElement.c_str(), textComparison.c_str());
			return false;
		}
		return true;
	}

	bool DynamicModel::checkInitialization(base::Matrix6d &inertiaMatrixPos,
										   base::Matrix6d &inertiaMatrixNeg)
	{
		std::string textElement;
		bool checkError = false;

		if(inertiaMatrixPos.determinant() == 0)
		{
			textElement = "positive";
			checkError = true;
		}
		else if(inertiaMatrixNeg.determinant() == 0)
		{
			textElement = "negative";
			checkError = true;
		}

		if(checkError)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
					  " The %s inertia matrix is not invertible.\x1b[0m\n\n",
					  textElement.c_str());
			return false;
		}
		return true;
	}

	void DynamicModel::checkPositiveMatrices(void)
	{
		std::string textElement;
		bool checkError = false;

		if (gInertiaMatrixPos == Eigen::MatrixXd::Zero(6,6))
		{
			textElement = "positive inertia matrix";
			checkError= true;
		}
		else if (gLinDampMatrixPos == Eigen::MatrixXd::Zero(6,6))
		{
			textElement = "positive linear damping matrix";
			checkError= true;;
		}
		else if (gThrustConfigMatrix.size() == 0)
		{
			textElement = "thrust configuration matrix";
			checkError= true;
		}

		if(checkError)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
					" The %s was not set.\x1b[0m\n\n", textElement.c_str());
		}
	}

	void DynamicModel::checkNegativeMatrices(base::Matrix6d &negativeMatrix,
											 const base::Matrix6d &positiveMatrix)
	{
		if (negativeMatrix == Eigen::MatrixXd::Zero(6, 6))
			negativeMatrix = positiveMatrix;
	}

	bool DynamicModel::checkControlInput(const base::samples::Joints &controlInput,
										 std::string jointsElement)
	{
		int inputSize = controlInput.size();
		std::string textElement;
		std::string textPosition;
		bool checkError = false;

		if (inputSize != gControlOrder)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				  " The system has %i control inputs, and not %i like it was"
				  " provided by the controlInput argument.\x1b[0m\n\n"
					, gControlOrder, inputSize);
			return false;
		}
		else if(jointsElement == "raw")
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if(isnan(controlInput.elements[i].hasRaw()))
				{
					textElement = jointsElement;
					textPosition = i;
					checkError = true;
					break;
				}
			}
		}
		else if(jointsElement == "speed")
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if(isnan(controlInput.elements[i].hasSpeed()))
				{
					textElement = jointsElement;
					textPosition = i;
					checkError = true;
					break;
				}
			}
		}
		else if(jointsElement == "effort")
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if(isnan(controlInput.elements[i].hasEffort()))
				{
					textElement = jointsElement;
					textPosition = i;
					checkError = true;
					break;
				}
			}
		}

		if(checkError)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The control input hasn't its %s element set"
				" at position %i.\x1b[0m\n\n", textElement.c_str(),
				textPosition.c_str());
			return false;
		}
		return true;
	}

	bool DynamicModel::checkPWMCoefficients(void)
	{
		std::string textElement;
		bool checkError = false;

		if(gUWVParameters.thrusterCoeffPWM.positive +
		   gUWVParameters.linThrusterCoeffPWM.positive +
		   gUWVParameters.quadThrusterCoeffPWM.positive == 0)
		{
			textElement = "positive";
			checkError = true;
		}
		else if (gUWVParameters.thrusterCoeffPWM.negative +
				gUWVParameters.linThrusterCoeffPWM.negative +
				gUWVParameters.quadThrusterCoeffPWM.negative == 0)
		{
			textElement = "negative";
			checkError = true;
		}

		if(checkError)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
					  " The %s PWM coefficients were not set.\x1b[0m\n\n",
					  textElement.c_str());
			return false;
		}

		return true;
	}

	bool DynamicModel::checkRPMCoefficients(void)
	{
		std::string textElement;
		bool checkError = false;

		if(gUWVParameters.thrusterCoeffRPM.positive == 0)
		{
			textElement = "positive";
			checkError = true;
		}
		else if (gUWVParameters.thrusterCoeffRPM.negative == 0)
		{
			textElement = "negative";
			checkError = true;
		}

		if(checkError)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
					" The %s RPM coefficient was not set.\x1b[0m\n\n",
					textElement.c_str());
			return false;
		}

		return true;
	}

	bool DynamicModel::checkErrors(void)
	{
		std::string textElement;
		bool checkError = false;

		if (errorModelInit)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
					" The model wasn't initialized. Call the method initParameters"
					" in order to do so or check if there was an error while doing"
					" it.\x1b[0m\n\n");
			return false;
		}
		else if(errorConstruction)
		{
			textElement = "during the construction of the class";
			checkError = true;
		}
		else if(errorControlInput)
		{
			textElement = "with the provided control input";
			checkError = true;
		}
		else if(errorInitialization)
		{
			textElement = "during the initialization of the model parameters";
			checkError = true;
		}

		if(checkError)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
					" There was an error %s.\x1b[0m\n\n", textElement.c_str());
			return false;
		}

		return true;
	}

};
	

	
