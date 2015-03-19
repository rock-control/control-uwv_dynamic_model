/***************************************************************************/
/*  Dynamic model for an underwater vehicle	                           */
/*                                                                         */
/* FILE --- uwv_dynamic_model.cpp	                                   */
/*                                                                         */
/* PURPOSE --- Header file for a Dynamic model of an 	                   */
/*             underwater vehicle. Based on T.I.Fossen & Giovanni Indiveri */
/*                                                                         */
/*  Sankaranarayanan Natarajan                                             */
/*  sankar.natarajan@dfki.de                                               */
/*  DFKI - BREMEN 2011                                                     */
/*                                                                         */
/*  This file was edited to include the full Fossen Model                  */
/*                                                                         */
/*  Bilal Wehbe                                                            */
/*  bilal.wehbe@dfki.de                                                    */
/*  DFKI - BREMEN 2015                                                     */
/***************************************************************************/

#ifndef _UWV_DYNAMIC_MODEL_H_
#define _UWV_DYNAMIC_MODEL_H_


#include "RK4Integrator.hpp"
#include "uwv_dataTypes.hpp"
#include "base/samples/Joints.hpp"
#include "base/Pose.hpp"


namespace uwv_dynamic_model
{
class DynamicModel : public RK4_SIM
{

public:
	DynamicModel(uint controlOrder, double samplingTime = 0.01,
				 uint simPerCycle = 10, double initialTime = 0.0);

	/**
	 * Initializes the model parameters. Run this function before sending control
	 * commands to the model.
	 */
	bool initParameters(const uwv_dynamic_model::Parameters &uwvParameters);

	/**
	 * Function for sending PWM commands to the model.
	 * @param controlInput - PWM commands that should be applied to the model
	 */
	bool sendPWMCommands(const base::samples::Joints &controlInput);

	/**
	 * Function for sending RPM commands to the model.
	 * @param controlInput - RPM commands that should be applied to the model
	 */
	bool sendRPMCommands(const base::samples::Joints &controlInput);

	/**
	 * Function for sending Effort commands to the model.
	 * @param controlInput - Effort commands that should be applied to the model
	 */
	bool sendEffortCommands(const base::samples::Joints &controlInput);

	/**
	 * Sets the general UWV parameters
	 * @param uwvParamaters - Structures containing the uwv parameters
	 */
	bool setUWVParameters(const uwv_dynamic_model::Parameters &uwvParameters);

	/**
	 * Resets position, orientation and velocities of the model
	 */
	void resetStates(void);

	/**
	 * Gets the underwater vehicle parameters
	 * @param uwvParameters - Underwater vehicle parameters
	 */
	void getUWVParameters(uwv_dynamic_model::Parameters &uwvParameters);

	/**
	 * Gets the position
	 * @param position - Position vector
	 */
	void getPosition(base::Position &position);

	/**
	 * Gets the euler orientation
	 * @param eulerOrientation - Euler orientation vector
	 */
	void getEulerOrientation(Eigen::Vector3d &eulerOrientation);

	/**
	 * Gets the quaternion orientation
	 * @param quatOrientation - Quaternion orientation
	 */
	void getQuatOrienration(base::Orientation &quatOrientation);

	/**
	 * Gets the linear velocity
	 * @param linearVelocity - Linear velocity vector
	 */
	void getLinearVelocity(base::Vector3d &linearVelocity);

	/**
	 * Gets the angular velocity
	 * @param angularVelocity - Angular velocity vector
	 */
	void getAngularVelocity(base::Vector3d &angularVelocity);

	/**
	 * Gets the linear acceleration
	 * @param linearAcceleration - Linear acceleration vector
	 */
	void getLinearAcceleration(base::Vector3d &linearAcceleration);

	/**
	 * Gets the angular acceleration
	 * @param angularAcceleration - Angular acceleration vector
	 */
	void getAngularAcceleration(base::Vector3d &angularAcceleration);

	/**
	 * Gets the the system states (pose and velocity)
	 * @param systemStates - System states vector (size = 12)
	 */
	void getStates(Eigen::VectorXd &systemStates);

	/**
	 * Gets the current efforts' vector
	 * @param efforts - Vector containing the current effort (forces and moments)
	 */
	void getEfforts(base::Vector6d &efforts);

	/**
	 * Gets the simulation time in seconds
	 * @param simulationTime - Simulation time in seconds
	 */
	void getSimulationTime(double &simulationTime);

	/**
	 * Gets the sampling time
	 * @para samplingTime - Sampling time variable
	 */
	void getSamplingTime(double &samplingTime);

	/**
	 * Gets the number of simulations (iterations) per cycle
	 * @param simPerCycle - Number of simulations per cycle
	 */
	void getSimPerCycle(int &simPerCycle);

	/**
	 * Converts from euler angles to quaternions.
	 * @param quaternion - Quaternion variable
	 * @param eulerAngles - Euler angles vector
	 */
	static void eulerToQuaternion(base::Quaterniond &quaternion,
						   const Eigen::Vector3d &eulerAngles);

	/**
	 * Converts the Body-Frame coordinates into World-Frame coordinates.
	 * @param worldCoordinates - Vector that will receive the world coordinates
	 * @param bodyCoordinates - Vector that contains the body coordinates
	 * @param eulerAngles - Current euler angles necessary to do the frame
	 * 						transformation
	 */
	static void convBodyToWorld(base::Vector6d &worldCoordinates,
								const base::Vector6d &bodyCoordinates,
								const base::Vector3d &eulerAngles);

	/**
	 * Converts the World-Frame coordinates into Body-Frame coordinates.
	 * @param bodyCoordinates - Vector that will receive the body coordinates
	 * @param worldCoordinates - Vector that contains the world coordinates
	 * @param eulerAngles - Current euler angles necessary to do the frame
	 * 						transformation
	 */
	static void convWorldToBody(base::Vector6d &bodyCoordinates,
								const base::Vector6d &worldCoordinates,
								const base::Vector3d &eulerAngles);

	/**
	 * Calculates the transformation matrix that is used to transform
	 * coordinates from Body-Frame to World-Frame.
	 * ( worldFrame = transMatrix * bodyFrame )
	 * @param transfMatrix - Transformation matrix
	 * @param eulerAngles - Current euler angles
	 */
	static void calcTransfMatrix(base::Matrix6d &transfMatrix,
								 const base::Vector3d &eulerAngles);

protected:

	/**
	 * Calculates the vehicle acceleration based on the current velocity,
	 * on the control input (efforts) and on the mathematical model.
	 * NOTE: This function should be used only by the RK4 Integrator.
	 * @param velocityAndAcceleration - Variable used to return the new
	 * 									accelerations together with the
	 * 									current velocities
	 * @param velocity - Current velocities
	 * @param controlInput - Current control input
	 */
	void calcAcceleration(Eigen::VectorXd &velocityAndAcceleration,
						  const base::Vector6d &velocity,
			  	  	  	  const base::Vector6d &controlInput);

private:

	/**
	 * Calculates the inverse of the inertia matrix. It considers both positive and negative
	 * inertia matrices.
	 */
	void calcInvInertiaMatrix(base::Matrix6d &invInertiaMatrix, const base::Vector6d &velocity);

	/**
	 * Functions for calculating the hydrodynamics effects. They consider both positive
	 * and negative hydrodynamic matrices.
	 */
	void calcCoriolisEffect(base::Vector6d &coriolisEffect, const base::Vector6d &velocity);
        void calcLiftEffect(base::Vector6d &LiftEffect, const base::Vector6d &velocity);
        void calcRBCoriolis(base::Vector6d &RBCoriolis, const base::Vector6d &velocity);
        void calcAddedMassCoriolis(base::Vector6d &AddedMassCoriolis, const base::Vector6d &velocity);
	void calcLinDamping(base::Vector6d &linDamping, const base::Vector6d &velocity);
	void calcQuadDamping(base::Vector6d &quadDamping, const base::Vector6d &velocity);
	void calcGravityBuoyancy(base::Vector6d &gravitybuoyancy, const Eigen::Vector3d &eulerOrientation);
        void calcModelCorrection(base::Vector6d &ModelCorrection, const base::Vector6d &velocity);

	/**
	 * Converts the PWM signal into its equivalent in DC voltage
	 */
	void pwmToDC(Eigen::VectorXd &dcVolt, const base::samples::Joints &controlInput);

	/**
	 * Converts the DC voltage into thrust force for each one of the thrusters
	 */
	void dcToThrustForce(Eigen::VectorXd &thrustForces, const Eigen::VectorXd &dcVolt);

	/**
	 * Converts the RPM commands into thrusters' forces
	 */
	void rpmToThrustForce(Eigen::VectorXd &thrustForces, const base::samples::Joints &controlInput);

	/**
	 * Converts the individual thrusters' forces into the general effort for the vehicle
	 */
	void thrustForceToEffort(base::Vector6d &forcesAndMoments, const Eigen::VectorXd &thrustInput);

	/**
	 * Updates the current states (pose and velocity)
	 */
	void updateStates(Eigen::VectorXd &newSystemStates);

	/**
	 * Sets the Inertia matrices
	 * @param inertiaMatrixPos - Inertia matrix for positive velocities
	 * @param inertiaMatrixNeg - Inertia matrix for negative velocities
	 */
	void setInertiaMatrix(const base::Matrix6d &inertiaMatrixPos,
						  const base::Matrix6d &inertiaMatrixNeg =
								Eigen::MatrixXd::Zero(6,6));

	/**
	 * Sets the Coriolis matrices
	 * @param coriolisMatrixPos - Coriolis matrix for positive velocities
	 * @param coriolisMatrixNeg - Coriolis matrix for negative velocities
	 */
	void setCoriolisMatrix(const base::Matrix6d &coriolisMatrixPos,
						   const base::Matrix6d &coriolisMatrixNeg =
								 Eigen::MatrixXd::Zero(6,6));

         /**
	 * Sets the Added Mass matrices
	 * @param AddedMassMatrixPos - Added mass matrix for positive velocities
	 * @param AddedMassMatrixMatrixNeg - Added mass matrix for negative velocities
	 */
	void setAddedMassMatrix(const base::Matrix6d &AddedMassMatrixPos,
						   const base::Matrix6d &AddedMassMatrixNeg =
								 Eigen::MatrixXd::Zero(6,6));


         /**
	 * Sets the Lift coefficients
	 * @param LiftCoefficients
	 */
         void setLiftCoefficients(const base::Vector4d &LiftCoefficients);

	/**
	 * Sets the Linear Damping matrices
	 * @param linDampingMatrixPos - Linear Damping matrix for positive velocities
	 * @param linDampingMatrixNeg - Linear Damping matrix for negative velocities
	 */
	void setLinDampingMatrix(const base::Matrix6d &linDampingMatrixPos,
						     const base::Matrix6d &linDampingMatrixNeg =
								   Eigen::MatrixXd::Zero(6,6));

	/**
	 * Sets the Quadratic Damping matrices
	 * @param quadDampingMatrixPos - Quadratic Damping matrix for positive velocities
	 * @param quadDampingMatrixNeg - Quadratic Damping matrix for negative velocities
	 */
	void setQuadDampingMatrix(const base::Matrix6d &quadDampingMatrixPos,
						      const base::Matrix6d &quadDampingMatrixNeg =
								    Eigen::MatrixXd::Zero(6,6));

/**
 * FUNCTIONS FOR CHECKING FOR USER'S MISUSE
 */

	/**
	 * Checks if the variables provided in the class construction are valid
	 */
	void checkConstruction(double &samplingTime, uint &simPerCycle, double &initialTime);

	/**
	 * Determinant of inertiaMatrix must be different from zero
	 */
	void checkParameters(const uwv_dynamic_model::Parameters &pwvParameters);

	/**
	 * Checks if the positive matrices were set.
	 */
	void checkPositiveMatrices(void);

	/**
	 * Checks if the negativeMatrix was set. If not, its value will be replaced by positiveMatrix.
	 */
	void checkNegativeMatrices(base::Matrix6d &negativeMatrix, const base::Matrix6d &positiveMatrix);

	/**
	 * Checks if the provided controlInput is valid
	 */
	void checkControlInput(const base::samples::Joints &controlInput, std::string element);

	/**
	 * Checks if at least one of the PWM coefficients was set
	 */
	void checkPWMCoefficients(void);

	/**
	 * Checks if the RPM coefficients were set
	 */
	void checkRPMCoefficients(void);

	/**
	 * Checks if any error flag is activated
	 */
	void checkErrors(void);


/**
 * SYSTEM STATES
 */

	/**
	 * Pose variables
	 */
	Eigen::Vector3d gPosition;
	Eigen::Vector3d gEulerOrientation;

	/**
	 * Velocity variables
	 */
	Eigen::Vector3d gLinearVelocity;
	Eigen::Vector3d gAngularVelocity;

	/**
	 * Acceleration variables
	 */
	Eigen::Vector3d gLinearAcceleration;
	Eigen::Vector3d gAngularAcceleration;

	/**
	 * Vector with forces and moments applied to the vehicle
	 */
	base::Vector6d gEfforts;

	/**
	 * Current time
	 */
	double gCurrentTime;


/**
 * SYSTEM'S DIMENSION
 */

	/**
	 * Number of system states
	 */
	int gSystemOrder;

	/**
	 * Number of control inputs
	 */
	int gControlOrder;


/**
 * SIMULATION PARAMETERS
 */

	double gSamplingTime;
	int gSimPerCycle;

/**
 * 	MODEL PARAMETERS
 */

	/**
	 * Intertia matrices for positive and negative speeds
	 */
	base::Matrix6d gInertiaMatrixPos;
	base::Matrix6d gInertiaMatrixNeg;

	/**
	 * Coriolis matrices for positive and negative speeds
	 */
	base::Matrix6d gCoriolisMatrixPos;
	base::Matrix6d gCoriolisMatrixNeg;

	/**
	 * Coriolis matrices for positive and negative speeds
	 */
	base::Matrix6d gAddedMassMatrixPos;
	base::Matrix6d gAddedMassMatrixNeg;

	/**
	 * Linear damping matrices for positive and negative speeds
	 */
	base::Matrix6d gLinDampMatrixPos;
	base::Matrix6d gLinDampMatrixNeg;

	/**
	 * Quadratic damping matrices for positive and negative speeds
	 */
	base::Matrix6d gQuadDampMatrixPos;
	base::Matrix6d gQuadDampMatrixNeg;

	/**
	 * Thrust configuration matrix
	 */
	Eigen::MatrixXd gThrustConfigMatrix;

	/**
	 * Lift coefficients
	 */
	Eigen::Vector4d gLiftCoefficients;

	/**
	 * Thrusters' coefficients for PWM and RPM
	 */
	Direction gThrusterCoeffPWM;
	Direction gLinThrusterCoeffPWM;
	Direction gQuadThrusterCoeffPWM;
	Direction gThrusterCoeffRPM;
	double gThrusterVoltage;


/**
 * RESTORING FORCES PARAMETERS
 */

	/**
	 * Total weight force of the vehicle
	 */
	double gWeight;

	/**
	 * Total buoyancy force of the vehicle
	 */
	double gBuoyancy;

	base::Vector3d gCenterOfGravity;
	base::Vector3d gCenterOfBuoyancy;
	bool gUWVFloat;
	double gUWVMass;
	double gUWVVolume;
	double gGravity;
	double gWaterDensity;


/**
* ERROR VARIABLES
*/
	bool errorModelInit;
	bool errorConstruction;
	bool errorControlInput;
	bool errorSetParameters;
	bool errorPWMCoeff;
	bool errorRPMCoeff;
	bool errorStatus;
};
};
#endif
