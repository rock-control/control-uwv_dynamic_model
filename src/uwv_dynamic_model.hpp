/***************************************************************************/
/*  Dynamic model for a underwater vehicle         	                   	   */
/*                                                                         */
/* FILE --- uwv_dynamic_model.hpp		                                   */
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


#include "RK4Integrator.hpp"
#include "uwv_dataTypes.hpp"
#include "base/samples/Joints.hpp"
#include "base/Pose.hpp"


namespace uwv_dynamic_model
{
class DynamicModel : public RK4_SIM
{

public:
	/********************* Data functions **************************************/
	DynamicModel(uint controlOrder, uint plantOrder = 12, double samplingTime = 0.01,
				 uint simPerCycle = 10, double initialTime = 0.0);

	/**
	 * Initializes the model parameters. Run this function before sending control
	 * commands to the model.
	 */
	void initParameters(const uwv_dynamic_model::Parameters &uwvParameters);

	/**
	 * Function for sending PWM commands to the model.
	 * @param controlInput - PWM commands that should be applied to the model
	 */
	void sendPWMCommands(const base::samples::Joints &controlInput);

	/**
	 * Function for sending RPM commands to the model.
	 * @param controlInput - RPM commands that should be applied to the model
	 */
	void sendRPMCommands(const base::samples::Joints &controlInput);

	/**
	 * Function for sending Effort commands to the model.
	 * @param controlInput - Effort commands that should be applied to the model
	 */
	void sendEffortCommands(const base::samples::Joints &controlInput);

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
						  const Eigen::VectorXd &velocity,
			  	  	  	  const Eigen::VectorXd &controlInput);

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
	 * Sets the Thrust Configuration Matrix
	 * @param thrustConfigMatrix - Thrust Configuration Matrix
	 */
	void setThrustConfigMatrix(const Eigen::MatrixXd &thrustConfigMatrix);

	/**
	 * Sets the vehicle position
	 * @param position - New position
	 */
	void setPosition(const Eigen::Vector3d &position);

	/**
	 * Sets the vehicle euler orientation
	 * @param eulerOrientation - New euler orientation
	 */
	void setEulerOrientation(const Eigen::Vector3d &eulerOrientation);

	/**
	 * Sets the vehicle quaternion orientation
	 * @param quatOrientation - New quaternion orientation
	 */
	void setQuatOrientation(const Eigen::Quaterniond &quatOrientation);

	/**
	 * Sets the vehicle linear velocity
	 * @param linearVelocity - New linear velocity
	 */
	void setLinearVelocity(const Eigen::Vector3d &linearVelocity);

	/**
	 * Sets the vehicle angular velocity
	 * @param angularVelocity - New angular velocity
	 */
	void setAngularVelocity(const Eigen::Vector3d &angularVelocity);

	/**
	 * Resets position, orientation and velocities of the model
	 */
	void resetAll(void);

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
	void getEulerOrientation(base::Vector3d &eulerOrientation);

	/**
	 * Gets the quaternion orientation
	 * @param quatOrientation - Quaternion orientation
	 */
	void getQuatOrienration(base::Quaterniond &quatOrientation);

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
	 * Gets the simulation time in seconds
	 * @param simulationTime - Simulation time in seconds
	 */
	void getSimulationTime(double &simulationTime);

	/**
	 * Converts from euler angles to quaternions.
	 */
	void eulerToQuaternion(Eigen::Quaterniond &quaternion, const Eigen::Vector3d &eulerAngles);

private:

	/**
	 * Calculates the inverse of the inertia matrix. It considers both positive and negative
	 * inertia matrices.
	 */
	void calcInvInertiaMatrix(base::Matrix6d &invInertiaMatrix, const Eigen::VectorXd &velocity);

	/**
	 * Functions for calculating the hydrodynamics effects. They consider both positive
	 * and negative hydrodynamic matrices.
	 */
	void calcCoriolisEffect(Eigen::VectorXd &coriolisEffect, const Eigen::VectorXd &velocity);
	void calcLinDamping(Eigen::VectorXd &linDamping, const Eigen::VectorXd &velocity);
	void calcQuadDamping(Eigen::VectorXd &quadDamping, const Eigen::VectorXd &velocity);
	void calcGravityBuoyancy(Eigen::VectorXd &gravitybuoyancy, const Eigen::Vector3d &eulerOrientation);
	void calcGravityBuoyancy(Eigen::VectorXd &gravitybuoyancy, const Eigen::Quaterniond &quatOrientation);

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
	void thrustForceToEffort(Eigen::VectorXd &forcesAndMoments, const Eigen::VectorXd &thrustInput);

	/**
	 * Updates the current states (pose and velocity)
	 */
	void updateStates(Eigen::VectorXd &newSystemStates);

/**
 * FUNCTIONS FOR CHECKING FOR USER'S MISUSE
 */

	/**
	 * Checks if the variables provided in the class construction are valid
	 */
	bool checkConstruction(double &samplingTime, uint &simPerCycle, double &initialTime);

	/**
	 * Determinant of inertiaMatrix must be different from zero
	 */
	bool checkInitialization(base::Matrix6d &inertiaMatrixPos, base::Matrix6d &inertiaMatrixNeg);

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
	bool checkControlInput(const base::samples::Joints &controlInput, std::string element);

	/**
	 * Checks if at least one of the PWM coefficients was set
	 */
	bool checkPWMCoefficients(void);

	/**
	 * Checks if the RPM coefficients were set
	 */
	bool checkRPMCoefficients(void);

	/**
	 * Checks if any error flag is activated
	 */
	bool checkErrors(void);


/**
 * SYSTEM STATES
 */

	uwv_dynamic_model::Parameters gUWVParameters;

	/**
	 * Pose variables
	 */
	Eigen::Vector3d gPosition;
	Eigen::Vector3d gEulerOrientation;
	Eigen::Quaterniond gQuatOrientation;

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
	Eigen::VectorXd gEfforts;

	/**
	 * Current system states
	 */
	Eigen::VectorXd gSystemStates;

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
	 * Thrusters' coefficients for PWM and RPM
	 */
	Direction gThrusterCoeffPWM;
	Direction gLinThrusterCoeffPWM;
	Direction gQuadThrusterCoeffPWM;
	Direction gThrusterCoeffRPM;


/**
 * RESTORING FORCES
 */

	/**
	 * Total weight force of the vehicle
	 */
	double gWeight;

	/**
	 * Total buoyancy force of the vehicle
	 */
	double gBuoyancy;

/**
* ERROR VARIABLES
*/
	bool errorModelInit;
	bool errorConstruction;
	bool errorControlInput;
	bool errorInitialization;
	bool errorStatus;
};
};
#endif
