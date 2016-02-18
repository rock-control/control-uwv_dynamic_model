/***************************************************************************/
/*  Dynamic model for an underwater vehicle	                               */
/*                                                                         */
/* FILE --- uwv_dynamic_model.hpp	                                       */
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
#include "base/samples/RigidBodyState.hpp"
#include "base/Pose.hpp"


namespace underwaterVehicle
{
class DynamicModel : public RK4_SIM
{

public:
    DynamicModel(double samplingTime = 0.01,
            int simPerCycle = 10, double initialTime = 0.0);

    /**
     * Initializes the model parameters. Run this function before sending control
     * commands to the model.
     */
    bool initParameters(const underwaterVehicle::Parameters &uwvParameters);

    /**
     * Sets the general UWV parameters
     * @param uwvParamaters - Structures containing the uwv parameters
     */
    bool setUWVParameters(const underwaterVehicle::Parameters &uwvParameters);

    /**
     * Resets position, orientation and velocities of the model
     */
    void resetStates(void);

    /**
     * Sets the current position of the vehicle
     * * @param position - New position value
     */
    void setPosition(const base::Vector3d &position);

    /**
     * Sets the current orientation of the vehicle
     * * @param quatOrientation - New orientation value
     */
    void setOrientation(const Eigen::Quaterniond &quatOrientation);

    /**
     * Sets the current linear velocity of the vehicle
     * * @param linearVelocity - New linear velocity value
     */
    void setLinearVelocity(const base::Vector3d &linearVelocity);

    /**
     * Sets the current angular velocity of the vehicle
     * * @param angularVelocity - New angular velocity value
     */
    void setAngularVelocity(const base::Vector3d &angularVelocity);

    /**
     * Sets the model type for the simulation (simple or complex)
     * @param modelType - Simple or complex model
     */
    void setModelType(const underwaterVehicle::ModelType &modelType);

    /**
     * Sets the samling time
     * * @param samplingTime - New sampling time value
     */
    void setSamplingTime(const double samplingTime);

    /**
     * Gets the underwater vehicle parameters
     * @return - Underwater vehicle parameters
     */
    underwaterVehicle::Parameters getUWVParameters(void) const;

    /**
     * Gets the position
     * @return Position vector
     */
    base::Position getPosition(void) const;

    /**
     * Gets the euler orientation
     * @return eulerOrientation - Euler orientation vector
     */
    base::Vector3d getEulerOrientation(void) const;

    /**
     * Gets the quaternion orientation
     * @return quatOrientation - Quaternion orientation
     */
    base::Orientation getQuatOrienration(void) const;

    /**
     * Gets the linear velocity
     * @return linearVelocity - Linear velocity vector
     */
    base::Vector3d getLinearVelocity(bool worldFrame = true) const;

    /**
     * Gets the angular velocity
     * @return angularVelocity - Angular velocity vector
     */
    base::Vector3d getAngularVelocity(bool worldFrame = true) const;

    /**
     * Gets the linear acceleration
     * @return linearAcceleration - Linear acceleration vector
     */
    base::Vector3d getLinearAcceleration(void) const;

    /**
     * Gets the angular acceleration
     * @return angularAcceleration - Angular acceleration vector
     */
    base::Vector3d getAngularAcceleration(void) const;

    /**
     * Gets the the system states (pose and velocity)
     * @return systemStates - System states vector (size = 12)
     */
    Eigen::VectorXd getStates(void) const;

    /**
     * Gets the current efforts' vector
     * @return efforts - Vector containing the current effort (forces and moments)
     */
    base::Vector6d getEfforts(void) const;

    /**
     * Gets the simulation time in seconds
     * @return simulationTime - Simulation time in seconds
     */
    double getSimulationTime(void) const;

    /**
     * Gets the sampling time
     * @return samplingTime - Sampling time variable
     */
    double getSamplingTime(void) const;

    /**
     * Gets the number of simulations (iterations) per cycle
     * @return simPerCycle - Number of simulations per cycle
     */
    int getSimPerCycle(void) const;

    /**
     * Converts from euler angles to quaternions.
     * @param eulerAngles - Euler angles vector
     * @return quaternion - Quaternion variable
     */
    static base::Quaterniond eulerToQuaternion( const base::Vector3d &eulerAngles) const;

protected:

    /**
     * Calculates the vehicle acceleration based on the current velocity,
     * on the control input (efforts) and on the mathematical model.
     * NOTE: This function should be used only by the RK4 Integrator.
     * @return velocityAndAcceleration - Variable used to return the new
     *                                  accelerations together with the
     *                                  current velocities
     * @param velocity - Current velocities
     * @param controlInput - Current control input
     */
    Eigen::VectorXd calcAcceleration( const base::Vector6d &velocity,
                          const base::Vector6d &controlInput);

private:

    /**
     * Calculates the inverse of the inertia matrix.
     */
    base::Matrix6d calcInvInertiaMatrix(void) const;

    /**
     * Functions for calculating the hydrodynamics effects.
     */

    /** Compute coriolis matrix
     *
     *  @param inertiaMatrix
     *  @param velocity
     *  @return coriolis matrix
     */
    base::Matrix6d calcCoriolisMatrix( const base::Matrix6d &inertiaMatrix, const base::Vector6d &velocity) const;

    /** Compute coriolis and centripetal forces
     *
     * @param coriolis matrix
     * @param velocity
     * @return forces and torques vector
     */
    base::Vector6d calcCoriolisEffect( const base::Matrix6d &coriolisMatrix, const base::Vector6d &velocity) const;

    /** Compute quadratic matrix for the COMPLEX mode
     *
     * @param vector of 6 quadDamping matrices
     * @param velocity vector
     * @return dampingMatrix
     */
    base::Matrix6d caclDampMatrix( const vector<base::Matrix6d> &quadDampMatrices, const base::Vector6d &velocity) const;

    /** Compute linear damping in SIMPLE mode
     *
     * @param linDamping matrix
     * @param velocity vector
     * @return forces and torques vector
     */
    base::Vector6d calcLinDamping(const base::Matrix6d &linDampMatrix, const base::Vector6d &velocity) const;

    /** Compute quadratic damping in SIMPLE mode
     *
     * @param quadDamping matrix
     * @param velocity vector
     * @return forces and torques vector
     */
    base::Vector6d calcQuadDamping( const base::Matrix6d &quadDampMatrix, const base::Vector6d &velocity) const;

    /** Compute gravity and buoyancy terms
     *
     * @param quaternion orientation
     * @param weight [N]
     * @param buoyancy [N]
     * @param vector center of gravity
     * @param vector center of bouyancy
     * @return forces and torques vector
     */
    base::Vector6d calcGravityBuoyancy( const Eigen::Quaterniond& orientation, const double& weight, const double& bouyancy, const base::Vector3d& cg, const base::Vector3d& cb) const;

    /**
     * Updates the current states (pose and velocity)
     */
    void updateStates(Eigen::VectorXd &newSystemStates);

    /**
     * Sets the Inertia matrix (inertia + added mass matrix)
     * @param inertiaMatrix - Inertia matrix
     */
    void setInertiaMatrix(const base::Matrix6d &inertiaMatrixPos);

    /**
     * Sets the Linear Damping matrix for SIMPLE CASE
     * @param linDampingMatrix - Linear Damping matrix
     */
    void setLinDampingMatrix(const base::Matrix6d &linDampingMatrix);

    /**
     * Sets the Quadratic Damping matrix
     * @param quadDampingMatrixPos - Quadratic Damping matrix
     */
    void setQuadDampingMatrix(const base::Matrix6d &quadDampingMatrix);

    /**
     * Sets the Damping matrices
     * @param dampingMatrices - Vector of Damping matrix
     */
    void setDampingMatrix(const base::vector<base::Matrix6d> &dampingMatrices);

    /**
     * FUNCTIONS FOR CHECKING FOR USER'S MISUSE
     */

    /**
     * Checks if the variables provided in the class construction are valid
     */
    void checkConstruction(double &samplingTime, int &simPerCycle, double &initialTime);

    /**
     * Determinant of inertiaMatrix must be different from zero
     */
    void checkParameters(const underwaterVehicle::Parameters &pwvParameters);

    /**
     * Checks if the positive matrices were set.
     */
    void checkPositiveMatrices(void);

    /**
     * Checks if the negativeMatrix was set. If not, its value will be replaced by positiveMatrix.
     */
    void checkNegativeMatrices(base::Matrix6d &negativeMatrix, const base::Matrix6d &positiveMatrix);

    /**
     * Checks if any error flag is activated
     */
    void checkErrors(void);

    underwaterVehicle::ModelType gModelType;


    /**
     * SYSTEM STATES
     */

    /**
     * Pose variables
     */
    base::Vector3d gPosition;
    base::Quaterniond gOrientation;

    /**
     * Velocity variables
     */
    base::Vector3d gLinearVelocity;
    base::Vector3d gAngularVelocity;

    /**
     * Acceleration variables
     */
    base::Vector3d gLinearAcceleration;
    base::Vector3d gAngularAcceleration;

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
     * SIMULATION PARAMETERS
     */

    double gSamplingTime;
    int gSimPerCycle;

    /**
     * MODEL PARAMETERS
     */

    /**
     * Inertia matrix
     */
    base::Matrix6d gInertiaMatrix;

    /**
     * Linear damping matrix for SIMPLE model type
     */
    base::Matrix6d gLinDampMatrix;

    /**
     * Quadratic damping matrix for SIMPLE model type
     */
    base::Matrix6d gQuadDampMatrix;

    /**
     * Quadratic damping matrices for COMPLEX model type
     */
    std::vector<base::Matrix6d> gQuadDampMatrices;

    /**
     * RESTORING FORCES PARAMETERS
     */

    double gWeight;
    double gBuoyancy;
    base::Vector3d gCenterOfGravity;
    base::Vector3d gCenterOfBuoyancy;

    /**
     * ERROR VARIABLES
     */
    bool errorModelInit;
    bool errorConstruction;
    bool errorControlInput;
    bool errorSetParameters;
    bool errorStatus;

};
};
#endif
