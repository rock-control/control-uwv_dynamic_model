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
#include "orocos/auv_control/6dControl.hpp"


namespace underwaterVehicle
{
class DynamicModel : public RK4_SIM
{

public:
    DynamicModel(double samplingTime = 0.01,
            int simPerCycle = 10, double initialTime = 0.0);

    ~DynamicModel();

    /**
     * Function for sending Effort commands to the model.
     * @param controlInput - Effort commands that should be applied to the model
     */
    void sendEffortCommands(const base::LinearAngular6DCommand &controlInput);

    /**
     * Sets the general UWV parameters
     * @param uwvParamaters - Structures containing the uwv parameters
     */
    void setUWVParameters(const UWVParameters &uwvParameters);

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
    void setOrientation(const base::Orientation &orientation);

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
     * Sets the samling time
     * * @param samplingTime - New sampling time value
     */
    void setSamplingTime(const double samplingTime);

    /**
     * Gets the underwater vehicle parameters
     * @return - Underwater vehicle parameters
     */
    UWVParameters getUWVParameters(void) const;

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
     * Set system state (pose and velocity)
     * @param RigidBodyState state
     */
    void setRigidBodyState(base::samples::RigidBodyState const &state);

    /**
     * Gets the system state (pose and velocity)
     * @return systemState as RigidBodyState
     */
    base::samples::RigidBodyState getRigidBodyState(void) const;

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
    static base::Orientation eulerToQuaternion( base::Vector3d eulerAngles);

    /**
     * Converts the Body-Frame coordinates into World-Frame coordinates.
     * @param bodyCoordinates - Vector that contains the body coordinates
     * @param orientation - Current quaternion necessary to do the frame transformation
     * @return worldCoordinates - Vector with the world coordinates
     */
    static base::Vector6d convBodyToWorld( const base::Vector6d &bodyCoordinates,
            const base::Orientation &orientation);

    /**
     * Converts the World-Frame coordinates into Body-Frame coordinates.
     * @param worldCoordinates - Vector that contains the world coordinates
     * @param orientation - Current quaternion necessary to do the frame transformation
     * @return bodyCoordinates - Vector that will receive the body coordinates
     */
    static base::Vector6d convWorldToBody( const base::Vector6d &worldCoordinates,
            const base::Orientation &orientation);

    /**
     * Calculates the transformation matrix that is used to transform
     * coordinates from Body-Frame to World-Frame.
     * ( worldFrame = transMatrix * bodyFrame )
     * @param orientation - Get current euler angles
     * @return transfMatrix - Transformation matrix
     */
    static base::Matrix6d calcTransfMatrix( const base::Orientation &orientation);

protected:

    /**
     * Calculates the vehicle acceleration based on the current velocity,
     * on the control input (efforts) and on the mathematical model.
     * NOTE: This function should be used only by the RK4 Integrator.
     * @return velocityAndAcceleration, linear and angular
     * @param velocity - Current velocities
     * @param controlInput - Current control input
     */
    Eigen::VectorXd calcAcceleration( const base::Vector6d &velocity,
                          const base::Vector6d &controlInput);

private:

    /**
     * Calculates the inverse of the inertia matrix.
     */
    base::Matrix6d calcInvInertiaMatrix(const base::Matrix6d &inertiaMatrix) const;

    /**
     * Functions for calculating the hydrodynamics effects.
     */

    /** Compute coriolis and centripetal forces
     *
     *  Based on Fossen[1994] and McFarland[2013]
     * @param coriolis matrix
     * @param velocity
     * @return forces and torques vector
     */
    base::Vector6d calcCoriolisEffect( const base::Matrix6d &inertiaMatrix, const base::Vector6d &velocity) const;

    /** Compute damping effect
     *
     * @param uwv_paramters
     * @param velocity vector
     * @return vecotr of damping effect
     */
    base::Vector6d caclDampingEffect( const UWVParameters &uwv_parameters, const base::Vector6d &velocity) const;

    /** Compute quadratic damping for the COMPLEX mode
     *
     *  Based on the general 6DOF coupled quadratic drag matrix proposed by McFarland[2013]
     * @param vector of 6 quadDamping matrices
     * @param velocity vector
     * @return dampingEffect
     */
    base::Vector6d caclSimpleDamping( const std::vector<base::Matrix6d> &dampMatrices, const base::Vector6d &velocity) const;

    /** Compute damping for the SIMPLE mode
     *
     *  Based on the usual linear + quadratic damping proposed by Fossen[1994].
     * @param vector of damping matrices
     * @param velocity vector
     * @return dampingEffect
     */
    base::Vector6d caclGeneralQuadDamping( const std::vector<base::Matrix6d> &quadDampMatrices, const base::Vector6d &velocity) const;

    /** Compute linear damping in SIMPLE mode
     *
     *  Based on the usual linear damping proposed by Fossen[1994].
     * @param linDamping diagonal matrix
     * @param velocity vector
     * @return forces and torques vector
     */
    base::Vector6d calcLinDamping(const base::Matrix6d &linDampMatrix, const base::Vector6d &velocity) const;

    /** Compute quadratic damping in SIMPLE mode
     *
     * Based on the usual quadratic damping proposed by Fossen[1994]
     * @param quadDamping diagonal matrix
     * @param velocity vector
     * @return forces and torques vector
     */
    base::Vector6d calcQuadDamping( const base::Matrix6d &quadDampMatrix, const base::Vector6d &velocity) const;

    /** Compute gravity and bouyancy terms
     * @param current orientation
     * @param uwv_parametes
     * @return vecto ofr forces and torques
     */
    base::Vector6d calcGravityBuoyancy(const Eigen::Quaterniond& orientation, const UWVParameters &uwv_parameters) const;

    /** Compute gravity and buoyancy terms
     *
     * @param quaternion orientation
     * @param weight [N]
     * @param buoyancy [N]
     * @param vector center of gravity
     * @param vector center of buoyancy
     * @return forces and torques vector
     */
    base::Vector6d calcGravityBuoyancy( const Eigen::Quaterniond& orientation,
            const double& weight, const double& bouyancy,
            const base::Vector3d& cg, const base::Vector3d& cb) const;

    /**
     * Updates the current states (pose and velocity)
     * @param newSystemStates as vector of states
     */
    void updateStates(Eigen::VectorXd &newSystemStates);

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
    void checkParameters(const UWVParameters &pwvParameters);

    /**
     * Check control input.
     */
    void checkControlInput(const base::LinearAngular6DCommand &controlInput) const;


    /**
     * SYSTEM STATES
     */

    /**
     * Pose variables
     */
    base::Vector3d gPosition;
    base::Orientation gOrientation;

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
    static const int gSystemOrder = 12;

    /**
     * SIMULATION PARAMETERS
     */

    double gSamplingTime;
    int gSimPerCycle;

    /**
     * MODEL PARAMETERS
     */

    UWVParameters gUwvParameters;

    /**
     * Inverse of inertia matrix
     */
    base::Matrix6d gInvInertiaMatrix;
};
};
#endif
