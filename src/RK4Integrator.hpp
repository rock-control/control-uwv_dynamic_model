/* 
 * PURPOSE --- Header file for a class for a 4'th order Runge-Kutta 
 * method for solving a system of n first order differential equations.
 * If the initial equation is in the form of n'th order differential equation
 * it must be converted to a system of n first order differential
 * equations.
 *
 */

#ifndef RK4_INTEGRATOR_HPP
#define RK4_INTEGRATOR_HPP

#include <base/Eigen.hpp>
#include "DataTypes.hpp"
#include "DataTypes.hpp"
#include "UwvDynamicModel.hpp"
#include "UwvKinematicModel.hpp"
#include <stdexcept>

namespace uwv_dynamic_model
{
class RK4_SIM
{
public:

    /* Constructor
     *
     * Constructor for Generic state space representation
     * @param step
     */
    RK4_SIM( double integration_step = 0.01);

    virtual ~RK4_SIM();

    /** Performs one step simulation.
     *
     *  Particular case of state space representation. PoseVelocityState structure instead of vector of states.
     *	@param actual state
     *	@param control_input
     *	@return next state
     */
    PoseVelocityState calcStates(const PoseVelocityState &states, const base::Vector6d &control_input);

    /* Compute derivatives of states
     *
     * Particular case of state space representation. PoseVelocityState structure instead of vector of states.
     * @param current state
     * @param control input
     * @return state derivatives
     *
     * This function is overloaded in the derived class.
     */
    PoseVelocityState deriv(const PoseVelocityState &current_states, const base::Vector6d &control_input);

    /** Compute derivative of velocity states
     *
     *  @param current state
     *  @param control input
     *  @return velocity derivatives
     *
     * This function is overloaded in the derived class.
     */
    virtual PoseVelocityState velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input);

    /** Compute derivative of pose states
     *
     *  @param current state
     *  @return pose derivatives
     *
     * This function is overloaded in the derived class.
     */
    virtual PoseVelocityState poseDeriv(const PoseVelocityState &current_states);


    /** Set step
     *
     *  @param step
     */
    void setIntegrationStep(double integration_step);

private:

    /**
     * Integration step size
     */
    double gIntegStep;


    void checkStep(double integration_step);
    void checkInputs(const PoseVelocityState &states, const base::Vector6d &control_input);
};

/**********************************************************
 * Dynamic Simulator
 * DYN_SIM
 * Defines velocity derivatives for the dynamic simulation
 **********************************************************/
class DYN_SIM: public RK4_SIM
{
public:
    DYN_SIM( double integration_step = 0.01);

    virtual ~DYN_SIM();

    /** Overrides
     *  Compute Acceleration (velocity derivatives in body frame)
     *
     *  @param current_state
     *  @param forces & torques
     *  @return Velocity derivatives
     */
    PoseVelocityState velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input);

    /** Overrides
     * Pose derivatives as zero
     *
     * @param current_states
     * @return zero derivatives
     */
    PoseVelocityState poseDeriv(const PoseVelocityState &current_states);

    /** Get acceleration
     *
     * @return Acceleration
     */
    AccelerationState getAcceleration() const;

    /**
     * Dynamic Model
     */
    DynamicModel gDynamicModel;

private:
    /**
     * Acceleration variables
     */
    AccelerationState gAcceleration;

};

/**********************************************************
 * Dynamic & Kinematic Simulator
 * DYN_KIN_SIM
 * Defines all state derivatives for simulation
 **********************************************************/
class DYN_KIN_SIM: public DYN_SIM
{
public:
    DYN_KIN_SIM( double integration_step = 0.01);

    virtual ~DYN_KIN_SIM();

    /** Overrides
     * Compute pose derivatives (velocities in world frame)
     *
     * @param current_states of pose and velocities
     */
    PoseVelocityState poseDeriv(const PoseVelocityState &current_states);

    /**
     *  Kinematic Model
     */
    KinematicModel gKinematicModel;
};

};

#endif

