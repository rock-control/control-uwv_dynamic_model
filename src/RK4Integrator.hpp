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

#include "DataTypes.hpp"

namespace uwv_dynamic_model
{
class RK4Integrator
{
public:

    /* Constructor
     *
     * Constructor for Generic state space representation
     * @param step
     */
    RK4Integrator( double integration_step = 0.01);

    virtual ~RK4Integrator();

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
    virtual PoseVelocityState velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input) = 0;

    /** Compute derivative of pose states
     *
     *  @param current state
     *  @return pose derivatives
     *
     * This function is overloaded in the derived class.
     */
    virtual PoseVelocityState poseDeriv(const PoseVelocityState &current_states) = 0;


    /** Set step
     *
     *  @param step
     */
    void setIntegrationStep(double step);

private:

    /**
     * Integration step size
     */
    double integration_step;


    void checkStep(double step);
    void checkInputs(const PoseVelocityState &states, const base::Vector6d &control_input);
};
};

#endif

