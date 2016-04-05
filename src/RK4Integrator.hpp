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
#include "uwv_dataTypes.hpp"
#include <stdexcept>

namespace underwaterVehicle
{
class RK4_SIM
{
public:

    /* Constructor
     *
     * Constructor for Generic state space representation
     * @param step
     */
    RK4_SIM( double integration_step);

    virtual ~RK4_SIM()=0;

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
    virtual PoseVelocityState DERIV(const PoseVelocityState &current_states, const base::Vector6d &control_input) = 0;

    void setIntegrationStep(double integration_step);

private:

    /**
     * Integration step size
     */
    double gIntegStep;


    void checkStep(double integration_step);
    void checkInputs(const PoseVelocityState &states, const base::Vector6d &control_input);
};
};

#endif

