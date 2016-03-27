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
#include <stdexcept>

namespace underwaterVehicle
{
class RK4_SIM
{
public:

    /*
     * Constructor
     */
    RK4_SIM( double integrationStep, int systemOrder);

    virtual ~RK4_SIM()=0;

    /** Performs one step simulation
     *
     *	@param actual state
     *	@param controlInput
     *	@return next state
     */
    Eigen::VectorXd calcStates(const Eigen::VectorXd &systemStates, const base::Vector6d &controlInput);

    /* Compute derivatives of states
     *
     * @param actual state
     * @param controlInput
     * @return state derivatives
     *
     * This function is overloaded in the derived class.
     */
    virtual Eigen::VectorXd DERIV( const Eigen::VectorXd &current_states,
            const base::Vector6d &controlInput) = 0;

    void setIntegrationStep(const double integrationStep);

private:

    /**
     * Number of system states
     */
    const int gSystemOrder;

    /**
     * Integration step size
     */
    double gIntegStep;


    void checkConstruction(double integrationStep, int systemOrder);
    void checkInputs(const Eigen::VectorXd &systemStates, const base::Vector6d &controlInput);
};
};

#endif

