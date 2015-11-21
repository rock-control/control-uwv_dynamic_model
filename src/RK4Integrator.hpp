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

namespace underwaterVehicle
{
class RK4_SIM
{
public:

    /*
     * Constructor
     */
    RK4_SIM(int &controlOrder,
            double integrationStep);

    /**
     *	Performs one step simulation
     */
    void calcStates(Eigen::VectorXd &systemStates,
            double &currentTime,
            const base::Vector6d &controlInput);

    /*
     * Uses the system's dynamic equations in order to calculate the
     * acceleration according to the current system states and control
     * input.
     * This function is overloaded in the derived class.
     */
    virtual void calcAcceleration(Eigen::VectorXd &acceleration,
            const base::Vector6d &velocity,
            const base::Vector6d &controlInput) = 0;

    void setIntegrationStep(const double integrationStep);

private:

    /**
     * Number of control inputs
     */
    const int gControlOrder;

    /**
     * Number of system states
     */
    const int gSystemOrder;

    /**
     * Integration step size
     */
    double gIntegStep;

    bool error;


    /**
     * Calculates the k1 coefficient of the Runge-Kutta Integration method
     */
    inline void calcK1 (Eigen::VectorXd &k1,
            const base::Vector6d &velocity,
            const base::Vector6d &controlInput);

    /**
     * Calculates the k2 coefficient of the Runge-Kutta Integration method
     */
    inline void calcK2 (Eigen::VectorXd &k2,
            const Eigen::VectorXd &k1,
            base::Vector6d velocity,
            const base::Vector6d &controlInput);

    /**
     * Calculates the k3 coefficient of the Runge-Kutta Integration method
     */
    inline void calcK3 (Eigen::VectorXd &k3,
            const Eigen::VectorXd &k2,
            base::Vector6d velocity,
            const base::Vector6d &controlInput);

    /**
     * Calculates the k4 coefficient of the Runge-Kutta Integration method
     */
    inline void calcK4 (Eigen::VectorXd &k4,
            const Eigen::VectorXd &k3,
            base::Vector6d velocity,
            const base::Vector6d &controlInput);

    /**
     * Update the given k coefficient according to the integration step value
     */
    inline void updateCoefficient(Eigen::VectorXd &k);

    void checkConstruction(int &controlOrder, double &integrationStep);
    void checkInputs(Eigen::VectorXd &systemStates,
            double &currentTime,
            const base::Vector6d &controlInput);
};
};

#endif

