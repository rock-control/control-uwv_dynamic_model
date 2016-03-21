/*
 * PURPOSE --- Implements 4th order Runge-Kutta differential equation
 * solver of an n'th order system.  User writes virtual function DERIV() 
 * containing system dynamics in the form of n first order differential  
 * equations, or x' = f(t,x,u). Therefore, if the initial equation is in 
 * the form of n'th order differential equation, it must be converted to 
 * a system of n first order differential equations. 
 *
 * Example : 
 * Given the differential equation
 * W'' + aW' + bW = cF 
 * Define
 * X1 = W 
 * X2 = W' 
 * To obtain the system of equations 
 * X1' = X2 
 * X2' = -aX2 - bX1 + cF 
 *
 * If your system is already in the form x' = f(t,x,u) you dont have
 * to do any transformation.
 */


#include "RK4Integrator.hpp"
#include <math.h>
#include <base/Logging.hpp>

namespace underwaterVehicle
{
RK4_SIM::RK4_SIM(double integrationStep)
:   gSystemOrder(12),
    gIntegStep(integrationStep)
{
    checkConstruction(integrationStep);
}

RK4_SIM::~RK4_SIM()
{
}

Eigen::VectorXd RK4_SIM::calcStates(const Eigen::VectorXd &systemStates,
        double &currentTime, const base::Vector6d &controlInput)
{
    /**
     * systemStates:
     *
     * [0] = x			[6]  = u	(SURGE)
     * [1] = y			[7]  = v	(SWAY)
     * [2] = z			[8]  = w	(HEAVE)
     * [3] = theta  	[9]  = p	(ROLL)
     * [4] = phi		[10] = q	(PITCH)
     * [5] = sigma		[11] = r	(YAW)
     *
     */
    Eigen::VectorXd system_states = systemStates;
    base::Vector6d velocity = systemStates.tail(6);

    // Runge-Kuta coefficients
    Eigen::VectorXd k1 = Eigen::VectorXd::Zero(gSystemOrder);
    Eigen::VectorXd k2 = Eigen::VectorXd::Zero(gSystemOrder);
    Eigen::VectorXd k3 = Eigen::VectorXd::Zero(gSystemOrder);
    Eigen::VectorXd k4 = Eigen::VectorXd::Zero(gSystemOrder);

    // Calculating Runge-Kutta coefficients
    k1 = calcK1(    velocity, controlInput);
    k2 = calcK2(k1, velocity, controlInput);
    k3 = calcK3(k2, velocity, controlInput);
    k4 = calcK4(k3, velocity, controlInput);

    // Updating simulation time
    currentTime += gIntegStep;

    // Calculating the system states
    system_states += (gIntegStep/6) * (k1 + 2*k2 + 2*k3 + k4);

    // Checking if the angle is between [-PI, PI], and if not, doing the
    // necessary corrections
    for (int i = 0; i < 3; i++)
    {
        if (system_states[i+3] > M_PI)
            system_states[i+3] -= 2*M_PI;

        else if (system_states[i+3] < -M_PI)
            system_states[i+3] += 2*M_PI;
    }
    return system_states;
}

inline Eigen::VectorXd RK4_SIM::calcK1 ( const base::Vector6d &velocity,
        const base::Vector6d &controlInput)
{
    return calcAcceleration( velocity, controlInput);
}

inline Eigen::VectorXd RK4_SIM::calcK2 (const Eigen::VectorXd &k1,
        base::Vector6d velocity,
        const base::Vector6d &controlInput)
{
    velocity += gIntegStep/2 * k1.tail(6);
    return calcAcceleration( velocity, controlInput);
}

inline Eigen::VectorXd RK4_SIM::calcK3 ( const Eigen::VectorXd &k2,
        base::Vector6d velocity,
        const base::Vector6d &controlInput)
{
    velocity += gIntegStep/2 * k2.tail(6);
    return calcAcceleration( velocity, controlInput);
}

inline Eigen::VectorXd RK4_SIM::calcK4 ( const Eigen::VectorXd &k3,
        base::Vector6d velocity,
        const base::Vector6d &controlInput)
{
    velocity += gIntegStep * k3.tail(6);
    return calcAcceleration( velocity, controlInput);
}

void RK4_SIM::updateCoefficient(Eigen::VectorXd &k)
{
    for (int i=0; i < gSystemOrder; i++)
        k[i] = gIntegStep * k[i];
}

void RK4_SIM::setIntegrationStep(const double integrationStep)
{
    if (integrationStep <= 0)
        throw std::runtime_error("Library: RK4Integrator.cpp: integrationStep is smaller than zero.");
    gIntegStep = integrationStep;
}

void RK4_SIM::checkConstruction(double &integrationStep)
{
    if (integrationStep <= 0)
        throw std::runtime_error("Library: RK4Integrator.cpp: integrationStep is smaller than zero.");
}

void RK4_SIM::checkInputs(const Eigen::VectorXd &systemStates, double &currentTime,
        const base::Vector6d &controlInput)
{
    if( systemStates.size() != 12)
        throw std::runtime_error( "Library: RK4Integrator.cpp: The systemStates should have a size equal to 12.");

    if( currentTime <= 0)
        throw std::runtime_error( "Library: RK4Integrator.cpp: The currentTime should be positive.");

    if(controlInput.hasNaN())
        throw std::runtime_error( "Library: RK4Integrator.cpp: controlInput is nan.");
}
};
