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
#include <iostream>

namespace underwaterVehicle
{
RK4_SIM::RK4_SIM(double integrationStep, int systemOrder)
:   gSystemOrder(systemOrder),
    gIntegStep(integrationStep)
{
    checkConstruction(integrationStep, systemOrder);
}

RK4_SIM::~RK4_SIM()
{
}

Eigen::VectorXd RK4_SIM::calcStates(const Eigen::VectorXd &systemStates, const base::Vector6d &controlInput)
{
    /**
     * systemStates:
     *
     * [0] = x			[7]  = u	(SURGE)
     * [1] = y			[8]  = v	(SWAY)
     * [2] = z			[9]  = w	(HEAVE)
     * [3] = e1  	    [10]  = p	(ROLL)
     * [4] = e2		    [11] = q	(PITCH)
     * [5] = e3         [12] = r    (YAW)
     * [6] = n
     *
     */
    checkInputs(systemStates, controlInput);
    Eigen::VectorXd system_states = systemStates;

    // Runge-Kuta coefficients
    Eigen::VectorXd k1 = DERIV(system_states,controlInput);
    Eigen::VectorXd k2 = DERIV(system_states + gIntegStep/2*k1, controlInput);
    Eigen::VectorXd k3 = DERIV(system_states + gIntegStep/2*k2, controlInput);
    Eigen::VectorXd k4 = DERIV(system_states + gIntegStep*k3, controlInput);

    // Calculating the system states
    system_states += (gIntegStep/6) * (k1 + 2*k2 + 2*k3 + k4);

    return system_states;
}

void RK4_SIM::setIntegrationStep(const double integrationStep)
{
    if (integrationStep <= 0)
        throw std::runtime_error("Library: RK4Integrator.cpp: integrationStep is smaller than zero.");
    gIntegStep = integrationStep;
}

void RK4_SIM::checkConstruction(double integrationStep, int systemOrder)
{
    if (integrationStep <= 0)
        throw std::runtime_error("Library: RK4Integrator.cpp: integrationStep is smaller than zero.");
    if (systemOrder <= 0)
        throw std::runtime_error("Library: RK4Integrator.cpp: systemOrder is smaller than zero.");
}

void RK4_SIM::checkInputs(const Eigen::VectorXd &systemStates, const base::Vector6d &controlInput)
{
    if( systemStates.size() != gSystemOrder)
        throw std::runtime_error( "Library: RK4Integrator.cpp: The systemStates should have a size equal to of the system state.");

    if( systemStates.hasNaN())
        throw std::runtime_error( "Library: RK4Integrator.cpp: The systemStates has a nan");

    if(controlInput.hasNaN())
        throw std::runtime_error( "Library: RK4Integrator.cpp: controlInput is nan.");
}
};
