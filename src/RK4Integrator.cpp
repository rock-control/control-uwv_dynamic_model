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

namespace uwv_dynamic_model
{
RK4_SIM::RK4_SIM(double integrationStep)
:    gIntegStep(integrationStep)
{
    checkStep(integrationStep);
}

RK4_SIM::~RK4_SIM()
{
}

PoseVelocityState RK4_SIM::calcStates(const PoseVelocityState &states, const base::Vector6d &control_input)
{
    checkInputs(states, control_input);
    PoseVelocityState system_states = states;

    // Runge-Kuta coefficients
    PoseVelocityState k1 = deriv(system_states, control_input);
    PoseVelocityState k2 = deriv(system_states + k1*(gIntegStep/2), control_input);
    PoseVelocityState k3 = deriv(system_states + k2*(gIntegStep/2), control_input);
    PoseVelocityState k4 = deriv(system_states + k3*gIntegStep, control_input);

    // Calculating the system states
    system_states += (k1 + k2*2 + k3*2 + k4)*(gIntegStep/6);
    return system_states;
}

PoseVelocityState RK4_SIM::deriv(const PoseVelocityState &current_states, const base::Vector6d &control_input)
{
    PoseVelocityState derivatives = poseDeriv(current_states);
    PoseVelocityState vel_deriv = velocityDeriv(current_states, control_input);
    derivatives.linear_velocity = vel_deriv.linear_velocity;
    derivatives.angular_velocity = vel_deriv.angular_velocity;
    return derivatives;
}

PoseVelocityState RK4_SIM::velocityDeriv(PoseVelocityState current_states, const base::Vector6d &control_input)
{
    return current_states*0;
}

PoseVelocityState RK4_SIM::poseDeriv(PoseVelocityState current_states)
{
    return current_states*0;
}

void RK4_SIM::setIntegrationStep(const double integration_step)
{
    checkStep(integration_step);
    gIntegStep = integration_step;
}

void RK4_SIM::checkStep(double integration_step)
{
    if (integration_step <= 0)
        throw std::runtime_error("uwv_dynamic_model: RK4Integrator.cpp: Integration step is equal or smaller than zero.");
}

void RK4_SIM::checkInputs(const PoseVelocityState &states, const base::Vector6d &control_input)
{
    if( states.hasNaN())
        throw std::runtime_error( "uwv_dynamic_model: RK4Integrator.cpp: The system states has a nan");
    if(control_input.hasNaN())
        throw std::runtime_error( "uwv_dynamic_model: RK4Integrator.cpp: Control input has a nan.");
}
};
