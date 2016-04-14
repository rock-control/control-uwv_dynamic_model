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

namespace uwv_dynamic_model
{
RK4Integrator::RK4Integrator(double step)
:    integration_step(step)
{
    checkStep(step);
}

RK4Integrator::~RK4Integrator()
{}

PoseVelocityState RK4Integrator::calcStates(const PoseVelocityState &states, const base::Vector6d &control_input)
{
    checkInputs(states, control_input);
    PoseVelocityState system_states = states;

    // Runge-Kuta coefficients
    PoseVelocityState k1 = deriv(system_states, control_input);
    PoseVelocityState k2 = deriv(system_states + ((integration_step/2)*k1), control_input);
    PoseVelocityState k3 = deriv(system_states + ((integration_step/2)*k2), control_input);
    PoseVelocityState k4 = deriv(system_states + (integration_step*k3), control_input);

    // Calculating the system states
    system_states += (integration_step/6)*(k1 + 2*k2 + 2*k3 + k4);
    return system_states;
}

PoseVelocityState RK4Integrator::deriv(const PoseVelocityState &current_states, const base::Vector6d &control_input)
{
    PoseVelocityState derivatives = poseDeriv(current_states);
    PoseVelocityState vel_deriv = velocityDeriv(current_states, control_input);
    derivatives.linear_velocity = vel_deriv.linear_velocity;
    derivatives.angular_velocity = vel_deriv.angular_velocity;
    return derivatives;
}

PoseVelocityState RK4Integrator::velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input)
{
    PoseVelocityState ret;
    return 0*ret;
}

PoseVelocityState RK4Integrator::poseDeriv(const PoseVelocityState &current_states)
{
    PoseVelocityState ret;
    return 0*ret;
}

void RK4Integrator::setIntegrationStep(const double step)
{
    checkStep(step);
    integration_step = step;
}

void RK4Integrator::checkStep(double step)
{
    if (step <= 0)
        throw std::runtime_error("uwv_dynamic_model: RK4Integrator.cpp: Integration step is equal or smaller than zero.");
}

void RK4Integrator::checkInputs(const PoseVelocityState &states, const base::Vector6d &control_input)
{
    if( states.hasNaN())
        throw std::runtime_error( "uwv_dynamic_model: RK4Integrator.cpp: The system states has a nan");
    if(control_input.hasNaN())
        throw std::runtime_error( "uwv_dynamic_model: RK4Integrator.cpp: Control input has a nan.");
}
};
