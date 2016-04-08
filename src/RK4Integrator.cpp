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
{}

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

PoseVelocityState RK4_SIM::velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input)
{
    PoseVelocityState ret;
    return ret*0;
}

PoseVelocityState RK4_SIM::poseDeriv(const PoseVelocityState &current_states)
{
    PoseVelocityState ret;
    return ret*0;
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

/**********************************************************
 * RK4_DYN_SIM
 * Defines velocity derivatives for the dynamic simulation
 **********************************************************/
DYN_SIM::DYN_SIM( double integration_step):
        RK4_SIM(integration_step) {}

DYN_SIM::~DYN_SIM()
{}

PoseVelocityState DYN_SIM::velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input)
{
    base::Vector6d velocity;
    velocity.head(3) = current_states.linear_velocity;
    velocity.tail(3) = current_states.angular_velocity;

    // Compute acceleration (velocity derivatives)
    base::Vector6d acceleration = gDynamicModel.calcAcceleration(control_input, velocity, current_states.orientation);

    PoseVelocityState deriv;
    deriv.linear_velocity = acceleration.head<3>();
    deriv.angular_velocity = acceleration.tail<3>();

    gAcceleration.linear_acceleration = acceleration.head<3>();
    gAcceleration.angular_acceleration = acceleration.tail<3>();

    return deriv;
}

PoseVelocityState DYN_SIM::poseDeriv(const PoseVelocityState &current_states)
{
    PoseVelocityState ret;
    return ret*0;
}

AccelerationState DYN_SIM::getAcceleration() const
{
    return gAcceleration;
}

/**********************************************************
 * RK4_KIN_SIM
 * Defines all state derivatives for simulation
 **********************************************************/
DYN_KIN_SIM::DYN_KIN_SIM( double integration_step):
        DYN_SIM(integration_step) {}

DYN_KIN_SIM::~DYN_KIN_SIM()
{}

PoseVelocityState DYN_KIN_SIM::poseDeriv(const PoseVelocityState &current_states)
{
    PoseVelocityState deriv;
    deriv.position = gKinematicModel.calcPoseDeriv(current_states.linear_velocity, current_states.orientation);
    deriv.orientation = gKinematicModel.calcOrientationDeriv(current_states.angular_velocity, current_states.orientation);
    return deriv;
}
};
