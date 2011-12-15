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
 * If your system is laready in the form x' = f(t,x,u) you dont have 
 * to do any transformation.
 */

#include "RK4Integrator.hpp"

/*************************************************************/

RK4_SIM::RK4_SIM(int _plant_order, 
	int _ctrl_order,
	double _integration_step, 
	double _initial_time, 
	double *_initial_state)  
:   plant_order(_plant_order),
    ctrl_order(_ctrl_order),
    plant_state(_plant_order),
    ctrl_input(_ctrl_order),
    f1(_plant_order),
    f2(_plant_order),
    f3(_plant_order),
    f4(_plant_order),
    temp(_plant_order)
{
    init_param(_integration_step, _initial_time, _initial_state);
}

RK4_SIM::~RK4_SIM()
{}

void RK4_SIM::init_param(double _integration_step, 
	double _initial_time, 
	double *_initial_state)
{
    integration_step = _integration_step;
    current_time = _initial_time; 

    if (_initial_state == NULL) // No initial conditions specified
    {
	for (int i=0; i < plant_order; i++)
	{
	    plant_state[i] = 0.0;
	}
    }
    else // Initialize with the provided initial conditions
    {
	for (int i=0; i < plant_order; i++)
	{
	    plant_state[i] = _initial_state[i];
	}
    }

    // Initial control is zero
    for (int i=0; i < ctrl_order; i++)
    {
	ctrl_input[i] = 0.0;
    }
}

void RK4_SIM::solve (void)
{
    // Determine Runge-Kutta coefficients
    F1 ();
    F2 ();
    F3 ();
    F4 ();

    // Update time and output values
    current_time += integration_step;
    for (int i=0; i < plant_order; i++)
    {
	plant_state[i] = plant_state[i] + 
	    (1.0/6.0) * (f1[i] + 2.0*f2[i] + 2.0*f3[i] + f4[i]);
    }
}

inline void RK4_SIM::F1 (void)
{
    DERIV (current_time, &(plant_state[0]), &(ctrl_input[0]), &(f1[0]));
    for (int i=0; i < plant_order; i++)
    {
	f1[i] = integration_step * f1[i];
    }
}

inline void RK4_SIM::F2 (void)
{
    for (int i=0; i < plant_order; i++)
    {
	temp[i] = plant_state[i] + 0.5*f1[i];
    }

    DERIV ((current_time + 0.5*integration_step), &(temp[0]), &(ctrl_input[0]), &(f2[0]));

    for (int i=0; i < plant_order; i++)
    {
	f2[i] = integration_step * f2[i];
    }
}

inline void RK4_SIM::F3 (void)
{
    for (int i=0; i < plant_order; i++)
    {
	temp[i] = plant_state[i] + 0.5*f2[i];
    }

    DERIV ((current_time + 0.5*integration_step), &(temp[0]), &(ctrl_input[0]), &(f3[0]));

    for (int i=0; i < plant_order; i++)
    {
	f3[i] = integration_step * f3[i];
    }
}

inline void RK4_SIM::F4 (void)
{
    for (int i=0; i < plant_order; i++)
    {
	temp[i] = plant_state[i] + f3[i];
    }

    DERIV ((current_time + integration_step), &(temp[0]), &(ctrl_input[0]), &(f4[0]));

    for (int i=0; i < plant_order; i++)
    {
	f4[i] = integration_step * f4[i];
    }
}
