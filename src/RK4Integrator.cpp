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
#include <math.h>
#include <base/Logging.hpp>

namespace uwv_dynamic_model
{
RK4_SIM::RK4_SIM(uint &controlOrder, uint systemOrder, double integrationStep)
:   gControlOrder(controlOrder),
    gSystemOrder(systemOrder),
    gIntegStep(integrationStep)
{
}

void RK4_SIM::calcStates(Eigen::VectorXd &systemStates,
		double &currentTime, const Eigen::VectorXd &controlInput)
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

	// Runge-Kuta coefficients
	Eigen::VectorXd k1 = Eigen::VectorXd::Zero(gSystemOrder);
	Eigen::VectorXd k2 = Eigen::VectorXd::Zero(gSystemOrder);
	Eigen::VectorXd k3 = Eigen::VectorXd::Zero(gSystemOrder);
	Eigen::VectorXd k4 = Eigen::VectorXd::Zero(gSystemOrder);

	// Calculating Runge-Kutta coefficients
	calcK1(k1, 	   systemStates, controlInput);
	calcK2(k2, k1, systemStates, controlInput);
	calcK3(k3, k2, systemStates, controlInput);
	calcK4(k4, k3, systemStates, controlInput);

	// Updating simulation time
	currentTime += gIntegStep;

	// Calculating the system states
	for (int i=0; i < gSystemOrder; i++)
		systemStates[i] +=	(1.0/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);

	// Checking if the angle is between [-PI, PI], and if not, doing the
	// necessary corrections
	for (int i = 0; i < 3; i++)
	{
		if (systemStates[i+3] > M_PI)
			systemStates[i+3] -= 2*M_PI;

		else if (systemStates[i+3] < -M_PI)
			systemStates[i+3] += 2*M_PI;
	}
}

inline void RK4_SIM::calcK1 (Eigen::VectorXd &k1,
							 const Eigen::VectorXd &systemStates,
							 const Eigen::VectorXd &controlInput)
{
	Eigen::VectorXd velocity = systemStates.block(6,0,6,1);
	calcAcceleration(k1, velocity, controlInput);

	updateCoefficient(k1);
}

inline void RK4_SIM::calcK2 (Eigen::VectorXd &k2, const Eigen::VectorXd &k1,
		const Eigen::VectorXd &systemStates,
		const Eigen::VectorXd &controlInput)
{
	Eigen::VectorXd velocity = Eigen::VectorXd::Zero(6);

	for (int i=0; i < 6; i++)
		velocity[i] = systemStates[i+6] + 0.5*k1[i+6];

	calcAcceleration(k2, velocity, controlInput);

	updateCoefficient(k2);
}

inline void RK4_SIM::calcK3 (Eigen::VectorXd &k3, const Eigen::VectorXd &k2,
		const Eigen::VectorXd &systemStates,
		const Eigen::VectorXd &controlInput)
{
	Eigen::VectorXd velocity = Eigen::VectorXd::Zero(6);

	for (int i=0; i < 6; i++)
		velocity[i] = systemStates[i+6] + 0.5*k2[i+6];

	calcAcceleration(k3, velocity, controlInput);

	updateCoefficient(k3);
}

inline void RK4_SIM::calcK4 (Eigen::VectorXd &k4, const Eigen::VectorXd &k3,
		const Eigen::VectorXd &systemStates,
		const Eigen::VectorXd &controlInput)
{
	Eigen::VectorXd velocity = Eigen::VectorXd::Zero(6);

	for (int i=0; i < 6; i++)
		velocity[i] = systemStates[i+6] + k3[i+6];

	calcAcceleration(k4, velocity, controlInput);

	updateCoefficient(k4);
}

void RK4_SIM::updateCoefficient(Eigen::VectorXd &k)
{
	for (int i=0; i < gSystemOrder; i++)
		k[i] = gIntegStep * k[i];
}

bool RK4_SIM::checkConstruction(uint &controlOrder, uint &systemOrder,
					   double &integrationStep)
{
	std::string textElement;
	bool checkError = false;

	if (controlOrder == 0)
	{
		textElement = "control order";
		checkError = true;
	}
	else if (systemOrder == 0)
	{
		textElement = "system order";
		checkError = true;
	}
	else if (integrationStep <= 0)
	{
		textElement = "integration step";
		checkError = true;
	}

	if(checkError)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: RK4Integrator.cpp)"
				  " The %s should be greater than zero.\x1b[0m\n\n",
					textElement.c_str());
		return false;
	}
	return true;
}
};
