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

namespace underwaterVehicle
{
RK4_SIM::RK4_SIM(int &controlOrder, double integrationStep)
:   gControlOrder(controlOrder),
    gSystemOrder(12),
    gIntegStep(integrationStep)
{
    error = false;
    checkConstruction(controlOrder, integrationStep);
}

void RK4_SIM::calcStates(Eigen::VectorXd &systemStates,
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

    if(!error)
    {
        base::Vector6d velocity = systemStates.tail(6);

        // Runge-Kuta coefficients
        Eigen::VectorXd k1 = Eigen::VectorXd::Zero(gSystemOrder);
        Eigen::VectorXd k2 = Eigen::VectorXd::Zero(gSystemOrder);
        Eigen::VectorXd k3 = Eigen::VectorXd::Zero(gSystemOrder);
        Eigen::VectorXd k4 = Eigen::VectorXd::Zero(gSystemOrder);

        // Calculating Runge-Kutta coefficients
        calcK1(k1, 	   velocity, controlInput);
        calcK2(k2, k1, velocity, controlInput);
        calcK3(k3, k2, velocity, controlInput);
        calcK4(k4, k3, velocity, controlInput);

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
}

inline void RK4_SIM::calcK1 (Eigen::VectorXd &k1,
        const base::Vector6d &velocity,
        const base::Vector6d &controlInput)
{
    calcAcceleration(k1, velocity, controlInput);

    updateCoefficient(k1);
}

inline void RK4_SIM::calcK2 (Eigen::VectorXd &k2, const Eigen::VectorXd &k1,
        base::Vector6d velocity,
        const base::Vector6d &controlInput)
{

    for (int i=0; i < 6; i++)
        velocity[i] += 0.5*k1[i+6];

    calcAcceleration(k2, velocity, controlInput);

    updateCoefficient(k2);
}

inline void RK4_SIM::calcK3 (Eigen::VectorXd &k3, const Eigen::VectorXd &k2,
        base::Vector6d velocity,
        const base::Vector6d &controlInput)
{

    for (int i=0; i < 6; i++)
        velocity[i] += 0.5*k2[i+6];

    calcAcceleration(k3, velocity, controlInput);

    updateCoefficient(k3);
}

inline void RK4_SIM::calcK4 (Eigen::VectorXd &k4, const Eigen::VectorXd &k3,
        base::Vector6d velocity,
        const base::Vector6d &controlInput)
{
    for (int i=0; i < 6; i++)
        velocity[i] += k3[i+6];

    calcAcceleration(k4, velocity, controlInput);

    updateCoefficient(k4);
}

void RK4_SIM::updateCoefficient(Eigen::VectorXd &k)
{
    for (int i=0; i < gSystemOrder; i++)
        k[i] = gIntegStep * k[i];
}

void RK4_SIM::setIntegrationStep(const double integrationStep)
{
    bool checkError = false;
    std::string textElement;

    if (integrationStep > 0)
        gIntegStep = integrationStep;
    else
    {
        textElement = "integration step";
        checkError = true;
    }

    if(checkError)
    {
        LOG_ERROR("\n\n\x1b[31m (Library: RK4Integrator.cpp)"
                " The %s should be greater than zero.\x1b[0m\n\n",
                textElement.c_str());
        error = true;
    }
}

void RK4_SIM::checkConstruction(int &controlOrder, double &integrationStep)
{
    std::string textElement;
    bool checkError = false;

    if (controlOrder <= 0)
    {
        textElement = "control order";
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
        error = true;
    }
}

void RK4_SIM::checkInputs(Eigen::VectorXd &systemStates, double &currentTime,
        const base::Vector6d &controlInput)
{
    if( !(systemStates.size() == 12))
    {
        LOG_ERROR("\n\n\x1b[31m (Library: RK4Integrator.cpp)"
                " The systemStates should have a size equal to 12,"
                " but its size is equal to %i.\x1b[0m\n\n",
                systemStates.size());
        error = true;
    }

    if( !(controlInput.size() > 0))
    {
        LOG_ERROR("\n\n\x1b[31m (Library: RK4Integrator.cpp)"
                " The controlInput has a size equal to 0 (zero).\x1b[0m\n\n");
        error = true;
    }

    if( !(currentTime > 0))
    {
        LOG_ERROR("\n\n\x1b[31m (Library: RK4Integrator.cpp)"
                " The currentTime shoud be positive.\x1b[0m\n\n");
        error = true;
    }
}
};
