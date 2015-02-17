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

namespace uwv_dynamic_model
{
class RK4_SIM
{
public:

	/*
	 * Constructor
	 */
	RK4_SIM(uint 		   &controlOrder,
			uint 			systemOrder,
			double 			integrationStep);

	/**
	 *	Performs one step simulation
	 */
	void calcStates(Eigen::VectorXd 		&systemStates,
			double 					&currentTime,
			const Eigen::VectorXd 	&controlInput);

	/*
	 * Uses the system's dynamic equations in order to calculate the
	 * acceleration according to the current system states and control
	 * input.
	 * This function is overloaded in the derived class.
	 */
	virtual void calcAcceleration(Eigen::VectorXd &acceleration,
			const Eigen::VectorXd &velocity,
			const Eigen::VectorXd &controlInput) = 0;

private:

	/**
	 * Number of system states
	 */
	const int gSystemOrder;

	/**
	 * Number of control inputs
	 */
	const int gControlOrder;

	/**
	 * Integration step size
	 */
	const double gIntegStep;


	/**
	 * Calculates the k1 coefficient of the Runge-Kutta Integration method
	 */
	inline void calcK1 (Eigen::VectorXd &k1,
			const Eigen::VectorXd &velocity,
			const Eigen::VectorXd &controlInput);

	/**
	 * Calculates the k2 coefficient of the Runge-Kutta Integration method
	 */
	inline void calcK2 (Eigen::VectorXd &k2,
			const Eigen::VectorXd &k1,
			const Eigen::VectorXd &velocity,
			const Eigen::VectorXd &controlInput);

	/**
	 * Calculates the k3 coefficient of the Runge-Kutta Integration method
	 */
	inline void calcK3 (Eigen::VectorXd &k3,
			const Eigen::VectorXd &k2,
			const Eigen::VectorXd &velocity,
			const Eigen::VectorXd &controlInput);

	/**
	 * Calculates the k4 coefficient of the Runge-Kutta Integration method
	 */
	inline void calcK4 (Eigen::VectorXd &k4,
			const Eigen::VectorXd &k3,
			const Eigen::VectorXd &velocity,
			const Eigen::VectorXd &controlInput);

	/**
	 * Update the given k coefficient according to the integration step value
	 */
	inline void updateCoefficient(Eigen::VectorXd &k);

	bool checkConstruction(uint &controlOrder, uint &systemOrder,
			double &integrationStep);
};
};

#endif

