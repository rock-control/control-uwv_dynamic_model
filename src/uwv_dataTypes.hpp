/****************************************************************************/
/*  Data types for an underwater vehicle 	                           	   	*/
/*                                                                         	*/
/* FILE --- uwv_dataTypes.h		                                   		   	*/
/*                                                                         	*/
/* PURPOSE --- Header file for a data types used in modeling a 		   		*/
/*             underwater vehicle. 					   						*/
/*                                                                         	*/
/*  Sankaranarayanan Natarajan                                             	*/
/*  sankar.natarajan@dfki.de                                               	*/
/*  DFKI - BREMEN 2011                                                     	*/
/****************************************************************************/
#ifndef _UWV_DATATYPES_H_
#define _UWV_DATATYPES_H_

#include "base/Eigen.hpp"
#include <vector>

namespace underwaterVehicle
{
/**
 * Structure used to define if the specific parameter is for positive or negative movement
 */
struct Direction
{
    double positive;
    double negative;
};

/**
 * Define which model the library is going to use.
 *
 * Simple Model:
 * acceleration  = invInertiaMatrix * ( gEfforts - linDamping - quadDamping - gravityBuoyancy)
 *
 * Complex Model:
 * acceleration  = invInertiaMatrix * ( gEfforts - coriolisEffect - RBCoriolis - AddedMassCoriolis -
 *                 LiftEffect  - linDamping - quadDamping - gravityBuoyancy - ModelCorrection)
 */


enum ModelType
{
    SIMPLE,
    COMPLEX
};

/**
 * Structure that contains all the necessary information for simulating the motion model
 */
struct Parameters
{
    /**
     * Number of controllable inputs
     */
    int ctrl_order;

    /**
     * Sampling time used in the simulation (in seconds)
     */
    double samplingtime;

    /**
     * Number of RK4 iterations per sampling interval
     */
    int sim_per_cycle;

    /**
     * Intertia matrices for positive and negative speeds
     */
    base::Matrix6d massMatrix;
    base::Matrix6d massMatrixNeg;

    /**
     * Coriolis matrices for positive and negative speeds
     * OBS: when setting coriolis matrix, make sure to set the added mass matrix
     *      to zero and add the added mass terms in the inertia matrix
     */
    base::Matrix6d coriolisMatrix;
    base::Matrix6d coriolisMatrixNeg;

    /**
     * Added Mass matrices for positive and negative speeds
     * OBS: only set theses matrices if you want to use the complete fossen's model,
     * otherwise add the added mass terms in the inertiaMatrix.
     */
    base::Matrix6d AddedMassMatrixPos;
    base::Matrix6d AddedMassMatrixNeg;

    /**
     * Linear damping matrices for positive and negative speeds
     */
    base::Matrix6d linDampMatrix;
    base::Matrix6d linDampMatrixNeg;

    /**
     * Quadratic damping matrices for positive and negative speeds
     */
    base::Matrix6d quadDampMatrix;
    base::Matrix6d quadDampMatrixNeg;

    /**
     * Lift coefficients (Yuv, Zuw, Muw, Nuv)
     */
    base::Vector4d LiftCoefficients;

    /**
     * Thrust configuration matrix
     */
    base::MatrixXd thruster_control_matrix;

    /**
     * Distance from the origin of the body-fixed frame to the center of buoyancy
     */
    base::Vector3d distance_body2centerofbuoyancy;

    /**
     * Distance from the origin of the body-fixed frame to the center of grabity
     */
    base::Vector3d distance_body2centerofgravity;

    /**
     * Weight of the vehicle
     */
    double weight;

    /**
     * Buoyancy of the vehicle
     */
    double buoyancy;

    /**
     * Variable used to convert the PWM signal into its corresponding DC Voltage
     */
    std::vector<double> thrusterVoltage;

    /**
     * Independent coefficient used to convert the thruster voltage into efforts
     */
    std::vector<Direction> thruster_coefficients_pwm;

    /**
     * Linear coefficient used to convert the thruster voltage into efforts
     */
    std::vector<Direction> linear_thruster_coefficients_pwm;

    /**
     * Quadratic coefficient used to convert the thruster voltage into efforts
     */
    std::vector<Direction> square_thruster_coefficients_pwm;

    /**
     * Coefficients used to convert the thruster rotational speed into efforts
     */
    std::vector<Direction> thruster_coefficient_rpm;

    /**
     * Initial condition used for simulation
     */
    double initial_condition[12];

    Parameters():
        ctrl_order(4),
        samplingtime(0.1),
        sim_per_cycle(10),
        massMatrix(Eigen::MatrixXd::Zero(6,6)),
        massMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
        coriolisMatrix(Eigen::MatrixXd::Zero(6,6)),
        coriolisMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
        AddedMassMatrixPos(Eigen::MatrixXd::Zero(6,6)),
        AddedMassMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
        linDampMatrix(Eigen::MatrixXd::Zero(6,6)),
        linDampMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
        quadDampMatrix(Eigen::MatrixXd::Zero(6,6)),
        quadDampMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
        LiftCoefficients(Eigen::VectorXd::Zero(4)),
        thruster_control_matrix(Eigen::MatrixXd::Zero(6,1)),
        distance_body2centerofbuoyancy(Eigen::VectorXd::Zero(3)),
        distance_body2centerofgravity(Eigen::VectorXd::Zero(3)),
        weight(0),
        buoyancy(0),
        thrusterVoltage(0)
    {
        thruster_coefficients_pwm.resize(1);
        linear_thruster_coefficients_pwm.resize(1);
        square_thruster_coefficients_pwm.resize(1);
        thruster_coefficient_rpm.resize(1);

        thruster_coefficients_pwm[0].positive = 0;
        thruster_coefficients_pwm[0].negative = 0;
        linear_thruster_coefficients_pwm[0].positive = 0;
        linear_thruster_coefficients_pwm[0].negative = 0;
        square_thruster_coefficients_pwm[0].positive = 0;
        square_thruster_coefficients_pwm[0].negative = 0;
        thruster_coefficient_rpm[0].positive = 0;
        thruster_coefficient_rpm[0].negative = 0;

        for(int i = 0; i < 12; i++)
            initial_condition[i] = 0;
    };
};

};
#endif
