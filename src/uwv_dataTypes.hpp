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
 * Define which model the library is going to use.
 *
 * Simple Model:
 * Based on Fossen[1994] and Smallwood & Whitcomb[2003]
 * Ignoring Coriolis. Damping = linearDamping + quadDamping
 * acceleration  = invInertiaMatrix * ( gEfforts - linDamping - quadDamping - gravityBuoyancy)
 *
 * Complex Model:
 * Based on Fossen[1994] and McFarland & Whitcomb[2013]
 * Considering Coriolis. Damping = sum(quadDamping_linx + quadDamping_liny + quadDamping_linz + quadDamping_angx + quadDamping_angy + quadDamping_angz)
 * acceleration  = invInertiaMatrix * ( gEfforts - coriolisEffect - quadDamping - gravityBuoyancy)
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
     * Number of RK4 iterations per sampling interval
     */
    int sim_per_cycle;

    /**
     * Inertia matrix. Including added mass.
     */
    base::Matrix6d inertiaMatrix;

    /**
     * Damping matrix
     * In SIMPLE case:
     *  dampMatrix[0] = linDamping; dampMatrix[1] = quadDamping
     * In COMPLEX case:
     *  dampMatrix[i] = quadDamping[i] / 0<=i<=5
     */
    std::vector<base::Matrix6d> dampMatrix;

    /**
     * Distance from the origin of the body-fixed frame to the center of buoyancy
     */
    base::Vector3d distance_body2centerofbuoyancy;

    /**
     * Distance from the origin of the body-fixed frame to the center of gravity
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
     * Initial condition used for simulation
     */
    double initial_condition[12];

    Parameters():
        sim_per_cycle(10),
        inertiaMatrix(Eigen::MatrixXd::Zero(6,6)),
        distance_body2centerofbuoyancy(Eigen::VectorXd::Zero(3)),
        distance_body2centerofgravity(Eigen::VectorXd::Zero(3)),
        weight(0),
        buoyancy(0)
    {
        dampMatrix.resize(2);
        for(int i = 0; i < dampMatrix.size(); i++)
            dampMatrix[i] = Eigen::MatrixXd::Zero(6,6);
        for(int i = 0; i < 12; i++)
            initial_condition[i] = 0;
    };
};

};
#endif
