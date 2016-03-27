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
 * Ignoring Coriolis (by now). Damping = linearDamping + quadDamping
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
struct UWVParameters
{
    /**
     * Type of model to be used
     */
    ModelType modelType;

    /**
     * Inertia matrix. Including added mass.
     */
    base::Matrix6d inertiaMatrix;

    /**
     * Damping matrix
     * In SIMPLE case:
     *  dampMatrices[0] = linDamping; dampMatrices[1] = quadDamping
     * In COMPLEX case:
     *  dampMatrices[i] = quadDamping[i] / 0<=i<=5
     */
    std::vector<base::Matrix6d> dampMatrices;

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


    UWVParameters():
        modelType(SIMPLE),
        inertiaMatrix(base::Matrix6d::Identity()),
        distance_body2centerofbuoyancy(Eigen::VectorXd::Zero(3)),
        distance_body2centerofgravity(Eigen::VectorXd::Zero(3)),
        weight(1),
        buoyancy(1)
    {
        dampMatrices.resize(2);
        for(size_t i = 0; i < dampMatrices.size(); i++)
            dampMatrices[i] = Eigen::MatrixXd::Zero(6,6);
    };
};

};
#endif
