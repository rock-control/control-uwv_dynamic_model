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
#include "base/Pose.hpp"
#include <vector>
#include <utility>

namespace uwv_dynamic_model
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
    ModelType model_type;

    /**
     * Inertia matrix. Including added mass.
     */
    base::Matrix6d inertia_matrix;

    /**
     * Damping matrix
     * In SIMPLE case:
     *  dampMatrices[0] = linDamping; dampMatrices[1] = quadDamping
     * In COMPLEX case:
     *  dampMatrices[i] = quadDamping[i] / 0<=i<=5
     */
    std::vector<base::Matrix6d> damping_matrices;

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
        model_type(SIMPLE),
        inertia_matrix(base::Matrix6d::Identity()),
        distance_body2centerofbuoyancy(Eigen::VectorXd::Zero(3)),
        distance_body2centerofgravity(Eigen::VectorXd::Zero(3)),
        weight(1),
        buoyancy(1)
    {
        damping_matrices.resize(2);
        for(size_t i = 0; i < damping_matrices.size(); i++)
            damping_matrices[i] = Eigen::MatrixXd::Zero(6,6);
    };
};

struct PoseVelocityState
{
    // Position in world-frame
    base::Vector3d position;
    // Orientation from body-frame to world-frame
    base::Orientation orientation;
    // Body-frame linear velocity
    base::Vector3d linear_velocity;
    // Body-frame angular velocity
    base::Vector3d angular_velocity;

    PoseVelocityState():
        position(base::Vector3d::Zero()),
        orientation(base::Orientation::Identity()),
        linear_velocity(base::Vector3d::Zero()),
        angular_velocity(base::Vector3d::Zero())
    {
    }

    inline bool hasNaN() const
    {
        return (this->position.hasNaN() ||
                this->linear_velocity.hasNaN() || this->angular_velocity.hasNaN());
    }
    inline PoseVelocityState& operator+= (const PoseVelocityState &value)
    {
        this->position += value.position;
        this->orientation.coeffs() += value.orientation.coeffs();
        this->linear_velocity += value.linear_velocity;
        this->angular_velocity += value.angular_velocity;
        return *this;
    }
    inline PoseVelocityState& operator-= (const PoseVelocityState &value)
    {
        this->position -= value.position;
        this->orientation.coeffs() -= value.orientation.coeffs();
        this->linear_velocity -= value.linear_velocity;
        this->angular_velocity -= value.angular_velocity;
        return *this;
    }
    inline PoseVelocityState& operator*= (double scalar)
    {
        this->position *= scalar;
        this->orientation.coeffs() *= scalar;
        this->linear_velocity *= scalar;
        this->angular_velocity *= scalar;
        return *this;
    }
    inline PoseVelocityState& operator/= (double scalar)
    {
        this->position /= scalar;
        this->orientation.coeffs() /= scalar;
        this->linear_velocity /= scalar;
        this->angular_velocity /= scalar;
        return *this;
    }
};

inline PoseVelocityState operator+ (PoseVelocityState value1, const PoseVelocityState &value2)
{
    return value1 += value2;
}
inline PoseVelocityState operator- (PoseVelocityState value1, const PoseVelocityState &value2)
{
    return value1 -= value2;
}
inline PoseVelocityState operator* (double scalar, PoseVelocityState value)
{
    return value *= scalar;
}
inline PoseVelocityState operator* (PoseVelocityState value, double scalar)
{
    return value *= scalar;
}
inline PoseVelocityState operator/ (PoseVelocityState value, double scalar)
{
    return value /= scalar;
}

struct AccelerationState
{
    // Body frame linear acceleration
    base::Vector3d linear_acceleration;
    // Body frame angular acceleration
    base::Vector3d angular_acceleration;

    AccelerationState():
        linear_acceleration(base::Vector3d::Zero()),
        angular_acceleration(base::Vector3d::Zero())
    {
    }

    inline AccelerationState& fromVector6d(const base::Vector6d &acceleration)
    {
        this->linear_acceleration = acceleration.head<3>();
        this->angular_acceleration = acceleration.tail<3>();
        return *this;
    }
};

};
#endif
