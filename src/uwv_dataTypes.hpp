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
#include <utility>      // std::swap

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

    inline PoseVelocityState operator+ (const PoseVelocityState &state)
    {
        PoseVelocityState new_state;
        new_state.position = this->position + state.position;
        new_state.orientation.coeffs() = this->orientation.coeffs() + state.orientation.coeffs();
        new_state.linear_velocity = this->linear_velocity + state.linear_velocity;
        new_state.angular_velocity = this->angular_velocity + state.angular_velocity;
        return new_state;
    }
    inline PoseVelocityState& operator+= (const PoseVelocityState &state)
    {
        this->position += state.position;
        this->orientation.coeffs() += state.orientation.coeffs();
        this->linear_velocity += state.linear_velocity;
        this->angular_velocity += state.angular_velocity;
        return *this;
    }
    inline PoseVelocityState operator- (const PoseVelocityState &state)
    {
        PoseVelocityState new_state;
        new_state.position = this->position - state.position;
        new_state.orientation.coeffs() = this->orientation.coeffs() - state.orientation.coeffs();
        new_state.linear_velocity = this->linear_velocity - state.linear_velocity;
        new_state.angular_velocity = this->angular_velocity - state.angular_velocity;
        return new_state;
    }
    inline PoseVelocityState& operator-= (const PoseVelocityState &state)
    {
        this->position -= state.position;
        this->orientation.coeffs() -= state.orientation.coeffs();
        this->linear_velocity -= state.linear_velocity;
        this->angular_velocity -= state.angular_velocity;
        return *this;
    }
    inline PoseVelocityState operator* (const double &number)
    {
        PoseVelocityState new_state;
        new_state.position = number*this->position;
        new_state.orientation.coeffs() = number*this->orientation.coeffs();
        new_state.linear_velocity = number*this->linear_velocity;
        new_state.angular_velocity = number*this->angular_velocity;
        return new_state;
    }
    inline PoseVelocityState& operator*= (const double &number)
    {
        this->position *= number;
        this->orientation.coeffs() *= number;
        this->linear_velocity *= number;
        this->angular_velocity *= number;
        return *this;
    }
};

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
