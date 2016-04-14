#ifndef _DYNAMIC_MODEL_H_
#define _DYNAMIC_MODEL_H_

#include "DataTypes.hpp"

namespace uwv_dynamic_model
{
class DynamicModel
{
public:
    DynamicModel();

    ~DynamicModel();

    /** Compute Acceleration
     *
     *  @param control input (forces and torques) in body frame.
     *  @param actual linear/angular velocity in body frame.
     *  @param actual orientation
     *  @return linear/angular acceleration in body frame
     */
    base::Vector6d calcAcceleration(const base::Vector6d &control_input, const base::Vector6d &velocity, const base::Orientation &orientation);

    /**
     * Sets the general UWV parameters
     * @param uwvParamaters - Structures containing the uwv parameters
     */
    void setUWVParameters(const UWVParameters &uwv_parameters);

    /**
     * Gets the underwater vehicle parameters
     * @return - Underwater vehicle parameters
     */
    UWVParameters getUWVParameters(void) const;

private:

    /**
     * Calculates the inverse of the inertia matrix.
     */
    base::Matrix6d calcInvInertiaMatrix(const base::Matrix6d &inertia_matrix) const;

    /**
     * Functions for calculating the hydrodynamics effects.
     */

    /** Compute coriolis and centripetal forces
     *
     *  Based on Fossen[1994] and McFarland[2013]
     * @param coriolis matrix
     * @param velocity
     * @return forces and torques vector
     */
    base::Vector6d calcCoriolisEffect( const base::Matrix6d &inertia_matrix, const base::Vector6d &velocity) const;

    /** Compute damping effect
     *
     * @param uwv_paramters
     * @param velocity vector
     * @return vecotr of damping effect
     */
    base::Vector6d caclDampingEffect( const UWVParameters &uwv_parameters, const base::Vector6d &velocity) const;

    /** Compute quadratic damping for the SIMPLE mode
     *
     *  Based on the usual linear + quadratic damping proposed by Fossen[1994].
     * @param vector of damping matrices
     * @param velocity vector
     * @return dampingEffect
     */
    base::Vector6d caclSimpleDamping( const std::vector<base::Matrix6d> &damp_matrices, const base::Vector6d &velocity) const;

    /** Compute damping for the COMPLEX mode
     *
     *  Based on the general 6DOF coupled quadratic drag matrix proposed by McFarland[2013]
     * @param vector of 6 quadDamping matrices
     * @param velocity vector
     * @return dampingEffect
     */
    base::Vector6d caclGeneralQuadDamping( const std::vector<base::Matrix6d> &quad_damp_matrices, const base::Vector6d &velocity) const;

    /** Compute linear damping in SIMPLE mode
     *
     *  Based on the usual linear damping proposed by Fossen[1994].
     * @param linDamping diagonal matrix
     * @param velocity vector
     * @return forces and torques vector
     */
    base::Vector6d calcLinDamping(const base::Matrix6d &lin_damp_matrix, const base::Vector6d &velocity) const;

    /** Compute quadratic damping in SIMPLE mode
     *
     * Based on the usual quadratic damping proposed by Fossen[1994]
     * @param quadDamping diagonal matrix
     * @param velocity vector
     * @return forces and torques vector
     */
    base::Vector6d calcQuadDamping( const base::Matrix6d &quad_damp_matrix, const base::Vector6d &velocity) const;

    /** Compute gravity and bouyancy terms
     * @param current orientation
     * @param uwv_parametes
     * @return vecto ofr forces and torques
     */
    base::Vector6d calcGravityBuoyancy(const base::Orientation& orientation, const UWVParameters &uwv_parameters) const;

    /** Compute gravity and buoyancy terms
     *
     * @param quaternion orientation
     * @param weight [N]
     * @param buoyancy [N]
     * @param vector center of gravity
     * @param vector center of buoyancy
     * @return forces and torques vector
     */
    base::Vector6d calcGravityBuoyancy( const base::Orientation& orientation,
            const double& weight, const double& bouyancy,
            const base::Vector3d& cg, const base::Vector3d& cb) const;

    /**
     * FUNCTIONS FOR CHECKING FOR USER'S MISUSE
     */

    /**
     * Determinant of inertiaMatrix must be different from zero
     */
    void checkParameters(const UWVParameters &uwv_parameters);

    /**
     * Check control input.
     */
    void checkControlInput(const base::Vector6d &control_input) const;

    /**
     * Check velocity
     */
    void checkVelocity(const base::Vector6d &velocity);

    /**
     * MODEL PARAMETERS
     */

    UWVParameters gUwvParameters;

    /**
     * Inverse of inertia matrix
     */
    base::Matrix6d gInvInertiaMatrix;
};
};
#endif
