#include "KinematicModel.hpp"

namespace uwv_dynamic_model
{
KinematicModel::KinematicModel()
{
}

KinematicModel::~KinematicModel()
{
}

base::Vector3d KinematicModel::calcPoseDeriv(const base::Vector3d &linear_velocity, const base::Orientation &orientation)
{
    checkVelocity(linear_velocity);
    return orientation.matrix()*linear_velocity;
}

base::Orientation KinematicModel::calcOrientationDeriv(const base::Vector3d &ang_vel, const base::Orientation &orientation)
{
    /** Based on Fossen[2011], Andrle[2013] & Wertz[1978]
     *
     *  qdot = 1/2*T(q)*w = 1/2*Omega(w)*q
     *  Omega(w) = [-J(w), w;
     *              -w^t , 0]
     *   J(w): skew-symmetric matrix
     *
     *   Quaternion representation:
     *   q = (q_r, q_i); q_r: real part, q_i: imaginary part
     *   Quaternion multiplication:
     *   q = q1*q2 = M(q2)*q1
     *   M(q) = [q_r,   q_i3, -q_i2, q_i1;
     *          -q_i3,  q_r,   q_i1, q_i2;
     *           q_i2, -q_i1,  q_r,  q_i3;
     *          -q_i1, -q_i2, -q_i3, q_r ]
     *   M((0,q_i)) = Omega(q_i)
     *
     *   => qdot = 1/2*orientation*w_as_quaternion
     *   w_as_quaternion.norm != 1 and qdot.norm != 1
     *
     * Fossen, Thor I. Handbook of marine craft hydrodynamics and motion control. John Wiley & Sons, 2011.
     * Andrle, Michael S., and John L. Crassidis. "Geometric integration of quaternions." Journal of Guidance, Control, and Dynamics 36.6 (2013): 1762-1767.
     * (Astrophysics and Space Science Library 73) James R. Wertz (auth.), James R. Wertz (eds.)-Spacecraft Attitude Determination and Control-Springer Netherlands (1978)
     */
    checkVelocity(ang_vel);
    return orientation * base::Orientation(0, ang_vel[0]*0.5, ang_vel[1]*0.5, ang_vel[2]*0.5);
}

void KinematicModel::checkVelocity(const base::Vector3d &velocity)
{
    if(velocity.hasNaN())
        throw std::runtime_error("KinematicModel checkVelocity: velocity is unset");
}

};
