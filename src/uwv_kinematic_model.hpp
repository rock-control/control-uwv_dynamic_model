#ifndef _UWV_KINEMATIC_MODEL_H_
#define _UWV_KINEMATIC_MODEL_H_

#include "uwv_dataTypes.hpp"

namespace underwaterVehicle
{
class KinematicModel
{
public:
    KinematicModel();

    ~KinematicModel();

    /** Compute position derivatives/ linear velocity in world-frame
     *
     *  @param linear velocity in body frame.
     *  @param actual orientation
     *  @return pose derivatives (linear velocity in world-frame)
     */
    static base::Vector3d calcPoseDeriv(const base::Vector3d &linear_velocity, const base::Orientation &orientation);

    /** Compute quaternion derivatives
     *
     *  @param angular velocity in body frame.
     *  @param actual orientation
     *  @return orientation derivatives (quaternion derivatives)
     */
    static base::Orientation calcOrientationDeriv(const base::Vector3d &ang_vel, const base::Orientation &orientation);

    /** Check velocity
     *
     *  Throw if velocity has a NaN
     *  @param velocity
     */
    static void checkVelocity(const base::Vector3d &velocity);
};
};
#endif
