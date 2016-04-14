#ifndef DYNAMIC_KINEMATIC_SIMULATOR_HPP
#define DYNAMIC_KINEMATIC_SIMULATOR_HPP

#include "DataTypes.hpp"
#include "DynamicSimulator.hpp"
#include "KinematicModel.hpp"


namespace uwv_dynamic_model
{
/**********************************************************
 * Dynamic & Kinematic Simulator
 * Defines all state derivatives for simulation
 **********************************************************/
class DynamicKinematicSimulator: public DynamicSimulator
{
public:
    DynamicKinematicSimulator( double integration_step = 0.01);

    virtual ~DynamicKinematicSimulator();

    /** Overrides
     * Compute pose derivatives (velocities in world frame)
     *
     * @param current_states of pose and velocities
     */
    PoseVelocityState poseDeriv(const PoseVelocityState &current_states);

private:
    /**
     *  Kinematic Model
     */
    KinematicModel kinematic_model;
};
};
#endif

