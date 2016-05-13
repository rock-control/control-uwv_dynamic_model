#include "DynamicKinematicSimulator.hpp"

namespace uwv_dynamic_model
{

/**********************************************************
 * DynamicKinematicSimulator
 * Defines all state derivatives for simulation
 **********************************************************/
DynamicKinematicSimulator::DynamicKinematicSimulator( double integration_step):
        DynamicSimulator(integration_step) {}

DynamicKinematicSimulator::~DynamicKinematicSimulator()
{}

PoseVelocityState DynamicKinematicSimulator::poseDeriv(const PoseVelocityState &current_states)
{
    PoseVelocityState deriv;
    deriv.position = kinematic_model.calcPoseDeriv(current_states.linear_velocity, current_states.orientation);
    deriv.orientation = kinematic_model.calcOrientationDeriv(current_states.angular_velocity, current_states.orientation);
    return deriv;
}

};
