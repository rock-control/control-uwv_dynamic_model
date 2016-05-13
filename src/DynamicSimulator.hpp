#ifndef DYNAMIC_SIMULATOR_HPP
#define DYNAMIC_SIMULATOR_HPP

#include "DataTypes.hpp"
#include "RK4Integrator.hpp"
#include "DynamicModel.hpp"

namespace uwv_dynamic_model
{
/**********************************************************
 * Dynamic Simulator
 * Defines velocity derivatives for the dynamic simulation
 **********************************************************/
class DynamicSimulator: public RK4Integrator
{
public:
    DynamicSimulator( double integration_step = 0.01);

    virtual ~DynamicSimulator();

    /** Overrides
     *  Compute Acceleration (velocity derivatives in body frame)
     *
     *  @param current_state
     *  @param forces & torques
     *  @return Velocity derivatives
     */
    PoseVelocityState velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input);

    /** Overrides
     * Pose derivatives as zero
     *
     * @param current_states
     * @return zero derivatives
     */
    PoseVelocityState poseDeriv(const PoseVelocityState &current_states);

    /** Get acceleration
     *
     * @return Acceleration
     */
    AccelerationState getAcceleration() const;

    /**
     * Access to dynamic_model
     */
    DynamicModel* getDynamicModel();

private:

    /**
     * Dynamic Model
     */
    DynamicModel dynamic_model;

    /**
     * Acceleration variables
     */
    AccelerationState acceleration;

};

};

#endif

