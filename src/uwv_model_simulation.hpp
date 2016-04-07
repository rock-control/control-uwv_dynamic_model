#ifndef _UWV_MODEL_SIMULATION_H_
#define _UWV_MODEL_SIMULATION_H_

#include "RK4Integrator.hpp"
#include "uwv_dataTypes.hpp"
#include "uwv_dynamic_model.hpp"
#include "uwv_kinematic_model.hpp"
#include "orocos/auv_control/6dControl.hpp"


namespace underwaterVehicle
{
class ModelSimulation : public RK4_SIM
{

public:
    ModelSimulation(double sampling_time = 0.01, int sim_per_cycle = 10,
                    double initial_time = 0.0, const UWVParameters &uwv_parameters = UWVParameters());

    ~ModelSimulation();

    /**
     * Function for sending Effort commands to the model.
     * @param controlInput - Effort commands applied to the model
     * @return computed pose state
     */
    PoseVelocityState sendEffort(const base::LinearAngular6DCommand &control_input);

    /** Send Effort commands to the model given the actual states.
     *
     * @param controlInput - Effort commands applied to the model
     * @param actual_pose
     * @return computed pose state
     */
    PoseVelocityState sendEffort(const base::LinearAngular6DCommand &control_input, const PoseVelocityState actual_pose);

    /** Reset pose states
     *
     */
    void resetStates(void);

    /** Set the current orientation of the vehicle
     *
     * @param orientation
     */
    void setOrientation(const base::Orientation &orientation);

    /** Get Effort applied
     *
     * @return Efforts
     */
    base::LinearAngular6DCommand getEfforts(void);

    const AccelerationState& getAcceleration() const {
        return gAcceleration;
    }

    void setAcceleration(const AccelerationState& acceleration) {
        gAcceleration = acceleration;
    }

    double getCurrentTime() const {
        return gCurrentTime;
    }

    void setCurrentTime(double currentTime) {
        checkSimulationTime(currentTime);
        gCurrentTime = currentTime;
    }

    const PoseVelocityState& getPose() const {
        return gPose;
    }

    void setPose(const PoseVelocityState& pose) {
        checkState(pose);
        gPose = pose;
    }

    double getSamplingTime() const {
        return gSamplingTime;
    }

    void setSamplingTime(double samplingTime) {
        checkSamplingTime(samplingTime);
        gSamplingTime = samplingTime;
    }

    int getSimPerCycle() const {
        return gSimPerCycle;
    }

protected:

    /* Compute derivatives of states
     *
     * @param actual state
     * @param control input
     * @return state derivatives
     */
    PoseVelocityState velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input);

private:

    /** Check if the variables provided in the class construction are valid
     *
     *  @param samplingTime
     *  @param simulations per cycle
     *  @param initialTime
     */
    void checkConstruction(double &sampling_time, int &sim_per_cycle, double &initial_time);

    /** Check sampling time
     *
     *  @param samplingTime
     */
    void checkSamplingTime(double sampling_time);

    /** Check simulation time
     *
     *  @param simulationTime
     */
    void checkSimulationTime(double simulation_time);

    /** Check control input
     *
     *  Throw exception if control input has a NaN
     *  @param control_input
     */
    void checkControlInput(const base::LinearAngular6DCommand &control_input);

    /** Check state
     *
     * Throw exception if state has a NaN
     * @param state
     */
    void checkState(const PoseVelocityState &state);

    /**
     * SYSTEM STATES
     */

    /**
     * Pose states
     */
    PoseVelocityState gPose;

    /**
     * Acceleration variables
     */
    AccelerationState gAcceleration;

    /**
     * Vector with forces and moments applied to the vehicle
     */
    base::Vector6d gEfforts;

    /**
     * Motion Model
     */
    DynamicModel gDynamicModel;

    /**
     * SIMULATION PARAMETERS
     */

    double gSamplingTime;
    int gSimPerCycle;

    /**
     * Current time
     */
    double gCurrentTime;
};
};
#endif
