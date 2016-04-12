#ifndef _UWV_MODEL_SIMULATION_H_
#define _UWV_MODEL_SIMULATION_H_

#include "RK4Integrator.hpp"
#include "DataTypes.hpp"
#include "UwvDynamicModel.hpp"
#include "UwvKinematicModel.hpp"

namespace uwv_dynamic_model
{
class BaseModelSimulation
{
public:
    BaseModelSimulation(double sampling_time = 0.01, int sim_per_cycle = 10,
                    double initial_time = 0.0);

    virtual ~BaseModelSimulation();

    /**
     * Function for sending Effort commands to the model.
     * @param controlInput - Effort commands applied to the model
     * @return computed pose state
     */
    PoseVelocityState sendEffort(const base::Vector6d &control_input);

    /** Send Effort commands to the model given the actual states.
     *
     * @param controlInput - Effort commands applied to the model
     * @param actual_pose
     * @return computed pose state
     */
    PoseVelocityState sendEffort(const base::Vector6d &control_input, const PoseVelocityState &actual_pose);

    /** Do one step simulation
     *
     *  To be override by specific simulator
     * @param actual_pose state
     * @param control_input
     * @param return computed pose state
     */
    virtual PoseVelocityState calcStates(const PoseVelocityState &actual_pose, const base::Vector6d &control_input);

    /** Get Acceleration
     *
     *  To be override by specific simulator
     *  @return Acceleration
     */
    virtual AccelerationState getAcceleration() const;

    /** Get UWV Parameters
     *
     *  To be override by specific simulator
     *  @return UWV Parameters
     */
    virtual UWVParameters getUWVParameters() const;

    /** Get UWV Parameters
     *
     *  To be override by specific simulator
     *  @param UWV Parameters
     */
    virtual void setUWVParameters(const UWVParameters &parameters);

    /** Set simulator step
     *
     *  To be override by specific simulator
     *  @param simulatorStep
     */
    virtual void setSimulatorStep(double step);

    /** Reset pose states
     *
     */
    void resetStates(void);

    /** Set the current orientation of the vehicle
     *
     * @param orientation
     */
    void setOrientation(const base::Orientation &orientation);

    /** Get Current Time
     *
     *  @return Current Time
     */
    double getCurrentTime() const;

    /** Set Current Time
     *
     *  @param Current Time
     */
    void setCurrentTime(double currentTime);

    /** Get Pose
     *
     *  @return Pose
     */
    PoseVelocityState getPose();

    /** Set Pose
     *
     *  @param  Pose
     */
    void setPose(const PoseVelocityState& pose);

    /** Get Sampling Time
     *
     *  @return sampling time
     */
    double getSamplingTime();

    /** Set Sampling Time
     *
     *  @param sampling time
     */
    void setSamplingTime(double samplingTime);

    /** Get Simulations per Cycle
     *
     *  @return simPerCycle
     */
    int getSimPerCycle() const;

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
    void checkControlInput(const base::Vector6d &control_input);

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
     * SIMULATION PARAMETERS
     */

    double gSamplingTime;
    int gSimPerCycle;

    /**
     * Current time
     */
    double gCurrentTime;
};

/**********************************************************
 * Simulation of velocity states
 * Use DYN_SIM simulator
 * Compute velocity states
 **********************************************************/
class DynamicModelSimulation: public BaseModelSimulation
{
public:
    DynamicModelSimulation(double sampling_time = 0.01, int sim_per_cycle = 10,
            double initial_time = 0.0);

    virtual ~DynamicModelSimulation();

    /** Compute one step simulation
     *
     * @param actual_sates
     * @param control_input
     * @return computed states
     */
    PoseVelocityState calcStates(const PoseVelocityState &actual_pose, const base::Vector6d &control_input);

    /** Return acceleration
     *  @return acceleration
     */
    AccelerationState getAcceleration() const;

    /** Get UWV Parameters
     *
     *  @return UWV Parameters
     */
    UWVParameters getUWVParameters() const;

    /** Get UWV Parameters
     *
     *  @param UWV Parameters
     */
    void setUWVParameters(const UWVParameters &parameters);

    /** Set simulator step
     *
     *  @param simulatorStep
     */
    void setSimulatorStep(double step);

private:
    /**
     * Dynamic Simulator
     */
    DYN_SIM simulator;
};


/**********************************************************
 * Simulation of velocity states
 * Use DYN_KIN_SIM simulator
 * Compute all state (pose & velocities)
 **********************************************************/
class ModelSimulation: public BaseModelSimulation
{
public:
    ModelSimulation(double sampling_time = 0.01, int sim_per_cycle = 10,
            double initial_time = 0.0);

    virtual ~ModelSimulation();

    /** Compute one step simulation
     *
     * @param actual_sates
     * @param control_input
     * @return computed states
     */
    PoseVelocityState calcStates(const PoseVelocityState &actual_pose, const base::Vector6d &control_input);

    /** Get acceleration
     *  @return acceleration
     */
    AccelerationState getAcceleration() const;

    /** Get UWV Parameters
     *
     *  @return UWV Parameters
     */
    UWVParameters getUWVParameters() const;

    /** Get UWV Parameters
     *
     *  @param UWV Parameters
     */
    void setUWVParameters(const UWVParameters &parameters);

    /** Set simulator step
     *
     *  @param simulatorStep
     */
    void setSimulatorStep(double step);

private:
    /**
     * Dynamic & Kinematic Simulator
     */
    DYN_KIN_SIM simulator;

};
};
#endif
