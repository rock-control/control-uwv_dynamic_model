#include "UwvModelSimulation.hpp"


namespace uwv_dynamic_model
{
BaseModelSimulation::BaseModelSimulation(double sampling_time, int sim_per_cycle,
                                 double initial_time)
{
    checkConstruction(sampling_time, sim_per_cycle, initial_time);
    gSamplingTime = sampling_time;
    gCurrentTime = initial_time;
    gSimPerCycle = sim_per_cycle;

    gPose = PoseVelocityState();
}

BaseModelSimulation::~BaseModelSimulation()
{
}

PoseVelocityState BaseModelSimulation::sendEffort(const base::Vector6d &control_input)
{
    PoseVelocityState actual_state = getPose();
    actual_state = sendEffort(control_input, actual_state);
    setPose(actual_state);
    return actual_state;
}

PoseVelocityState BaseModelSimulation::sendEffort(const base::Vector6d &control_input, const PoseVelocityState &actual_pose)
{
    // Checks control input and states
    checkControlInput(control_input);
    checkState(actual_pose);

    PoseVelocityState state = actual_pose;

    // Performs iterations to calculate the new system's states
    for (int i=0; i < gSimPerCycle; i++)
    {
        state = calcStates(state, control_input);
        //Brute force normalization of quaternions due the RK4 integration.
        state.orientation.normalize();
    }
    gCurrentTime += gSamplingTime;
    return state;
}

PoseVelocityState BaseModelSimulation::calcStates(const PoseVelocityState &actual_pose, const base::Vector6d &control_input)
{
    PoseVelocityState ret;
    return ret;
}

AccelerationState BaseModelSimulation::getAcceleration() const
{
    AccelerationState ret;
    return ret;
}

UWVParameters BaseModelSimulation::getUWVParameters() const
{
    UWVParameters ret;
    return ret;
}

void BaseModelSimulation::setUWVParameters(const UWVParameters &parameters)
{}

void BaseModelSimulation::resetStates()
{
    gPose.position = base::Vector3d::Zero();
    gPose.orientation = base::Orientation::Identity();
    gPose.linear_velocity = base::Vector3d::Zero();
    gPose.angular_velocity = base::Vector3d::Zero();
}

void BaseModelSimulation::setOrientation(const base::Orientation &orientation)
{
    gPose.orientation = orientation;
}

double BaseModelSimulation::getCurrentTime() const
{
    return gCurrentTime;
}

void BaseModelSimulation::setCurrentTime(double currentTime)
{
    checkSimulationTime(currentTime);
    gCurrentTime = currentTime;
}

PoseVelocityState BaseModelSimulation::getPose()
{
    return gPose;
}

void BaseModelSimulation::setPose(const PoseVelocityState& pose)
{
    checkState(pose);
    gPose = pose;
}

double BaseModelSimulation::getSamplingTime()
{
    return gSamplingTime;
}

void BaseModelSimulation::setSamplingTime(double samplingTime)
{
    checkSamplingTime(samplingTime);
    gSamplingTime = samplingTime;
}

int BaseModelSimulation::getSimPerCycle() const
{
    return gSimPerCycle;
}

void BaseModelSimulation::checkConstruction(double &samplingTime,
        int &simPerCycle, double &initialTime)
{
    checkSamplingTime(samplingTime);
    checkSimulationTime(initialTime);
    if (simPerCycle <= 0)
        throw std::runtime_error("simPerCycle must be positive");
}

void BaseModelSimulation::checkSamplingTime(double samplingTime)
{
    if (samplingTime <= 0)
        throw std::runtime_error("samplingTime must be positive");
}

void BaseModelSimulation::checkSimulationTime(double simulationTime)
{
    if (simulationTime < 0)
        throw std::runtime_error("simulationTime must be positive or equal to zero");
}

void BaseModelSimulation::checkControlInput(const base::Vector6d &control_input)
{
    if(control_input.hasNaN())
        throw std::runtime_error("control input has a NaN.");
}

void BaseModelSimulation::checkState(const PoseVelocityState &state)
{
    if(state.hasNaN())
        throw std::runtime_error("state has a NaN.");
}


/**********************************************************
 * Simulation of velocity states
 * Use RK4_DYN_SIM simulator
 * Compute velocity states
 **********************************************************/
DynamicModelSimulation::DynamicModelSimulation(double sampling_time, int sim_per_cycle, double initial_time)
    :BaseModelSimulation(sampling_time, sim_per_cycle, initial_time)
{
    simulator.setIntegrationStep(sampling_time/sim_per_cycle);
}

DynamicModelSimulation::~DynamicModelSimulation(){}

PoseVelocityState DynamicModelSimulation::calcStates(const PoseVelocityState &actual_pose, const base::Vector6d &control_input)
{
    return simulator.calcStates(actual_pose,control_input);
}

AccelerationState DynamicModelSimulation::getAcceleration() const
{
    return simulator.getAcceleration();
}

UWVParameters DynamicModelSimulation::getUWVParameters() const
{
    return simulator.gDynamicModel.getUWVParameters();
}

void DynamicModelSimulation::setUWVParameters(const UWVParameters &parameters)
{
    simulator.gDynamicModel.setUWVParameters(parameters);
}

/**********************************************************
 * Simulation of velocity states
 * Use RK4_KIN_SIM simulator
 * Compute all state (pose & velocities)
 **********************************************************/
ModelSimulation::ModelSimulation(double sampling_time, int sim_per_cycle, double initial_time)
    :BaseModelSimulation(sampling_time, sim_per_cycle, initial_time)
{
    simulator.setIntegrationStep(sampling_time/sim_per_cycle);
}

ModelSimulation::~ModelSimulation(){}

PoseVelocityState ModelSimulation::calcStates(const PoseVelocityState &actual_pose, const base::Vector6d &control_input)
{
    return simulator.calcStates(actual_pose,control_input);
}

AccelerationState ModelSimulation::getAcceleration() const
{
    return simulator.getAcceleration();
}

UWVParameters ModelSimulation::getUWVParameters() const
{
    return simulator.gDynamicModel.getUWVParameters();
}

void ModelSimulation::setUWVParameters(const UWVParameters &parameters)
{
    simulator.gDynamicModel.setUWVParameters(parameters);
}
}
