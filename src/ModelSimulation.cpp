#include "ModelSimulation.hpp"


namespace uwv_dynamic_model
{
ModelSimulation::ModelSimulation(double sampling_time, int sim_per_cycle,
                                 double initial_time, DynamicSimulator* sim)
{
    checkConstruction(sampling_time, sim_per_cycle, initial_time);
    gSamplingTime = sampling_time;
    gCurrentTime = initial_time;
    gSimPerCycle = sim_per_cycle;
    gPose = PoseVelocityState();
    simulator = ( sim ) ? sim: new DynamicSimulator(sampling_time / sim_per_cycle);
}

ModelSimulation::~ModelSimulation()
{
}

PoseVelocityState ModelSimulation::sendEffort(const base::Vector6d &control_input)
{
    PoseVelocityState actual_state = getPose();
    actual_state = sendEffort(control_input, actual_state);
    setPose(actual_state);
    return actual_state;
}

PoseVelocityState ModelSimulation::sendEffort(const base::Vector6d &control_input, const PoseVelocityState &actual_pose)
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

PoseVelocityState ModelSimulation::calcStates(const PoseVelocityState &actual_pose, const base::Vector6d &control_input)
{
    return simulator->calcStates(actual_pose,control_input);
}

AccelerationState ModelSimulation::getAcceleration() const
{
    return simulator->getAcceleration();
}

UWVParameters ModelSimulation::getUWVParameters() const
{
    return simulator->getDynamicModel()->getUWVParameters();
}

void ModelSimulation::setUWVParameters(const UWVParameters &parameters)
{
    return simulator->getDynamicModel()->setUWVParameters(parameters);
}

void ModelSimulation::resetStates()
{
    gPose.position = base::Vector3d::Zero();
    gPose.orientation = base::Orientation::Identity();
    gPose.linear_velocity = base::Vector3d::Zero();
    gPose.angular_velocity = base::Vector3d::Zero();
}

void ModelSimulation::setOrientation(const base::Orientation &orientation)
{
    gPose.orientation = orientation;
}

double ModelSimulation::getCurrentTime() const
{
    return gCurrentTime;
}

void ModelSimulation::setCurrentTime(double currentTime)
{
    checkSimulationTime(currentTime);
    gCurrentTime = currentTime;
}

PoseVelocityState ModelSimulation::getPose()
{
    return gPose;
}

void ModelSimulation::setPose(const PoseVelocityState& pose)
{
    checkState(pose);
    gPose = pose;
}

double ModelSimulation::getSamplingTime()
{
    return gSamplingTime;
}

void ModelSimulation::setSamplingTime(double samplingTime)
{
    checkSamplingTime(samplingTime);
    gSamplingTime = samplingTime;
    simulator->setIntegrationStep(samplingTime/getSimPerCycle());
}

int ModelSimulation::getSimPerCycle() const
{
    return gSimPerCycle;
}

void ModelSimulation::checkConstruction(double &samplingTime,
        int &simPerCycle, double &initialTime)
{
    checkSamplingTime(samplingTime);
    checkSimulationTime(initialTime);
    if (simPerCycle <= 0)
        throw std::runtime_error("simPerCycle must be positive");
}

void ModelSimulation::checkSamplingTime(double samplingTime)
{
    if (samplingTime <= 0)
        throw std::runtime_error("samplingTime must be positive");
}

void ModelSimulation::checkSimulationTime(double simulationTime)
{
    if (simulationTime < 0)
        throw std::runtime_error("simulationTime must be positive or equal to zero");
}

void ModelSimulation::checkControlInput(const base::Vector6d &control_input)
{
    if(control_input.hasNaN())
        throw std::runtime_error("control input has a NaN.");
}

void ModelSimulation::checkState(const PoseVelocityState &state)
{
    if(state.hasNaN())
        throw std::runtime_error("state has a NaN.");
}
}
