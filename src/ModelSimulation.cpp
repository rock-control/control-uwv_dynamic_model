#include "ModelSimulation.hpp"


namespace uwv_dynamic_model
{
ModelSimulation::ModelSimulation(ModelSimulator sim, double sampling_time, int sim_per_cycle,
                                 double initial_time)
{
    checkConstruction(sampling_time, sim_per_cycle, initial_time);
    if(sim == DYNAMIC)
        simulator = new DynamicSimulator();
    else if(sim == DYNAMIC_KINEMATIC)
        simulator = new DynamicKinematicSimulator();
    else
        throw std::runtime_error("Unknown ModelSimulator.");
    current_time = initial_time;
    simulations_per_cycle = sim_per_cycle;
    pose = PoseVelocityState();
    setSamplingTime(sampling_time);
}

ModelSimulation::~ModelSimulation()
{
    delete simulator;
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
    for (int i=0; i < simulations_per_cycle; i++)
        state = calcStates(state, control_input);

    current_time += sampling_time;
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
    pose.position = base::Vector3d::Zero();
    pose.orientation = base::Orientation::Identity();
    pose.linear_velocity = base::Vector3d::Zero();
    pose.angular_velocity = base::Vector3d::Zero();
}

void ModelSimulation::setOrientation(const base::Orientation &orientation)
{
    pose.orientation = orientation;
}

double ModelSimulation::getCurrentTime() const
{
    return current_time;
}

void ModelSimulation::setCurrentTime(double time)
{
    checkSimulationTime(time);
    current_time = time;
}

PoseVelocityState ModelSimulation::getPose()
{
    return pose;
}

void ModelSimulation::setPose(const PoseVelocityState& current_pose)
{
    checkState(current_pose);
    pose = current_pose;
}

double ModelSimulation::getSamplingTime()
{
    return sampling_time;
}

void ModelSimulation::setSamplingTime(double step_time)
{
    // sampling time set equally in simulation and simulator
    checkSamplingTime(step_time);
    sampling_time = step_time;
    simulator->setIntegrationStep(step_time/getSimPerCycle());
}

int ModelSimulation::getSimPerCycle() const
{
    return simulations_per_cycle;
}

void ModelSimulation::checkConstruction(double &sampling_time,
        int &sim_per_cycle, double &initial_time)
{
    checkSamplingTime(sampling_time);
    checkSimulationTime(initial_time);
    if (sim_per_cycle <= 0)
        throw std::runtime_error("simulations_per_cycle must be positive");
}

void ModelSimulation::checkSamplingTime(double step_time)
{
    if (step_time <= 0)
        throw std::runtime_error("sampling_time must be positive");
}

void ModelSimulation::checkSimulationTime(double simulation_time)
{
    if (simulation_time < 0)
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
