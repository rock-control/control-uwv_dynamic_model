#include "UwvModelSimulation.hpp"


namespace uwv_dynamic_model
{
ModelSimulation::ModelSimulation(double sampling_time, int sim_per_cycle,
                                 double initial_time, const UWVParameters &uwv_parameters)
: RK4_SIM((sampling_time/(double)sim_per_cycle))
{
    checkConstruction(sampling_time, sim_per_cycle, initial_time);
    gSamplingTime = sampling_time;
    gCurrentTime = initial_time;
    gSimPerCycle = sim_per_cycle;

    gPose = PoseVelocityState();
    gAcceleration = AccelerationState();
    gEfforts = base::Vector6d::Zero();

    gDynamicModel.setUWVParameters(uwv_parameters);
}

ModelSimulation::~ModelSimulation()
{
}

PoseVelocityState ModelSimulation::sendEffort(const base::LinearAngular6DCommand &control_input)
{
    PoseVelocityState actual_state = getPose();
    actual_state = sendEffort(control_input, actual_state);
    setPose(actual_state);
    return actual_state;
}

PoseVelocityState ModelSimulation::sendEffort(const base::LinearAngular6DCommand &control_input, const PoseVelocityState actual_pose)
{
    // Checks if the control input is valid
    checkControlInput(control_input);
    checkState(actual_pose);
    // Puts efforts in a vector
    gEfforts.head(3) = control_input.linear;
    gEfforts.tail(3) = control_input.angular;

    PoseVelocityState state = actual_pose;

    // Performs iterations to calculate the new system's states
    for (int i=0; i < gSimPerCycle; i++)
    {
        state = calcStates(state, gEfforts);
        //Brute force normalization of quaternions due the RK4 integration.
        state.orientation.normalize();
    }
    gCurrentTime += gSamplingTime;
    return state;
}

PoseVelocityState ModelSimulation::velocityDeriv(const PoseVelocityState &current_states, const base::Vector6d &control_input)
{
    base::Vector6d velocity;
    velocity.head(3) = current_states.linear_velocity;
    velocity.tail(3) = current_states.angular_velocity;

    // Compute acceleration (velocity derivatives)
    gAcceleration.fromVector6d(gDynamicModel.calcAcceleration(control_input, velocity, current_states.orientation));

    PoseVelocityState state_derivatives;
    state_derivatives.linear_velocity = gAcceleration.linear_acceleration;
    state_derivatives.angular_velocity = gAcceleration.angular_acceleration;

    return state_derivatives;
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

base::LinearAngular6DCommand ModelSimulation::getEfforts()
{
    base::LinearAngular6DCommand efforts;
    efforts.linear = gEfforts.head(3);
    efforts.angular = gEfforts.tail(3);
    return efforts;
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

void ModelSimulation::checkControlInput(const base::LinearAngular6DCommand &control_input)
{
    if(control_input.linear.hasNaN() || control_input.angular.hasNaN())
        throw std::runtime_error("control input has a NaN.");
}

void ModelSimulation::checkState(const PoseVelocityState &state)
{
    if(state.hasNaN())
        throw std::runtime_error("state has a NaN.");
}

}
