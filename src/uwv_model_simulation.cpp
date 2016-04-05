#include "uwv_model_simulation.hpp"


namespace underwaterVehicle
{
ModelSimulation::ModelSimulation(double sampling_time, int sim_per_cycle,
                                 double initial_time, bool pose_orientaion_integration)
: RK4_SIM((sampling_time/(double)sim_per_cycle)), DynamicModel()
{
    checkConstruction(sampling_time, sim_per_cycle, initial_time);
    gSamplingTime = sampling_time;
    gCurrentTime = initial_time;
    gSimPerCycle = sim_per_cycle;
    gPoseOrientationIntegration = pose_orientaion_integration;

    gPose = PoseVelocityState();
    gAcceleration = AccelerationState();
    gEfforts = base::Vector6d::Zero();
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
        //Brute force normalization of quaternions
        if(gPoseOrientationIntegration)
            state.orientation.normalize();
    }
    gCurrentTime += gSamplingTime;
    return state;
}

PoseVelocityState ModelSimulation::DERIV(const PoseVelocityState &current_states, const base::Vector6d &control_input)
{
    base::Vector6d velocity;
    velocity.head(3) = current_states.linear_velocity;
    velocity.tail(3) = current_states.angular_velocity;

    // Compute acceleration
    gAcceleration.fromVector6d(calcAcceleration(control_input, velocity, current_states.orientation));

    PoseVelocityState state_derivatives;
    state_derivatives.linear_velocity = gAcceleration.linear_acceleration;
    state_derivatives.angular_velocity = gAcceleration.angular_acceleration;

    // If the position and orientation should be integrated, compute derivatives. Otherwise derivatives are zero and pose/orientation are constants.
    if(gPoseOrientationIntegration)
    {
        state_derivatives.position = KinematicModel::calcPoseDeriv(current_states.linear_velocity, current_states.orientation);
        state_derivatives.orientation = KinematicModel::calcOrientationDeriv(current_states.angular_velocity, current_states.orientation);
    }
    else
        state_derivatives.orientation = base::Orientation(0,0,0,0);
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

PoseVelocityState ModelSimulation::fromRBS(const base::samples::RigidBodyState &state)
{
    PoseVelocityState new_state;
    new_state.position = state.position;
    new_state.orientation = state.orientation;
    // RBS velocity expressed in target frame, PoseVelocityState velocity expressed in body-frame
    new_state.linear_velocity = state.orientation.inverse()*state.velocity;
    new_state.angular_velocity = state.angular_velocity;
    return new_state;
}

base::samples::RigidBodyState ModelSimulation::toRBS(const PoseVelocityState &state)
{
    base::samples::RigidBodyState new_state;
    new_state.position = state.position;
    new_state.orientation = state.orientation;
    // RBS velocity expressed in target frame, PoseVelocityState velocity expressed in body-frame
    new_state.velocity = state.orientation.matrix()*state.linear_velocity;
    new_state.angular_velocity = state.angular_velocity;
    return new_state;
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
