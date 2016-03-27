/***************************************************************************/
/*  Dynamic model for an underwater vehicle	                           */
/*                                                                         */
/* FILE --- uwv_dynamic_model.cpp	                                   */
/*                                                                         */
/* PURPOSE --- Source file for a Dynamic model of an 	                   */
/*             underwater vehicle. Based on T.I.Fossen & Giovanni Indiveri */
/*                                                                         */
/*  Sankaranarayanan Natarajan                                             */
/*  sankar.natarajan@dfki.de                                               */
/*  DFKI - BREMEN 2011                                                     */
/*                                                                         */
/*  This file was edited to include the full Fossen Model                  */
/*                                                                         */
/*  Bilal Wehbe                                                            */
/*  bilal.wehbe@dfki.de                                                    */
/*  DFKI - BREMEN 2015                                                     */
/***************************************************************************/


#include "uwv_dynamic_model.hpp"
#include <base/Logging.hpp>

namespace underwaterVehicle
{
DynamicModel::DynamicModel(double samplingTime,
        int simPerCycle,  double initialTime)
: RK4_SIM((samplingTime/(double)simPerCycle), gSystemOrder)
{
    checkConstruction(samplingTime, simPerCycle, initialTime);
    gSamplingTime = samplingTime;
    gSimPerCycle = simPerCycle;
    gCurrentTime = initialTime;

    // uwv model parameters
    UWVParameters uwvParameters;
    setUWVParameters(uwvParameters);

    // States variables
    resetStates();

    gLinearAcceleration = Eigen::VectorXd::Zero(3);
    gAngularAcceleration = Eigen::VectorXd::Zero(3);
    gEfforts = Eigen::VectorXd::Zero(6);

}

DynamicModel::~DynamicModel()
{
}

base::samples::RigidBodyState DynamicModel::sendEffortCommands(const base::LinearAngular6DCommand &controlInput)
{
    // Checks if the control input is valid
    checkControlInput(controlInput);

    // Puts the efforts in a vector
    gEfforts.head(3) = controlInput.linear;
    gEfforts.tail(3) = controlInput.angular;

    // Gets a vector with the current system states (pose and velocities)
    Eigen::VectorXd systemStates = getStates();

    // Performs iterations to calculate the new system's states
    for (int ii=0; ii < gSimPerCycle; ii++)
    {
        systemStates = calcStates(systemStates, gEfforts);
        //Brute force normalization of quaternions
        systemStates.segment(3,4) /= systemStates.segment(3,4).norm();
    }

    // Updates the new system's states
    updateStates(systemStates);
    return getRigidBodyState();
}

Eigen::VectorXd DynamicModel::DERIV( const Eigen::VectorXd &current_states,
        const base::Vector6d &controlInput)
{
    /**
     * velocityAndAcceleration:
     *
     * [0] = x          [7]  = u    (SURGE)
     * [1] = y          [8]  = v    (SWAY)
     * [2] = z          [9]  = w    (HEAVE)
     * [3] = e1         [10]  = p   (ROLL)
     * [4] = e2         [11] = q    (PITCH)
     * [5] = e3         [12] = r    (YAW)
     * [6] = n
     *
     */

    checkStates(current_states);

    //states
    Eigen::VectorXd velocityAndAcceleration = Eigen::VectorXd::Zero(gSystemOrder);
    base::Vector6d velocity = current_states.tail(6);
    base::Vector4d quat = current_states.segment(3,4);
    base::Orientation orientation(quat[3],quat[0],quat[1],quat[2]);

    // Updating the RK4 vector with the velocity and acceleration values
    // Pose derivatives
    velocityAndAcceleration.head<3>() = orientation.matrix()*velocity.head<3>();
    // Quaternion derivatives
    velocityAndAcceleration.segment<4>(3) = calQuatDeriv(velocity.tail<3>(), orientation);
    // Calculating the acceleration based on all the hydrodynamics effects
    velocityAndAcceleration.tail<6>() = calcAcceleration(controlInput, velocity, orientation);
    // Updating global acceleration variables
    gLinearAcceleration  = velocityAndAcceleration.segment<3>(7);
    gAngularAcceleration = velocityAndAcceleration.segment<3>(10);



    return velocityAndAcceleration;
}

base::Vector6d DynamicModel::calcAcceleration(const base::Vector6d &controlInput, const base::Vector6d &velocity, const base::Orientation &orientation)
{
    // Calculating the acceleration based on all the hydrodynamics effects
    base::Vector6d acceleration = Eigen::VectorXd::Zero(6);
    switch(gUwvParameters.modelType)
    {
    case SIMPLE:
        acceleration  = gInvInertiaMatrix * ( controlInput + caclDampingEffect(gUwvParameters, velocity) + calcGravityBuoyancy(orientation, gUwvParameters));
        break;
    case COMPLEX:
        acceleration  = gInvInertiaMatrix * ( controlInput + calcCoriolisEffect(gUwvParameters.inertiaMatrix, velocity) + caclDampingEffect(gUwvParameters, velocity) + calcGravityBuoyancy(orientation, gUwvParameters));
        break;
    }
    return acceleration;
}

void DynamicModel::setUWVParameters(const UWVParameters &uwvParameters)
{
    // Checks if there is any parameter inconsistency
    checkParameters(uwvParameters);
    gUwvParameters = uwvParameters;
    gInvInertiaMatrix = calcInvInertiaMatrix(uwvParameters.inertiaMatrix);
}

void DynamicModel::resetStates()
{
    base::Vector3d resetVector3d = Eigen::VectorXd::Zero(3);
    gPosition           = resetVector3d;
    gOrientation        = base::Orientation::Identity();
    gLinearVelocity     = resetVector3d;
    gAngularVelocity    = resetVector3d;
}

void DynamicModel::setPosition(const base::Vector3d &position)
{
    if(!position.hasNaN())
        gPosition = position;
}

void DynamicModel::setOrientation(const base::Orientation &orientation)
{
    gOrientation = orientation.normalized();
}

void DynamicModel::setLinearVelocity(const base::Vector3d &linearVelocity)
{
    if(!linearVelocity.hasNaN())
        gLinearVelocity = linearVelocity;
}

void DynamicModel::setAngularVelocity(const base::Vector3d &angularVelocity)
{
    if(!angularVelocity.hasNaN())
    gAngularVelocity = angularVelocity;
}

void DynamicModel::setSamplingTime(const double samplingTime)
{
    if(samplingTime > 0)
    {
        gSamplingTime = samplingTime;
        setIntegrationStep(gSamplingTime/(double)gSimPerCycle);
    }
}

UWVParameters DynamicModel::getUWVParameters(void) const
{
    return gUwvParameters;
}

base::Position DynamicModel::getPosition(void) const
{
    return gPosition;
}

base::Orientation DynamicModel::getOrienration(void) const
{
     return gOrientation;
}

base::Vector3d DynamicModel::getLinearVelocity( bool worldFrame) const
{
    if(worldFrame)
        return gOrientation.matrix()*gLinearVelocity;
    return gLinearVelocity;
}

base::Vector3d DynamicModel::getAngularVelocity(void) const
{
    return gAngularVelocity;
}

base::Vector3d DynamicModel::getLinearAcceleration(void) const
{
    return gLinearAcceleration;
}

base::Vector3d DynamicModel::getAngularAcceleration(void) const
{
    return gAngularAcceleration;
}

Eigen::VectorXd DynamicModel::getStates(void) const
{
    Eigen::VectorXd systemStates = Eigen::VectorXd::Zero(gSystemOrder);
    systemStates.segment(0,3) = gPosition;
    systemStates.segment(3,4) = gOrientation.coeffs();
    systemStates.segment(7,3) = gLinearVelocity;
    systemStates.segment(10,3) = gAngularVelocity;
    return systemStates;
}

base::Vector6d DynamicModel::getEfforts(void) const
{
    return gEfforts;
}

double DynamicModel::getSimulationTime(void) const
{
    return gCurrentTime;
}

double DynamicModel::getSamplingTime(void) const
{
    return gSamplingTime;
}

int DynamicModel::getSimPerCycle(void) const
{
    return gSimPerCycle;
}

base::Vector4d DynamicModel::calQuatDeriv(const base::Vector3d &ang_vel, const base::Orientation &orientation)
{
    /** Based on Fossen[2011], Andrle[2013] & Wertz[1978]
     *
     *  qdot = 1/2*T(q)*w = 1/2*Omega(w)*q
     *  Omega(w) = [-J(w), w;
     *              -w^t , 0]
     *   J(w): skew-symmetric matrix
     *
     *   Quaternion representation:
     *   q = (q_r, q_i); q_r: real part, q_i: imaginary part
     *   Quaternion multiplication:
     *   q = q1*q2 = M(q2)*q1
     *   M(q) = [q_r,   q_i3, -q_i2, q_i1;
     *          -q_i3,  q_r,   q_i1, q_i2;
     *           q_i2, -q_i1,  q_r,  q_i3;
     *          -q_i1, -q_i2, -q_i3, q_r ]
     *   M((0,q_i)) = Omega(q_i)
     *
     *   => qdot = 1/2*orientation*w_as_quaternion
     *
     * Fossen, Thor I. Handbook of marine craft hydrodynamics and motion control. John Wiley & Sons, 2011.
     * Andrle, Michael S., and John L. Crassidis. "Geometric integration of quaternions." Journal of Guidance, Control, and Dynamics 36.6 (2013): 1762-1767.
     * (Astrophysics and Space Science Library 73) James R. Wertz (auth.), James R. Wertz (eds.)-Spacecraft Attitude Determination and Control-Springer Netherlands (1978)
     */
    base::Orientation quat_ang_vel(0, ang_vel[0], ang_vel[1], ang_vel[2]);
    return (orientation*quat_ang_vel).coeffs()/2;
}

base::Matrix6d DynamicModel::calcInvInertiaMatrix(const base::Matrix6d &inertiaMatrix) const
{
    /**
     * M * M^(-1) = I
     * A*x = b
     */
    Eigen::JacobiSVD<base::Matrix6d> svd(inertiaMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.solve(base::Matrix6d::Identity());
}

base::Vector6d DynamicModel::calcCoriolisEffect(const base::Matrix6d &inertiaMatrix, const base::Vector6d &velocity) const
{
    /**
     * Based on McFarland[2013] and Fossen[1994]
     * coriolisEffect = H(M*v)*v
     * M = inertiaMatrix; v = velocity
     * Operator H: R^6 -> R^(6x6).
     *      H(v) = [0(3x3), J(v.head(3));
     *              J(v.head(3)),  J(v.tail(3))]
     * Operator J: R^3 -> R^(3x3) (the so(3) operator, skew-symmetric matrix)
     *      J([v1; v2; v3]) = [ 0 ,-v3, v2;
     *                          v3, 0 ,-v1;
     *                         -v2, v1, 0]
     * Cross product:
     *      J(v.head(3)) * v.tail(3) = v.head(3) X v.tail(3)
     */

    base::Vector6d coriloisEffect;
    base::Vector6d prod = inertiaMatrix * velocity;
    coriloisEffect << prod.head<3>().cross(velocity.tail<3>()),
                prod.head<3>().cross(velocity.head<3>()) + prod.tail<3>().cross(velocity.tail<3>());
    return coriloisEffect;
}

base::Vector6d DynamicModel::caclDampingEffect( const UWVParameters &uwv_parameters, const base::Vector6d &velocity) const
{
    // Damping term as a dissipative term. Considering the damping term as positive semidefinite, returns a negative term.
    if(uwv_parameters.modelType == SIMPLE)
        return -caclSimpleDamping(uwv_parameters.dampMatrices, velocity);
    else if(uwv_parameters.modelType == COMPLEX)
        return -caclGeneralQuadDamping(uwv_parameters.dampMatrices, velocity);
    else
        throw std::runtime_error("unknown modelType.");
}

base::Vector6d DynamicModel::caclGeneralQuadDamping( const std::vector<base::Matrix6d> &quadDampMatrices, const base::Vector6d &velocity) const
{
    /**
     *  Based on McFarland[2013]
     *  damping effect = sum(Di * |vi|) * v, i=1...6
     *  D = quadDampMatrix; v = velocity
     */
    if(quadDampMatrices.size() != 6)
        throw std::runtime_error("quadDampMatrices does not have 6 elements.");

    base::Matrix6d dampMatrix = base::Matrix6d::Zero();
    for(size_t i=0; i < quadDampMatrices.size(); i++)
        dampMatrix += quadDampMatrices[i] * velocity.cwiseAbs()[i];

    return dampMatrix * velocity;
}

base::Vector6d DynamicModel::caclSimpleDamping( const std::vector<base::Matrix6d> &dampMatrices, const base::Vector6d &velocity) const
{
    /**
     *  Based on Fossen[1994]
     *  damping effect = quadDampingMatrix*|vi|*v + linDampingMatrix*v
     */
    if(dampMatrices.size() != 2)
        throw std::runtime_error("dampMatrices does not have 2 elements.");
    return calcLinDamping(dampMatrices[0], velocity) + calcQuadDamping(dampMatrices[1], velocity);
}

base::Vector6d DynamicModel::calcLinDamping(const base::Matrix6d &linDampMatrix, const base::Vector6d &velocity) const
{
    return linDampMatrix * velocity;
}

base::Vector6d DynamicModel::calcQuadDamping( const base::Matrix6d &quadDampMatrix, const base::Vector6d &velocity) const
{
    return quadDampMatrix * velocity.cwiseAbs().asDiagonal() * velocity;
}

base::Vector6d DynamicModel::calcGravityBuoyancy(const base::Orientation& orientation, const UWVParameters &uwv_parameters) const
{
    return calcGravityBuoyancy(orientation, uwv_parameters.weight, uwv_parameters.buoyancy, uwv_parameters.distance_body2centerofgravity, uwv_parameters.distance_body2centerofbuoyancy);
}

base::Vector6d DynamicModel::calcGravityBuoyancy( const base::Orientation& orientation,
        const double& weight, const double& bouyancy,
        const base::Vector3d& cg, const base::Vector3d& cb) const
{
    /** Based on McFarland[2013] and Fossen[1994]
     * gravityBuoyancy = [R^T * e3 * (W-B);
     *                    (cg*W - cb*B) X R^T * e3]
     *  R: Rotation matrix from body-frame to world-frame
     *  e3 = [0; 0; 1]
     *
     *  In Rock framework, positive z is pointing up, in marine/undewater literature positive z is pointing down.
     */
    base::Vector6d gravityEffect;
    gravityEffect << orientation.inverse() * Eigen::Vector3d(0, 0, (weight-bouyancy)),
            (cg*weight - cb*bouyancy).cross(orientation.inverse() * Eigen::Vector3d(0, 0, 1));
    return -gravityEffect;
}


void DynamicModel::updateStates(const Eigen::VectorXd &newSystemStates)
{
    checkStates(newSystemStates);
    gPosition           = newSystemStates.segment(0,3);
    base::Vector4d quat = newSystemStates.segment(3,4);
    base::Orientation ori(quat[3],quat[0],quat[1],quat[2]);
    gOrientation = ori;
    gLinearVelocity     = newSystemStates.segment(7,3);
    gAngularVelocity    = newSystemStates.segment(10,3);
}


void DynamicModel::setRigidBodyState(base::samples::RigidBodyState const& newSystemStates)
{
    if( !newSystemStates.hasValidPosition() || !newSystemStates.hasValidOrientation()
       || !newSystemStates.hasValidVelocity() || !newSystemStates.hasValidAngularVelocity())
        throw std::runtime_error("uwv_dynamic_model updateStates: newSystemState has invalid data.");

    setPosition(newSystemStates.position);
    setOrientation(newSystemStates.orientation);
    setLinearVelocity(newSystemStates.velocity);
    setAngularVelocity(newSystemStates.angular_velocity);
}

base::samples::RigidBodyState DynamicModel::getRigidBodyState(void) const
{
    base::samples::RigidBodyState state;

    state.position = getPosition();
    state.orientation = getOrienration();
    state.velocity = getLinearVelocity(true);
    state.angular_velocity = getAngularVelocity();
    return state;
}

void DynamicModel::checkConstruction(double &samplingTime,
        int &simPerCycle, double &initialTime)
{
    if (samplingTime <= 0)
        throw std::runtime_error("samplingTime must be positive");
    if (simPerCycle <= 0)
        throw std::runtime_error("simPerCycle must be positive");
    if (initialTime < 0)
        throw std::runtime_error("initialTime must be positive or equal to zero");
}

void DynamicModel::checkParameters(const UWVParameters &uwvParameters)
{
    if(uwvParameters.modelType == SIMPLE && uwvParameters.dampMatrices.size() != 2)
        throw std::runtime_error("in SIMPLE model, dampMatrices should have two elements, the linDampingMatrix and quadDampingMatrix");

    if(uwvParameters.modelType == COMPLEX && uwvParameters.dampMatrices.size() != 6)
        throw std::runtime_error("in COMPLEX model, dampMatrices should have six elements, one quadDampingMatrix / DOF");

    if(uwvParameters.weight <= 0)
        throw std::runtime_error("weight must be a positive value");
    if(uwvParameters.buoyancy <= 0)
        throw std::runtime_error("buoyancy must be a positive value");
}


void DynamicModel::checkControlInput(const base::LinearAngular6DCommand &controlInput) const
{
    if(controlInput.angular.hasNaN() || controlInput.linear.hasNaN() )
        throw std::runtime_error("control input is unset");
}

void DynamicModel::checkStates(const base::VectorXd &states) const
{
    /**
    *
    * [0] = x          [7]  = u    (SURGE)
    * [1] = y          [8]  = v    (SWAY)
    * [2] = z          [9]  = w    (HEAVE)
    * [3] = e1         [10]  = p   (ROLL)
    * [4] = e2         [11] = q    (PITCH)
    * [5] = e3         [12] = r    (YAW)
    * [6] = n
    */
    if(states.size() != gSystemOrder)
        throw std::runtime_error("uwv_dynamic_model checkStates: SystemState doesn't have size 13");
    if(states.hasNaN())
        throw std::runtime_error("uwv_dynamic_model checkStates: SystemState has a NaN");
}
};
