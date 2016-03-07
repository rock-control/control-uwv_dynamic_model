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
: RK4_SIM((samplingTime/(double)simPerCycle))
{
    checkConstruction(samplingTime, simPerCycle, initialTime);
    gSamplingTime = samplingTime;
    gSimPerCycle = simPerCycle;
    gCurrentTime = initialTime;

    // uwv model parameters
    UWVParameters uwvParameters;
    setUWVParameters(uwvParameters);

    // States variables
    Eigen::VectorXd statesInit = Eigen::VectorXd::Zero(gSystemOrder);
    updateStates(statesInit);

    gLinearAcceleration = Eigen::VectorXd::Zero(3);
    gAngularAcceleration = Eigen::VectorXd::Zero(3);
    gEfforts = Eigen::VectorXd::Zero(6);

}

DynamicModel::~DynamicModel()
{
}

void DynamicModel::sendEffortCommands(const base::LinearAngular6DCommand &controlInput)
{
    // Checks if the control input is valid
    checkControlInput(controlInput);

    // Puts the efforts in a vector
    for (int i = 0; i < 3; i++)
    {
        gEfforts[i] = controlInput.linear[i];
        gEfforts[i+3] = controlInput.angular[i];
    }

    // Gets a vector with the current system states (pose and velocities)
    Eigen::VectorXd systemStates = Eigen::VectorXd::Zero(gSystemOrder);
    systemStates = getStates();

    // Performs iterations to calculate the new system's states
    for (int ii=0; ii < gSimPerCycle; ii++)
        calcStates(systemStates, gCurrentTime, gEfforts);

    // Updates the new system's states
    updateStates(systemStates);

}

void DynamicModel::calcAcceleration(Eigen::VectorXd &velocityAndAcceleration,
        const base::Vector6d &velocity,
        const base::Vector6d &controlInput)
{
    /**
     * velocityAndAcceleration:
     *
     * [0] = u		[6]  = u_dot	(SURGE)
     * [1] = v		[7]  = v_dot	(SWAY)
     * [2] = w		[8]  = w_dot	(HEAVE)
     * [3] = p		[9]  = p_dot	(ROLL)
     * [4] = q		[10] = q_dot	(PITCH)
     * [5] = r		[11] = r_dot	(YAW)
     *
     */

    // Forces and Moments vectors
    base::Matrix6d invInertiaMatrix  = Eigen::MatrixXd::Zero(6,6);
    base::Vector6d linDamping        = Eigen::VectorXd::Zero(6);
    base::Vector6d quadDamping       = Eigen::VectorXd::Zero(6);
    base::Vector6d gravityBuoyancy   = Eigen::VectorXd::Zero(6);
    base::Vector6d worldVelocity     = Eigen::VectorXd::Zero(6);
    base::Vector6d acceleration      = Eigen::VectorXd::Zero(6);

    // Calculating the efforts for each one of the hydrodynamics effects
    calcInvInertiaMatrix(invInertiaMatrix, velocity);
    calcLinDamping(linDamping, velocity);
    calcQuadDamping(quadDamping, velocity);
    calcGravityBuoyancy(gravityBuoyancy, gEulerOrientation);

    // Calculating the acceleration based on all the hydrodynamics effects
    switch(gModelType)
    {
    case SIMPLE:
        acceleration  = invInertiaMatrix * ( gEfforts - linDamping - quadDamping - gravityBuoyancy);
        break;
    case COMPLEX:
        base::Vector6d coriolisEffect    = Eigen::VectorXd::Zero(6);
        base::Vector6d RBCoriolis        = Eigen::VectorXd::Zero(6);
        base::Vector6d AddedMassCoriolis = Eigen::VectorXd::Zero(6);
        base::Vector6d LiftEffect        = Eigen::VectorXd::Zero(6);
        base::Vector6d ModelCorrection   = Eigen::VectorXd::Zero(6);
        calcCoriolisEffect(coriolisEffect, velocity);
        calcRBCoriolis(RBCoriolis, velocity);
        calcAddedMassCoriolis(AddedMassCoriolis, velocity);
        calcLiftEffect(LiftEffect, velocity);
        calcModelCorrection(ModelCorrection, velocity);

        acceleration  = invInertiaMatrix * ( gEfforts - coriolisEffect - RBCoriolis - AddedMassCoriolis - LiftEffect  -
                linDamping - quadDamping - gravityBuoyancy - ModelCorrection);
        break;
    }

    // Converting the body velocity to world velocity. This is necessary because
    // when the integration takes place in order to find the position, the velocity
    // should be expressed in the world frame, just like the position is.
    convBodyToWorld(worldVelocity, velocity, gEulerOrientation);

    // Updating the RK4 vector with the velocity and acceleration values
    for (int i = 0; i < 6; i++)
    {
        velocityAndAcceleration[i] = worldVelocity[i];
        velocityAndAcceleration[i+6] = acceleration[i];
    }

    // Updating global acceleration variables
    for(int i = 0; i < 3; i++)
    {
        gLinearAcceleration[i]  = acceleration[i];
        gAngularAcceleration[i] = acceleration[i+3];
    }
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
    gPosition = position;
}

void DynamicModel::setOrientation(const base::Orientation &orientation)
{
    gOrientation = orientation;
}

void DynamicModel::setLinearVelocity(const base::Vector3d &linearVelocity)
{
    gLinearVelocity = linearVelocity;
}

void DynamicModel::setAngularVelocity(const base::Vector3d &angularVelocity)
{
    gAngularVelocity = angularVelocity;
}

void DynamicModel::setSamplingTime(const double samplingTime)
{
    gSamplingTime = samplingTime;
    setIntegrationStep(gSamplingTime/(double)gSimPerCycle);
}

UWVParameters DynamicModel::getUWVParameters(void) const
{
    return gUwvParameters;
}

base::Position DynamicModel::getPosition(void) const
{
    return gPosition;
}

base::Vector3d DynamicModel::getEulerOrientation(void) const
{
    base::Vector3d euler_angles;
    euler_angles[0] = base::getRoll(gOrientation);
    euler_angles[1] = base::getPitch(gOrientation);
    euler_angles[2] = base::getYaw(gOrientation);
    return euler_angles;
}

base::Orientation DynamicModel::getQuatOrienration(void) const
{
     return gOrientation;
}

base::Vector3d DynamicModel::getLinearVelocity( bool worldFrame) const
{
    if(worldFrame)
    {
        // Body to world frame convertion
        base::Vector6d bodyVelocity;
        base::Vector6d worldVelocity;

        bodyVelocity.head(3) = gLinearVelocity;
        bodyVelocity.tail(3) = gAngularVelocity;

        worldVelocity = convBodyToWorld(bodyVelocity, gOrientation);

        return worldVelocity.head(3);
    }
    else
    {
        return gLinearVelocity;
    }
}

base::Vector3d DynamicModel::getAngularVelocity(bool worldFrame) const
{
    if(worldFrame)
    {
        // Body to world frame convertion
        base::Vector6d bodyVelocity;
        base::Vector6d worldVelocity;

        bodyVelocity.head(3) = gLinearVelocity;
        bodyVelocity.tail(3) = gAngularVelocity;

        worldVelocity = convBodyToWorld( bodyVelocity, gOrientation);

        return worldVelocity.tail(3);
    }
    else
    {
        return gAngularVelocity;
    }
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
    systemStates.segment(3,3) = getEulerOrientation();
    systemStates.segment(6,3) = gLinearVelocity;
    systemStates.segment(9,3) = gAngularVelocity;
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

base::Quaterniond DynamicModel::eulerToQuaternion( base::Vector3d eulerAngles)
{
    base::Quaterniond quaternion;
    quaternion.w() = ( cos(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) +
            ( sin(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
    quaternion.x() = ( sin(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) -
            ( cos(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
    quaternion.y() = ( cos(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) +
            ( sin(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
    quaternion.z() = ( cos(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*sin(eulerAngles(2)/2) ) -
            ( sin(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*cos(eulerAngles(2)/2) );
    return quaternion;
}

base::Vector6d DynamicModel::convBodyToWorld( const base::Vector6d &bodyCoordinates,
        const base::Orientation &orientation)
 {
    base::Matrix6d transfMatrix = Eigen::MatrixXd::Zero(6,6);

    transfMatrix = calcTransfMatrix(orientation);

    return transfMatrix * bodyCoordinates;
 }

base::Vector6d DynamicModel::convWorldToBody( const base::Vector6d &worldCoordinates,
        const base::Orientation &orientation)
 {
    base::Matrix6d invTransfMatrix = Eigen::MatrixXd::Zero(6,6);

    invTransfMatrix = calcTransfMatrix(orientation).inverse();

    return invTransfMatrix * worldCoordinates;
}

base::Matrix6d DynamicModel::calcTransfMatrix( const base::Orientation &orientation)
{
    base::Matrix6d transfMatrix = Eigen::MatrixXd::Zero(6,6);;

    // TODO Need an elegant solution, using rotation matrix
    double phi   = base::getRoll(orientation);
    double theta = base::getPitch(orientation);
    double psi   = base::getYaw(orientation);
    base::Matrix3d J1;
    base::Matrix3d J2;

    J1 << cos(psi)*cos(theta),   -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi),   sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta),
            sin(psi)*cos(theta),    cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi),  -cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi),
            -sin(theta),                        cos(theta)*sin(phi),               cos(theta)*cos(phi)                 ;


    J2 <<  1,       sin(phi)*tan(theta),       cos(phi)*tan(theta),
            0,            cos(phi)      ,              -sin(phi)     ,
            0,       sin(phi)/cos(theta),       cos(phi)/cos(theta);

    transfMatrix.block<3,3>(0,0) = J1;
    transfMatrix.block<3,3>(3,3) = J2;
    return transfMatrix;
 }

base::Matrix6d DynamicModel::calcInvInertiaMatrix(const base::Matrix6d &inertiaMatrix) const
{
    /**
     * M * M^(-1) = I
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
    return coriloisEffect << prod.head(3).cross(velocity.tail(3)),
                prod.head(3).cross(velocity.head(3)) + prod.tail(3).cross(velocity.tail(3));
}

base::Vector6d DynamicModel::caclDampingEffect( const std::vector<base::Matrix6d> &dampMatrices, const base::Vector6d &velocity, const ModelType &modelType) const
{
    if(modelType == SIMPLE)
        return caclSimpleDamping(dampMatrices, velocity);
    else if(modelType == COMPLEX)
        return caclGeneralQuadDamping(dampMatrices, velocity);
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
        dampMatrix += quadDampMatrices[i] * velocity.abs()[i];

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
    return quadDampMatrix * velocity.abs().asDiagonal() * velocity;
}

base::Vector6d DynamicModel::calcGravityBuoyancy( const Eigen::Quaterniond& orientation,
        const double& weight, const double& bouyancy,
        const base::Vector3d& cg, const base::Vector3d& cb) const
{
    /** Based on McFarland[2013] and Fossen[1994]
     * gravityBuoyancy = [R^T * e3 * (W-B);
     *                    (cg*W - cb*B) X R^T * e3]
     *  R: Rotation matrix from body-frame to world-frame
     *  e3 = [0; 0; 1]
     */
    base::Vector6d gravityEffect;
    return gravityEffect << orientation.inverse() * Eigen::Vector3d(0, 0, (weight-bouyancy)),
            (cg*weight - cb*bouyancy).cross(orientation.inverse() * Eigen::Vector3d(0, 0, 1));
}


void DynamicModel::updateStates(Eigen::VectorXd &newSystemStates)
{
    if(newSystemStates.size() != gSystemOrder)
        throw std::runtime_error("uwv_dynamic_model updateStates: newSystemState doesn't have size 12");
    gPosition           = newSystemStates.segment(0,3);
    gOrientation        = eulerToQuaternion(newSystemStates.segment(3,3));
    gLinearVelocity     = newSystemStates.segment(6,3);
    gAngularVelocity    = newSystemStates.segment(9,3);
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
    state.orientation = getQuatOrienration();
    state.velocity = getLinearVelocity(false);
    state.angular_velocity = getAngularVelocity(false);
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
}


void DynamicModel::checkControlInput(const base::LinearAngular6DCommand &controlInput) const
{
    for (size_t i=0; i<3; i++)
    {
        if(std::isnan(controlInput.linear[i]) || std::isnan(controlInput.angular[i]))
            throw std::runtime_error("control input is nan");
    }
}

};
