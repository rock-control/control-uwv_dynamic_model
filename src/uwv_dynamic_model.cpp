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
: RK4_SIM(controlOrder, (samplingTime/(double)simPerCycle))
{
    underwaterVehicle::DynamicModel::iniatilizeClass(samplingTime, simPerCycle, initialTime);
}

void DynamicModel::iniatilizeClass(double samplingTime,
        int simPerCycle,  double initialTime)
{
    // Error flags. The errorModelInit will be unset when the model is initialized
    errorModelInit      = true;
    errorConstruction   = false;
    errorControlInput   = false;
    errorSetParameters  = false;
    errorStatus         = false;

    // Checks the arguments provided to the constructor and then initialize the
    // members of the class
    checkConstruction(samplingTime, simPerCycle, initialTime);

    if(!errorConstruction)
    {
        gSystemOrder = 12;
        gSamplingTime = samplingTime;
        gSimPerCycle = simPerCycle;
        gCurrentTime = initialTime;

        // States variables
        Eigen::VectorXd statesInit = Eigen::VectorXd::Zero(12);
        updateStates(statesInit);

        gLinearAcceleration = Eigen::VectorXd::Zero(3);
        gAngularAcceleration = Eigen::VectorXd::Zero(3);

        gEfforts = Eigen::VectorXd::Zero(6);

        // Model parameters
        setInertiaMatrix(Eigen::MatrixXd::Zero(6,6));
        setCoriolisMatrix(Eigen::MatrixXd::Zero(6,6));
        setAddedMassMatrix(Eigen::MatrixXd::Zero(6,6));
        setLinDampingMatrix(Eigen::MatrixXd::Zero(6,6));
        setQuadDampingMatrix(Eigen::MatrixXd::Zero(6,6));
        setLiftCoefficients(Eigen::VectorXd::Zero(4));
        setAddedMassMatrix(Eigen::MatrixXd::Zero(6,6));
        gThrustConfigMatrix = Eigen::MatrixXd::Zero(6,1);

        // Restoring forces' variables
        gWeight = 0;
        gBuoyancy = 0;
        gCenterOfGravity = Eigen::VectorXd::Zero(3);
        gCenterOfBuoyancy = Eigen::VectorXd::Zero(3);

        gModelType = underwaterVehicle::SIMPLE;
    }
}

bool DynamicModel::initParameters(const underwaterVehicle::Parameters &uwvParameters)
{
    // Checks if the model wasn't initialized yet
    if(errorModelInit)
    {
        // Checks if there was any error in the library
        if(!errorConstruction)
        {
            // Unsets the errorModelInit because the model is being initialized
            errorModelInit = false;

            Eigen::VectorXd statesInit = Eigen::VectorXd::Zero(12);
            for(int i = 0; i < 12; i++)
                statesInit[i] = uwvParameters.initial_condition[i];

            // Updates initial system states
            updateStates(statesInit);

            // Sets the uwv parameters
            if(setUWVParameters(uwvParameters))
            {
                // Checks if the positive inertia, positive linear damping and thrust
                // configuration matrices were set
                checkPositiveMatrices();
            }

            if(!errorStatus)
                return true;
            else
                return false;
        }
        else
            return false;
    }
    else
    {
        LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
                " The model was already initialized, use the function"
                " setUWVParameters instead if you want to change any"
                " UWV parameter.\x1b[0m\n\n");
        return false;
    }
}



bool DynamicModel::sendEffortCommands(const base::samples::Joints &controlInput)
{
    // Checks if the control input is valid
    checkControlInput(controlInput, "effort");

    // Checks if there is any error flag activated
    checkErrors();

    // Checks if there was any error in the library
    if(!errorStatus)
    {
        Eigen::VectorXd systemStates = Eigen::VectorXd::Zero(12);

        // Puts the efforts in a vector
        for (int i = 0; i < gSystemOrder/2; i++)
            gEfforts[i] = controlInput[i].effort;

        // Gets a vector with the current system states (pose and velocities)
        getStates(systemStates);

        // Performs iterations to calculate the new system's states
        for (int ii=0; ii < gSimPerCycle; ii++)
            calcStates(systemStates, gCurrentTime, gEfforts);

        // Updates the new system's states
        updateStates(systemStates);

        return true;
    }
    else
        return false;
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
    case underwaterVehicle::SIMPLE:
        acceleration  = invInertiaMatrix * ( gEfforts - linDamping - quadDamping - gravityBuoyancy);
        break;
    case underwaterVehicle::COMPLEX:
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

bool DynamicModel::setUWVParameters(const underwaterVehicle::Parameters &uwvParameters)
{
    // Checks if there is any parameter inconsistency
    checkParameters(uwvParameters);

    // Checks if there is any error flag activated
    checkErrors();

    // Checks if the model was already initialized
    if(!errorModelInit)
    {
        if(!errorStatus)
        {
            setInertiaMatrix(uwvParameters.massMatrix,
                    uwvParameters.massMatrixNeg);
            setCoriolisMatrix(uwvParameters.coriolisMatrix,
                    uwvParameters.coriolisMatrixNeg);
            setAddedMassMatrix(uwvParameters.AddedMassMatrixPos,
                    uwvParameters.AddedMassMatrixNeg);
            setLiftCoefficients(uwvParameters.LiftCoefficients);
            setLinDampingMatrix(uwvParameters.linDampMatrix,
                    uwvParameters.linDampMatrixNeg);
            setQuadDampingMatrix(uwvParameters.quadDampMatrix,
                    uwvParameters.quadDampMatrixNeg);



            gWeight                     = uwvParameters.weight;
            gBuoyancy                   = uwvParameters.buoyancy;

            gCenterOfGravity  	= uwvParameters.distance_body2centerofgravity;
            gCenterOfBuoyancy 	= uwvParameters.distance_body2centerofbuoyancy;

            gSimPerCycle        = uwvParameters.sim_per_cycle;

            return true;
        }
        else
            return false;
    }
    else
    {
        LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
                " The model wasn't initialized yet. Use the function"
                " initParameters first to set the model parameters,"
                " and then, if you want to modify any of them, use the"
                " function setUWVParameters.\x1b[0m\n\n");
        return false;
    }
}

void DynamicModel::resetStates()
{
    base::Vector3d resetVector3d = Eigen::VectorXd::Zero(3);

    gPosition           = resetVector3d;
    gEulerOrientation   = resetVector3d;
    gLinearVelocity     = resetVector3d;
    gAngularVelocity    = resetVector3d;
}

void DynamicModel::setPosition(const base::Vector3d &position)
{
    gPosition = position;
}

void DynamicModel::setOrientation(const Eigen::Quaterniond &quatOrientation)
{
    base::samples::RigidBodyState orientation;
    orientation.orientation = quatOrientation;

    gEulerOrientation[0] = orientation.getRoll();
    gEulerOrientation[1] = orientation.getPitch();
    gEulerOrientation[2] = orientation.getYaw();
}

void DynamicModel::setLinearVelocity(const base::Vector3d &linearVelocity)
{
    gLinearVelocity = linearVelocity;
}

void DynamicModel::setAngularVelocity(const base::Vector3d &angularVelocity)
{
    gAngularVelocity = angularVelocity;
}

void DynamicModel::setModelType(const underwaterVehicle::ModelType &modelType)
{
    gModelType = modelType;
}

void DynamicModel::setSamplingTime(const double samplingTime)
{
    gSamplingTime = samplingTime;
    setIntegrationStep(gSamplingTime/(double)gSimPerCycle);
}


void DynamicModel::getUWVParameters(underwaterVehicle::Parameters &uwvParameters)
{
    uwvParameters.massMatrix                    = gInertiaMatrixPos;
    uwvParameters.massMatrixNeg                 = gInertiaMatrixNeg;
    uwvParameters.coriolisMatrix                = gCoriolisMatrixPos;
    uwvParameters.coriolisMatrixNeg             = gCoriolisMatrixNeg;
    uwvParameters.AddedMassMatrixPos            = gAddedMassMatrixPos;
    uwvParameters.AddedMassMatrixNeg            = gAddedMassMatrixNeg;
    uwvParameters.linDampMatrix                 = gLinDampMatrixPos;
    uwvParameters.linDampMatrixNeg              = gLinDampMatrixNeg;
    uwvParameters.quadDampMatrix                = gQuadDampMatrixPos;
    uwvParameters.quadDampMatrixNeg             = gQuadDampMatrixNeg;
    uwvParameters.thruster_control_matrix       = gThrustConfigMatrix;
    uwvParameters.LiftCoefficients              = gLiftCoefficients;


    uwvParameters.distance_body2centerofgravity         = gCenterOfGravity;
    uwvParameters.distance_body2centerofbuoyancy        = gCenterOfBuoyancy;
    uwvParameters.sim_per_cycle                         = gSimPerCycle;

    uwvParameters.ctrl_order    = gControlOrder;
    uwvParameters.samplingtime  = gSamplingTime;
}

void DynamicModel::getPosition(base::Position &position)
{
    position = gPosition;
}

void DynamicModel::getEulerOrientation(base::Vector3d &eulerOrientation)
{
    eulerOrientation = gEulerOrientation;
}

void DynamicModel::getQuatOrienration(base::Orientation &quatOrientation)
{
    eulerToQuaternion(quatOrientation, gEulerOrientation);
}

void DynamicModel::getLinearVelocity(base::Vector3d &linearVelocity, bool worldFrame)
{
    if(worldFrame)
    {
        // Body to world frame convertion
        base::Vector6d bodyVelocity;
        base::Vector6d worldVelocity;

        bodyVelocity.head(3) = gLinearVelocity;
        bodyVelocity.tail(3) = gAngularVelocity;

        convBodyToWorld(worldVelocity, bodyVelocity, gEulerOrientation);

        linearVelocity = worldVelocity.head(3);
    }
    else
    {
        linearVelocity = gLinearVelocity;
    }
}

void DynamicModel::getAngularVelocity(base::Vector3d &angularVelocity, bool worldFrame)
{
    if(worldFrame)
    {
        // Body to world frame convertion
        base::Vector6d bodyVelocity;
        base::Vector6d worldVelocity;

        bodyVelocity.head(3) = gLinearVelocity;
        bodyVelocity.tail(3) = gAngularVelocity;

        convBodyToWorld(worldVelocity, bodyVelocity, gEulerOrientation);

        angularVelocity = worldVelocity.tail(3);
    }
    else
    {
        angularVelocity = gAngularVelocity;
    }
}

void DynamicModel::getLinearAcceleration(base::Vector3d &linearAcceleration)
{
    linearAcceleration = gLinearAcceleration;
}

void DynamicModel::getAngularAcceleration(base::Vector3d &angularAcceleration)
{
    angularAcceleration = gAngularAcceleration;
}

void DynamicModel::getStates(Eigen::VectorXd &systemStates)
{
    systemStates.segment(0,3) = gPosition;
    systemStates.segment(3,3) = gEulerOrientation;
    systemStates.segment(6,3) = gLinearVelocity;
    systemStates.segment(9,3) = gAngularVelocity;
}

void DynamicModel::getEfforts(base::Vector6d &efforts)
{
    efforts = gEfforts;
}

void DynamicModel::getSimulationTime(double &simulationTime)
{
    simulationTime = gCurrentTime;
}

void DynamicModel::getSamplingTime(double &samplingTime)
{
    samplingTime = gSamplingTime;
}

void DynamicModel::getSimPerCycle(int &simPerCycle)
{
    simPerCycle = gSimPerCycle;
}


base::Matrix6d DynamicModel::calcInvInertiaMatrix(const base::Matrix6d &inertiaMatrix) const
{
    /**
     * M * M^(-1) = I
     */
    Eigen::JacobiSVD<base::Matrix6d> svd(inertiaMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.solve(base::Matrix6d::Identitity());
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

base::Vector6d DynamicModel::caclDampingEffect( const std::vector<base::Matrix6d> &quadDampMatrices, const base::Vector6d &velocity, const ModelType &modelType) const
{
    if(modelType == SIMPLE)
    {
        if(quadDampMatrices.size() != 2)
            throw std::runtime_error("quadDampMatrices does not have 2 elements, as expected for the SIMPLE mode.");
        return calcLinDamping(quadDampMatrices[0], velocity) + calcQuadDamping(quadDampMatrices[1], velocity);
    }
    else if(modelType == COMPLEX)
        return caclGeneralQuadDamping(quadDampMatrices, velocity);
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
    gPosition           = newSystemStates.segment(0,3);
    gEulerOrientation   = newSystemStates.segment(3,3);
    gLinearVelocity     = newSystemStates.segment(6,3);
    gAngularVelocity    = newSystemStates.segment(9,3);
}

void DynamicModel::setInertiaMatrix(const base::Matrix6d &inertiaMatrix)
{
    gInertiaMatrixPos = inertiaMatrix;
}

void DynamicModel::setLinDampingMatrix(const base::Matrix6d &linDampingMatrix)
{
    gLinDampMatrix = linDampingMatrix;
}

void DynamicModel::setQuadDampingMatrix(const base::Matrix6d &quadDampingMatrix)
{
    gQuadDampMatrixPos = quadDampingMatrix;
}

void DynamicModel::setDampingMatrices(const std::vector<base::Matrix6d> &dampingMatrices)
{
    gQuadDampMatrices = dampingMatrices;
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

void DynamicModel::checkParameters(const underwaterVehicle::Parameters &uwvParameters)
{
    if(uwvParameters.sim_per_cycle <= 0)
        throw std::runtime_error("The sim_per_cycle should be a positive value.");

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
            throw runtime_error("control input is nan");
    }
}

void DynamicModel::checkErrors(void)
{
    std::string textElement;
    bool checkError = false;

    if (errorModelInit && !errorStatus)
    {
        LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
                " The model wasn't initialized. Call the method initParameters"
                " in order to do so or check if there was an error while doing"
                " it.\x1b[0m\n\n");
        errorStatus = true;
    }
    else if(errorConstruction && !errorStatus)
    {
        textElement = "during the construction of the class";
        checkError = true;
    }
    else if(errorSetParameters && !errorStatus)
    {
        textElement = "while setting the model parameters";
        checkError = true;
    }
    else if(errorPWMCoeff && !errorStatus)
    {
        textElement = "with the PWM coefficients";
        checkError = true;
    }
    else if(errorRPMCoeff && !errorStatus)
    {
        textElement = "with the RPM coefficients";
        checkError = true;
    }
    else if(errorControlInput && !errorStatus)
    {
        textElement = "with the provided control input";
        checkError = true;
    }
    if(checkError && !errorStatus)
    {
        LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
                " There was an error %s.\x1b[0m\n\n", textElement.c_str());
        errorStatus = true;
    }

    if(!errorModelInit          &&
       !errorConstruction       &&
       !errorSetParameters      &&
       !errorPWMCoeff           &&
       !errorRPMCoeff           &&
       !errorControlInput       &&
       errorStatus)
    {
        errorStatus = false;
    }
}
};
