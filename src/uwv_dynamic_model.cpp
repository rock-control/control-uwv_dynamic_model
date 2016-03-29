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
DynamicModel::DynamicModel()
{
    // uwv model parameters
    UWVParameters uwvParameters;
    setUWVParameters(uwvParameters);
}

DynamicModel::~DynamicModel()
{
}

base::Vector6d DynamicModel::calcAcceleration(const base::LinearAngular6DCommand &controlInput, const base::Vector6d &velocity, const base::Orientation &orientation)
{
    // Check inputs
    checkControlInput(controlInput);
    checkVelocity(velocity);

    base::Vector6d control_input;
    control_input.head(3) = controlInput.linear;
    control_input.tail(3) = controlInput.angular;

    // Calculating the acceleration based on all the hydrodynamics effects
    base::Vector6d acceleration = Eigen::VectorXd::Zero(6);
    switch(gUwvParameters.modelType)
    {
    case SIMPLE:
        acceleration  = gInvInertiaMatrix * ( control_input + caclDampingEffect(gUwvParameters, velocity) + calcGravityBuoyancy(orientation, gUwvParameters));
        break;
    case COMPLEX:
        acceleration  = gInvInertiaMatrix * ( control_input + calcCoriolisEffect(gUwvParameters.inertiaMatrix, velocity) + caclDampingEffect(gUwvParameters, velocity) + calcGravityBuoyancy(orientation, gUwvParameters));
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

UWVParameters DynamicModel::getUWVParameters(void) const
{
    return gUwvParameters;
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
        throw std::runtime_error("DynamicModel checkControlInput: control input is unset");
}

void DynamicModel::checkVelocity(const base::Vector6d &velocity)
{
    if(velocity.hasNaN())
        throw std::runtime_error("DynamicModel checkVelocity: velocity is unset");
}

};
