#include "DynamicModel.hpp"
#include <base/Logging.hpp>

namespace uwv_dynamic_model
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

base::Vector6d DynamicModel::calcAcceleration(const base::Vector6d &control_input, const base::Vector6d &velocity, const base::Orientation &orientation)
{
    // Check inputs
    checkControlInput(control_input);
    checkVelocity(velocity);

    // Calculating the acceleration based on all the hydrodynamics effects
    base::Vector6d acceleration = base::Vector6d::Zero();

    acceleration = control_input - calcGravityBuoyancy(orientation, gUwvParameters);
    switch(gUwvParameters.modelType)
    {
    case SIMPLE:
        acceleration  -=  caclSimpleDamping(gUwvParameters.dampMatrices, velocity);
        break;
    case COMPLEX:
        acceleration  -= (calcCoriolisEffect(gUwvParameters.inertiaMatrix, velocity) + caclGeneralQuadDamping(gUwvParameters.dampMatrices, velocity));
        break;
    }
    return gInvInertiaMatrix*acceleration;
}

void DynamicModel::setUWVParameters(const UWVParameters &uwv_parameters)
{
    // Checks if there is any parameter inconsistency
    checkParameters(uwv_parameters);
    gUwvParameters = uwv_parameters;
    gInvInertiaMatrix = calcInvInertiaMatrix(uwv_parameters.inertiaMatrix);
}

UWVParameters DynamicModel::getUWVParameters(void) const
{
    return gUwvParameters;
}

base::Matrix6d DynamicModel::calcInvInertiaMatrix(const base::Matrix6d &inertia_matrix) const
{
    /**
     * M * M^(-1) = I
     * A*x = b
     */
    Eigen::JacobiSVD<base::Matrix6d> svd(inertia_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.solve(base::Matrix6d::Identity());
}

base::Vector6d DynamicModel::calcCoriolisEffect(const base::Matrix6d &inertia_matrix, const base::Vector6d &velocity) const
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
    base::Vector6d prod = inertia_matrix * velocity;
    coriloisEffect << prod.head<3>().cross(velocity.tail<3>()),
                prod.head<3>().cross(velocity.head<3>()) + prod.tail<3>().cross(velocity.tail<3>());
    return -coriloisEffect;
}

base::Vector6d DynamicModel::caclGeneralQuadDamping( const std::vector<base::Matrix6d> &quad_damp_matrices, const base::Vector6d &velocity) const
{
    /**
     *  Based on McFarland[2013]
     *  damping effect = sum(Di * |vi|) * v, i=1...6
     *  D = quadDampMatrix; v = velocity
     */
    if(quad_damp_matrices.size() != 6)
        throw std::runtime_error("quadDampMatrices does not have 6 elements.");

    base::Matrix6d dampMatrix = base::Matrix6d::Zero();
    for(size_t i=0; i < quad_damp_matrices.size(); i++)
        dampMatrix += quad_damp_matrices[i] * velocity.cwiseAbs()[i];

    return dampMatrix * velocity;
}

base::Vector6d DynamicModel::caclSimpleDamping( const std::vector<base::Matrix6d> &damp_matrices, const base::Vector6d &velocity) const
{
    /**
     *  Based on Fossen[1994]
     *  damping effect = quadDampingMatrix*|vi|*v + linDampingMatrix*v
     */
    if(damp_matrices.size() != 2)
        throw std::runtime_error("dampMatrices does not have 2 elements.");
    return calcLinDamping(damp_matrices[0], velocity) + calcQuadDamping(damp_matrices[1], velocity);
}

base::Vector6d DynamicModel::calcLinDamping(const base::Matrix6d &lin_damp_matrix, const base::Vector6d &velocity) const
{
    return lin_damp_matrix * velocity;
}

base::Vector6d DynamicModel::calcQuadDamping( const base::Matrix6d &quad_damp_matrix, const base::Vector6d &velocity) const
{
    return quad_damp_matrix * velocity.cwiseAbs().asDiagonal() * velocity;
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
     *  In Rock framework, positive z is pointing up, in marine/underwater literature positive z is pointing down.
     */
    base::Vector6d gravityEffect;
    gravityEffect << orientation.inverse() * Eigen::Vector3d(0, 0, (weight-bouyancy)),
            (cg*weight - cb*bouyancy).cross(orientation.inverse() * Eigen::Vector3d(0, 0, 1));
    return gravityEffect;
}

void DynamicModel::checkParameters(const UWVParameters &uwv_parameters)
{
    if(uwv_parameters.modelType == SIMPLE && uwv_parameters.dampMatrices.size() != 2)
        throw std::invalid_argument("in SIMPLE model, dampMatrices should have two elements, the linDampingMatrix and quadDampingMatrix");

    if(uwv_parameters.modelType == COMPLEX && uwv_parameters.dampMatrices.size() != 6)
        throw std::invalid_argument("in COMPLEX model, dampMatrices should have six elements, one quadDampingMatrix / DOF");

    if(uwv_parameters.weight <= 0)
        throw std::invalid_argument("weight must be a positive value");
    if(uwv_parameters.buoyancy <= 0)
        throw std::invalid_argument("buoyancy must be a positive value");
}

void DynamicModel::checkControlInput(const base::Vector6d &control_input) const
{
    if(control_input.hasNaN())
        throw std::runtime_error("DynamicModel checkControlInput: control input is unset");
}

void DynamicModel::checkVelocity(const base::Vector6d &velocity)
{
    if(velocity.hasNaN())
        throw std::runtime_error("DynamicModel checkVelocity: velocity is unset");
}

};
