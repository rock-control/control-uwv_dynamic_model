#define BOOST_TEST_MODULE UWV_DYNAMIC_MODEL
#include <boost/test/included/unit_test.hpp>
#include <uwv_dynamic_model/ModelSimulation.hpp>
#include <iostream>

/**
 * Commands for testing:
 *
 * # ./unit_test --log_level=test_suite --run_test=CONSTRUCTOR
 *
 * Constructor was just an example, it could be any of the test suites, or
 * you can just hide this last parameter and the test will run for all
 * the test suites, for example:
 *
 * # ./unit_test --log_level=test_suite
 */

using namespace uwv_dynamic_model;
using namespace base;

UWVParameters loadParameters(void);

BOOST_AUTO_TEST_SUITE (CONSTRUCTOR)

BOOST_AUTO_TEST_CASE( null_control_order )
{
	BOOST_REQUIRE_NO_THROW( DynamicModel vehicle);
}
BOOST_AUTO_TEST_SUITE_END()


BOOST_AUTO_TEST_SUITE (OPERATORS)

BOOST_AUTO_TEST_CASE(operators )
{
    PoseVelocityState pose;
    pose.position = Vector3d::Ones();
    pose.linear_velocity = Vector3d::Ones();
    pose.angular_velocity = Vector3d::Ones();

    pose = 2*pose;
    pose = pose*2;
    pose *= 2;
    pose += pose;

    for(size_t i=0; i<3; i++)
    {
        BOOST_REQUIRE_EQUAL(pose.position[i], 16);
        BOOST_REQUIRE_EQUAL(pose.linear_velocity[i], 16);
        BOOST_REQUIRE_EQUAL(pose.angular_velocity[i],16);
        BOOST_REQUIRE_EQUAL(pose.orientation.coeffs()[i], (16*Orientation::Identity().coeffs())[i]);
    }
    BOOST_REQUIRE_EQUAL(pose.orientation.coeffs()[3], (16*Orientation::Identity().coeffs())[3]);
}
BOOST_AUTO_TEST_SUITE_END()




BOOST_AUTO_TEST_SUITE (INITPARAMETERS)

BOOST_AUTO_TEST_CASE( normal )
{
    DynamicModel vehicle;
    BOOST_REQUIRE_NO_THROW( vehicle.setUWVParameters(loadParameters()));
}

BOOST_AUTO_TEST_CASE( unset_inertia_matrix)
{
    DynamicModel vehicle;
    UWVParameters parameters = loadParameters();


	// Test
	parameters.inertia_matrix = Eigen::MatrixXd::Zero(6,6);

	// Invert inertia set as zero
	BOOST_REQUIRE_NO_THROW(vehicle.setUWVParameters(parameters));

}


BOOST_AUTO_TEST_CASE( wrong_set_damp_matrix_1 )
{
    DynamicModel vehicle;
    UWVParameters parameters = loadParameters();

	// Test
	parameters.damping_matrices.resize(1);
	BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE( wrong_set_damp_matrix_2 )
{
    DynamicModel vehicle;
    UWVParameters parameters = loadParameters();

    // Test
    parameters.damping_matrices.resize(4);
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE( damp_matrix_and_Model_Type_inconsistent_1 )
{
    DynamicModel vehicle;
    UWVParameters parameters = loadParameters();

    // Test
    parameters.model_type = COMPLEX;
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE( damp_matrix_and_Model_Type_inconsistent_2 )
{
    DynamicModel vehicle;
    UWVParameters parameters = loadParameters();

    // Test
    parameters.damping_matrices.resize(6);
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE( send_command )
{
    DynamicModel vehicle;

    // Test
    base::Vector6d control_input;
    base::Vector6d velocity(base::Vector6d::Zero());
    base::Orientation orientation(base::Orientation::Identity());
    BOOST_REQUIRE_NO_THROW(vehicle.setUWVParameters(loadParameters()));
    BOOST_REQUIRE_NO_THROW(vehicle.calcAcceleration(control_input, velocity, orientation));
}

BOOST_AUTO_TEST_CASE( uwv_weight )
{

    DynamicModel vehicle;
    UWVParameters parameters = loadParameters();

    // Test
    parameters.weight = -1;
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE( uwv_buoyancy )
{
    DynamicModel vehicle;
    UWVParameters parameters = loadParameters();

    // Test
    parameters.buoyancy = -1;
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::invalid_argument);

}

BOOST_AUTO_TEST_SUITE_END()






BOOST_AUTO_TEST_SUITE (SEND_COMMANDS)

BOOST_AUTO_TEST_CASE( normal )
{
    DynamicModel vehicle;
    UWVParameters parameters = loadParameters();

    BOOST_REQUIRE_NO_THROW(vehicle.setUWVParameters(parameters));

    base::Vector6d controlInput(base::Vector6d::Zero());
    controlInput[0] = 2;

    base::Vector6d velocity(base::Vector6d::Zero());
    base::Orientation orientation(base::Orientation::Identity());


    BOOST_REQUIRE_NO_THROW(vehicle.calcAcceleration(controlInput, velocity, orientation));

}

BOOST_AUTO_TEST_CASE( unset_command )
{
    ModelSimulation vehicle(DYNAMIC);
    UWVParameters parameters = loadParameters();

    BOOST_REQUIRE_NO_THROW(vehicle.setUWVParameters(parameters));

    base::Vector6d controlInput(base::Vector6d::Zero());
    controlInput[4] = std::numeric_limits<double>::quiet_NaN();
    controlInput[0] = 2;

    BOOST_REQUIRE_THROW(vehicle.sendEffort(controlInput), std::runtime_error);

}


BOOST_AUTO_TEST_SUITE_END()



UWVParameters loadParameters(void)
{
    UWVParameters parameters;
    parameters.inertia_matrix            = Eigen::MatrixXd::Identity(6,6);
	parameters.damping_matrices[0] 			= Eigen::MatrixXd::Identity(6,6);
	parameters.damping_matrices[1] 			= Eigen::MatrixXd::Identity(6,6);

	return parameters;
}
