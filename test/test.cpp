#define BOOST_TEST_MODULE UWV_DYNAMIC_MODEL
#include <boost/test/included/unit_test.hpp>
#include <uwv_dynamic_model/uwv_dynamic_model.hpp>
#include <uwv_dynamic_model/uwv_dataTypes.hpp>
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

underwaterVehicle::UWVParameters loadParameters(void);

BOOST_AUTO_TEST_SUITE (CONSTRUCTOR)

BOOST_AUTO_TEST_CASE( null_control_order )
{
	BOOST_REQUIRE_NO_THROW( underwaterVehicle::DynamicModel vehicle);
}
BOOST_AUTO_TEST_SUITE_END()



BOOST_AUTO_TEST_SUITE (INITPARAMETERS)

BOOST_AUTO_TEST_CASE( normal )
{
    underwaterVehicle::DynamicModel vehicle;
    BOOST_REQUIRE_NO_THROW( vehicle.setUWVParameters(loadParameters()));
}

BOOST_AUTO_TEST_CASE( unset_inertia_matrix)
{
	underwaterVehicle::DynamicModel vehicle;
	underwaterVehicle::UWVParameters parameters = loadParameters();


	// Test
	parameters.inertiaMatrix = Eigen::MatrixXd::Zero(6,6);

	// Invert inertia set as zero
	BOOST_REQUIRE_NO_THROW(vehicle.setUWVParameters(parameters));

}


BOOST_AUTO_TEST_CASE( wrong_set_damp_matrix_1 )
{
	underwaterVehicle::DynamicModel vehicle;
	underwaterVehicle::UWVParameters parameters = loadParameters();

	// Test
	parameters.dampMatrices.resize(1);
	BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::runtime_error);
}

BOOST_AUTO_TEST_CASE( wrong_set_damp_matrix_2 )
{
    underwaterVehicle::DynamicModel vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();

    // Test
    parameters.dampMatrices.resize(4);
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::runtime_error);
}

BOOST_AUTO_TEST_CASE( damp_matrix_and_Model_Type_inconsistent_1 )
{
    underwaterVehicle::DynamicModel vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();

    // Test
    parameters.modelType = underwaterVehicle::COMPLEX;
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::runtime_error);
}

BOOST_AUTO_TEST_CASE( damp_matrix_and_Model_Type_inconsistent_2 )
{
    underwaterVehicle::DynamicModel vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();

    // Test
    parameters.dampMatrices.resize(6);
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::runtime_error);
}

BOOST_AUTO_TEST_CASE( unset_command )
{
    underwaterVehicle::DynamicModel vehicle;

    // Test
    base::LinearAngular6DCommand controlInput;
    base::Vector6d velocity(base::Vector6d::Zero());
    base::Orientation orientation(base::Orientation::Identity());
    BOOST_REQUIRE_NO_THROW(vehicle.setUWVParameters(loadParameters()));
    BOOST_REQUIRE_THROW(vehicle.calcAcceleration(controlInput, velocity, orientation), std::runtime_error);
}

BOOST_AUTO_TEST_CASE( uwv_weight )
{

    underwaterVehicle::DynamicModel vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();

    // Test
    parameters.weight = -1;
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::runtime_error);
}

BOOST_AUTO_TEST_CASE( uwv_buoyancy )
{
    underwaterVehicle::DynamicModel vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();

    // Test
    parameters.buoyancy = -1;
    BOOST_REQUIRE_THROW(vehicle.setUWVParameters(parameters), std::runtime_error);

}

BOOST_AUTO_TEST_SUITE_END()






BOOST_AUTO_TEST_SUITE (SEND_COMMANDS)

BOOST_AUTO_TEST_CASE( normal )
{
    underwaterVehicle::DynamicModel vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();

    BOOST_REQUIRE_NO_THROW(vehicle.setUWVParameters(parameters));

    base::LinearAngular6DCommand controlInput;
    controlInput.linear = base::Vector3d(2,0,0);
    controlInput.angular = base::Vector3d(0,0,0);

    base::Vector6d velocity(base::Vector6d::Zero());
    base::Orientation orientation(base::Orientation::Identity());


    BOOST_REQUIRE_NO_THROW(vehicle.calcAcceleration(controlInput, velocity, orientation));

}

BOOST_AUTO_TEST_CASE( unset_command )
{
    underwaterVehicle::DynamicModel vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();

    BOOST_REQUIRE_NO_THROW(vehicle.setUWVParameters(parameters));

    base::LinearAngular6DCommand controlInput;

    controlInput.linear = base::Vector3d(2,0,0);

    base::Vector6d velocity(base::Vector6d::Zero());
    base::Orientation orientation(base::Orientation::Identity());

    BOOST_REQUIRE_THROW(vehicle.calcAcceleration(controlInput, velocity, orientation), std::runtime_error);

}


BOOST_AUTO_TEST_SUITE_END()



underwaterVehicle::UWVParameters loadParameters(void)
{
    underwaterVehicle::UWVParameters parameters;
	parameters.inertiaMatrix			= Eigen::MatrixXd::Identity(6,6);
	parameters.dampMatrices[0] 			= Eigen::MatrixXd::Identity(6,6);
	parameters.dampMatrices[1] 			= Eigen::MatrixXd::Identity(6,6);

	return parameters;
}

