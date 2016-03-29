#define BOOST_TEST_MODULE UWV_DYNAMIC_MODEL
#include <boost/test/included/unit_test.hpp>
#include <uwv_dynamic_model/uwv_dynamic_model.hpp>
#include <uwv_dynamic_model/uwv_dataTypes.hpp>
#include <base/samples/Joints.hpp>
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


BOOST_AUTO_TEST_CASE( normal )
{
	underwaterVehicle::DynamicModel vehicle;
	vehicle.setUWVParameters(loadParameters());

	base::LinearAngular6DCommand controlInput;

	controlInput.linear = base::Vector3d(2,0,0);
	controlInput.angular = base::Vector3d(0,0,0);

	base::Vector6d velocity(base::Vector6d::Zero());
	base::Orientation orientation(base::Orientation::Identity());

	// For init velocity=0 and inertia term=1, acceleration in the surge dof must be equal to the force.
	BOOST_REQUIRE_EQUAL(vehicle.calcAcceleration(controlInput, velocity, orientation)[0], controlInput.linear[0]);
}


BOOST_AUTO_TEST_CASE( buoyancy )
{
    underwaterVehicle::DynamicModel vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();
    parameters.buoyancy = 3;
    parameters.weight = 1;

    vehicle.setUWVParameters(parameters);

    // No forces being applied
    base::LinearAngular6DCommand controlInput;
    controlInput.linear = base::Vector3d(0,0,0);
    controlInput.angular = base::Vector3d(0,0,0);

    base::Vector6d velocity(base::Vector6d::Zero());
    base::Orientation orientation(base::Orientation::Identity());

    BOOST_REQUIRE_EQUAL(vehicle.calcAcceleration(controlInput, velocity, orientation)[2], 2);
    // For init velocity=0 and inertia term=1, acceleration in the heave dof must be equal to gravity force of 2

}


BOOST_AUTO_TEST_SUITE_END()



underwaterVehicle::UWVParameters loadParameters(void)
{
    underwaterVehicle::UWVParameters parameters;
	parameters.inertiaMatrix << 1,   0,   0,   0,   0,    0,
			   	   	   	   	   	   0, 	1,   0,   0,   0,    0,
								   0,   0, 	 1,   0,   0,    0,
								   0,   0,   0,   1,   0,    0,
								   0,   0,   0,   0,   1,  	 0,
								   0,   0,   0,   0,   0, 	 1;
	parameters.dampMatrices.resize(2);
	parameters.dampMatrices[0] << 1,   0,   0,   0,   0,    0,
			   	   	   	   	   	   0,   1,   0,   0,   0,    0,
								   0,   0,   1,   0,   0,    0,
								   0,   0,   0,   1,   0,    0,
								   0,   0,   0,   0,   1,  	 0,
								   0,   0,   0,   0,   0, 	 1;
	parameters.dampMatrices[1] << 1,   0,   0,   0,   0,    0,
									0,   1,   0,   0,   0,    0,
									0,   0,   1,   0,   0,    0,
									0,   0,   0,   1,   0,    0,
									0,   0,   0,   0,   1,    0,
									0,   0,   0,   0,   0, 	  1;
	return parameters;
}
