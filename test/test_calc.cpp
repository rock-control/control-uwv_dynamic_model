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
	underwaterVehicle::DynamicModel vehicle(0.2, 8, 1);
	vehicle.setUWVParameters(loadParameters());

	base::LinearAngular6DCommand controlInput;

	controlInput.linear = base::Vector3d(2,0,0);
	controlInput.angular = base::Vector3d(0,0,0);

	for (int i = 0; i < 20; i++)
		vehicle.sendEffortCommands(controlInput);

	std::cout << "position\n" << vehicle.getPosition() << std::endl;
	std::cout << "linearVelocity\n" << vehicle.getLinearVelocity() << std::endl;
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

