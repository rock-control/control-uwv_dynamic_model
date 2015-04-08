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

void loadParameters(underwaterVehicle::Parameters &parameters);

BOOST_AUTO_TEST_SUITE (CONSTRUCTOR)


BOOST_AUTO_TEST_CASE( normal )
{
	underwaterVehicle::DynamicModel vehicle(4, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);


	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(4);

	controlInput[0].raw = 1.0005;
	controlInput[1].raw = 0.9995;
	controlInput[2].raw = 0;
	controlInput[3].raw = 0;

	for (int i = 0; i < 20; i++)
		vehicle.sendPWMCommands(controlInput);

	base::Vector3d position;
	base::Vector3d linearVelocity;

	vehicle.getPosition(position);
	vehicle.getLinearVelocity(linearVelocity);

	std::cout << "position\n" << position << std::endl;
	std::cout << "linearVelocity\n" << linearVelocity << std::endl;
}





BOOST_AUTO_TEST_SUITE_END()



void loadParameters(underwaterVehicle::Parameters &parameters)
{
	parameters.massMatrix << 1,   0,   0,   0,   0,    0,
			   	   	   	   	   	   0, 	1,   0,   0,   0,    0,
								   0,   0, 	 1,   0,   0,    0,
								   0,   0,   0,   1,   0,    0,
								   0,   0,   0,   0,   1,  	 0,
								   0,   0,   0,   0,   0, 	 1;
	parameters.linDampMatrix << 1,   0,   0,   0,   0,    0,
			   	   	   	   	   	   0,   1,   0,   0,   0,    0,
								   0,   0,   1,   0,   0,    0,
								   0,   0,   0,   1,   0,    0,
								   0,   0,   0,   0,   1,  	 0,
								   0,   0,   0,   0,   0, 	 1;
	parameters.quadDampMatrix << 1,   0,   0,   0,   0,    0,
									0,   1,   0,   0,   0,    0,
									0,   0,   1,   0,   0,    0,
									0,   0,   0,   1,   0,    0,
									0,   0,   0,   0,   1,    0,
									0,   0,   0,   0,   0, 	  1;

	parameters.thruster_control_matrix.resize(6,4);
	parameters.thruster_control_matrix <<   1,   	   	1,   	0,   	0,
			   	   	   	   	   	       0,   		0,   	0,  	 1,
									   0,   		0,   	1,   	0,
									   0,   		0,   	0,   	1,
									   0,   	   	0,   	0,   	0,
									   0.05,	-0.05,   	0,   	0.015;
	parameters.thruster_coefficients_pwm.resize(4);
	parameters.thruster_coefficient_rpm.resize(4);
	parameters.thrusterVoltage.resize(4);

	for(int i = 0; i < 4; i++)
	{
		parameters.thruster_coefficients_pwm[i].positive 	= 1;
		parameters.thruster_coefficients_pwm[i].negative 	= 1;
		parameters.thruster_coefficient_rpm[i].positive 	= 1;
		parameters.thruster_coefficient_rpm[i].negative 	= 1;
		parameters.thrusterVoltage[i] 	= 1;
	}
}

