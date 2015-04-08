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

void loadParameters(underwaterVehicle::Parameters &parameters);

BOOST_AUTO_TEST_SUITE (CONSTRUCTOR)

BOOST_AUTO_TEST_CASE( null_control_order )
{
	uint controlOrder = 0;
	double samplingTime = 0.2;
	uint simPerCycle = 8;
	double initialTime = 1;
	underwaterVehicle::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_CASE( negative_sampling_time )
{
	uint controlOrder = 4;
	double samplingTime = -0.2;
	uint simPerCycle = 8;
	double initialTime = 1;
	underwaterVehicle::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_CASE( null_sampling_time )
{
	uint controlOrder = 4;
	double samplingTime = 0;
	uint simPerCycle = 8;
	double initialTime = 1;
	underwaterVehicle::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_CASE( null_sim_per_cycle )
{
	uint controlOrder = 4;
	double samplingTime = 0.2;
	uint simPerCycle = 0;
	double initialTime = 1;
	underwaterVehicle::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_CASE( negative_initial_time )
{
	uint controlOrder = 4;
	double samplingTime = 0.2;
	uint simPerCycle = 8;
	double initialTime = -1;
	underwaterVehicle::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_SUITE_END()



BOOST_AUTO_TEST_SUITE (INITPARAMETERS)

BOOST_AUTO_TEST_CASE( normal )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);


	BOOST_CHECK(vehicle.initParameters(parameters) == true);

}

BOOST_AUTO_TEST_CASE( unset_inertia_matrix_pos )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.massMatrix = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters) == false);

}

BOOST_AUTO_TEST_CASE( unset_inertia_matrix_neg )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.massMatrixNeg = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

	vehicle.getUWVParameters(parameters);
	std::cout << "parameters\n\n" << parameters.massMatrixNeg << std::endl;
}

BOOST_AUTO_TEST_CASE( unset_coriolis_matrix_pos )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.coriolisMatrix = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);
}

BOOST_AUTO_TEST_CASE( unset_coriolis_matrix_neg )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.coriolisMatrixNeg = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

	vehicle.getUWVParameters(parameters);
	std::cout << "parameters\n\n" << parameters.coriolisMatrixNeg << std::endl;
}

BOOST_AUTO_TEST_CASE( unset_lin_damp_matrix_pos )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.linDampMatrix = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);
}

BOOST_AUTO_TEST_CASE( unset_lin_damp_matrix_neg )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.linDampMatrixNeg = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

	vehicle.getUWVParameters(parameters);
	std::cout << "parameters\n\n" << parameters.linDampMatrixNeg << std::endl;
}

BOOST_AUTO_TEST_CASE( unset_quad_damp_matrix_pos )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.quadDampMatrix = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

}

BOOST_AUTO_TEST_CASE( unset_quad_damp_matrix_neg )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.quadDampMatrixNeg = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

	vehicle.getUWVParameters(parameters);
	std::cout << "parameters\n\n" << parameters.quadDampMatrixNeg << std::endl;
}

BOOST_AUTO_TEST_CASE( thrust_config_matrix_rows )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thruster_control_matrix = Eigen::MatrixXd::Zero(5,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thrust_config_matrix_cols )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thruster_control_matrix = Eigen::MatrixXd::Zero(6,5);

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( uwv_mass )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.uwv_mass = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( uwv_volume )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.uwv_volume = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( water_density )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.waterDensity = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( gravity )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.gravity = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_voltage )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thrusterVoltage[0] = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_pwm_pos )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thruster_coefficients_pwm[5].positive = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_pwm_neg )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thruster_coefficients_pwm[5].negative = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( lin_thruster_pwm_pos )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.linear_thruster_coefficients_pwm.resize(6);

	// Test
	parameters.linear_thruster_coefficients_pwm[5].positive = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( lin_thruster_pwm_neg )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.linear_thruster_coefficients_pwm.resize(6);

	// Test
	parameters.linear_thruster_coefficients_pwm[5].negative = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( quad_thruster_pwm_pos )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.square_thruster_coefficients_pwm.resize(6);

	// Test
	parameters.square_thruster_coefficients_pwm[5].positive = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( quad_thruster_pwm_neg )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.square_thruster_coefficients_pwm.resize(6);

	// Test
	parameters.square_thruster_coefficients_pwm[5].negative = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_rpm_pos )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thruster_coefficient_rpm[5].positive = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_rpm_neg )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thruster_coefficient_rpm[5].negative = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_SUITE_END()






BOOST_AUTO_TEST_SUITE (SEND_PWM_COMMANDS)

BOOST_AUTO_TEST_CASE( normal )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 1;

	std::cout << "size: " << parameters.linear_thruster_coefficients_pwm.size() << std::endl;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == true);

}

BOOST_AUTO_TEST_CASE( wrong_input_size )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(5);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( raw_unset )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].effort = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( pwm_pos_unset )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.thruster_coefficients_pwm[0].positive = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( pwm_neg_unset )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.thruster_coefficients_pwm[0].negative = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( null_thruster_voltage )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.thrusterVoltage[0] = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_SUITE_END()




BOOST_AUTO_TEST_SUITE (SEND_RPM_COMMANDS)

BOOST_AUTO_TEST_CASE( wrong_input_size )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(5);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].speed = 0;

	BOOST_CHECK(vehicle.sendRPMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( speed_unset )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].effort = 0;

	BOOST_CHECK(vehicle.sendRPMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( rpm_pos_unset )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.thruster_coefficient_rpm[0].positive = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].speed = 0;

	BOOST_CHECK(vehicle.sendRPMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( rpm_neg_unset )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	parameters.thruster_coefficient_rpm[0].negative = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].speed = 0;

	BOOST_CHECK(vehicle.sendRPMCommands(controlInput) == false);

}


BOOST_AUTO_TEST_SUITE_END()


BOOST_AUTO_TEST_SUITE (SEND_EFFORT_COMMANDS)

BOOST_AUTO_TEST_CASE( wrong_input_size )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(5);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].effort = 0;

	BOOST_CHECK(vehicle.sendEffortCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( effort_unset )
{
	underwaterVehicle::DynamicModel vehicle(6, 0.2, 8, 1);
	underwaterVehicle::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(uint i = 0; i < controlInput.size(); i++)
		controlInput[i].speed = 0;

	BOOST_CHECK(vehicle.sendEffortCommands(controlInput) == false);

}




BOOST_AUTO_TEST_SUITE_END()



void loadParameters(underwaterVehicle::Parameters &parameters)
{
	parameters.massMatrix 				= Eigen::MatrixXd::Identity(6,6);
	parameters.coriolisMatrix 			= Eigen::MatrixXd::Identity(6,6);
	parameters.linDampMatrix 			= Eigen::MatrixXd::Identity(6,6);
	parameters.quadDampMatrix 			= Eigen::MatrixXd::Identity(6,6);
	parameters.thruster_control_matrix 	= Eigen::MatrixXd::Identity(6,6);

	parameters.thruster_coefficients_pwm.resize(6);
	parameters.thruster_coefficient_rpm.resize(6);
	parameters.thrusterVoltage.resize(6);

	for(int i = 0; i < 6; i++)
	{
		parameters.thruster_coefficients_pwm[i].positive 	= 1;
		parameters.thruster_coefficients_pwm[i].negative 	= 1;
		parameters.thruster_coefficient_rpm[i].positive 	= 1;
		parameters.thruster_coefficient_rpm[i].negative 	= 1;
		parameters.thrusterVoltage[i] 	= 1;
	}
}

