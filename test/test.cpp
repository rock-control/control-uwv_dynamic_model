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

void loadParameters(uwv_dynamic_model::Parameters &parameters);

BOOST_AUTO_TEST_SUITE (CONSTRUCTOR)

BOOST_AUTO_TEST_CASE( null_control_order )
{
	uint controlOrder = 0;
	double samplingTime = 0.2;
	uint simPerCycle = 8;
	double initialTime = 1;
	uwv_dynamic_model::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_CASE( negative_sampling_time )
{
	uint controlOrder = 4;
	double samplingTime = -0.2;
	uint simPerCycle = 8;
	double initialTime = 1;
	uwv_dynamic_model::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_CASE( null_sampling_time )
{
	uint controlOrder = 4;
	double samplingTime = 0;
	uint simPerCycle = 8;
	double initialTime = 1;
	uwv_dynamic_model::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_CASE( null_sim_per_cycle )
{
	uint controlOrder = 4;
	double samplingTime = 0.2;
	uint simPerCycle = 0;
	double initialTime = 1;
	uwv_dynamic_model::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_CASE( negative_initial_time )
{
	uint controlOrder = 4;
	double samplingTime = 0.2;
	uint simPerCycle = 8;
	double initialTime = -1;
	uwv_dynamic_model::DynamicModel vehicle(controlOrder, samplingTime, simPerCycle, initialTime);
}

BOOST_AUTO_TEST_SUITE_END()



BOOST_AUTO_TEST_SUITE (INITPARAMETERS)

BOOST_AUTO_TEST_CASE( normal )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);


	BOOST_CHECK(vehicle.initParameters(parameters) == true);

}

BOOST_AUTO_TEST_CASE( unset_inertia_matrix_pos )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.inertiaMatrixPos = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters) == false);

}

BOOST_AUTO_TEST_CASE( unset_inertia_matrix_neg )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.inertiaMatrixNeg = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

	vehicle.getUWVParameters(parameters);
	std::cout << "parameters\n\n" << parameters.inertiaMatrixNeg << std::endl;
}

BOOST_AUTO_TEST_CASE( unset_coriolis_matrix_pos )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.coriolisMatrixPos = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);
}

BOOST_AUTO_TEST_CASE( unset_coriolis_matrix_neg )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.coriolisMatrixNeg = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

	vehicle.getUWVParameters(parameters);
	std::cout << "parameters\n\n" << parameters.coriolisMatrixNeg << std::endl;
}

BOOST_AUTO_TEST_CASE( unset_lin_damp_matrix_pos )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.linDampMatrixPos = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);
}

BOOST_AUTO_TEST_CASE( unset_lin_damp_matrix_neg )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.linDampMatrixNeg = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

	vehicle.getUWVParameters(parameters);
	std::cout << "parameters\n\n" << parameters.linDampMatrixNeg << std::endl;
}

BOOST_AUTO_TEST_CASE( unset_quad_damp_matrix_pos )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.quadDampMatrixPos = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

}

BOOST_AUTO_TEST_CASE( unset_quad_damp_matrix_neg )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.quadDampMatrixNeg = Eigen::MatrixXd::Zero(6,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == true);

	vehicle.getUWVParameters(parameters);
	std::cout << "parameters\n\n" << parameters.quadDampMatrixNeg << std::endl;
}

BOOST_AUTO_TEST_CASE( thrust_config_matrix_rows )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thrustConfigMatrix = Eigen::MatrixXd::Zero(5,6);

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thrust_config_matrix_cols )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thrustConfigMatrix = Eigen::MatrixXd::Zero(6,5);

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( uwv_mass )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.uwvMass = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( uwv_volume )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.uwvVolume = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( water_density )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.waterDensity = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( gravity )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.gravity = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_voltage )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thrusterVoltage = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_pwm_pos )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thrusterCoeffPWM.positive = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_pwm_neg )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thrusterCoeffPWM.negative = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( lin_thruster_pwm_pos )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.linThrusterCoeffPWM.positive = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( lin_thruster_pwm_neg )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.linThrusterCoeffPWM.negative = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( quad_thruster_pwm_pos )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.quadThrusterCoeffPWM.positive = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( quad_thruster_pwm_neg )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.quadThrusterCoeffPWM.negative = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_rpm_pos )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thrusterCoeffRPM.positive = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_CASE( thruster_rpm_neg )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	// Test
	parameters.thrusterCoeffRPM.negative = -1;

	BOOST_CHECK(vehicle.initParameters(parameters)  == false);

}

BOOST_AUTO_TEST_SUITE_END()






BOOST_AUTO_TEST_SUITE (SEND_PWM_COMMANDS)

BOOST_AUTO_TEST_CASE( wrong_input_size )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(5);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( raw_unset )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].effort = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( pwm_pos_unset )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	parameters.thrusterCoeffPWM.positive = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( pwm_neg_unset )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	parameters.thrusterCoeffPWM.negative = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( null_thruster_voltage )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	parameters.thrusterVoltage = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].raw = 0;

	BOOST_CHECK(vehicle.sendPWMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_SUITE_END()




BOOST_AUTO_TEST_SUITE (SEND_RPM_COMMANDS)

BOOST_AUTO_TEST_CASE( wrong_input_size )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(5);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].speed = 0;

	BOOST_CHECK(vehicle.sendRPMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( speed_unset )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].effort = 0;

	BOOST_CHECK(vehicle.sendRPMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( rpm_pos_unset )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	parameters.thrusterCoeffRPM.positive = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].speed = 0;

	BOOST_CHECK(vehicle.sendRPMCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( rpm_neg_unset )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	parameters.thrusterCoeffRPM.negative = 0;

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].speed = 0;

	BOOST_CHECK(vehicle.sendRPMCommands(controlInput) == false);

}


BOOST_AUTO_TEST_SUITE_END()


BOOST_AUTO_TEST_SUITE (SEND_EFFORT_COMMANDS)

BOOST_AUTO_TEST_CASE( wrong_input_size )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(5);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].effort = 0;

	BOOST_CHECK(vehicle.sendEffortCommands(controlInput) == false);

}

BOOST_AUTO_TEST_CASE( effort_unset )
{
	uwv_dynamic_model::DynamicModel vehicle(6, 0.2, 8, 1);
	uwv_dynamic_model::Parameters parameters;
	loadParameters(parameters);

	BOOST_CHECK(vehicle.initParameters(parameters) == true);

	base::samples::Joints controlInput;
	controlInput.resize(6);

	for(int i = 0; i < controlInput.size(); i++)
		controlInput[i].speed = 0;

	BOOST_CHECK(vehicle.sendEffortCommands(controlInput) == false);

}




BOOST_AUTO_TEST_SUITE_END()



void loadParameters(uwv_dynamic_model::Parameters &parameters)
{
	parameters.inertiaMatrixPos 	= Eigen::MatrixXd::Identity(6,6);
	parameters.coriolisMatrixPos 	= Eigen::MatrixXd::Identity(6,6);
	parameters.linDampMatrixPos 	= Eigen::MatrixXd::Identity(6,6);
	parameters.quadDampMatrixPos 	= Eigen::MatrixXd::Identity(6,6);
	parameters.thrustConfigMatrix 	= Eigen::MatrixXd::Identity(6,6);
	parameters.thrusterCoeffPWM.positive 	= 1;
	parameters.thrusterCoeffPWM.negative 	= 1;
	parameters.thrusterCoeffRPM.positive 	= 1;
	parameters.thrusterCoeffRPM.negative 	= 1;
	parameters.thrusterVoltage 	= 1;
}

