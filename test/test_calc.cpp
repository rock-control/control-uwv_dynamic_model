#define BOOST_TEST_MODULE UWV_DYNAMIC_MODEL
#include <boost/test/included/unit_test.hpp>
#include <uwv_dynamic_model/uwv_model_simulation.hpp>
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
underwaterVehicle::UWVParameters loadRotationalParameters(void);
base::Vector3d calcOmega(base::Vector3d omega0, double t, double omegan);
base::Orientation calcOrientation(base::Orientation init_ori, double t, double wn, double wi, base::Vector3d init_ang_mom);

BOOST_AUTO_TEST_SUITE (CONSTRUCTOR)


BOOST_AUTO_TEST_CASE( normal )
{
	underwaterVehicle::ModelSimulation vehicle;
	vehicle.setUWVParameters(loadParameters());

	base::LinearAngular6DCommand controlInput;

	controlInput.linear = base::Vector3d(2,0,0);
	controlInput.angular = base::Vector3d(0,0,0);

	base::Vector6d velocity(base::Vector6d::Zero());
	base::Orientation orientation(base::Orientation::Identity());

	for(int i=0; i<200; i++)
	    vehicle.sendEffort(controlInput);

	// For init velocity=0 and inertia term=1, quadratic and linear damping equal to 1, the steady state velocity in surge dof must be 1.
	BOOST_REQUIRE_CLOSE(vehicle.getPose().linear_velocity[0], 1, 1);
}


BOOST_AUTO_TEST_CASE( buoyancy )
{
    underwaterVehicle::ModelSimulation vehicle;
    underwaterVehicle::UWVParameters parameters = loadParameters();
    parameters.buoyancy = 3;
    parameters.weight = 1;

    vehicle.setUWVParameters(parameters);

    // No forces being applied
    base::LinearAngular6DCommand controlInput;
    controlInput.linear = base::Vector3d(0,0,0);
    controlInput.angular = base::Vector3d(0,0,0);

    for(int i; i<200; i++)
        vehicle.sendEffort(controlInput);

    // For init velocity=0 and inertia term=1, quadratic and linear damping equal to 1 and gravitational force equal 2, the steady state velocity in heave dof must be 1.
    BOOST_REQUIRE_CLOSE(vehicle.getPose().linear_velocity[2], 1, 1);
    // For init velocity=0 and inertia term=1, acceleration in the heave dof must be equal to gravity force of 2

}

BOOST_AUTO_TEST_CASE(constant_yaw_velocity )
{
    double deltaT = 0.1;
    // One hour simulation
    double t = 60*60*1;
    underwaterVehicle::ModelSimulation vehicle(deltaT, 10, 0, true);

    underwaterVehicle::UWVParameters parameters = loadRotationalParameters();
    vehicle.setUWVParameters(parameters);

    // Constant velocity in yaw DOF
    base::Vector3d omega0(0, 0, 0.10);
    underwaterVehicle::PoseVelocityState init_state;
    init_state.angular_velocity = omega0;
    vehicle.setPose(init_state);

    // Final orientation at time t after constant angular velocity omega0 in yaw DOF
    // q = qw * q(0); qw: rotation about axis w/||w|| through the angle t*||w||
    base::Orientation orientation = Eigen::AngleAxisd(omega0.norm() * t, omega0/omega0.norm()) * base::Orientation::Identity();

    // Initial yaw angle
    BOOST_REQUIRE_EQUAL(base::getYaw(vehicle.getPose().orientation), 0);

    // No torque been applied
    base::LinearAngular6DCommand controlInput;
    controlInput.linear = base::Vector3d::Zero();
    controlInput.angular = base::Vector3d::Zero();

    // Model simulation
    for (int i = 0; i < t/deltaT; i++)
        vehicle.sendEffort(controlInput);

    std::cout << std::endl << "Constant Yaw velocity" << std::endl;
    std::cout << "vehicle roll: " << base::getRoll(vehicle.getPose().orientation) << ". analytical: " << base::getRoll(orientation) << std::endl;
    std::cout << "vehicle pitch: " << base::getPitch(vehicle.getPose().orientation) << ". analytical: " << base::getPitch(orientation) << std::endl;
    std::cout << "vehicle yaw: " << base::getYaw(vehicle.getPose().orientation) << ". analytical: " << base::getYaw(orientation) << std::endl;

    // Compare simulation result with analytical one.
    BOOST_REQUIRE_EQUAL(base::getRoll(vehicle.getPose().orientation), base::getRoll(orientation));
    BOOST_REQUIRE_EQUAL(base::getPitch(vehicle.getPose().orientation), base::getPitch(orientation));
    //Yaw angle comparison
    BOOST_REQUIRE_CLOSE(base::getYaw(vehicle.getPose().orientation)/base::getYaw(orientation), 1, 10^-9);

}

BOOST_AUTO_TEST_CASE( angular )
{
    // Example from:
    // Andrle, Michael S., and John L. Crassidis. "Geometric integration of quaternions." Journal of Guidance, Control, and Dynamics 36.6 (2013): 1762-1767.
    double deltaT = 0.1;
    // Simulation of 1 hour.
    double t = 60*60*1;
    // Inertia parameters
    double Jt = 200;
    double J3 = 100;
    underwaterVehicle::ModelSimulation vehicle(deltaT, 10, 0, true);

    underwaterVehicle::UWVParameters parameters = loadRotationalParameters();
    parameters.inertiaMatrix << 0,   0,   0,   0,   0,    0,
                                0,   0,   0,   0,   0,    0,
                                0,   0,   0,   0,   0,    0,
                                0,   0,   0,   Jt,   0,    0,
                                0,   0,   0,   0,   Jt,    0,
                                0,   0,   0,   0,   0,    J3;
    vehicle.setUWVParameters(parameters);

    // Initial angular velocity
    base::Vector3d omega0(0.05, 0, 0.01);
    underwaterVehicle::PoseVelocityState init_state;
    init_state.angular_velocity = omega0;
    vehicle.setPose(init_state);


    // Body nutation rate
    double wn = omega0[2]*(Jt - J3)/Jt;
    // Initial angular momentum (in this example it should be constant once there will be no torques being applied)
    base::Vector3d init_ang_mom = parameters.inertiaMatrix.bottomRightCorner<3,3>()*omega0;
    // Inertial nutation rate
    double wi = init_ang_mom.norm()/Jt;

    // Final angular velocity at time t
    base::Vector3d omega = calcOmega(omega0, t, wn);
    // Final orientation at time t
    base::Orientation orientation = calcOrientation(base::Orientation::Identity(), t, wn, wi, init_ang_mom);

    // No torque been applied
    base::LinearAngular6DCommand controlInput;
    controlInput.linear = base::Vector3d(0,0,0);
    controlInput.angular = base::Vector3d(0,0,0);

    // Metrics
    // error_quaternion = qs*qa^-1; qs:=simulation; qa:=analytical solution
    // erro_quaternion = [error_imaginary^t error_real]^t
    // small angles: error_imaginary =~ error+_angles/2
    //
    base::Vector3d error_angles = base::Vector3d::Zero();
    base::Orientation error_quaternion;

    // Simulation
    for (int i = 1; i <= t/deltaT; i++)
    {
        vehicle.sendEffort(controlInput);
        error_quaternion = vehicle.getPose().orientation*calcOrientation(base::Orientation::Identity(), i*deltaT, wn, wi, init_ang_mom).inverse();
        for(size_t i=0; i<3; i++)
        {
            // Get maximum value for each DOF
            if(error_quaternion.coeffs()[i]*2 > error_angles[i])
                 {
                    error_angles[i] = error_quaternion.coeffs()[i]*2;
                 }
        }
    }

    std::cout << std::endl << "Angular case" << std::endl;
    for(size_t i=0; i<3; i++)
        std::cout << "log(max(error_angle["<< i <<"])) = " << log(error_angles[i]) << std::endl;

    std::cout << "error roll: " << base::getRoll(error_quaternion) << std::endl;
    std::cout << "error pitch: " << base::getPitch(error_quaternion) << std::endl;
    std::cout << "error yaw: " << base::getYaw(error_quaternion) << std::endl;

    std::cout << "vehicle roll: " << base::getRoll(vehicle.getPose().orientation) << ". analytical: " << base::getRoll(orientation) << std::endl;
    std::cout << "vehicle pitch: " << base::getPitch(vehicle.getPose().orientation) << ". analytical: " << base::getPitch(orientation) << std::endl;
    std::cout << "vehicle yaw: " << base::getYaw(vehicle.getPose().orientation) << ". analytical: " << base::getYaw(orientation) << std::endl;

    BOOST_REQUIRE_CLOSE(base::getRoll(vehicle.getPose().orientation)/base::getRoll(orientation), 1, 10^-5);
    BOOST_REQUIRE_CLOSE(base::getPitch(vehicle.getPose().orientation)/base::getPitch(orientation), 1, 10^-5);
    BOOST_REQUIRE_CLOSE(base::getYaw(vehicle.getPose().orientation)/base::getYaw(orientation), 1, 10^-5);
    BOOST_REQUIRE_CLOSE(vehicle.getPose().angular_velocity[0]/omega[0], 1, 10^-5 );

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


underwaterVehicle::UWVParameters loadRotationalParameters(void)
{
    underwaterVehicle::UWVParameters parameters;
    parameters.modelType = underwaterVehicle::COMPLEX;
    parameters.inertiaMatrix << 0,   0,   0,   0,   0,    0,
                                   0,   0,   0,   0,   0,    0,
                                   0,   0,   0,   0,   0,    0,
                                   0,   0,   0,   1,   0,    0,
                                   0,   0,   0,   0,   1,    0,
                                   0,   0,   0,   0,   0,    1;
    parameters.dampMatrices.resize(6);
    for(size_t i=0; i<parameters.dampMatrices.size(); i++)
    {
        parameters.dampMatrices[i] << 0,   0,   0,   0,   0,    0,
                                        0,   0,   0,   0,   0,    0,
                                        0,   0,   0,   0,   0,    0,
                                        0,   0,   0,   0,   0,    0,
                                        0,   0,   0,   0,   0,    0,
                                        0,   0,   0,   0,   0,    0;
    }
    return parameters;
}


base::Vector3d calcOmega(base::Vector3d omega0, double t, double omegan)
{
    base::Vector3d ome;
    ome[0] = omega0[0]*cos(omegan*t) + omega0[1]*sin(omegan*t);
    ome[1] = omega0[1]*cos(omegan*t) - omega0[0]*sin(omegan*t);
    ome[2] = omega0[2];
    return ome;
}

base::Orientation calcOrientation(base::Orientation init_ori, double t, double wn, double wi, base::Vector3d init_ang_mom)
{
    base::Vector3d h0 = init_ang_mom/init_ang_mom.norm();
    double alpha = wn*t/2;
    double betha = wi*t/2;

    base::Vector4d y;
    y[0] = h0[0]*cos(alpha)*sin(betha) + h0[1]*sin(alpha)*sin(betha);
    y[1] = h0[1]*cos(alpha)*sin(betha) - h0[0]*sin(alpha)*sin(betha);
    y[2] = h0[2]*cos(alpha)*sin(betha) + sin(alpha)*cos(betha);
    y[3] = cos(alpha)*cos(betha) - h0[2]*sin(alpha)*sin(betha);

    base::Orientation Y(y[3], y[0], y[1], y[2] );
    return Y*init_ori;
}
