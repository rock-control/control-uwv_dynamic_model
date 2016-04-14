#define BOOST_TEST_MODULE UWV_DYNAMIC_MODEL
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
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
UWVParameters loadRotationalParameters(void);
Vector3d calcOmega(base::Vector3d omega0, double t, double omegan);
Orientation calcOrientation(base::Orientation init_ori, double t, double wn, double wi, base::Vector3d init_ang_mom);

BOOST_AUTO_TEST_SUITE (CONSTRUCTOR)


BOOST_AUTO_TEST_CASE( normal )
{
    DynamicKinematicSimulator *simulator = new DynamicKinematicSimulator();
	ModelSimulation vehicle(0.01, 10, 0, simulator);

	vehicle.setUWVParameters(loadParameters());

	Vector6d controlInput(base::Vector6d::Zero());
	controlInput[0] = 2;

	Vector6d velocity(Vector6d::Zero());
	Orientation orientation(Orientation::Identity());

	for(int i=0; i<200; i++)
	    vehicle.sendEffort(controlInput);

	// For init velocity=0 and inertia term=1, quadratic and linear damping equal to 1, the steady state velocity in surge dof must be 1.
	BOOST_REQUIRE_CLOSE(vehicle.getPose().linear_velocity[0], 1, 1);
}


BOOST_AUTO_TEST_CASE( buoyancy )
{
    ModelSimulation vehicle;
    UWVParameters parameters = loadParameters();
    parameters.buoyancy = 3;
    parameters.weight = 1;

    vehicle.setUWVParameters(parameters);

    // No forces being applied
    Vector6d controlInput(base::Vector6d::Zero());

    for(int i=0 ; i<200; i++)
        vehicle.sendEffort(controlInput);

    // For init velocity=0 and inertia term=1, quadratic and linear damping equal to 1 and gravitational force equal 2, the steady state velocity in heave dof must be 1.
    BOOST_REQUIRE_CLOSE(vehicle.getPose().linear_velocity[2], 1, 1);

}

BOOST_AUTO_TEST_CASE(constant_yaw_velocity )
{
    double deltaT = 0.1;
    // One hour simulation
    double t = 60*60*1;

    DynamicKinematicSimulator *simulator = new DynamicKinematicSimulator();
    ModelSimulation vehicle(deltaT, 10, 0, simulator);

    UWVParameters parameters = loadRotationalParameters();
    vehicle.setUWVParameters(parameters);

    // Constant velocity in yaw DOF
    Vector3d omega0(0, 0, 0.10);
    PoseVelocityState init_state;
    init_state.angular_velocity = omega0;
    vehicle.setPose(init_state);

    // Final orientation at time t after constant angular velocity omega0 in yaw DOF
    // q = qw * q(0); qw: rotation about axis w/||w|| through the angle t*||w||
    Orientation orientation = Eigen::AngleAxisd(omega0.norm() * t, omega0/omega0.norm()) * Orientation::Identity();

    // Initial yaw angle
    BOOST_REQUIRE_EQUAL(getYaw(vehicle.getPose().orientation), 0);

    // No torque been applied
    Vector6d controlInput(base::Vector6d::Zero());

    // Model simulation
    for (int i = 0; i < t/deltaT; i++)
        vehicle.sendEffort(controlInput);

    // Compare simulation result with analytical one.
    BOOST_CHECK_EQUAL( getRoll(vehicle.getPose().orientation), getRoll(orientation));
    BOOST_CHECK_EQUAL( getPitch(vehicle.getPose().orientation), getPitch(orientation));
    //Yaw angle comparison
    BOOST_CHECK_CLOSE( getYaw(vehicle.getPose().orientation) / getYaw(orientation), 1, 10^-10);

}


BOOST_AUTO_TEST_CASE(constant_yaw_velocity_diff_deltaT)
{
    double deltaT = 0.2;
    // One hour simulation
    double t = 60*60*1;

    DynamicKinematicSimulator *simulator = new DynamicKinematicSimulator();
    ModelSimulation vehicle(deltaT, 10, 0, simulator);

    UWVParameters parameters = loadRotationalParameters();
    vehicle.setUWVParameters(parameters);

    // Constant velocity in yaw DOF
    Vector3d omega0(0, 0, 0.10);
    PoseVelocityState init_state;
    init_state.angular_velocity = omega0;
    vehicle.setPose(init_state);

    // Final orientation at time t after constant angular velocity omega0 in yaw DOF
    // q = qw * q(0); qw: rotation about axis w/||w|| through the angle t*||w||
    Orientation orientation = Eigen::AngleAxisd(omega0.norm() * t, omega0/omega0.norm()) * Orientation::Identity();

    // Initial yaw angle
    BOOST_REQUIRE_EQUAL(getYaw(vehicle.getPose().orientation), 0);

    // No torque been applied
    Vector6d controlInput(base::Vector6d::Zero());

    // Model simulation
    for (int i = 0; i < t/deltaT; i++)
        vehicle.sendEffort(controlInput);

    // Compare simulation result with analytical one.
    BOOST_CHECK_EQUAL( getRoll(vehicle.getPose().orientation), getRoll(orientation));
    BOOST_CHECK_EQUAL( getPitch(vehicle.getPose().orientation), getPitch(orientation));
    //Yaw angle comparison
    BOOST_CHECK_CLOSE( getYaw(vehicle.getPose().orientation) / getYaw(orientation), 1, 10^-10);

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
    DynamicKinematicSimulator *simulator = new DynamicKinematicSimulator();
    ModelSimulation vehicle(deltaT, 10, 0, simulator);

    UWVParameters parameters = loadRotationalParameters();
    parameters.inertiaMatrix << 0,   0,   0,   0,   0,    0,
                                0,   0,   0,   0,   0,    0,
                                0,   0,   0,   0,   0,    0,
                                0,   0,   0,   Jt,   0,    0,
                                0,   0,   0,   0,   Jt,    0,
                                0,   0,   0,   0,   0,    J3;
    vehicle.setUWVParameters(parameters);

    // Initial angular velocity
    Vector3d omega0(0.05, 0, 0.01);
    PoseVelocityState init_state;
    init_state.angular_velocity = omega0;
    vehicle.setPose(init_state);


    // Body nutation rate
    double wn = omega0[2]*(Jt - J3)/Jt;
    // Initial angular momentum (in this example it should be constant once there will be no torques being applied)
    Vector3d init_ang_mom = parameters.inertiaMatrix.bottomRightCorner<3,3>()*omega0;
    // Inertial nutation rate
    double wi = init_ang_mom.norm()/Jt;

    // Final angular velocity at time t
    Vector3d omega = calcOmega(omega0, t, wn);
    // Final orientation at time t
    Orientation final_orientation = calcOrientation(Orientation::Identity(), t, wn, wi, init_ang_mom);

    // No torque been applied
    Vector6d controlInput(Vector6d::Zero());

    // Metrics
    // error_quaternion = qs*qa^-1; qs:=simulation; qa:=analytical solution
    // erro_quaternion = [error_imaginary^t error_real]^t
    // small angles: error_imaginary =~ error+_angles/2
    //
    Vector3d error_angles = Vector3d::Zero();
    Orientation error_quaternion;

    // Simulation
    for (int i = 1; i <= t/deltaT; i++)
    {
        vehicle.sendEffort(controlInput);
        error_quaternion = vehicle.getPose().orientation*calcOrientation(Orientation::Identity(), i*deltaT, wn, wi, init_ang_mom).inverse();
        for(size_t i=0; i<3; i++)
        {
            // Get maximum value for each DOF
            if(error_quaternion.coeffs()[i]*2 > error_angles[i])
                    error_angles[i] = error_quaternion.coeffs()[i]*2;
        }
    }

    for(size_t i=0; i<3; i++)
            BOOST_CHECK_SMALL(error_angles[i], 0.000000000001);

    BOOST_CHECK_CLOSE( getRoll(vehicle.getPose().orientation) / getRoll(final_orientation), 1, 10^-10);
    BOOST_CHECK_CLOSE( getPitch(vehicle.getPose().orientation) / getPitch(final_orientation), 1, 10^-10);
    BOOST_CHECK_CLOSE( getYaw(vehicle.getPose().orientation) /  getYaw(final_orientation), 1, 10^-10);
    BOOST_CHECK_CLOSE(vehicle.getPose().angular_velocity[0] / omega[0], 1, 10^-10 );

}


BOOST_AUTO_TEST_SUITE_END()



uwv_dynamic_model::UWVParameters loadParameters(void)
{
    uwv_dynamic_model::UWVParameters parameters;
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


uwv_dynamic_model::UWVParameters loadRotationalParameters(void)
{
    uwv_dynamic_model::UWVParameters parameters;
    parameters.modelType = uwv_dynamic_model::COMPLEX;
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
