/***************************************************************************/
/*  Data types for an underwater vehicle 	                           */
/*                                                                         */
/* FILE --- uwv_dataTypes.h		                                   */
/*                                                                         */
/* PURPOSE --- Header file for a data types used in modeling a 		   */
/*             underwater vehicle. 					   */
/*                                                                         */
/*  Sankaranarayanan Natarajan                                             */
/*  sankar.natarajan@dfki.de                                               */
/*  DFKI - BREMEN 2011                                                     */
/***************************************************************************/
#ifndef _UWV_DATATYPES_H_
#define _UWV_DATATYPES_H_

#include <base/eigen.h>
#include <math.h>
#include <vector>

#define DOF 6			// degree of freedom


namespace underwaterVehicle
{
	struct Direction
	{
		double positive;						// thruster co-efficient in positive direction
		double negative;						// thruster co-efficient in negative direction
	};
	struct ThrusterCoefficientRPM
	{
		Direction surge;						// thruster co-efficient in surge	
		Direction sway;							// thruster co-efficient in sway	
		Direction heave;						// thruster co-efficient in heave	
		Direction roll;							// thruster co-efficient in roll	
		Direction pitch;						// thruster co-efficient in pitch	
		Direction yaw;							// thruster co-efficient in yaw			
	};
	struct ThrusterPWMModelCoefficient
	{
		double coefficient_a;
		double coefficient_b;
	};
	struct DirectionPWM
	{
		ThrusterPWMModelCoefficient positive;				// thruster co-efficient in positive direction
		ThrusterPWMModelCoefficient negative;				// thruster co-efficient in negative direction
	};
	struct ThrusterCoefficientPWM
	{
		DirectionPWM surge;						// thruster co-efficient in surge	
		DirectionPWM sway;						// thruster co-efficient in sway	
		DirectionPWM heave;						// thruster co-efficient in heave	
		DirectionPWM roll;						// thruster co-efficient in roll	
		DirectionPWM pitch;						// thruster co-efficient in pitch	
		DirectionPWM yaw;						// thruster co-efficient in yaw			
	};
	enum MOTION_MODE
        {
		SURGE = 0,
		SWAY  = 1,
		HEAVE = 2,
		ROLL  = 3,
		PITCH = 4,
		YAW   = 5,
		UNINITIALISED =6
        };
	struct ThrusterMapping
	{
		std::vector<MOTION_MODE> thruster_mapped_names;
		std::vector<double> thruster_value;

	 	void resize(int size)
		{	
			thruster_mapped_names.resize(size);
			std::fill(thruster_value.begin(), thruster_value.end(), UNINITIALISED);		
			thruster_value.resize(size);
			std::fill(thruster_value.begin(), thruster_value.end(), 6);
		}
	
	};	
	struct Parameters
	{			
		/************** UWV dimensional parameters **************/											
		base::Vector3d distance_body2centerofmass;			// vector position from the origin of body-fixed frame to center of mass in body-fixed frame				
		base::Vector3d distance_body2centerofbuoyancy;			// vector position from the origin of body-fixed frame to center of buoyancy				
		base::Vector3d distance_body2centerofgravity;			// vector position from the origin of body-fixed frame to center of gravity		
		/************** UWV physical parameters *****************/
		double uwv_mass;   						// total mass of the vehicle 
		double uwv_volume;	   					// total volume of the vehicle
		bool   uwv_float;						// to assume that the vehicle floats . i.e gravity == buoyancy
		/************** Environment parameters *****************/
		double waterDensity; 						// density of the water kg/m^3
		double gravity;							// gravity

		/************** Thrusters parameters ********************/
		ThrusterCoefficientRPM thruster_coefficient_rpm;
		ThrusterCoefficientPWM thruster_coefficient_pwm;
		double maxSurgePWM, minSurgePWM;
		double maxSwayPWM, minSwayPWM;
		double maxHeavePWM, minHeavePWM;
		double maxRollPWM, minRollPWM;
		double maxPitchPWM, minPitchPWM;
		double maxYawPWM, minYawPWM;
		double thrusterVoltage;						// thruster voltage - used for covnerting pwm to corresponding volatage
		
		std::vector<double> thruster_coefficients_pwm;
		std::vector<double> linear_thruster_coefficients_pwm;
		std::vector<double> square_thruster_coefficients_pwm;

		/************** UWV variables ***************************/		    	
		std::vector<double> mass_matrix;				// mass Matrix  w.r.t body-fixed frame 
		Direction massCoefficient[DOF];					// inertia + added mass for uncoupled 6 DOF
		Direction linDampCoeff[DOF];					// lineal damping coefficient
		Direction quadDampCoeff[DOF];					// quadratic damping coefficient
		std::vector<double> thruster_control_matrix;			// thruster Control Matrix(TCM)
		
		
		ThrusterMapping thrusters;
		
		int sim_per_cycle;						// number of RK4 simulations per sampling interval
		int plant_order;						// plant order - number of states in the model - generally 12 states -> 3 position, 3 orientation and 6 linear&angular velocity  
		int ctrl_order;							// ctrl order - number of controllable inputs
		double samplingtime;           					// Sampling time used in simulation
		double initial_condition[12];					// Initial conditions used for simulation - currently it 12 states
		double initial_time;              				// Initial time	used for simulation*/
	

	};								    	
	
};
#endif
