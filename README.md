# uwv\_dynamic\_model: Implementation of a dynamic model for undewater vehicles

Assumptions:

 - A vehicle, considered a rigid body, is represented by a point in space, been this point the center of gravity (CoG).
 - The CoG is the origin of the right-hand side body-frame.
 - There is no representation of multiples objects, neither contact or interaction betwen them.
 - There is no representation of the environment (floor, surface, water currents, waves, etc)[^TODO].
 - The vehicle is considered to be surrended by fluid infinitly in all direction.
 - The gravity field is considered uniform.

## General presentation

The present dynamic model implements the formulation proposed mainly by [Fossen][].

It implements the dynamic and kinematic of a vehicle and its hydrodynamic effect. See [UWV Parameters](#parameters) section.
There are three induced hydrodynamic efforts:

 1. Added mass due to the inertia of the surrouding fluid.
 2. Damping effect.
 3. Restoring gravitational forces (weight and buoyancy).

The hydrodynamic effect is embedded in the parameters passed to the model.
It is required that the parameters are identified previously. (The code doesn't estimated them).

In order to add some flexibility on how each term would affect the model, the present library has 
a ModelType property. It allows the user to choose a specific formulation of the damping term or 
to ignore the Corilois effect for a simplified model, for example. See [UWV Parameters](#parameters) section.

The control efforts applied in the vechile are considered to be in the CoG.
In case a set of propellers need to be represented, the dynamic model just requires
the resultant effect (in terms of forces and torques) in the CoG.

In the 6 degree of freedom (DOF) formulation, the order of representing each DOF is:

 1. linear x
 2. linear y
 3. linear z
 4. angular x
 5. angular y
 6. angular z

The efforts applied as well as the velocities (both linear as angular) are represented in body-frame.
The position and the orientation are represented is fixed world-frame.
 
As the main focus of the present library is the dynamic effect, it is possile to make the simulator ignores 
the kinematic part of the formulation. ModelSimulator defines if the integration from velocities to pose and
orientation would be done or not. See [Simulation](#simulation) section.

## UWV Parameters <a id="parameters"></a>

The parameters that need to be set for the dynamic model are:

- Model Type
- Inertia Matrix
- Damping Matrices
- Distance Body to Center of Buoyancy
- Weight
- Buoyancy

### Model Type
 
Model Type defines which effects will be present in the model.
At the moment, three Model Types are defined:

 1. SIMPLE: 
	 It ignores the Coriolis effect and consider the Damping Matrices with two terms,
	the fist the linear damping and the second quadratic damping.
	It is based on [Smallwood][].

	> **Note:**
	For considering a decoupled model, one must set the matrices of inertia and damping as diagonal.


 2. COMPLEX:
	It consider the Coriollis effect and a Damping Matrices with six terms,
	all quadratic damping, been relative to the 3 linear and 3 angular DOF respectivelly.
	It is based on [McFarland][].
	
 3. INTERMEDIATE:
	It consider the Coriollis effect and a Damping Matrices with two terms,
	the first the linear and the second the quadratic damping.

Other ways to formulate the model could be defined.

### Inertia Matrix

The $6 \times 6$ inertia matrix consider both the inertia of the vehicle and the added mass.

$$ M = M_{vehicle} + M_{added\_mass} $$

with, 

$$ M_{vehicle} = 
 \left( \begin{array}{cc}
mass*Identity(3) & Zero(3) \\
Zero(3) & Inertia \end{array} \right) 
$$	

### Damping Matrices

The Damping Matrices is a vector of $6 \times 6$ matrices and his terms are defined according the Model Type.
For the SIMPLE and INTERMEDIATE model type, the damping is represented by a linear and a quadratic term.

$$ D = D_{linear}*v + D_{quadratic}*v*|v|$$

For the COMPLEX case, the damping is represented by six quadratic terms, one for each DOF.

$$ D = \left(\sum_{i=1}^{6}(|v_i|*D_i)\right) * v $$

### Weight

$$ Weight = Mass*gravity $$

### Buoyancy

Resultant buoyancy


## Simulation <a id="simulation"></a>

The simulation performs a integration of the states derivatives (acceleration and velocities)
to the states of interest (velocities and position/orientation) using for that a 4th order Runge-Kutta.
The parameters of the simulation are:

 - Model Simulator
 - Sampling Time 
 - Simulation per Cycle
 - Initial Time

### Model Simulator

Model Simulator defines if the integration of velocities to position/orientation will be made.
The two possibilities are:

 1. DYNAMIC: Only the dynamic is take in account. It integrates the accelerations to update the velocities. 
	For this case, updated orientation need to be set.

 2. DYNAMIC_KINEMATIC: It consider both the dynamic and the kinematic and do the intergration of all states.

### Sampling Time
 The step used in integration

### Simulation per Cycle
 Can be increased for precision purpose of the integration. 

	
[Fossen]:	https://scholar.google.com.br/citations?view_op=view_citation&hl=pt-BR&user=Sn8fzegAAAAJ&citation_for_view=Sn8fzegAAAAJ:u5HHmVD_uO8C
[Smallwood]: http://ieeexplore.ieee.org/document/1208328/?arnumber=1208328&tag=1
[McFarland]: http://ieeexplore.ieee.org/document/6631233/

[^TODO]: Future improvement on the library may include the definition of an environment, with a floor (inferior limit) and a surface (superior limit) and the presence of water current. 
It also may include the possibility to add more then one object and defines their interaction. 

