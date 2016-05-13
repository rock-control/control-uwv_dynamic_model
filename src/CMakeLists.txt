rock_library(uwv_dynamic_model
    SOURCES RK4Integrator.cpp DynamicModel.cpp KinematicModel.cpp DynamicSimulator.cpp DynamicKinematicSimulator.cpp ModelSimulation.cpp
    HEADERS DataTypes.hpp RK4Integrator.hpp DynamicModel.hpp KinematicModel.hpp DynamicSimulator.hpp DynamicKinematicSimulator.hpp ModelSimulation.hpp
    DEPS_PKGCONFIG base-types base-lib)