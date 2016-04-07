rock_library(uwv_dynamic_model
    SOURCES UwvDynamicModel.cpp RK4Integrator.cpp UwvKinematicModel.cpp UwvModelSimulation.cpp
    HEADERS UwvDynamicModel.hpp RK4Integrator.hpp DataTypes.hpp UwvKinematicModel.hpp UwvModelSimulation.hpp
    DEPS_PKGCONFIG base-types base-lib)