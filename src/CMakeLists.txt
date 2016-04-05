rock_library(uwv_dynamic_model
    SOURCES uwv_dynamic_model.cpp RK4Integrator.cpp uwv_kinematic_model.cpp uwv_model_simulation.cpp
    HEADERS uwv_dynamic_model.hpp RK4Integrator.hpp uwv_dataTypes.hpp uwv_kinematic_model.hpp uwv_model_simulation.hpp
    DEPS_PKGCONFIG base-types base-lib)