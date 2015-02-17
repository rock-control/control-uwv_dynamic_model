rock_library(uwv_dynamic_model
    SOURCES uwv_dynamic_model.cpp RK4Integrator.cpp
    HEADERS uwv_dynamic_model.hpp RK4Integrator.hpp uwv_dataTypes.hpp
    DEPS_PKGCONFIG base-types)