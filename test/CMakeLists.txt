rock_testsuite(unit_test test.cpp
    DEPS_PKGCONFIG base-types
    DEPS uwv_dynamic_model)
    
rock_testsuite(values_test test_calc.cpp
    DEPS_PKGCONFIG base-types
    DEPS uwv_dynamic_model)