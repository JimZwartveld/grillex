# CMakeLists.txt for C++ tests using Catch2

# Include Catch2's CMake integration
list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)

# Test executable for boundary condition tests
add_executable(test_boundary_conditions
    test_boundary_conditions.cpp
)
target_link_libraries(test_boundary_conditions PRIVATE
    grillex_core_static
    Catch2::Catch2WithMain
)

# Test executable for eigenvalue solver tests
add_executable(test_eigenvalue_solver
    test_eigenvalue_solver.cpp
)
target_link_libraries(test_eigenvalue_solver PRIVATE
    grillex_core_static
    Catch2::Catch2WithMain
)

# Test executable for singularity diagnostics tests
add_executable(test_singularity_diagnostics
    test_singularity_diagnostics.cpp
)
target_link_libraries(test_singularity_diagnostics PRIVATE
    grillex_core_static
    Catch2::Catch2WithMain
)

# Test executable for constraint handler tests
add_executable(test_constraints
    test_constraints.cpp
)
target_link_libraries(test_constraints PRIVATE
    grillex_core_static
    Catch2::Catch2WithMain
)

# Test executable for internal actions tests
add_executable(test_internal_actions
    test_internal_actions.cpp
)
target_link_libraries(test_internal_actions PRIVATE
    grillex_core_static
    Catch2::Catch2WithMain
)

# Test executable for warping tests
add_executable(test_warping
    test_warping.cpp
)
target_link_libraries(test_warping PRIVATE
    grillex_core_static
    Catch2::Catch2WithMain
)

# Register tests with CTest using Catch2's discovery
catch_discover_tests(test_boundary_conditions)
catch_discover_tests(test_eigenvalue_solver)
catch_discover_tests(test_singularity_diagnostics)
catch_discover_tests(test_constraints)
catch_discover_tests(test_internal_actions)
catch_discover_tests(test_warping)
