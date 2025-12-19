# C++ core library and Python bindings for Grillex

# Create a placeholder library target for the C++ core
add_library(grillex_core INTERFACE)
target_include_directories(grillex_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(grillex_core INTERFACE Eigen3::Eigen)

# Create the pybind11 module
pybind11_add_module(_grillex_cpp
    bindings/bindings.cpp
    src/placeholder.cpp
    src/node.cpp
    src/node_registry.cpp
    src/material.cpp
    src/section.cpp
    src/local_axes.cpp
    src/beam_element.cpp
    src/dof_handler.cpp
    src/assembler.cpp
    src/boundary_condition.cpp
    src/solver.cpp
    src/load_case.cpp
    src/model.cpp
    src/constraints.cpp
    src/internal_actions_axial.cpp
    src/internal_actions_bending_euler.cpp
    src/internal_actions_bending_timoshenko.cpp
)

target_link_libraries(_grillex_cpp PRIVATE grillex_core)

# Set output directory for the Python module
set_target_properties(_grillex_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../src/grillex"
)

# On Windows, also set the runtime output directory
if(WIN32)
    set_target_properties(_grillex_cpp PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../src/grillex"
    )
endif()
