#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "grillex/placeholder.hpp"

namespace py = pybind11;

/**
 * Grillex C++ Python bindings module.
 * This module exposes C++ functionality to Python via pybind11.
 */
PYBIND11_MODULE(_grillex_cpp, m) {
    m.doc() = "Grillex C++ core module - Structural analysis FE engine";

    // Placeholder functions for testing
    m.def("get_greeting", &grillex::get_greeting,
          "Returns a greeting message from the C++ core");

    m.def("add_numbers", &grillex::add_numbers,
          py::arg("a"), py::arg("b"),
          "Add two numbers together (test function)");

    // Version information
    m.attr("__version__") = "2.0.0-dev";
}
