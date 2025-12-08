#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "grillex/placeholder.hpp"
#include "grillex/node.hpp"
#include "grillex/node_registry.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"

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

    // ========================================================================
    // Phase 1: Core Data Structures
    // ========================================================================

    // Node class
    py::class_<grillex::Node>(m, "Node",
        "Node represents a point in the structural model with up to 6 DOFs")
        .def(py::init<int, double, double, double>(),
             py::arg("id"), py::arg("x"), py::arg("y"), py::arg("z"),
             "Construct a node at position (x, y, z)")
        .def_readwrite("id", &grillex::Node::id, "Node ID")
        .def_readwrite("x", &grillex::Node::x, "X coordinate [m]")
        .def_readwrite("y", &grillex::Node::y, "Y coordinate [m]")
        .def_readwrite("z", &grillex::Node::z, "Z coordinate [m]")
        .def_readwrite("dof_active", &grillex::Node::dof_active,
                       "DOF activation flags [UX, UY, UZ, RX, RY, RZ]")
        .def_readwrite("global_dof_numbers", &grillex::Node::global_dof_numbers,
                       "Global DOF numbers (assigned during assembly)")
        .def("position", &grillex::Node::position,
             "Get position as Eigen::Vector3d")
        .def("__repr__", [](const grillex::Node &n) {
            return "<Node id=" + std::to_string(n.id) +
                   " pos=(" + std::to_string(n.x) + ", " +
                              std::to_string(n.y) + ", " +
                              std::to_string(n.z) + ")>";
        });

    // NodeRegistry class
    py::class_<grillex::NodeRegistry>(m, "NodeRegistry",
        "Registry for managing nodes with automatic merging based on tolerance")
        .def(py::init<double>(), py::arg("tolerance") = 1e-6,
             "Construct a NodeRegistry with specified tolerance [m]")
        .def("get_or_create_node", &grillex::NodeRegistry::get_or_create_node,
             py::arg("x"), py::arg("y"), py::arg("z"),
             "Get existing node or create new one at (x, y, z)",
             py::return_value_policy::reference_internal)
        .def("get_node_by_id", &grillex::NodeRegistry::get_node_by_id,
             py::arg("id"), "Get node by ID",
             py::return_value_policy::reference_internal)
        .def("all_nodes", [](const grillex::NodeRegistry &reg) {
            py::list result;
            for (const auto& node : reg.all_nodes()) {
                result.append(node.get());
            }
            return result;
        }, "Get all nodes in registry", py::return_value_policy::reference_internal)
        .def("set_tolerance", &grillex::NodeRegistry::set_tolerance,
             py::arg("tolerance"), "Set distance tolerance [m]")
        .def("get_tolerance", &grillex::NodeRegistry::get_tolerance,
             "Get distance tolerance [m]")
        .def("__len__", [](const grillex::NodeRegistry &reg) {
            return reg.all_nodes().size();
        })
        .def("__repr__", [](const grillex::NodeRegistry &reg) {
            return "<NodeRegistry nodes=" + std::to_string(reg.all_nodes().size()) +
                   " tol=" + std::to_string(reg.get_tolerance()) + ">";
        });

    // Material class
    py::class_<grillex::Material>(m, "Material",
        "Material properties for structural elements")
        .def(py::init<int, std::string, double, double, double>(),
             py::arg("id"), py::arg("name"), py::arg("E"),
             py::arg("nu"), py::arg("rho"),
             "Construct a material with E [kN/m²], nu, rho [mT/m³]")
        .def_readwrite("id", &grillex::Material::id, "Material ID")
        .def_readwrite("name", &grillex::Material::name, "Material name")
        .def_readwrite("E", &grillex::Material::E, "Young's modulus [kN/m²]")
        .def_readwrite("G", &grillex::Material::G, "Shear modulus [kN/m²]")
        .def_readwrite("nu", &grillex::Material::nu, "Poisson's ratio")
        .def_readwrite("rho", &grillex::Material::rho, "Density [mT/m³]")
        .def_static("compute_G", &grillex::Material::compute_G,
                    py::arg("E"), py::arg("nu"),
                    "Compute shear modulus from E and nu")
        .def("__repr__", [](const grillex::Material &m) {
            return "<Material '" + m.name + "' E=" + std::to_string(m.E) +
                   " nu=" + std::to_string(m.nu) + ">";
        });

    // Section class
    py::class_<grillex::Section>(m, "Section",
        "Cross-section properties for beam elements")
        .def(py::init<int, std::string, double, double, double, double>(),
             py::arg("id"), py::arg("name"), py::arg("A"),
             py::arg("Iy"), py::arg("Iz"), py::arg("J"),
             "Construct a section with area [m²] and moments [m⁴]")
        .def_readwrite("id", &grillex::Section::id, "Section ID")
        .def_readwrite("name", &grillex::Section::name, "Section name")
        .def_readwrite("A", &grillex::Section::A, "Cross-sectional area [m²]")
        .def_readwrite("Iy", &grillex::Section::Iy, "Second moment about y [m⁴]")
        .def_readwrite("Iz", &grillex::Section::Iz, "Second moment about z [m⁴]")
        .def_readwrite("J", &grillex::Section::J, "Torsional constant [m⁴]")
        .def_readwrite("Iw", &grillex::Section::Iw, "Warping constant [m⁶]")
        .def_readwrite("Asy", &grillex::Section::Asy, "Shear area y [m²]")
        .def_readwrite("Asz", &grillex::Section::Asz, "Shear area z [m²]")
        .def_readwrite("zy_top", &grillex::Section::zy_top, "Fibre distance y-top [m]")
        .def_readwrite("zy_bot", &grillex::Section::zy_bot, "Fibre distance y-bottom [m]")
        .def_readwrite("zz_top", &grillex::Section::zz_top, "Fibre distance z-top [m]")
        .def_readwrite("zz_bot", &grillex::Section::zz_bot, "Fibre distance z-bottom [m]")
        .def("set_warping_constant", &grillex::Section::set_warping_constant,
             py::arg("Iw"), "Set warping constant [m⁶]")
        .def("set_shear_areas", &grillex::Section::set_shear_areas,
             py::arg("Asy"), py::arg("Asz"), "Set shear areas [m²]")
        .def("set_fibre_distances", &grillex::Section::set_fibre_distances,
             py::arg("zy_top"), py::arg("zy_bot"),
             py::arg("zz_top"), py::arg("zz_bot"),
             "Set distances to extreme fibres [m]")
        .def("__repr__", [](const grillex::Section &s) {
            return "<Section '" + s.name + "' A=" + std::to_string(s.A) + ">";
        });
}
