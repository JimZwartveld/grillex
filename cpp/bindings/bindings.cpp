#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "grillex/placeholder.hpp"
#include "grillex/node.hpp"
#include "grillex/node_registry.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/local_axes.hpp"
#include "grillex/beam_element.hpp"

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

    // ========================================================================
    // Phase 2: Beam Element Foundation
    // ========================================================================

    // BeamFormulation enum
    py::enum_<grillex::BeamFormulation>(m, "BeamFormulation",
        "Beam formulation type (Euler-Bernoulli or Timoshenko)")
        .value("EulerBernoulli", grillex::BeamFormulation::EulerBernoulli,
               "Classical beam theory (no shear deformation)")
        .value("Timoshenko", grillex::BeamFormulation::Timoshenko,
               "Beam theory with shear deformation effects")
        .export_values();

    // LocalAxes class
    py::class_<grillex::LocalAxes>(m, "LocalAxes",
        "Local coordinate system for beam elements")
        .def(py::init<const Eigen::Vector3d&, const Eigen::Vector3d&, double>(),
             py::arg("end_a"), py::arg("end_b"), py::arg("roll_angle") = 0.0,
             "Construct local axes from two points with optional roll angle")
        .def_readonly("rotation_matrix", &grillex::LocalAxes::rotation_matrix,
                      "3x3 rotation matrix (global to local)")
        .def_readonly("x_axis", &grillex::LocalAxes::x_axis,
                      "Local x-axis (along beam)")
        .def_readonly("y_axis", &grillex::LocalAxes::y_axis,
                      "Local y-axis")
        .def_readonly("z_axis", &grillex::LocalAxes::z_axis,
                      "Local z-axis")
        .def("to_local", &grillex::LocalAxes::to_local,
             py::arg("global"), "Transform vector from global to local coordinates")
        .def("to_global", &grillex::LocalAxes::to_global,
             py::arg("local"), "Transform vector from local to global coordinates")
        .def("__repr__", [](const grillex::LocalAxes &ax) {
            return "<LocalAxes x=" + std::to_string(ax.x_axis[0]) + "," +
                   std::to_string(ax.x_axis[1]) + "," + std::to_string(ax.x_axis[2]) + ">";
        });

    // BeamElement class
    py::class_<grillex::BeamElement>(m, "BeamElement",
        "3D Euler-Bernoulli beam element with 12 DOFs")
        .def(py::init<int, grillex::Node*, grillex::Node*,
                      grillex::Material*, grillex::Section*, double>(),
             py::arg("id"), py::arg("node_i"), py::arg("node_j"),
             py::arg("material"), py::arg("section"), py::arg("roll") = 0.0,
             "Construct a beam element")
        .def_readwrite("id", &grillex::BeamElement::id, "Element ID")
        .def_readonly("node_i", &grillex::BeamElement::node_i,
                      "First node", py::return_value_policy::reference)
        .def_readonly("node_j", &grillex::BeamElement::node_j,
                      "Second node", py::return_value_policy::reference)
        .def_readonly("material", &grillex::BeamElement::material,
                      "Material", py::return_value_policy::reference)
        .def_readonly("section", &grillex::BeamElement::section,
                      "Section", py::return_value_policy::reference)
        .def_readonly("local_axes", &grillex::BeamElement::local_axes,
                      "Local coordinate system")
        .def_readonly("length", &grillex::BeamElement::length,
                      "Element length [m]")
        .def_readwrite("offset_i", &grillex::BeamElement::offset_i,
                       "End offset at node i [m]")
        .def_readwrite("offset_j", &grillex::BeamElement::offset_j,
                       "End offset at node j [m]")
        .def("local_stiffness_matrix", &grillex::BeamElement::local_stiffness_matrix,
             py::arg("formulation") = grillex::BeamFormulation::EulerBernoulli,
             "Compute 12x12 local stiffness matrix")
        .def("transformation_matrix", &grillex::BeamElement::transformation_matrix,
             "Compute 12x12 transformation matrix")
        .def("global_stiffness_matrix", &grillex::BeamElement::global_stiffness_matrix,
             "Compute 12x12 global stiffness matrix")
        .def("local_mass_matrix", &grillex::BeamElement::local_mass_matrix,
             py::arg("formulation") = grillex::BeamFormulation::EulerBernoulli,
             "Compute 12x12 local mass matrix")
        .def("global_mass_matrix", &grillex::BeamElement::global_mass_matrix,
             "Compute 12x12 global mass matrix")
        .def("set_offsets", &grillex::BeamElement::set_offsets,
             py::arg("offset_i"), py::arg("offset_j"),
             "Set end offsets in local coordinates [m]")
        .def("has_offsets", &grillex::BeamElement::has_offsets,
             "Check if element has any offsets")
        .def("effective_length", &grillex::BeamElement::effective_length,
             "Compute effective beam length accounting for offsets [m]")
        .def("__repr__", [](const grillex::BeamElement &e) {
            return "<BeamElement id=" + std::to_string(e.id) +
                   " nodes=[" + std::to_string(e.node_i->id) + "," +
                   std::to_string(e.node_j->id) + "] L=" +
                   std::to_string(e.length) + ">";
        });
}
