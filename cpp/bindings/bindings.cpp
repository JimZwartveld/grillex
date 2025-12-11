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
#include "grillex/dof_handler.hpp"
#include "grillex/assembler.hpp"
#include "grillex/boundary_condition.hpp"

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
        "Node represents a point in the structural model with up to 7 DOFs (6 standard + 1 optional warping)")
        .def(py::init<int, double, double, double>(),
             py::arg("id"), py::arg("x"), py::arg("y"), py::arg("z"),
             "Construct a node at position (x, y, z)")
        .def_readwrite("id", &grillex::Node::id, "Node ID")
        .def_readwrite("x", &grillex::Node::x, "X coordinate [m]")
        .def_readwrite("y", &grillex::Node::y, "Y coordinate [m]")
        .def_readwrite("z", &grillex::Node::z, "Z coordinate [m]")
        .def_readwrite("dof_active", &grillex::Node::dof_active,
                       "DOF activation flags [UX, UY, UZ, RX, RY, RZ, WARP]")
        .def_readwrite("global_dof_numbers", &grillex::Node::global_dof_numbers,
                       "Global DOF numbers (assigned during assembly)")
        .def("position", &grillex::Node::position,
             "Get position as Eigen::Vector3d")
        .def("enable_warping_dof", &grillex::Node::enable_warping_dof,
             "Enable the warping DOF (7th DOF) for this node")
        .def("has_warping_dof", &grillex::Node::has_warping_dof,
             "Check if warping DOF is enabled")
        .def("num_active_dofs", &grillex::Node::num_active_dofs,
             "Get number of active DOFs at this node (6 or 7)")
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
        .def_readwrite("requires_warping", &grillex::Section::requires_warping,
                       "Indicates if section requires warping analysis")
        .def_readwrite("omega_max", &grillex::Section::omega_max,
                       "Maximum sectorial coordinate [m²]")
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
        .def("enable_warping", &grillex::Section::enable_warping,
             py::arg("Iw"), py::arg("omega_max") = 0.0,
             "Enable warping analysis with warping constant and max sectorial coordinate")
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

    // BeamConfig struct
    py::class_<grillex::BeamConfig>(m, "BeamConfig",
        "Configuration for beam element creation")
        .def(py::init<>(), "Construct with default configuration (Euler-Bernoulli, no warping)")
        .def_readwrite("formulation", &grillex::BeamConfig::formulation,
                       "Beam formulation type")
        .def_readwrite("include_warping", &grillex::BeamConfig::include_warping,
                       "Include warping DOF (7th DOF at each end)")
        .def_readwrite("include_shear_deformation", &grillex::BeamConfig::include_shear_deformation,
                       "Alias for Timoshenko formulation")
        .def("get_formulation", &grillex::BeamConfig::get_formulation,
             "Get the effective beam formulation")
        .def("__repr__", [](const grillex::BeamConfig &c) {
            std::string form = (c.get_formulation() == grillex::BeamFormulation::EulerBernoulli)
                             ? "EulerBernoulli" : "Timoshenko";
            return "<BeamConfig formulation=" + form +
                   " include_warping=" + (c.include_warping ? "True" : "False") + ">";
        });

    // BeamElementBase abstract class
    py::class_<grillex::BeamElementBase>(m, "BeamElementBase",
        "Abstract base class for beam elements with different formulations")
        .def("compute_local_stiffness", &grillex::BeamElementBase::compute_local_stiffness,
             "Compute local stiffness matrix (12x12 or 14x14)")
        .def("compute_local_mass", &grillex::BeamElementBase::compute_local_mass,
             "Compute local mass matrix (12x12 or 14x14)")
        .def("compute_transformation", &grillex::BeamElementBase::compute_transformation,
             "Compute transformation matrix (12x12 or 14x14)")
        .def("num_dofs", &grillex::BeamElementBase::num_dofs,
             "Get number of DOFs (12 for standard, 14 for warping)")
        .def("get_formulation", &grillex::BeamElementBase::get_formulation,
             "Get the beam formulation used")
        .def("has_warping", &grillex::BeamElementBase::has_warping,
             "Check if element includes warping DOF");

    // EndRelease struct
    py::class_<grillex::EndRelease>(m, "EndRelease",
        "End release configuration for beam elements")
        .def(py::init<>(), "Construct with no releases (fully fixed)")
        .def_readwrite("release_ux_i", &grillex::EndRelease::release_ux_i,
                       "Axial release at end i (sliding joint)")
        .def_readwrite("release_uy_i", &grillex::EndRelease::release_uy_i,
                       "Shear y release at end i")
        .def_readwrite("release_uz_i", &grillex::EndRelease::release_uz_i,
                       "Shear z release at end i")
        .def_readwrite("release_rx_i", &grillex::EndRelease::release_rx_i,
                       "Torsion release at end i")
        .def_readwrite("release_ry_i", &grillex::EndRelease::release_ry_i,
                       "Moment about y release at end i")
        .def_readwrite("release_rz_i", &grillex::EndRelease::release_rz_i,
                       "Moment about z release at end i")
        .def_readwrite("release_warp_i", &grillex::EndRelease::release_warp_i,
                       "Warping release at end i (free to warp)")
        .def_readwrite("release_ux_j", &grillex::EndRelease::release_ux_j,
                       "Axial release at end j (sliding joint)")
        .def_readwrite("release_uy_j", &grillex::EndRelease::release_uy_j,
                       "Shear y release at end j")
        .def_readwrite("release_uz_j", &grillex::EndRelease::release_uz_j,
                       "Shear z release at end j")
        .def_readwrite("release_rx_j", &grillex::EndRelease::release_rx_j,
                       "Torsion release at end j")
        .def_readwrite("release_ry_j", &grillex::EndRelease::release_ry_j,
                       "Moment about y release at end j")
        .def_readwrite("release_rz_j", &grillex::EndRelease::release_rz_j,
                       "Moment about z release at end j")
        .def_readwrite("release_warp_j", &grillex::EndRelease::release_warp_j,
                       "Warping release at end j (free to warp)")
        .def("release_moment_i", &grillex::EndRelease::release_moment_i,
             "Release both bending moments at end i (pin connection)")
        .def("release_moment_j", &grillex::EndRelease::release_moment_j,
             "Release both bending moments at end j (pin connection)")
        .def("release_all_rotations_i", &grillex::EndRelease::release_all_rotations_i,
             "Release all rotations at end i (true pin/ball joint)")
        .def("release_all_rotations_j", &grillex::EndRelease::release_all_rotations_j,
             "Release all rotations at end j (true pin/ball joint)")
        .def("has_any_release", &grillex::EndRelease::has_any_release,
             "Check if any releases are active")
        .def("get_released_indices", &grillex::EndRelease::get_released_indices,
             py::arg("has_warping"),
             "Get indices of released DOFs")
        .def("__repr__", [](const grillex::EndRelease &r) {
            int count = 0;
            if (r.release_ux_i || r.release_uy_i || r.release_uz_i ||
                r.release_rx_i || r.release_ry_i || r.release_rz_i || r.release_warp_i) count++;
            if (r.release_ux_j || r.release_uy_j || r.release_uz_j ||
                r.release_rx_j || r.release_ry_j || r.release_rz_j || r.release_warp_j) count++;
            return "<EndRelease " + std::to_string(count) + " ends with releases>";
        });

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
    py::class_<grillex::BeamElement, grillex::BeamElementBase>(m, "BeamElement",
        "3D Euler-Bernoulli beam element with 12 DOFs")
        .def(py::init<int, grillex::Node*, grillex::Node*,
                      grillex::Material*, grillex::Section*, double>(),
             py::arg("id"), py::arg("node_i"), py::arg("node_j"),
             py::arg("material"), py::arg("section"), py::arg("roll") = 0.0,
             "Construct a beam element")
        .def(py::init<int, grillex::Node*, grillex::Node*,
                      grillex::Material*, grillex::Section*,
                      const grillex::BeamConfig&, double>(),
             py::arg("id"), py::arg("node_i"), py::arg("node_j"),
             py::arg("material"), py::arg("section"),
             py::arg("config"), py::arg("roll") = 0.0,
             "Construct a beam element with configuration")
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
        .def_readwrite("releases", &grillex::BeamElement::releases,
                       "End release configuration")
        .def_readwrite("config", &grillex::BeamElement::config,
                       "Beam configuration (formulation and features)")
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
        .def("offset_transformation_matrix", &grillex::BeamElement::offset_transformation_matrix,
             "Compute 12x12 offset transformation matrix")
        .def("offset_transformation_matrix_warping", &grillex::BeamElement::offset_transformation_matrix_warping,
             "Compute 14x14 offset transformation matrix for warping elements")
        .def("local_stiffness_matrix_warping", &grillex::BeamElement::local_stiffness_matrix_warping,
             py::arg("formulation") = grillex::BeamFormulation::EulerBernoulli,
             "Compute 14x14 local stiffness matrix including warping DOF")
        .def("local_mass_matrix_warping", &grillex::BeamElement::local_mass_matrix_warping,
             py::arg("formulation") = grillex::BeamFormulation::EulerBernoulli,
             "Compute 14x14 local mass matrix including warping DOF")
        .def("transformation_matrix_warping", &grillex::BeamElement::transformation_matrix_warping,
             "Compute 14x14 transformation matrix for warping elements")
        .def("global_stiffness_matrix_warping", &grillex::BeamElement::global_stiffness_matrix_warping,
             py::arg("formulation") = grillex::BeamFormulation::EulerBernoulli,
             "Compute 14x14 global stiffness matrix including warping DOF")
        .def("global_mass_matrix_warping", &grillex::BeamElement::global_mass_matrix_warping,
             py::arg("formulation") = grillex::BeamFormulation::EulerBernoulli,
             "Compute 14x14 global mass matrix including warping DOF")
        .def("direction_vector", &grillex::BeamElement::direction_vector,
             "Get the direction vector of the beam (normalized, from node_i to node_j)")
        .def("__repr__", [](const grillex::BeamElement &e) {
            return "<BeamElement id=" + std::to_string(e.id) +
                   " nodes=[" + std::to_string(e.node_i->id) + "," +
                   std::to_string(e.node_j->id) + "] L=" +
                   std::to_string(e.length) + ">";
        });

    // Factory function
    m.def("create_beam_element", &grillex::create_beam_element,
          py::arg("id"), py::arg("node_i"), py::arg("node_j"),
          py::arg("material"), py::arg("section"),
          py::arg("config") = grillex::BeamConfig{},
          py::arg("roll") = 0.0,
          "Factory function to create beam elements with different configurations");

    // are_elements_collinear helper function
    m.def("are_elements_collinear", &grillex::are_elements_collinear,
          py::arg("elem1"), py::arg("elem2"), py::arg("shared_node_id"),
          py::arg("angle_tolerance_deg") = 5.0,
          "Check if two elements sharing a node are collinear within the angle tolerance");

    // ========================================================================
    // Phase 3: Assembly & Solver
    // ========================================================================

    // WarpingDOFInfo struct
    py::class_<grillex::WarpingDOFInfo>(m, "WarpingDOFInfo",
        "Information about a warping DOF for a specific element at a node")
        .def(py::init<>())
        .def_readwrite("element_id", &grillex::WarpingDOFInfo::element_id,
                       "Element ID")
        .def_readwrite("node_id", &grillex::WarpingDOFInfo::node_id,
                       "Node ID where this warping DOF is located")
        .def_readwrite("is_node_i", &grillex::WarpingDOFInfo::is_node_i,
                       "True if this is at node_i of the element")
        .def_readwrite("global_dof", &grillex::WarpingDOFInfo::global_dof,
                       "Assigned global DOF number")
        .def("__repr__", [](const grillex::WarpingDOFInfo &info) {
            return "<WarpingDOFInfo elem=" + std::to_string(info.element_id) +
                   " node=" + std::to_string(info.node_id) +
                   " global_dof=" + std::to_string(info.global_dof) + ">";
        });

    // WarpingCoupling struct
    py::class_<grillex::WarpingCoupling>(m, "WarpingCoupling",
        "Group of warping DOFs that should be coupled (share the same global DOF)")
        .def(py::init<>())
        .def_readwrite("coupled_dofs", &grillex::WarpingCoupling::coupled_dofs,
                       "DOFs that share the same global DOF")
        .def_readwrite("master_dof", &grillex::WarpingCoupling::master_dof,
                       "The global DOF number used by all coupled DOFs")
        .def("__repr__", [](const grillex::WarpingCoupling &coupling) {
            return "<WarpingCoupling master_dof=" + std::to_string(coupling.master_dof) +
                   " num_coupled=" + std::to_string(coupling.coupled_dofs.size()) + ">";
        });

    // DOFHandler class
    py::class_<grillex::DOFHandler>(m, "DOFHandler",
        "Handles global DOF numbering for structural analysis with element-specific warping DOFs")
        .def(py::init<>(), "Construct an empty DOFHandler")
        .def("number_dofs", &grillex::DOFHandler::number_dofs,
             py::arg("registry"),
             "Assign global DOF numbers to all nodes (legacy mode, nodal warping)")
        .def("number_dofs_with_elements", &grillex::DOFHandler::number_dofs_with_elements,
             py::arg("registry"), py::arg("elements"),
             py::arg("collinearity_tolerance_deg") = 5.0,
             "Assign DOF numbers with element-specific warping handling and automatic collinearity detection")
        .def("total_dofs", &grillex::DOFHandler::total_dofs,
             "Get total number of DOFs in the system")
        .def("get_global_dof", &grillex::DOFHandler::get_global_dof,
             py::arg("node_id"), py::arg("local_dof"),
             "Get global DOF number for a specific node and local DOF (standard DOFs 0-5)")
        .def("get_warping_dof", &grillex::DOFHandler::get_warping_dof,
             py::arg("element_id"), py::arg("node_id"),
             "Get warping DOF for a specific element at a specific node")
        .def("get_location_array", &grillex::DOFHandler::get_location_array,
             py::arg("elem"),
             "Get location array for an element (maps local to global DOFs)")
        .def("has_warping_dofs", &grillex::DOFHandler::has_warping_dofs,
             "Check if any warping DOF exists")
        .def("clear", &grillex::DOFHandler::clear,
             "Clear all DOF numbering")
        .def("set_warping_continuous", &grillex::DOFHandler::set_warping_continuous,
             py::arg("node_id"), py::arg("element_ids"),
             "Manually specify that warping should be continuous between elements at a node")
        .def("release_warping_coupling", &grillex::DOFHandler::release_warping_coupling,
             py::arg("node_id"), py::arg("element1_id"), py::arg("element2_id"),
             "Manually release warping coupling between two elements at a node")
        .def("get_warping_couplings", &grillex::DOFHandler::get_warping_couplings,
             "Get all warping coupling information")
        .def("get_collinearity_tolerance", &grillex::DOFHandler::get_collinearity_tolerance,
             "Get the collinearity tolerance used for warping DOF coupling (degrees)")
        .def("__repr__", [](const grillex::DOFHandler &dh) {
            return "<DOFHandler total_dofs=" + std::to_string(dh.total_dofs()) +
                   " has_warping=" + (dh.has_warping_dofs() ? "True" : "False") +
                   " collinearity_tol=" + std::to_string(dh.get_collinearity_tolerance()) + "deg>";
        });

    // ========================================================================
    // Phase 3: Assembly & Solver
    // ========================================================================

    // Assembler class
    py::class_<grillex::Assembler>(m, "Assembler",
        "Assembles global stiffness and mass matrices from element matrices")
        .def(py::init<grillex::DOFHandler&>(),
             py::arg("dof_handler"),
             "Construct an Assembler with a DOFHandler",
             py::keep_alive<1, 2>())  // Keep DOFHandler alive as long as Assembler exists
        .def("assemble_stiffness", &grillex::Assembler::assemble_stiffness,
             py::arg("elements"),
             "Assemble global stiffness matrix from element stiffness matrices")
        .def("assemble_mass", &grillex::Assembler::assemble_mass,
             py::arg("elements"),
             "Assemble global mass matrix from element mass matrices")
        .def("get_dof_handler", &grillex::Assembler::get_dof_handler,
             "Get the DOFHandler used by this assembler",
             py::return_value_policy::reference_internal)
        .def("__repr__", [](const grillex::Assembler &asm_) {
            return "<Assembler total_dofs=" +
                   std::to_string(asm_.get_dof_handler().total_dofs()) + ">";
        });

    // DOFIndex enum
    py::enum_<grillex::DOFIndex>(m, "DOFIndex",
        "Local DOF indices for beam elements")
        .value("UX", grillex::DOFIndex::UX, "Translation in X direction")
        .value("UY", grillex::DOFIndex::UY, "Translation in Y direction")
        .value("UZ", grillex::DOFIndex::UZ, "Translation in Z direction")
        .value("RX", grillex::DOFIndex::RX, "Rotation about X axis")
        .value("RY", grillex::DOFIndex::RY, "Rotation about Y axis")
        .value("RZ", grillex::DOFIndex::RZ, "Rotation about Z axis")
        .value("WARP", grillex::DOFIndex::WARP, "Warping displacement (7th DOF)")
        .export_values();

    // FixedDOF struct
    py::class_<grillex::FixedDOF>(m, "FixedDOF",
        "Represents a fixed degree of freedom with optional prescribed value")
        .def(py::init<int, int, double>(),
             py::arg("node_id"),
             py::arg("local_dof"),
             py::arg("value") = 0.0,
             "Construct a fixed DOF")
        .def_readwrite("node_id", &grillex::FixedDOF::node_id,
                      "Node ID where DOF is fixed")
        .def_readwrite("local_dof", &grillex::FixedDOF::local_dof,
                      "Local DOF index (0-6)")
        .def_readwrite("value", &grillex::FixedDOF::value,
                      "Prescribed value")
        .def("__repr__", [](const grillex::FixedDOF &fd) {
            return "<FixedDOF node=" + std::to_string(fd.node_id) +
                   " dof=" + std::to_string(fd.local_dof) +
                   " value=" + std::to_string(fd.value) + ">";
        });

    // BCHandler class
    py::class_<grillex::BCHandler>(m, "BCHandler",
        "Boundary Condition Handler for managing fixed DOFs and prescribed displacements")
        .def(py::init<>(), "Construct an empty boundary condition handler")
        .def("add_fixed_dof", &grillex::BCHandler::add_fixed_dof,
             py::arg("node_id"),
             py::arg("local_dof"),
             py::arg("value") = 0.0,
             "Add a single fixed DOF with optional prescribed value")
        .def("fix_node", &grillex::BCHandler::fix_node,
             py::arg("node_id"),
             "Fix all 6 standard DOFs at a node (full fixity, no warping)")
        .def("fix_node_with_warping", &grillex::BCHandler::fix_node_with_warping,
             py::arg("node_id"),
             "Fix all 7 DOFs at a node (including warping)")
        .def("pin_node", &grillex::BCHandler::pin_node,
             py::arg("node_id"),
             "Fix only translations at a node (pin support)")
        .def("fork_support", &grillex::BCHandler::fork_support,
             py::arg("node_id"),
             "Apply fork support (fix translations, free rotations and warping)")
        .def("apply_to_system", &grillex::BCHandler::apply_to_system,
             py::arg("K"),
             py::arg("F"),
             py::arg("dof_handler"),
             "Apply boundary conditions to system matrices using penalty method")
        .def("get_fixed_global_dofs", &grillex::BCHandler::get_fixed_global_dofs,
             py::arg("dof_handler"),
             "Get list of global DOF indices that are fixed")
        .def("num_fixed_dofs", &grillex::BCHandler::num_fixed_dofs,
             "Get number of fixed DOFs")
        .def("clear", &grillex::BCHandler::clear,
             "Clear all boundary conditions")
        .def("get_fixed_dofs", &grillex::BCHandler::get_fixed_dofs,
             "Get const reference to fixed DOFs list")
        .def("__repr__", [](const grillex::BCHandler &bc) {
            return "<BCHandler num_fixed=" + std::to_string(bc.num_fixed_dofs()) + ">";
        });
}
