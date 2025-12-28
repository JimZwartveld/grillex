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
#include "grillex/solver.hpp"
#include "grillex/load_case.hpp"
#include "grillex/model.hpp"
#include "grillex/constraints.hpp"
#include "grillex/internal_actions.hpp"
#include "grillex/spring_element.hpp"
#include "grillex/point_mass.hpp"
#include "grillex/plate_element.hpp"
#include "grillex/plate_element_8.hpp"
#include "grillex/plate_element_9.hpp"
#include "grillex/errors.hpp"
#include "grillex/warnings.hpp"
#include "grillex/nonlinear_solver.hpp"
#include "grillex/eigenvalue_solver.hpp"
#include "grillex/singularity_diagnostics.hpp"

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
    py::class_<grillex::NodeRegistry, std::unique_ptr<grillex::NodeRegistry, py::nodelete>>(m, "NodeRegistry",
        "Registry for managing nodes with automatic merging based on tolerance")
        .def(py::init<double>(), py::arg("tolerance") = 1e-6,
             "Construct a NodeRegistry with specified tolerance [m]")
        .def("get_or_create_node", &grillex::NodeRegistry::get_or_create_node,
             py::arg("x"), py::arg("y"), py::arg("z"),
             "Get existing node or create new one at (x, y, z)",
             py::return_value_policy::reference_internal)
        .def("create_node", &grillex::NodeRegistry::create_node,
             py::arg("x"), py::arg("y"), py::arg("z"),
             "Force create a new node at (x, y, z) without merging",
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
        .def("equivalent_nodal_forces", &grillex::BeamElement::equivalent_nodal_forces,
             py::arg("w_start"), py::arg("w_end"),
             "Compute equivalent nodal forces for a distributed line load.\n\n"
             "Args:\n"
             "    w_start: Load intensity vector at start [kN/m] (global coords)\n"
             "    w_end: Load intensity vector at end [kN/m] (global coords)\n\n"
             "Returns:\n"
             "    12x1 vector of equivalent nodal forces in global coordinates\n"
             "    [Fx_i, Fy_i, Fz_i, Mx_i, My_i, Mz_i, Fx_j, Fy_j, Fz_j, Mx_j, My_j, Mz_j]")
        .def("get_element_displacements_local", &grillex::BeamElement::get_element_displacements_local,
             py::arg("global_displacements"), py::arg("dof_handler"),
             "Get element displacements in local coordinates.\n\n"
             "Extracts element DOFs from global displacement vector and transforms to local coords.\n\n"
             "Args:\n"
             "    global_displacements: Global displacement vector from analysis\n"
             "    dof_handler: DOF handler for global-to-local mapping\n\n"
             "Returns:\n"
             "    Local displacement vector (12 or 14 components)")
        .def("get_displacements_at",
             static_cast<grillex::DisplacementLine (grillex::BeamElement::*)(
                 double, const Eigen::VectorXd&, const grillex::DOFHandler&, const grillex::LoadCase*) const>(
                 &grillex::BeamElement::get_displacements_at),
             py::arg("x"), py::arg("global_displacements"), py::arg("dof_handler"),
             py::arg("load_case") = nullptr,
             "Get displacements and rotations at position x along element.\n\n"
             "Uses analytical beam equations to compute exact displacements and rotations\n"
             "at any position along the beam, including distributed load effects.\n\n"
             "Args:\n"
             "    x: Position along element [0, L] in meters\n"
             "    global_displacements: Global displacement vector from analysis\n"
             "    dof_handler: DOF handler for global-to-local mapping\n"
             "    load_case: Optional load case for distributed load effects\n\n"
             "Returns:\n"
             "    DisplacementLine with u, v, w, theta_x, theta_y, theta_z, phi_prime")
        .def("compute_end_forces", &grillex::BeamElement::compute_end_forces,
             py::arg("global_displacements"), py::arg("dof_handler"),
             "Compute end forces in local coordinates.\n\n"
             "Computes internal forces at both element ends: f = K * u\n\n"
             "Args:\n"
             "    global_displacements: Global displacement vector from analysis\n"
             "    dof_handler: DOF handler for global-to-local mapping\n\n"
             "Returns:\n"
             "    Tuple (EndForces_i, EndForces_j) with forces at each end")
        // Task 7.0: Distributed Load Query Methods
        .def("get_distributed_load_y", &grillex::BeamElement::get_distributed_load_y,
             py::arg("load_case"),
             "Get distributed load in local y direction from a load case.\n\n"
             "Queries the load case for line loads on this element, transforms them\n"
             "from global to local coordinates, and returns the local y-component.\n\n"
             "Args:\n"
             "    load_case: LoadCase containing line loads\n\n"
             "Returns:\n"
             "    DistributedLoad with q_start and q_end in local y [kN/m]")
        .def("get_distributed_load_z", &grillex::BeamElement::get_distributed_load_z,
             py::arg("load_case"),
             "Get distributed load in local z direction from a load case.\n\n"
             "Queries the load case for line loads on this element, transforms them\n"
             "from global to local coordinates, and returns the local z-component.\n\n"
             "Args:\n"
             "    load_case: LoadCase containing line loads\n\n"
             "Returns:\n"
             "    DistributedLoad with q_start and q_end in local z [kN/m]")
        .def("get_distributed_load_axial", &grillex::BeamElement::get_distributed_load_axial,
             py::arg("load_case"),
             "Get distributed axial load in local x direction from a load case.\n\n"
             "Queries the load case for line loads on this element, transforms them\n"
             "from global to local coordinates, and returns the local x-component.\n\n"
             "Args:\n"
             "    load_case: LoadCase containing line loads\n\n"
             "Returns:\n"
             "    DistributedLoad with q_start and q_end in local x (axial) [kN/m]")
        // Task 7.2: Internal Action Functions Along Beam
        .def("get_internal_actions", &grillex::BeamElement::get_internal_actions,
             py::arg("x"),
             py::arg("global_displacements"),
             py::arg("dof_handler"),
             py::arg("load_case") = nullptr,
             "Get internal actions at position x along element.\n\n"
             "Computes internal forces and moments at any position along the beam\n"
             "using analytical closed-form solutions from differential equations.\n\n"
             "Args:\n"
             "    x: Position along beam [0, L] in meters\n"
             "    global_displacements: Displacement vector from analysis\n"
             "    dof_handler: DOF numbering manager\n"
             "    load_case: Optional load case for distributed load effects\n\n"
             "Returns:\n"
             "    InternalActions with N, Vy, Vz, Mx, My, Mz at position x")
        .def("find_moment_extremes", &grillex::BeamElement::find_moment_extremes,
             py::arg("axis"),
             py::arg("global_displacements"),
             py::arg("dof_handler"),
             py::arg("load_case") = nullptr,
             "Find moment extrema along element.\n\n"
             "Finds locations and values of maximum and minimum bending moment\n"
             "along the element. For distributed loads, extrema occur where shear is zero.\n\n"
             "Args:\n"
             "    axis: 'y' or 'z' for bending plane (char)\n"
             "    global_displacements: Displacement vector from analysis\n"
             "    dof_handler: DOF numbering manager\n"
             "    load_case: Optional load case for distributed load effects\n\n"
             "Returns:\n"
             "    Tuple (min_extreme, max_extreme) with ActionExtreme objects")
        // Task 7.2b: Warping Internal Actions
        .def("get_warping_internal_actions", &grillex::BeamElement::get_warping_internal_actions,
             py::arg("x"),
             py::arg("global_displacements"),
             py::arg("dof_handler"),
             "Get warping-specific internal actions at position x.\n\n"
             "For 14-DOF elements with warping DOF, computes bimoment, St. Venant torsion,\n"
             "warping torsion, and maximum warping stress. For 12-DOF elements, returns\n"
             "standard internal actions with zero warping values.\n\n"
             "Args:\n"
             "    x: Position along beam [0, L] in meters\n"
             "    global_displacements: Displacement vector from analysis\n"
             "    dof_handler: DOF numbering manager\n\n"
             "Returns:\n"
             "    WarpingInternalActions with B, Mx_sv, Mx_w, sigma_w_max")
        .def("compute_warping_stress", &grillex::BeamElement::compute_warping_stress,
             py::arg("bimoment"),
             "Compute maximum warping normal stress.\n\n"
             "σ_w = -B × ω_max / Iw\n\n"
             "Args:\n"
             "    bimoment: Bimoment at position [kN·m²]\n\n"
             "Returns:\n"
             "    Maximum warping stress [kN/m²]")
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
        .def("assemble_mass",
             static_cast<Eigen::SparseMatrix<double> (grillex::Assembler::*)(
                 const std::vector<grillex::BeamElement*>&) const>(
                 &grillex::Assembler::assemble_mass),
             py::arg("elements"),
             "Assemble global mass matrix from element mass matrices")
        .def("assemble_mass_with_point_masses",
             static_cast<Eigen::SparseMatrix<double> (grillex::Assembler::*)(
                 const std::vector<grillex::BeamElement*>&,
                 const std::vector<grillex::PointMass*>&) const>(
                 &grillex::Assembler::assemble_mass),
             py::arg("beam_elements"),
             py::arg("point_masses"),
             "Assemble global mass matrix including point masses")
        .def("compute_total_mass", &grillex::Assembler::compute_total_mass,
             py::arg("beam_elements"),
             py::arg("point_masses"),
             "Compute total translational mass from beam elements and point masses [mT]")
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
        .def_readwrite("element_id", &grillex::FixedDOF::element_id,
                      "Element ID for element-specific warping DOF (-1 for standard DOFs)")
        .def("__repr__", [](const grillex::FixedDOF &fd) {
            std::string repr = "<FixedDOF node=" + std::to_string(fd.node_id) +
                   " dof=" + std::to_string(fd.local_dof) +
                   " value=" + std::to_string(fd.value);
            if (fd.element_id >= 0) {
                repr += " element=" + std::to_string(fd.element_id);
            }
            repr += ">";
            return repr;
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
        .def("add_fixed_warping_dof", &grillex::BCHandler::add_fixed_warping_dof,
             py::arg("element_id"),
             py::arg("node_id"),
             py::arg("value") = 0.0,
             "Add a fixed warping DOF for a specific element at a node")
        .def("fix_node", &grillex::BCHandler::fix_node,
             py::arg("node_id"),
             "Fix all 6 standard DOFs at a node (full fixity, no warping)")
        .def("fix_node_with_warping",
             py::overload_cast<int>(&grillex::BCHandler::fix_node_with_warping),
             py::arg("node_id"),
             "Fix all 7 DOFs at a node (including warping for all elements)")
        .def("fix_node_with_warping",
             py::overload_cast<int, int>(&grillex::BCHandler::fix_node_with_warping),
             py::arg("node_id"),
             py::arg("element_id"),
             "Fix all 7 DOFs at a node with element-specific warping fixity")
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

    // ========================================================================
    // Phase 3 (continued): Linear Solver
    // ========================================================================

    // LinearSolver::Method enum
    py::enum_<grillex::LinearSolver::Method>(m, "SolverMethod",
        "Solver method for linear systems")
        .value("SparseLU", grillex::LinearSolver::Method::SparseLU,
               "Direct solver using SparseLU (general sparse matrices)")
        .value("SimplicialLDLT", grillex::LinearSolver::Method::SimplicialLDLT,
               "Direct solver using SimplicialLDLT (symmetric matrices, default)")
        .value("ConjugateGradient", grillex::LinearSolver::Method::ConjugateGradient,
               "Iterative solver using Conjugate Gradient (large systems)")
        .export_values();

    // LinearSolver class
    py::class_<grillex::LinearSolver>(m, "LinearSolver",
        "Linear solver for finite element systems K * u = F")
        .def(py::init<grillex::LinearSolver::Method>(),
             py::arg("method") = grillex::LinearSolver::Method::SimplicialLDLT,
             "Construct a linear solver with specified method (default: SimplicialLDLT)")
        .def("solve", &grillex::LinearSolver::solve,
             py::arg("K"),
             py::arg("F"),
             "Solve linear system K * u = F, returns displacement vector u")
        .def("is_singular", &grillex::LinearSolver::is_singular,
             "Check if the last solve detected a singular system")
        .def("get_error_message", &grillex::LinearSolver::get_error_message,
             "Get error message from last solve (empty if no error)")
        .def("get_method", &grillex::LinearSolver::get_method,
             "Get current solver method")
        .def("set_method", &grillex::LinearSolver::set_method,
             py::arg("method"),
             "Set solver method")
        .def("get_iterations", &grillex::LinearSolver::get_iterations,
             "Get number of iterations (for iterative solvers, 1 for direct solvers)")
        .def("get_error", &grillex::LinearSolver::get_error,
             "Get estimated error (for iterative solvers, 0.0 for direct solvers)")
        .def("set_max_iterations", &grillex::LinearSolver::set_max_iterations,
             py::arg("max_iter"),
             "Set maximum iterations for iterative solvers (default: 1000)")
        .def("set_tolerance", &grillex::LinearSolver::set_tolerance,
             py::arg("tol"),
             "Set convergence tolerance for iterative solvers (default: 1e-10)")
        .def("__repr__", [](const grillex::LinearSolver &s) {
            std::string method_name;
            switch (s.get_method()) {
                case grillex::LinearSolver::Method::SparseLU:
                    method_name = "SparseLU";
                    break;
                case grillex::LinearSolver::Method::SimplicialLDLT:
                    method_name = "SimplicialLDLT";
                    break;
                case grillex::LinearSolver::Method::ConjugateGradient:
                    method_name = "ConjugateGradient";
                    break;
                default:
                    method_name = "Unknown";
            }
            return "<LinearSolver method=" + method_name + ">";
        });

    // ========================================================================
    // Phase 5: Load Cases
    // ========================================================================

    // LoadCaseType enum
    py::enum_<grillex::LoadCaseType>(m, "LoadCaseType",
        "Load case type classification")
        .value("Permanent", grillex::LoadCaseType::Permanent,
               "Dead loads, self-weight, fixed equipment")
        .value("Variable", grillex::LoadCaseType::Variable,
               "Live loads, imposed loads, traffic")
        .value("Environmental", grillex::LoadCaseType::Environmental,
               "Wind, snow, temperature")
        .value("Accidental", grillex::LoadCaseType::Accidental,
               "Impact, explosion, seismic (ultimate limit)")
        .export_values();

    // NodalLoad struct
    py::class_<grillex::NodalLoad>(m, "NodalLoad",
        "Concentrated force/moment at a node")
        .def(py::init<int, int, double>(),
             py::arg("node_id"),
             py::arg("local_dof"),
             py::arg("value"),
             "Construct a nodal load")
        .def_readwrite("node_id", &grillex::NodalLoad::node_id, "Node where load is applied")
        .def_readwrite("local_dof", &grillex::NodalLoad::local_dof, "Local DOF index (0-6)")
        .def_readwrite("value", &grillex::NodalLoad::value, "Load magnitude [kN] or [kN·m]")
        .def("__repr__", [](const grillex::NodalLoad &nl) {
            return "<NodalLoad node=" + std::to_string(nl.node_id) +
                   " dof=" + std::to_string(nl.local_dof) +
                   " value=" + std::to_string(nl.value) + ">";
        });

    // DistributedLoad struct (for Phase 7 internal actions)
    py::class_<grillex::DistributedLoad>(m, "DistributedLoad",
        "Distributed load representation for internal actions computation.\n\n"
        "Represents a linearly varying load in local element coordinates.\n"
        "Load varies: q(x) = q_start + (q_end - q_start) * x / L")
        .def(py::init<>(), "Construct a zero distributed load")
        .def_readwrite("q_start", &grillex::DistributedLoad::q_start,
                       "Load intensity at element start [kN/m]")
        .def_readwrite("q_end", &grillex::DistributedLoad::q_end,
                       "Load intensity at element end [kN/m]")
        .def("is_uniform", &grillex::DistributedLoad::is_uniform,
             "Check if load is uniform (constant along length)")
        .def("is_zero", &grillex::DistributedLoad::is_zero,
             "Check if load is zero")
        .def("at", &grillex::DistributedLoad::at,
             py::arg("x"), py::arg("L"),
             "Get load intensity at position x along element")
        .def("__repr__", [](const grillex::DistributedLoad &dl) {
            return "<DistributedLoad q_start=" + std::to_string(dl.q_start) +
                   " q_end=" + std::to_string(dl.q_end) + ">";
        });

    // LineLoad struct
    py::class_<grillex::LineLoad>(m, "LineLoad",
        "Distributed load along a beam element")
        .def(py::init<int, const Eigen::Vector3d&, const Eigen::Vector3d&>(),
             py::arg("element_id"),
             py::arg("w_start"),
             py::arg("w_end"),
             "Construct a line load")
        .def_readwrite("element_id", &grillex::LineLoad::element_id, "Beam element ID")
        .def_readwrite("w_start", &grillex::LineLoad::w_start, "Load intensity at start [kN/m]")
        .def_readwrite("w_end", &grillex::LineLoad::w_end, "Load intensity at end [kN/m]")
        .def("__repr__", [](const grillex::LineLoad &ll) {
            return "<LineLoad elem=" + std::to_string(ll.element_id) + ">";
        });

    // LoadCase class
    py::class_<grillex::LoadCase>(m, "LoadCase",
        "Load case containing all loads for a specific scenario")
        .def_property_readonly("id", &grillex::LoadCase::id, "Load case ID")
        .def_property_readonly("name", &grillex::LoadCase::name, "Load case name")
        .def_property_readonly("type", &grillex::LoadCase::type, "Load case type")
        .def("add_nodal_load", &grillex::LoadCase::add_nodal_load,
             py::arg("node_id"),
             py::arg("local_dof"),
             py::arg("value"),
             "Add a nodal load (accumulated if called multiple times for same DOF)")
        .def("add_line_load", &grillex::LoadCase::add_line_load,
             py::arg("element_id"),
             py::arg("w_start"),
             py::arg("w_end"),
             "Add a distributed line load to a beam element")
        .def("set_acceleration_field", &grillex::LoadCase::set_acceleration_field,
             py::arg("acceleration"),
             py::arg("ref_point") = Eigen::Vector3d::Zero(),
             "Set acceleration field for gravity or rotating reference frame")
        .def("clear", &grillex::LoadCase::clear,
             "Clear all loads in this load case")
        .def("is_empty", &grillex::LoadCase::is_empty,
             "Check if load case has any loads")
        .def("get_nodal_loads", &grillex::LoadCase::get_nodal_loads,
             py::return_value_policy::reference_internal,
             "Get list of nodal loads")
        .def("get_line_loads", &grillex::LoadCase::get_line_loads,
             py::return_value_policy::reference_internal,
             "Get list of line loads")
        .def("get_acceleration", &grillex::LoadCase::get_acceleration,
             "Get acceleration field [ax, ay, az, αx, αy, αz]")
        .def("get_acceleration_ref_point", &grillex::LoadCase::get_acceleration_ref_point,
             "Get acceleration field reference point")
        .def("__repr__", [](const grillex::LoadCase &lc) {
            return "<LoadCase '" + lc.name() + "' id=" + std::to_string(lc.id()) + ">";
        });

    // LoadCaseResult struct
    py::class_<grillex::LoadCaseResult>(m, "LoadCaseResult",
        "Results for a single load case analysis.\n\n"
        "Extended for nonlinear analysis with spring states and iteration info.")
        .def_readonly("load_case", &grillex::LoadCaseResult::load_case,
                     "Associated load case")
        .def_readonly("displacements", &grillex::LoadCaseResult::displacements,
                     "Global displacement vector")
        .def_readonly("reactions", &grillex::LoadCaseResult::reactions,
                     "Reaction forces at constraints")
        .def_readonly("success", &grillex::LoadCaseResult::success,
                     "Analysis succeeded")
        .def_readonly("error_message", &grillex::LoadCaseResult::error_message,
                     "Error message if failed")
        .def_readonly("iterations", &grillex::LoadCaseResult::iterations,
                     "Number of solver iterations (1 for linear)")
        .def_readonly("solver_message", &grillex::LoadCaseResult::solver_message,
                     "Solver convergence message")
        .def_readonly("spring_states", &grillex::LoadCaseResult::spring_states,
                     "Spring states at convergence: [(spring_id, active_states[6])]")
        .def_readonly("spring_forces", &grillex::LoadCaseResult::spring_forces,
                     "Spring forces at convergence: [(spring_id, forces[6])] [kN or kN*m]")
        .def("__repr__", [](const grillex::LoadCaseResult &res) {
            return "<LoadCaseResult success=" + std::string(res.success ? "True" : "False") +
                   " iterations=" + std::to_string(res.iterations) + ">";
        });

    // LoadCombinationResult struct
    py::class_<grillex::LoadCombinationResult>(m, "LoadCombinationResult",
        "Results for a load combination analysis.\n\n"
        "Used by analyze_combination() for direct nonlinear combination solving.\n"
        "Required because nonlinear springs invalidate superposition.")
        .def_readonly("displacements", &grillex::LoadCombinationResult::displacements,
                     "Combined displacement vector")
        .def_readonly("reactions", &grillex::LoadCombinationResult::reactions,
                     "Reaction forces at constraints")
        .def_readonly("converged", &grillex::LoadCombinationResult::converged,
                     "Analysis converged successfully")
        .def_readonly("iterations", &grillex::LoadCombinationResult::iterations,
                     "Total iterations for solve")
        .def_readonly("message", &grillex::LoadCombinationResult::message,
                     "Solver message")
        .def_readonly("spring_states", &grillex::LoadCombinationResult::spring_states,
                     "Spring states at convergence: [(spring_id, active_states[6])]")
        .def_readonly("spring_forces", &grillex::LoadCombinationResult::spring_forces,
                     "Spring forces at convergence: [(spring_id, forces[6])] [kN or kN*m]")
        .def("__repr__", [](const grillex::LoadCombinationResult &res) {
            return "<LoadCombinationResult converged=" + std::string(res.converged ? "True" : "False") +
                   " iterations=" + std::to_string(res.iterations) + ">";
        });

    // LoadCombinationTerm struct
    py::class_<grillex::LoadCombinationTerm>(m, "LoadCombinationTerm",
        "Term in a load combination (load case + factor)")
        .def(py::init<grillex::LoadCase*, double, bool>(),
             py::arg("load_case"),
             py::arg("factor"),
             py::arg("explicit_factor") = false,
             "Construct a combination term")
        .def_readonly("load_case", &grillex::LoadCombinationTerm::load_case,
                     "Load case (non-owning pointer)")
        .def_readonly("factor", &grillex::LoadCombinationTerm::factor,
                     "Load factor to apply")
        .def_readonly("explicit_factor", &grillex::LoadCombinationTerm::explicit_factor,
                     "True if factor was explicitly set")
        .def("__repr__", [](const grillex::LoadCombinationTerm &term) {
            std::string lc_name = term.load_case ? term.load_case->name() : "null";
            return "<LoadCombinationTerm '" + lc_name +
                   "' factor=" + std::to_string(term.factor) +
                   " explicit=" + (term.explicit_factor ? "True" : "False") + ">";
        });

    // LoadCombination class
    py::class_<grillex::LoadCombination>(m, "LoadCombination",
        "Load combination for code-based analysis (ULS, SLS, etc.)\n\n"
        "Combines multiple load cases with factors. Two approaches:\n"
        "1. Type-based factors: Set factors for Permanent, Variable, Environmental,\n"
        "   Accidental types in constructor. Load cases automatically get their type factor.\n"
        "2. Explicit factors: Override type-based factor when adding a load case.\n\n"
        "Example (Eurocode ULS):\n"
        "    combo = LoadCombination(1, 'ULS-STR', 1.35, 1.5, 1.5, 1.0)\n"
        "    combo.add_load_case(dead_load)   # Uses 1.35 (Permanent)\n"
        "    combo.add_load_case(live_load)   # Uses 1.5 (Variable)\n"
        "    combo.add_load_case(special, 0.9)  # Explicit factor 0.9")
        .def(py::init<int, const std::string&, double, double, double, double>(),
             py::arg("id"),
             py::arg("name"),
             py::arg("permanent_factor") = 1.0,
             py::arg("variable_factor") = 1.0,
             py::arg("environmental_factor") = 1.0,
             py::arg("accidental_factor") = 1.0,
             "Construct a load combination with type-based factors")
        .def_property_readonly("id", &grillex::LoadCombination::id, "Combination ID")
        .def_property_readonly("name", &grillex::LoadCombination::name, "Combination name")
        .def("get_type_factor", &grillex::LoadCombination::get_type_factor,
             py::arg("type"),
             "Get factor for a load case type")
        .def("set_type_factor", &grillex::LoadCombination::set_type_factor,
             py::arg("type"),
             py::arg("factor"),
             "Set factor for a load case type")
        .def("add_load_case", py::overload_cast<grillex::LoadCase*>(&grillex::LoadCombination::add_load_case),
             py::arg("load_case"),
             "Add a load case using its type-based factor")
        .def("add_load_case", py::overload_cast<grillex::LoadCase*, double>(&grillex::LoadCombination::add_load_case),
             py::arg("load_case"),
             py::arg("factor"),
             "Add a load case with an explicit factor (overrides type-based)")
        .def("remove_load_case", &grillex::LoadCombination::remove_load_case,
             py::arg("load_case"),
             "Remove a load case from the combination")
        .def("clear", &grillex::LoadCombination::clear,
             "Clear all load cases from the combination")
        .def("get_terms", &grillex::LoadCombination::get_terms,
             py::return_value_policy::reference_internal,
             "Get all terms (load cases + factors)")
        .def("__len__", &grillex::LoadCombination::size,
             "Get number of load cases in the combination")
        .def("empty", &grillex::LoadCombination::empty,
             "Check if the combination is empty")
        .def("get_combined_displacements", &grillex::LoadCombination::get_combined_displacements,
             py::arg("results"),
             "Get combined displacements from individual load case results")
        .def("get_combined_reactions", &grillex::LoadCombination::get_combined_reactions,
             py::arg("results"),
             "Get combined reactions from individual load case results")
        .def("__repr__", [](const grillex::LoadCombination &combo) {
            return "<LoadCombination '" + combo.name() +
                   "' id=" + std::to_string(combo.id()) +
                   " terms=" + std::to_string(combo.size()) + ">";
        });

    // ========================================================================
    // Phase 15: NonlinearSolverSettings (needed before Model for default args)
    // ========================================================================

    // NonlinearSolverSettings struct - MUST be bound before Model
    py::class_<grillex::NonlinearSolverSettings>(m, "NonlinearSolverSettings",
        "Settings for nonlinear spring solver.")
        .def(py::init<>())
        .def_readwrite("max_iterations", &grillex::NonlinearSolverSettings::max_iterations,
             "Maximum iterations before giving up (default: 50)")
        .def_readwrite("gap_tolerance", &grillex::NonlinearSolverSettings::gap_tolerance,
             "Tolerance for spring activation threshold [m] (default: 1e-10)")
        .def_readwrite("displacement_tolerance", &grillex::NonlinearSolverSettings::displacement_tolerance,
             "Relative displacement tolerance for convergence (default: 1e-8)")
        .def_readwrite("allow_all_springs_inactive", &grillex::NonlinearSolverSettings::allow_all_springs_inactive,
             "Allow solution where all nonlinear springs are inactive (default: false)")
        .def_readwrite("enable_oscillation_damping", &grillex::NonlinearSolverSettings::enable_oscillation_damping,
             "Enable oscillation detection and damping (default: true)")
        .def_readwrite("oscillation_history_size", &grillex::NonlinearSolverSettings::oscillation_history_size,
             "Number of iterations to look back for oscillation detection (default: 4)")
        .def_readwrite("oscillation_damping_factor", &grillex::NonlinearSolverSettings::oscillation_damping_factor,
             "Damping factor when oscillation detected (default: 0.5)")
        .def_readwrite("use_partial_stiffness", &grillex::NonlinearSolverSettings::use_partial_stiffness,
             "Use partial stiffness (0.5*k) for oscillating springs (default: false)")
        .def_readwrite("hysteresis_band", &grillex::NonlinearSolverSettings::hysteresis_band,
             "Hysteresis band width for state changes [m or rad] (default: 0.0)")
        .def_readwrite("enable_line_search", &grillex::NonlinearSolverSettings::enable_line_search,
             "Enable line search damping for convergence (default: false)")
        .def_readwrite("line_search_factor", &grillex::NonlinearSolverSettings::line_search_factor,
             "Line search damping factor, typical 0.1-0.5 (default: 0.1)")
        .def_readwrite("linear_method", &grillex::NonlinearSolverSettings::linear_method,
             "Linear solver method to use (default: SimplicialLDLT)")
        .def("__repr__", [](const grillex::NonlinearSolverSettings &s) {
            return "<NonlinearSolverSettings max_iter=" +
                   std::to_string(s.max_iterations) + ">";
        });

    // ========================================================================
    // Phase 16: Eigenvalue Analysis Types (must be before Model)
    // ========================================================================

    // EigensolverMethod enum
    py::enum_<grillex::EigensolverMethod>(m, "EigensolverMethod",
        "Eigenvalue solver method selection.\n\n"
        "Dense: Uses Eigen's GeneralizedSelfAdjointEigenSolver (all modes, small systems)\n"
        "SubspaceIteration: Iterative method for large systems (selected modes)\n"
        "ShiftInvert: Shift-and-invert for targeting specific frequencies")
        .value("Dense", grillex::EigensolverMethod::Dense,
               "Dense solver - computes all eigenvalues (suitable for small systems)")
        .value("SubspaceIteration", grillex::EigensolverMethod::SubspaceIteration,
               "Subspace iteration - efficient for finding first n modes of large systems")
        .value("ShiftInvert", grillex::EigensolverMethod::ShiftInvert,
               "Shift-and-invert - for finding modes near a target frequency")
        .export_values();

    // EigensolverSettings struct
    py::class_<grillex::EigensolverSettings>(m, "EigensolverSettings",
        "Configuration settings for eigenvalue analysis.\n\n"
        "Controls solver behavior, convergence criteria, and output options.\n\n"
        "Example:\n"
        "    settings = EigensolverSettings()\n"
        "    settings.n_modes = 10           # Find first 10 modes\n"
        "    settings.tolerance = 1e-8       # Convergence tolerance\n"
        "    settings.method = EigensolverMethod.SubspaceIteration")
        .def(py::init<>())
        .def_readwrite("n_modes", &grillex::EigensolverSettings::n_modes,
             "Number of modes to compute (lowest frequencies first)")
        .def_readwrite("shift", &grillex::EigensolverSettings::shift,
             "Frequency shift for shift-and-invert [rad/s]^2. Set to 0 for lowest modes.")
        .def_readwrite("tolerance", &grillex::EigensolverSettings::tolerance,
             "Convergence tolerance for eigenvalue relative change")
        .def_readwrite("max_iterations", &grillex::EigensolverSettings::max_iterations,
             "Maximum iterations for iterative solvers")
        .def_readwrite("method", &grillex::EigensolverSettings::method,
             "Solver method to use (Dense, SubspaceIteration, ShiftInvert)")
        .def_readwrite("compute_participation", &grillex::EigensolverSettings::compute_participation,
             "Whether to compute participation factors")
        .def_readwrite("mass_normalize", &grillex::EigensolverSettings::mass_normalize,
             "Whether to mass-normalize mode shapes (phi^T M phi = 1)")
        .def_readwrite("rigid_body_threshold", &grillex::EigensolverSettings::rigid_body_threshold,
             "Threshold for rigid body mode detection [rad/s]^2")
        .def("__repr__", [](const grillex::EigensolverSettings &s) {
            return "<EigensolverSettings n_modes=" + std::to_string(s.n_modes) +
                   " method=" + std::to_string(static_cast<int>(s.method)) + ">";
        });

    // ModeResult struct
    py::class_<grillex::ModeResult>(m, "ModeResult",
        "Result for a single vibration mode.\n\n"
        "Contains eigenvalue, frequency, period, mode shape, and modal quantities.\n"
        "Mode shapes are mass-normalized if settings.mass_normalize is true.\n\n"
        "Attributes:\n"
        "    mode_number: Mode number (1-based)\n"
        "    eigenvalue: lambda = omega^2 [rad/s]^2\n"
        "    omega: Natural circular frequency [rad/s]\n"
        "    frequency_hz: Natural frequency [Hz]\n"
        "    period_s: Natural period [s]\n"
        "    mode_shape: Eigenvector (mass-normalized)")
        .def(py::init<>())
        .def_readonly("mode_number", &grillex::ModeResult::mode_number,
             "Mode number (1-based indexing)")
        .def_readonly("eigenvalue", &grillex::ModeResult::eigenvalue,
             "Eigenvalue lambda = omega^2 [rad/s]^2")
        .def_readonly("omega", &grillex::ModeResult::omega,
             "Natural circular frequency omega [rad/s]")
        .def_readonly("frequency_hz", &grillex::ModeResult::frequency_hz,
             "Natural frequency f = omega/(2*pi) [Hz]")
        .def_readonly("period_s", &grillex::ModeResult::period_s,
             "Natural period T = 1/f [s]")
        .def_readonly("mode_shape", &grillex::ModeResult::mode_shape,
             "Mode shape vector (mass-normalized if enabled)")
        .def_readonly("is_rigid_body_mode", &grillex::ModeResult::is_rigid_body_mode,
             "Whether this is a rigid body mode (omega approx 0)")
        .def_readonly("participation_x", &grillex::ModeResult::participation_x,
             "Participation factor for X translation")
        .def_readonly("participation_y", &grillex::ModeResult::participation_y,
             "Participation factor for Y translation")
        .def_readonly("participation_z", &grillex::ModeResult::participation_z,
             "Participation factor for Z translation")
        .def_readonly("participation_rx", &grillex::ModeResult::participation_rx,
             "Participation factor for rotation about X")
        .def_readonly("participation_ry", &grillex::ModeResult::participation_ry,
             "Participation factor for rotation about Y")
        .def_readonly("participation_rz", &grillex::ModeResult::participation_rz,
             "Participation factor for rotation about Z")
        .def_readonly("effective_mass_x", &grillex::ModeResult::effective_mass_x,
             "Effective modal mass for X translation [mT]")
        .def_readonly("effective_mass_y", &grillex::ModeResult::effective_mass_y,
             "Effective modal mass for Y translation [mT]")
        .def_readonly("effective_mass_z", &grillex::ModeResult::effective_mass_z,
             "Effective modal mass for Z translation [mT]")
        .def_readonly("effective_mass_pct_x", &grillex::ModeResult::effective_mass_pct_x,
             "Effective modal mass percentage for X [%]")
        .def_readonly("effective_mass_pct_y", &grillex::ModeResult::effective_mass_pct_y,
             "Effective modal mass percentage for Y [%]")
        .def_readonly("effective_mass_pct_z", &grillex::ModeResult::effective_mass_pct_z,
             "Effective modal mass percentage for Z [%]")
        .def("__repr__", [](const grillex::ModeResult &m) {
            return "<ModeResult mode=" + std::to_string(m.mode_number) +
                   " f=" + std::to_string(m.frequency_hz) + " Hz" +
                   (m.is_rigid_body_mode ? " (rigid body)" : "") + ">";
        });

    // EigensolverResult struct
    py::class_<grillex::EigensolverResult>(m, "EigensolverResult",
        "Complete eigenvalue analysis result.\n\n"
        "Contains all computed modes plus summary statistics.\n\n"
        "Attributes:\n"
        "    converged: Whether solver converged successfully\n"
        "    iterations: Number of iterations (for iterative solvers)\n"
        "    message: Status/error message\n"
        "    modes: List of ModeResult objects, sorted by frequency\n"
        "    n_rigid_body_modes: Number of rigid body modes detected")
        .def(py::init<>())
        .def_readonly("converged", &grillex::EigensolverResult::converged,
             "Whether solver converged successfully")
        .def_readonly("iterations", &grillex::EigensolverResult::iterations,
             "Number of iterations (for iterative solvers)")
        .def_readonly("message", &grillex::EigensolverResult::message,
             "Status/error message")
        .def_readonly("modes", &grillex::EigensolverResult::modes,
             "Computed modes, sorted by frequency (ascending)")
        .def_readonly("n_rigid_body_modes", &grillex::EigensolverResult::n_rigid_body_modes,
             "Number of rigid body modes detected")
        .def_readonly("total_mass_x", &grillex::EigensolverResult::total_mass_x,
             "Total translational mass in X direction [mT]")
        .def_readonly("total_mass_y", &grillex::EigensolverResult::total_mass_y,
             "Total translational mass in Y direction [mT]")
        .def_readonly("total_mass_z", &grillex::EigensolverResult::total_mass_z,
             "Total translational mass in Z direction [mT]")
        .def_readonly("cumulative_mass_pct_x", &grillex::EigensolverResult::cumulative_mass_pct_x,
             "Cumulative effective mass percentage for X [%]")
        .def_readonly("cumulative_mass_pct_y", &grillex::EigensolverResult::cumulative_mass_pct_y,
             "Cumulative effective mass percentage for Y [%]")
        .def_readonly("cumulative_mass_pct_z", &grillex::EigensolverResult::cumulative_mass_pct_z,
             "Cumulative effective mass percentage for Z [%]")
        .def_readonly("reduced_to_full", &grillex::EigensolverResult::reduced_to_full,
             "DOF mapping from reduced to full system")
        .def_readonly("total_dofs", &grillex::EigensolverResult::total_dofs,
             "Total number of DOFs (before reduction)")
        .def("get_mode", &grillex::EigensolverResult::get_mode,
             py::arg("mode_number"),
             "Get mode by number (1-based)",
             py::return_value_policy::reference_internal)
        .def("get_frequencies", &grillex::EigensolverResult::get_frequencies,
             "Get vector of natural frequencies [Hz], sorted ascending")
        .def("get_periods", &grillex::EigensolverResult::get_periods,
             "Get vector of natural periods [s]")
        .def("expand_mode_shape", &grillex::EigensolverResult::expand_mode_shape,
             py::arg("reduced_shape"),
             "Expand reduced mode shape to full DOF vector")
        .def("get_expanded_mode_shape", &grillex::EigensolverResult::get_expanded_mode_shape,
             py::arg("mode_number"),
             "Get expanded mode shape for a specific mode (1-based)")
        .def("__repr__", [](const grillex::EigensolverResult &r) {
            return "<EigensolverResult converged=" +
                   std::string(r.converged ? "True" : "False") +
                   " n_modes=" + std::to_string(r.modes.size()) +
                   " rigid_body=" + std::to_string(r.n_rigid_body_modes) + ">";
        });

    // ========================================================================
    // Phase 3 (continued): Model Class (Orchestration)
    // ========================================================================

    // Model class
    py::class_<grillex::Model>(m, "Model",
        "Top-level Model class for structural analysis")
        .def(py::init<double, grillex::LinearSolver::Method>(),
             py::arg("node_tolerance") = 1e-6,
             py::arg("solver_method") = grillex::LinearSolver::Method::SimplicialLDLT,
             "Construct an empty model with node tolerance and solver method")
        // Node access methods (delegate to internal NodeRegistry)
        .def("get_or_create_node", [](grillex::Model &m, double x, double y, double z) {
            return m.nodes.get_or_create_node(x, y, z);
        }, py::arg("x"), py::arg("y"), py::arg("z"),
           py::return_value_policy::reference_internal,
           "Get existing node or create new one at (x, y, z)")
        .def("create_node", [](grillex::Model &m, double x, double y, double z) {
            return m.nodes.create_node(x, y, z);
        }, py::arg("x"), py::arg("y"), py::arg("z"),
           py::return_value_policy::reference_internal,
           "Force create a new node at (x, y, z) without merging")
        .def("get_all_nodes", [](grillex::Model &m) {
            py::list result;
            for (const auto& node : m.nodes.all_nodes()) {
                result.append(node.get());
            }
            return result;
        }, "Get all nodes in the model")
        .def_property_readonly("materials", [](const grillex::Model &m) {
            py::list result;
            for (const auto& mat : m.materials) {
                result.append(mat.get());
            }
            return result;
        }, "List of materials in the model")
        .def_property_readonly("sections", [](const grillex::Model &m) {
            py::list result;
            for (const auto& sec : m.sections) {
                result.append(sec.get());
            }
            return result;
        }, "List of sections in the model")
        .def_property_readonly("elements", [](const grillex::Model &m) {
            py::list result;
            for (const auto& elem : m.elements) {
                result.append(elem.get());
            }
            return result;
        }, "List of beam elements in the model")
        .def_property_readonly("spring_elements", [](const grillex::Model &m) {
            py::list result;
            for (const auto& spring : m.spring_elements) {
                result.append(spring.get());
            }
            return result;
        }, "List of spring elements in the model")
        .def_property_readonly("point_masses", [](const grillex::Model &m) {
            py::list result;
            for (const auto& pm : m.point_masses) {
                result.append(pm.get());
            }
            return result;
        }, "List of point mass elements in the model")
        .def_property_readonly("plate_elements", [](const grillex::Model &m) {
            py::list result;
            for (const auto& plate : m.plate_elements) {
                result.append(plate.get());
            }
            return result;
        }, "List of plate elements in the model")
        .def_readwrite("boundary_conditions", &grillex::Model::boundary_conditions,
                      "BCHandler for managing boundary conditions")
        .def("create_material", &grillex::Model::create_material,
             py::arg("name"),
             py::arg("E"),
             py::arg("nu"),
             py::arg("rho"),
             "Create a material and add to model",
             py::return_value_policy::reference_internal)
        .def("create_section", &grillex::Model::create_section,
             py::arg("name"),
             py::arg("A"),
             py::arg("Iy"),
             py::arg("Iz"),
             py::arg("J"),
             "Create a section and add to model",
             py::return_value_policy::reference_internal)
        .def("create_beam", &grillex::Model::create_beam,
             py::arg("node_i"),
             py::arg("node_j"),
             py::arg("material"),
             py::arg("section"),
             py::arg("config") = grillex::BeamConfig{},
             "Create a beam element and add to model",
             py::return_value_policy::reference_internal)
        .def("create_spring", &grillex::Model::create_spring,
             py::arg("node_i"),
             py::arg("node_j"),
             "Create a spring element and add to model.\n\n"
             "Example:\n"
             "    spring = model.create_spring(n1, n2)\n"
             "    spring.kx = 1000.0  # Axial stiffness [kN/m]\n"
             "    spring.kz = 500.0   # Vertical stiffness [kN/m]",
             py::return_value_policy::reference_internal)
        .def("create_point_mass", &grillex::Model::create_point_mass,
             py::arg("node"),
             "Create a point mass element and add to model.\n\n"
             "Example:\n"
             "    pm = model.create_point_mass(node)\n"
             "    pm.mass = 10.0  # 10 mT\n"
             "    pm.set_inertia(5.0, 5.0, 3.0)  # Rotational inertias",
             py::return_value_policy::reference_internal)
        .def("create_plate", &grillex::Model::create_plate,
             py::arg("n1"), py::arg("n2"), py::arg("n3"), py::arg("n4"),
             py::arg("thickness"), py::arg("material"),
             "Create a 4-node plate element and add to model.\n\n"
             "Parameters:\n"
             "    n1: Node 1 at corner (-1,-1) in natural coordinates\n"
             "    n2: Node 2 at corner (+1,-1)\n"
             "    n3: Node 3 at corner (+1,+1)\n"
             "    n4: Node 4 at corner (-1,+1)\n"
             "    thickness: Plate thickness [m]\n"
             "    material: Material properties\n\n"
             "Example:\n"
             "    plate = model.create_plate(n1, n2, n3, n4, 0.01, steel)\n\n"
             "Uses Mindlin plate theory with MITC4 formulation for locking-free behavior.",
             py::return_value_policy::reference_internal)
        .def("add_rigid_link", &grillex::Model::add_rigid_link,
             py::arg("slave_node"), py::arg("master_node"), py::arg("offset"),
             "Add a rigid link constraint between two nodes.\n\n"
             "Creates a kinematic constraint where the slave node's motion is\n"
             "determined by the master node's motion:\n"
             "    u_slave = u_master + θ_master × offset\n"
             "    θ_slave = θ_master\n\n"
             "Parameters:\n"
             "    slave_node: Slave (constrained) node - its motion follows master\n"
             "    master_node: Master (independent) node\n"
             "    offset: Offset vector from master to slave [m] as numpy array\n\n"
             "Example:\n"
             "    cog = model.get_or_create_node(5, 5, 2)\n"
             "    foot = model.get_or_create_node(5, 5, 0)\n"
             "    model.add_rigid_link(foot, cog, np.array([0, 0, -2]))")
        .def_readonly("constraints", &grillex::Model::constraints,
             "MPC and rigid link constraint handler")
        .def("remove_element", &grillex::Model::remove_element,
             py::arg("element_id"),
             "Remove a beam element from the model by ID.\n\n"
             "Used for beam subdivision when a beam needs to be split at internal nodes.\n"
             "Returns True if element was found and removed, False otherwise.")
        .def("get_element", &grillex::Model::get_element,
             py::arg("element_id"),
             "Get a beam element by ID.\n\n"
             "Returns the element, or None if not found.",
             py::return_value_policy::reference_internal)
        // Load case management
        .def("create_load_case", &grillex::Model::create_load_case,
             py::arg("name"),
             py::arg("type") = grillex::LoadCaseType::Permanent,
             "Create a new load case",
             py::return_value_policy::reference_internal)
        .def("get_default_load_case", &grillex::Model::get_default_load_case,
             "Get the default load case (auto-created if needed)",
             py::return_value_policy::reference_internal)
        .def("set_active_load_case", &grillex::Model::set_active_load_case,
             py::arg("load_case"),
             "Set the active load case for result queries")
        .def("get_active_load_case", &grillex::Model::get_active_load_case,
             "Get the currently active load case",
             py::return_value_policy::reference_internal)
        .def("get_load_cases", &grillex::Model::get_load_cases,
             "Get all load cases in the model",
             py::return_value_policy::reference_internal)
        .def("get_result", &grillex::Model::get_result,
             py::arg("load_case"),
             "Get result for a specific load case",
             py::return_value_policy::reference_internal)
        .def("get_all_results", &grillex::Model::get_all_results,
             py::return_value_policy::reference_internal,
             "Get all load case results (for use with LoadCombination)")
        .def("analyze", &grillex::Model::analyze,
             "Run analysis for all load cases (linear solver)")
        .def("has_nonlinear_springs", &grillex::Model::has_nonlinear_springs,
             "Check if model has nonlinear (tension/compression-only) springs")
        .def("analyze_nonlinear", &grillex::Model::analyze_nonlinear,
             "Run nonlinear analysis for all load cases.\n\n"
             "Uses iterative solver for tension/compression-only springs.\n"
             "If no nonlinear springs exist, falls back to efficient linear analysis.")
        .def("analyze_combination", &grillex::Model::analyze_combination,
             py::arg("combo"),
             py::arg("settings") = grillex::NonlinearSolverSettings{},
             "Analyze a specific load combination with nonlinear spring support.\n\n"
             "IMPORTANT: With nonlinear springs, load combinations cannot use\n"
             "superposition (summing individual results). Each combination must\n"
             "be solved directly as a single nonlinear problem.\n\n"
             "Static->Dynamic Sequencing: This method first solves the 'static base'\n"
             "(Permanent loads only) to establish the baseline contact pattern,\n"
             "then solves the full combination starting from the static state.")
        .def("nonlinear_settings",
             py::overload_cast<>(&grillex::Model::nonlinear_settings),
             py::return_value_policy::reference_internal,
             "Get nonlinear solver settings (can be modified)")
        .def("is_analyzed", &grillex::Model::is_analyzed,
             "Check if model has been analyzed successfully")
        .def("get_displacements", &grillex::Model::get_displacements,
             "Get global displacement vector (requires analyze() first)")
        .def("get_node_displacement", &grillex::Model::get_node_displacement,
             py::arg("node_id"),
             py::arg("local_dof"),
             "Get displacement at a specific node and DOF")
        .def("get_reactions", &grillex::Model::get_reactions,
             "Get global reaction vector (requires analyze() first)")
        .def("total_dofs", &grillex::Model::total_dofs,
             "Get total number of DOFs in the model")
        .def("get_dof_handler", &grillex::Model::get_dof_handler,
             "Get the DOFHandler (for advanced users)",
             py::return_value_policy::reference_internal)
        .def("get_solver", py::overload_cast<>(&grillex::Model::get_solver),
             "Get the LinearSolver for configuration",
             py::return_value_policy::reference_internal)
        .def("get_error_message", &grillex::Model::get_error_message,
             "Get analysis error message (if analyze() failed)")
        .def("clear", &grillex::Model::clear,
             "Clear the model (remove all entities except nodes)")
        // Eigenvalue analysis methods
        .def("analyze_eigenvalues", &grillex::Model::analyze_eigenvalues,
             py::arg("settings") = grillex::EigensolverSettings{},
             "Run eigenvalue analysis to compute natural frequencies and mode shapes.\n\n"
             "Solves the generalized eigenvalue problem: K*phi = omega^2*M*phi\n\n"
             "Args:\n"
             "    settings: EigensolverSettings with n_modes, tolerance, method, etc.\n\n"
             "Returns:\n"
             "    True if analysis converged successfully\n\n"
             "After analysis, access results via:\n"
             "  - get_eigenvalue_result(): Full EigensolverResult\n"
             "  - get_natural_frequencies(): List of frequencies [Hz]\n"
             "  - get_periods(): List of periods [s]\n"
             "  - get_mode_shape(n): Mode shape vector for mode n (1-based)")
        .def("has_eigenvalue_results", &grillex::Model::has_eigenvalue_results,
             "Check if eigenvalue results are available")
        .def("get_eigenvalue_result", &grillex::Model::get_eigenvalue_result,
             py::return_value_policy::reference_internal,
             "Get eigenvalue analysis results (raises RuntimeError if not available)")
        .def("get_natural_frequencies", &grillex::Model::get_natural_frequencies,
             "Get natural frequencies from eigenvalue analysis [Hz]")
        .def("get_periods", &grillex::Model::get_periods,
             "Get natural periods from eigenvalue analysis [s]")
        .def("get_mode_shape", &grillex::Model::get_mode_shape,
             py::arg("mode_number"),
             "Get mode shape for a specific mode (1-based indexing)\n\n"
             "Returns full mode shape vector (size = total_dofs) with zeros at fixed DOFs.")
        .def("__repr__", [](const grillex::Model &mdl) {
            std::ostringstream oss;
            oss << "<Model nodes=" << mdl.nodes.all_nodes().size()
                << " materials=" << mdl.materials.size()
                << " sections=" << mdl.sections.size()
                << " elements=" << mdl.elements.size()
                << " analyzed=" << (mdl.is_analyzed() ? "True" : "False")
                << ">";
            return oss.str();
        });

    // ========================================================================
    // Phase 6: Multi-Point Constraints (MPC)
    // ========================================================================

    // EqualityConstraint struct
    py::class_<grillex::EqualityConstraint>(m, "EqualityConstraint",
        "Equality constraint between two DOFs (slave = master)")
        .def(py::init<int, int, int, int>(),
             py::arg("slave_node"),
             py::arg("slave_dof"),
             py::arg("master_node"),
             py::arg("master_dof"),
             "Construct an equality constraint")
        .def_readwrite("slave_node_id", &grillex::EqualityConstraint::slave_node_id,
                       "Node ID of the slave DOF")
        .def_readwrite("slave_local_dof", &grillex::EqualityConstraint::slave_local_dof,
                       "Local DOF index at slave (0-6)")
        .def_readwrite("master_node_id", &grillex::EqualityConstraint::master_node_id,
                       "Node ID of the master DOF")
        .def_readwrite("master_local_dof", &grillex::EqualityConstraint::master_local_dof,
                       "Local DOF index at master (0-6)")
        .def("__repr__", [](const grillex::EqualityConstraint &eq) {
            return "<EqualityConstraint slave=(" + std::to_string(eq.slave_node_id) +
                   "," + std::to_string(eq.slave_local_dof) + ") master=(" +
                   std::to_string(eq.master_node_id) + "," +
                   std::to_string(eq.master_local_dof) + ")>";
        });

    // RigidLink struct
    py::class_<grillex::RigidLink>(m, "RigidLink",
        "Rigid link constraint between master and slave nodes.\n\n"
        "Kinematics:\n"
        "  u_slave = u_master + θ_master × offset\n"
        "  θ_slave = θ_master")
        .def(py::init<int, int, const Eigen::Vector3d&>(),
             py::arg("slave_node"),
             py::arg("master_node"),
             py::arg("offset"),
             "Construct a rigid link")
        .def_readwrite("slave_node_id", &grillex::RigidLink::slave_node_id,
                       "Node ID of the slave (constrained) node")
        .def_readwrite("master_node_id", &grillex::RigidLink::master_node_id,
                       "Node ID of the master (independent) node")
        .def_readwrite("offset", &grillex::RigidLink::offset,
                       "Offset from master to slave in global coords [m]")
        .def("skew_matrix", &grillex::RigidLink::skew_matrix,
             "Get 3x3 skew-symmetric matrix for θ × r computation")
        .def("transformation_block_6x6", &grillex::RigidLink::transformation_block_6x6,
             "Get full 6x6 transformation block: [u_S; θ_S] = T * [u_M; θ_M]\n\n"
             "Returns:\n"
             "  T = [I  R]  where I is 3x3 identity\n"
             "      [0  I]  and R is skew-symmetric matrix")
        .def("__repr__", [](const grillex::RigidLink &rl) {
            return "<RigidLink slave=" + std::to_string(rl.slave_node_id) +
                   " master=" + std::to_string(rl.master_node_id) +
                   " offset=(" + std::to_string(rl.offset.x()) + "," +
                   std::to_string(rl.offset.y()) + "," +
                   std::to_string(rl.offset.z()) + ")>";
        });

    // ConstraintHandler::ReducedSystem struct
    py::class_<grillex::ConstraintHandler::ReducedSystem>(m, "ReducedSystem",
        "Result of system reduction with MPC constraints")
        .def_readonly("K_reduced", &grillex::ConstraintHandler::ReducedSystem::K_reduced,
                     "Reduced stiffness matrix")
        .def_readonly("F_reduced", &grillex::ConstraintHandler::ReducedSystem::F_reduced,
                     "Reduced force vector")
        .def_readonly("T", &grillex::ConstraintHandler::ReducedSystem::T,
                     "Transformation matrix (u_full = T * u_reduced)")
        .def_readonly("n_full", &grillex::ConstraintHandler::ReducedSystem::n_full,
                     "Number of full DOFs")
        .def_readonly("n_reduced", &grillex::ConstraintHandler::ReducedSystem::n_reduced,
                     "Number of reduced (independent) DOFs")
        .def("__repr__", [](const grillex::ConstraintHandler::ReducedSystem &rs) {
            return "<ReducedSystem full=" + std::to_string(rs.n_full) +
                   " reduced=" + std::to_string(rs.n_reduced) + ">";
        });

    // ConstraintHandler class
    py::class_<grillex::ConstraintHandler>(m, "ConstraintHandler",
        "Multi-point constraint (MPC) handler for structural analysis.\n\n"
        "Manages kinematic constraints between DOFs:\n"
        "- Equality constraints: u_slave = u_master\n"
        "- Rigid links: slave motion determined by master via rigid body kinematics\n\n"
        "Uses transformation matrix approach:\n"
        "  u_full = T * u_reduced\n"
        "  K_reduced = T^T * K * T\n"
        "  F_reduced = T^T * F\n\n"
        "Example:\n"
        "    constraints = ConstraintHandler()\n"
        "    constraints.add_rigid_link(slave_node, master_node, offset)\n"
        "    reduced = constraints.reduce_system(K, F, dof_handler)\n"
        "    u_red = solver.solve(reduced.K_reduced, reduced.F_reduced)\n"
        "    u_full = constraints.expand_displacements(u_red, reduced.T)")
        .def(py::init<>(), "Construct an empty constraint handler")
        .def("add_equality_constraint", &grillex::ConstraintHandler::add_equality_constraint,
             py::arg("slave_node"),
             py::arg("slave_dof"),
             py::arg("master_node"),
             py::arg("master_dof"),
             "Add a simple equality constraint: u_slave = u_master")
        .def("add_rigid_link", &grillex::ConstraintHandler::add_rigid_link,
             py::arg("slave_node"),
             py::arg("master_node"),
             py::arg("offset"),
             "Add a rigid link constraint (constrains all 6 DOFs of slave node)")
        .def("build_transformation_matrix", &grillex::ConstraintHandler::build_transformation_matrix,
             py::arg("dof_handler"),
             "Build the transformation matrix T (u_full = T * u_reduced)")
        .def("reduce_system", &grillex::ConstraintHandler::reduce_system,
             py::arg("K"),
             py::arg("F"),
             py::arg("dof_handler"),
             "Reduce system using constraints: K_red = T^T * K * T, F_red = T^T * F")
        .def("expand_displacements", &grillex::ConstraintHandler::expand_displacements,
             py::arg("u_reduced"),
             py::arg("T"),
             "Expand reduced displacements to full: u_full = T * u_reduced")
        .def("get_equality_constraints", &grillex::ConstraintHandler::get_equality_constraints,
             py::return_value_policy::reference_internal,
             "Get all equality constraints")
        .def("get_rigid_links", &grillex::ConstraintHandler::get_rigid_links,
             py::return_value_policy::reference_internal,
             "Get all rigid link constraints")
        .def("num_slave_dofs", &grillex::ConstraintHandler::num_slave_dofs,
             py::arg("dof_handler"),
             "Get number of slave (dependent) DOFs")
        .def("has_constraints", &grillex::ConstraintHandler::has_constraints,
             "Check if any constraints are defined")
        .def("clear", &grillex::ConstraintHandler::clear,
             "Clear all constraints")
        .def("__repr__", [](const grillex::ConstraintHandler &ch) {
            return "<ConstraintHandler equalities=" +
                   std::to_string(ch.get_equality_constraints().size()) +
                   " rigid_links=" +
                   std::to_string(ch.get_rigid_links().size()) + ">";
        });

    // ========================================================================
    // Phase 7: Internal Actions
    // ========================================================================

    // ReleaseCombo4DOF enum
    py::enum_<grillex::ReleaseCombo4DOF>(m, "ReleaseCombo4DOF",
        "Release combinations for bending (4-DOF: w1, φ1, w2, φ2).\n\n"
        "Each combination represents which DOFs are FIXED (active) vs FREE (released)\n"
        "at the two ends of a beam for a single bending plane.")
        .value("FIXED_FIXED_FIXED_FIXED", grillex::ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED,
               "All DOFs fixed (standard continuous beam)")
        .value("FIXED_FIXED_FREE_FIXED", grillex::ReleaseCombo4DOF::FIXED_FIXED_FREE_FIXED)
        .value("FIXED_FIXED_FIXED_FREE", grillex::ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE,
               "Hinge at end j (propped cantilever)")
        .value("FIXED_FIXED_FREE_FREE", grillex::ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE,
               "Cantilever from end i")
        .value("FIXED_FREE_FIXED_FIXED", grillex::ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED,
               "Hinge at end i")
        .value("FIXED_FREE_FREE_FIXED", grillex::ReleaseCombo4DOF::FIXED_FREE_FREE_FIXED)
        .value("FIXED_FREE_FIXED_FREE", grillex::ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE,
               "Simply supported (double hinge)")
        .value("FIXED_FREE_FREE_FREE", grillex::ReleaseCombo4DOF::FIXED_FREE_FREE_FREE)
        .value("FREE_FIXED_FIXED_FIXED", grillex::ReleaseCombo4DOF::FREE_FIXED_FIXED_FIXED)
        .value("FREE_FIXED_FREE_FIXED", grillex::ReleaseCombo4DOF::FREE_FIXED_FREE_FIXED)
        .value("FREE_FIXED_FIXED_FREE", grillex::ReleaseCombo4DOF::FREE_FIXED_FIXED_FREE)
        .value("FREE_FIXED_FREE_FREE", grillex::ReleaseCombo4DOF::FREE_FIXED_FREE_FREE)
        .value("FREE_FREE_FIXED_FIXED", grillex::ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED,
               "Cantilever from end j")
        .value("FREE_FREE_FREE_FIXED", grillex::ReleaseCombo4DOF::FREE_FREE_FREE_FIXED)
        .value("FREE_FREE_FIXED_FREE", grillex::ReleaseCombo4DOF::FREE_FREE_FIXED_FREE)
        .value("FREE_FREE_FREE_FREE", grillex::ReleaseCombo4DOF::FREE_FREE_FREE_FREE,
               "Unstable (rigid body motion)")
        .export_values();

    // ReleaseCombo2DOF enum
    py::enum_<grillex::ReleaseCombo2DOF>(m, "ReleaseCombo2DOF",
        "Release combinations for axial/torsion (2-DOF: u1/θ1, u2/θ2).")
        .value("FIXED_FIXED", grillex::ReleaseCombo2DOF::FIXED_FIXED,
               "Both ends fixed (standard)")
        .value("FIXED_FREE", grillex::ReleaseCombo2DOF::FIXED_FREE,
               "Start fixed, end free")
        .value("FREE_FIXED", grillex::ReleaseCombo2DOF::FREE_FIXED,
               "Start free, end fixed")
        .value("FREE_FREE", grillex::ReleaseCombo2DOF::FREE_FREE,
               "Unstable (rigid body motion)")
        .export_values();

    // ReleaseComboWarping enum
    py::enum_<grillex::ReleaseComboWarping>(m, "ReleaseComboWarping",
        "Release combinations for warping (4-DOF: θ₁, φ₁, θ₂, φ₂).\n\n"
        "For thin-walled open sections with warping DOF:\n"
        "- θ = twist angle (rotation DOF)\n"
        "- φ = rate of twist / warping DOF (dθ/dx)")
        .value("FIXED_FIXED_FIXED_FIXED", grillex::ReleaseComboWarping::FIXED_FIXED_FIXED_FIXED,
               "All fixed (full warping restraint)")
        .value("FIXED_FIXED_FIXED_FREE", grillex::ReleaseComboWarping::FIXED_FIXED_FIXED_FREE)
        .value("FIXED_FIXED_FREE_FIXED", grillex::ReleaseComboWarping::FIXED_FIXED_FREE_FIXED)
        .value("FIXED_FIXED_FREE_FREE", grillex::ReleaseComboWarping::FIXED_FIXED_FREE_FREE,
               "Cantilever (warping free at tip)")
        .value("FIXED_FREE_FIXED_FIXED", grillex::ReleaseComboWarping::FIXED_FREE_FIXED_FIXED)
        .value("FIXED_FREE_FIXED_FREE", grillex::ReleaseComboWarping::FIXED_FREE_FIXED_FREE,
               "Pure St. Venant (no warping restraint)")
        .value("FIXED_FREE_FREE_FIXED", grillex::ReleaseComboWarping::FIXED_FREE_FREE_FIXED)
        .value("FIXED_FREE_FREE_FREE", grillex::ReleaseComboWarping::FIXED_FREE_FREE_FREE)
        .value("FREE_FIXED_FIXED_FIXED", grillex::ReleaseComboWarping::FREE_FIXED_FIXED_FIXED)
        .value("FREE_FIXED_FIXED_FREE", grillex::ReleaseComboWarping::FREE_FIXED_FIXED_FREE)
        .value("FREE_FIXED_FREE_FIXED", grillex::ReleaseComboWarping::FREE_FIXED_FREE_FIXED)
        .value("FREE_FIXED_FREE_FREE", grillex::ReleaseComboWarping::FREE_FIXED_FREE_FREE)
        .value("FREE_FREE_FIXED_FIXED", grillex::ReleaseComboWarping::FREE_FREE_FIXED_FIXED,
               "Reverse cantilever")
        .value("FREE_FREE_FIXED_FREE", grillex::ReleaseComboWarping::FREE_FREE_FIXED_FREE)
        .value("FREE_FREE_FREE_FIXED", grillex::ReleaseComboWarping::FREE_FREE_FREE_FIXED)
        .value("FREE_FREE_FREE_FREE", grillex::ReleaseComboWarping::FREE_FREE_FREE_FREE,
               "All free (rigid body)")
        .export_values();

    // DisplacementLine struct
    py::class_<grillex::DisplacementLine>(m, "DisplacementLine",
        "Displacements and rotations at a position along the beam.\n\n"
        "Used for deflection diagrams and displacement queries.")
        .def(py::init<>(), "Construct with zero displacements")
        .def(py::init<double>(), py::arg("x"),
             "Construct at position x with zero displacements")
        .def_readwrite("x", &grillex::DisplacementLine::x,
                       "Position along beam [0, L] in meters")
        .def_readwrite("u", &grillex::DisplacementLine::u,
                       "Axial displacement [m]")
        .def_readwrite("v", &grillex::DisplacementLine::v,
                       "Lateral displacement in y [m]")
        .def_readwrite("w", &grillex::DisplacementLine::w,
                       "Lateral displacement in z [m]")
        .def_readwrite("theta_x", &grillex::DisplacementLine::theta_x,
                       "Twist rotation (torsion) [rad]")
        .def_readwrite("theta_y", &grillex::DisplacementLine::theta_y,
                       "Bending rotation about y [rad]")
        .def_readwrite("theta_z", &grillex::DisplacementLine::theta_z,
                       "Bending rotation about z [rad]")
        .def_readwrite("phi_prime", &grillex::DisplacementLine::phi_prime,
                       "Warping parameter (rate of twist) [rad]")
        .def("__repr__", [](const grillex::DisplacementLine &d) {
            std::ostringstream oss;
            oss << "<DisplacementLine x=" << d.x
                << " u=" << d.u << " v=" << d.v << " w=" << d.w << ">";
            return oss.str();
        });

    // EndForces struct
    py::class_<grillex::EndForces>(m, "EndForces",
        "Element end forces in local coordinates.\n\n"
        "Sign conventions:\n"
        "- Axial N: positive = tension\n"
        "- Shear V: positive in positive local y/z direction\n"
        "- Moment M: positive per right-hand rule")
        .def(py::init<>(), "Construct with zero forces")
        .def(py::init<double, double, double, double, double, double, double>(),
             py::arg("N"), py::arg("Vy"), py::arg("Vz"),
             py::arg("Mx"), py::arg("My"), py::arg("Mz"),
             py::arg("B") = 0.0,
             "Construct from individual components")
        .def_readwrite("N", &grillex::EndForces::N,
                       "Axial force [kN] (positive = tension)")
        .def_readwrite("Vy", &grillex::EndForces::Vy,
                       "Shear force in local y [kN]")
        .def_readwrite("Vz", &grillex::EndForces::Vz,
                       "Shear force in local z [kN]")
        .def_readwrite("Mx", &grillex::EndForces::Mx,
                       "Torsion moment [kN·m]")
        .def_readwrite("My", &grillex::EndForces::My,
                       "Bending moment about local y [kN·m]")
        .def_readwrite("Mz", &grillex::EndForces::Mz,
                       "Bending moment about local z [kN·m]")
        .def_readwrite("B", &grillex::EndForces::B,
                       "Bimoment [kN·m²] (for 14-DOF warping elements)")
        .def("to_vector6", &grillex::EndForces::to_vector6,
             "Convert to 6-component vector [N, Vy, Vz, Mx, My, Mz]")
        .def("to_vector7", &grillex::EndForces::to_vector7,
             "Convert to 7-component vector [N, Vy, Vz, Mx, My, Mz, B]")
        .def("__repr__", [](const grillex::EndForces &f) {
            std::ostringstream oss;
            oss << "<EndForces N=" << f.N << " Vy=" << f.Vy << " Vz=" << f.Vz
                << " Mx=" << f.Mx << " My=" << f.My << " Mz=" << f.Mz;
            if (std::abs(f.B) > 1e-10) {
                oss << " B=" << f.B;
            }
            oss << ">";
            return oss.str();
        });

    // InternalActions struct
    py::class_<grillex::InternalActions>(m, "InternalActions",
        "Internal actions at a position along the beam.\n\n"
        "Represents internal forces and moments at any position x along the element.")
        .def(py::init<>(), "Construct with zero actions")
        .def(py::init<double>(), py::arg("x"),
             "Construct at position x with zero actions")
        .def(py::init<double, double, double, double, double, double, double>(),
             py::arg("x"), py::arg("N"), py::arg("Vy"), py::arg("Vz"),
             py::arg("Mx"), py::arg("My"), py::arg("Mz"),
             "Construct with all values")
        .def_readwrite("x", &grillex::InternalActions::x,
                       "Position along beam [0, L] in meters")
        .def_readwrite("N", &grillex::InternalActions::N,
                       "Axial force [kN]")
        .def_readwrite("Vy", &grillex::InternalActions::Vy,
                       "Shear force in y [kN]")
        .def_readwrite("Vz", &grillex::InternalActions::Vz,
                       "Shear force in z [kN]")
        .def_readwrite("Mx", &grillex::InternalActions::Mx,
                       "Torsion moment [kN·m]")
        .def_readwrite("My", &grillex::InternalActions::My,
                       "Moment about y [kN·m]")
        .def_readwrite("Mz", &grillex::InternalActions::Mz,
                       "Moment about z [kN·m]")
        .def("__repr__", [](const grillex::InternalActions &a) {
            std::ostringstream oss;
            oss << "<InternalActions x=" << a.x
                << " N=" << a.N << " Vy=" << a.Vy << " Vz=" << a.Vz
                << " Mx=" << a.Mx << " My=" << a.My << " Mz=" << a.Mz << ">";
            return oss.str();
        });

    // WarpingInternalActions struct (inherits from InternalActions)
    py::class_<grillex::WarpingInternalActions, grillex::InternalActions>(m, "WarpingInternalActions",
        "Warping-specific internal actions (extends InternalActions).\n\n"
        "For thin-walled open sections (I-beams, channels) under torsion:\n"
        "- St. Venant torsion: Mx_sv = GJ × dθ/dx\n"
        "- Warping torsion: Mx_w = -EIw × d³θ/dx³\n"
        "- Total torsion: Mx = Mx_sv + Mx_w\n"
        "- Bimoment: B = -EIw × d²θ/dx²\n"
        "- Warping stress: σ_w = -B × ω / Iw")
        .def(py::init<>(), "Construct with zero actions")
        .def(py::init<double>(), py::arg("x"),
             "Construct at position x with zero actions")
        .def_readwrite("B", &grillex::WarpingInternalActions::B,
                       "Bimoment [kN·m²]")
        .def_readwrite("Mx_sv", &grillex::WarpingInternalActions::Mx_sv,
                       "St. Venant torsion component [kN·m]")
        .def_readwrite("Mx_w", &grillex::WarpingInternalActions::Mx_w,
                       "Warping torsion component [kN·m]")
        .def_readwrite("sigma_w_max", &grillex::WarpingInternalActions::sigma_w_max,
                       "Maximum warping normal stress [kN/m²]")
        .def("__repr__", [](const grillex::WarpingInternalActions &a) {
            std::ostringstream oss;
            oss << "<WarpingInternalActions x=" << a.x
                << " Mx=" << a.Mx << " B=" << a.B
                << " Mx_sv=" << a.Mx_sv << " Mx_w=" << a.Mx_w
                << " σ_w=" << a.sigma_w_max << ">";
            return oss.str();
        });

    // ActionExtreme struct
    py::class_<grillex::ActionExtreme>(m, "ActionExtreme",
        "Extremum location and value for moment/shear along beam elements.")
        .def(py::init<>(), "Construct with zero position and value")
        .def(py::init<double, double>(),
             py::arg("x"), py::arg("value"),
             "Construct with position and value")
        .def_readwrite("x", &grillex::ActionExtreme::x,
                       "Position along beam [m]")
        .def_readwrite("value", &grillex::ActionExtreme::value,
                       "Value at extremum")
        .def("__repr__", [](const grillex::ActionExtreme &e) {
            return "<ActionExtreme x=" + std::to_string(e.x) +
                   " value=" + std::to_string(e.value) + ">";
        });

    // ========================================================================
    // Phase 8: Additional Element Types
    // ========================================================================

    // LoadingCondition enum for spring elements
    py::enum_<grillex::LoadingCondition>(m, "LoadingCondition",
        "Loading condition for spring elements.\n\n"
        "Controls when a spring is active based on load case type.\n"
        "Used for modeling cargo connections that behave differently\n"
        "for static (set-down) vs dynamic (environmental) loads.\n\n"
        "- All: Active for all load cases (default)\n"
        "- Static: Only active for Permanent load cases (e.g., bearing pads)\n"
        "- Dynamic: Only active for Variable/Environmental load cases (e.g., seafastening)")
        .value("All", grillex::LoadingCondition::All,
               "Active for all load cases (default)")
        .value("Static", grillex::LoadingCondition::Static,
               "Only active for Permanent load cases (gravity, dead load)")
        .value("Dynamic", grillex::LoadingCondition::Dynamic,
               "Only active for Variable/Environmental/Accidental load cases")
        .export_values();

    // SpringBehavior enum (Phase 15)
    py::enum_<grillex::SpringBehavior>(m, "SpringBehavior",
        "Spring behavior type for nonlinear analysis.\n\n"
        "- Linear: Always active (default)\n"
        "- TensionOnly: Active only when elongated (δ > gap)\n"
        "- CompressionOnly: Active only when compressed (δ < -gap)")
        .value("Linear", grillex::SpringBehavior::Linear,
               "Always active (default)")
        .value("TensionOnly", grillex::SpringBehavior::TensionOnly,
               "Active only when elongated (δ > gap)")
        .value("CompressionOnly", grillex::SpringBehavior::CompressionOnly,
               "Active only when compressed (δ < -gap)")
        .export_values();

    // SpringElement class
    py::class_<grillex::SpringElement>(m, "SpringElement",
        "Spring element connecting two nodes with independent stiffness for each DOF.\n\n"
        "Stiffness values:\n"
        "- kx, ky, kz: Translational stiffness [kN/m]\n"
        "- krx, kry, krz: Rotational stiffness [kN·m/rad]")
        .def(py::init<int, grillex::Node*, grillex::Node*>(),
             py::arg("id"), py::arg("node_i"), py::arg("node_j"),
             "Construct a spring element between two nodes")
        .def_readwrite("id", &grillex::SpringElement::id, "Element ID")
        .def_readonly("node_i", &grillex::SpringElement::node_i, "Start node")
        .def_readonly("node_j", &grillex::SpringElement::node_j, "End node")
        .def_readwrite("kx", &grillex::SpringElement::kx, "Translational stiffness in X [kN/m]")
        .def_readwrite("ky", &grillex::SpringElement::ky, "Translational stiffness in Y [kN/m]")
        .def_readwrite("kz", &grillex::SpringElement::kz, "Translational stiffness in Z [kN/m]")
        .def_readwrite("krx", &grillex::SpringElement::krx, "Rotational stiffness about X [kN·m/rad]")
        .def_readwrite("kry", &grillex::SpringElement::kry, "Rotational stiffness about Y [kN·m/rad]")
        .def_readwrite("krz", &grillex::SpringElement::krz, "Rotational stiffness about Z [kN·m/rad]")
        .def("num_dofs", &grillex::SpringElement::num_dofs, "Get number of DOFs (always 12)")
        .def("has_warping", &grillex::SpringElement::has_warping, "Check for warping DOF (always false)")
        .def("global_stiffness_matrix", &grillex::SpringElement::global_stiffness_matrix,
             "Get 12x12 global stiffness matrix")
        .def("global_mass_matrix", &grillex::SpringElement::global_mass_matrix,
             "Get 12x12 global mass matrix (zeros - springs are massless)")
        .def("set_translational_stiffness", &grillex::SpringElement::set_translational_stiffness,
             py::arg("kx"), py::arg("ky"), py::arg("kz"),
             "Set all translational stiffnesses at once")
        .def("set_rotational_stiffness", &grillex::SpringElement::set_rotational_stiffness,
             py::arg("krx"), py::arg("kry"), py::arg("krz"),
             "Set all rotational stiffnesses at once")
        .def("has_stiffness", &grillex::SpringElement::has_stiffness,
             "Check if element has any non-zero stiffness")
        .def_readwrite("loading_condition", &grillex::SpringElement::loading_condition,
             "Loading condition controlling when this spring is active.\n\n"
             "- LoadingCondition.All: Active for all load cases (default)\n"
             "- LoadingCondition.Static: Only for Permanent load cases\n"
             "- LoadingCondition.Dynamic: Only for Variable/Environmental load cases")
        .def("is_active_for_load_case", &grillex::SpringElement::is_active_for_load_case,
             py::arg("load_case_type"),
             "Check if this spring is active for a given load case type.\n\n"
             "Args:\n"
             "    load_case_type: The LoadCaseType to check against\n\n"
             "Returns:\n"
             "    True if this spring should contribute to the load case")
        // === Nonlinear spring properties and methods (Phase 15) ===
        .def_readwrite("behavior", &grillex::SpringElement::behavior,
             "Per-DOF behavior type array [6].\n\n"
             "DOF indices: 0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ")
        .def_readwrite("gap", &grillex::SpringElement::gap,
             "Per-DOF gap values array [6].\n\n"
             "Units: [m] for translation (0-2), [rad] for rotation (3-5)")
        .def_readwrite("is_active", &grillex::SpringElement::is_active,
             "Per-DOF active state array [6].\n\n"
             "Updated by update_state() during nonlinear iteration.")
        .def_readonly("deformation", &grillex::SpringElement::deformation,
             "Per-DOF deformation array [6]: δ = u_j - u_i.\n\n"
             "Units: [m] for translation, [rad] for rotation.")
        .def("update_state", &grillex::SpringElement::update_state,
             py::arg("displacements"), py::arg("dof_handler"),
             "Update spring state based on current displacements.\n\n"
             "Computes deformation and updates is_active based on behavior and gap.")
        .def("state_changed", &grillex::SpringElement::state_changed,
             "Check if any DOF state changed in last update_state() call.")
        .def("has_gap", &grillex::SpringElement::has_gap,
             "Check if any DOF has a non-zero gap.")
        .def("is_nonlinear", &grillex::SpringElement::is_nonlinear,
             "Check if any DOF has nonlinear behavior (not Linear).")
        .def("compute_forces", &grillex::SpringElement::compute_forces,
             "Compute spring forces for each DOF [kN or kN·m].\n\n"
             "Returns array of 6 force values.")
        .def("compute_gap_forces", &grillex::SpringElement::compute_gap_forces,
             "Compute gap closure forces for solver RHS (12x1 vector).\n\n"
             "Returns force offset term for active gap springs.")
        .def("current_stiffness_matrix", &grillex::SpringElement::current_stiffness_matrix,
             "Get 12x12 stiffness matrix respecting current active state.\n\n"
             "Inactive DOFs have zero contribution.")
        .def("set_behavior", &grillex::SpringElement::set_behavior,
             py::arg("dof"), py::arg("behavior"),
             "Set behavior for a specific DOF (0-5).")
        .def("set_all_behavior", &grillex::SpringElement::set_all_behavior,
             py::arg("behavior"),
             "Set behavior for all DOFs at once.")
        .def("set_gap", &grillex::SpringElement::set_gap,
             py::arg("dof"), py::arg("gap"),
             "Set gap for a specific DOF (0-5).\n\n"
             "Units: [m] for translation, [rad] for rotation.")
        .def("set_all_gaps", &grillex::SpringElement::set_all_gaps,
             py::arg("gap"),
             "Set gap for all DOFs at once.")
        .def("get_gap_tolerance", &grillex::SpringElement::get_gap_tolerance,
             "Get the gap tolerance used for state determination [m].")
        .def("set_gap_tolerance", &grillex::SpringElement::set_gap_tolerance,
             py::arg("tolerance"),
             "Set the gap tolerance used for state determination [m].")
        .def("update_state_with_hysteresis", &grillex::SpringElement::update_state_with_hysteresis,
             py::arg("displacements"), py::arg("dof_handler"), py::arg("hysteresis_band"),
             "Update spring state with hysteresis band.\n\n"
             "Uses different thresholds for activation vs deactivation:\n"
             "- Activate when deformation > gap + hysteresis_band\n"
             "- Deactivate when deformation < gap - hysteresis_band")
        .def("set_oscillating", &grillex::SpringElement::set_oscillating,
             py::arg("oscillating"),
             "Mark this spring as oscillating (for partial stiffness).")
        .def("is_oscillating", &grillex::SpringElement::is_oscillating,
             "Check if spring is marked as oscillating.")
        .def("__repr__", [](const grillex::SpringElement &s) {
            std::string cond_str;
            switch (s.loading_condition) {
                case grillex::LoadingCondition::All: cond_str = "All"; break;
                case grillex::LoadingCondition::Static: cond_str = "Static"; break;
                case grillex::LoadingCondition::Dynamic: cond_str = "Dynamic"; break;
            }
            return "<SpringElement id=" + std::to_string(s.id) +
                   " kx=" + std::to_string(s.kx) +
                   " ky=" + std::to_string(s.ky) +
                   " kz=" + std::to_string(s.kz) +
                   " loading_condition=" + cond_str + ">";
        });

    // PointMass class
    py::class_<grillex::PointMass>(m, "PointMass",
        "Point mass element at a single node.\n\n"
        "Properties:\n"
        "- mass: Translational mass [mT]\n"
        "- Ixx, Iyy, Izz: Moments of inertia [mT·m²]\n"
        "- Ixy, Ixz, Iyz: Products of inertia [mT·m²]")
        .def(py::init<int, grillex::Node*>(),
             py::arg("id"), py::arg("node"),
             "Construct a point mass at a node")
        .def_readwrite("id", &grillex::PointMass::id, "Element ID")
        .def_readonly("node", &grillex::PointMass::node, "Associated node")
        .def_readwrite("mass", &grillex::PointMass::mass, "Translational mass [mT]")
        .def_readwrite("Ixx", &grillex::PointMass::Ixx, "Moment of inertia about X [mT·m²]")
        .def_readwrite("Iyy", &grillex::PointMass::Iyy, "Moment of inertia about Y [mT·m²]")
        .def_readwrite("Izz", &grillex::PointMass::Izz, "Moment of inertia about Z [mT·m²]")
        .def_readwrite("Ixy", &grillex::PointMass::Ixy, "Product of inertia XY [mT·m²]")
        .def_readwrite("Ixz", &grillex::PointMass::Ixz, "Product of inertia XZ [mT·m²]")
        .def_readwrite("Iyz", &grillex::PointMass::Iyz, "Product of inertia YZ [mT·m²]")
        .def("num_dofs", &grillex::PointMass::num_dofs, "Get number of DOFs (always 6)")
        .def("has_warping", &grillex::PointMass::has_warping, "Check for warping DOF (always false)")
        .def("mass_matrix", &grillex::PointMass::mass_matrix,
             "Get 6x6 mass matrix including full inertia tensor")
        .def("set_mass", &grillex::PointMass::set_mass,
             py::arg("m"), "Set translational mass [mT]")
        .def("set_inertia", &grillex::PointMass::set_inertia,
             py::arg("ixx"), py::arg("iyy"), py::arg("izz"),
             "Set diagonal moments of inertia")
        .def("set_products_of_inertia", &grillex::PointMass::set_products_of_inertia,
             py::arg("ixy"), py::arg("ixz"), py::arg("iyz"),
             "Set products of inertia (off-diagonal terms)")
        .def("set_full_inertia", &grillex::PointMass::set_full_inertia,
             py::arg("ixx"), py::arg("iyy"), py::arg("izz"),
             py::arg("ixy"), py::arg("ixz"), py::arg("iyz"),
             "Set full inertia tensor at once")
        .def("is_valid", &grillex::PointMass::is_valid,
             "Check if mass matrix is physically valid")
        .def("get_total_mass", &grillex::PointMass::get_total_mass,
             "Get total translational mass [mT]")
        .def("__repr__", [](const grillex::PointMass &pm) {
            return "<PointMass id=" + std::to_string(pm.id) +
                   " mass=" + std::to_string(pm.mass) +
                   " Ixx=" + std::to_string(pm.Ixx) +
                   " Iyy=" + std::to_string(pm.Iyy) +
                   " Izz=" + std::to_string(pm.Izz) + ">";
        });

    // PlateElement class
    py::class_<grillex::PlateElement>(m, "PlateElement",
        "4-node Mindlin plate element (MITC4 formulation).\n\n"
        "A plate element for bending analysis using Mindlin plate theory\n"
        "with MITC4 interpolation to avoid shear locking.\n\n"
        "Node numbering (natural coordinates):\n"
        "   4 (-1,+1) -------- 3 (+1,+1)\n"
        "       |                  |\n"
        "       |     (0,0)        |\n"
        "       |                  |\n"
        "   1 (-1,-1) -------- 2 (+1,-1)\n\n"
        "Properties:\n"
        "- thickness: Plate thickness [m]\n"
        "- material: Material properties (E, nu, rho)")
        .def(py::init<int, grillex::Node*, grillex::Node*, grillex::Node*,
                      grillex::Node*, double, grillex::Material*>(),
             py::arg("id"), py::arg("n1"), py::arg("n2"), py::arg("n3"),
             py::arg("n4"), py::arg("thickness"), py::arg("material"),
             "Construct a plate element with 4 corner nodes")
        .def_readwrite("id", &grillex::PlateElement::id, "Element ID")
        .def_readonly("nodes", &grillex::PlateElement::nodes, "Array of 4 corner nodes")
        .def_readwrite("thickness", &grillex::PlateElement::thickness, "Plate thickness [m]")
        .def_readonly("material", &grillex::PlateElement::material, "Material properties")
        .def_readonly("x_axis", &grillex::PlateElement::x_axis, "Local x-axis (from node 1 to node 2)")
        .def_readonly("y_axis", &grillex::PlateElement::y_axis, "Local y-axis")
        .def_readonly("z_axis", &grillex::PlateElement::z_axis, "Local z-axis (plate normal)")
        .def("num_dofs", &grillex::PlateElement::num_dofs, "Get number of DOFs (always 24)")
        .def("has_warping", &grillex::PlateElement::has_warping, "Check for warping DOF (always false)")
        .def("global_stiffness_matrix", &grillex::PlateElement::global_stiffness_matrix,
             "Get 24x24 global stiffness matrix")
        .def("global_mass_matrix", &grillex::PlateElement::global_mass_matrix,
             "Get 24x24 global mass matrix (lumped)")
        .def("area", &grillex::PlateElement::area, "Get plate element area [m²]")
        .def("centroid", &grillex::PlateElement::centroid, "Get centroid position in global coordinates")
        .def("to_local", &grillex::PlateElement::to_local,
             py::arg("global_vec"),
             "Transform vector from global to local coordinates")
        .def("to_global", &grillex::PlateElement::to_global,
             py::arg("local_vec"),
             "Transform vector from local to global coordinates")
        .def("__repr__", [](const grillex::PlateElement &p) {
            return "<PlateElement id=" + std::to_string(p.id) +
                   " thickness=" + std::to_string(p.thickness) +
                   " area=" + std::to_string(p.area()) + ">";
        });

    // PlateElement8 class (8-node serendipity)
    py::class_<grillex::PlateElement8>(m, "PlateElement8",
        "8-node serendipity Mindlin plate element (MITC8 formulation).\n\n"
        "A higher-order plate element for bending analysis using Mindlin plate theory\n"
        "with quadratic shape functions (serendipity family).\n\n"
        "Node numbering (natural coordinates):\n"
        "   4 (-1,+1) --- 7 (0,+1) --- 3 (+1,+1)\n"
        "       |                          |\n"
        "   8 (-1,0)                    6 (+1,0)\n"
        "       |                          |\n"
        "   1 (-1,-1) --- 5 (0,-1) --- 2 (+1,-1)\n\n"
        "Properties:\n"
        "- thickness: Plate thickness [m]\n"
        "- material: Material properties (E, nu, rho)")
        .def(py::init<int, grillex::Node*, grillex::Node*, grillex::Node*,
                      grillex::Node*, grillex::Node*, grillex::Node*,
                      grillex::Node*, grillex::Node*, double, grillex::Material*>(),
             py::arg("id"), py::arg("n1"), py::arg("n2"), py::arg("n3"),
             py::arg("n4"), py::arg("n5"), py::arg("n6"), py::arg("n7"),
             py::arg("n8"), py::arg("thickness"), py::arg("material"),
             "Construct an 8-node plate element")
        .def_readwrite("id", &grillex::PlateElement8::id, "Element ID")
        .def_readonly("nodes", &grillex::PlateElement8::nodes, "Array of 8 nodes")
        .def_readwrite("thickness", &grillex::PlateElement8::thickness, "Plate thickness [m]")
        .def_readonly("material", &grillex::PlateElement8::material, "Material properties")
        .def_readonly("x_axis", &grillex::PlateElement8::x_axis, "Local x-axis")
        .def_readonly("y_axis", &grillex::PlateElement8::y_axis, "Local y-axis")
        .def_readonly("z_axis", &grillex::PlateElement8::z_axis, "Local z-axis (plate normal)")
        .def("num_dofs", &grillex::PlateElement8::num_dofs, "Get number of DOFs (always 48)")
        .def("has_warping", &grillex::PlateElement8::has_warping, "Check for warping DOF (always false)")
        .def("global_stiffness_matrix", &grillex::PlateElement8::global_stiffness_matrix,
             "Get 48x48 global stiffness matrix")
        .def("global_mass_matrix", &grillex::PlateElement8::global_mass_matrix,
             "Get 48x48 global mass matrix (lumped)")
        .def("area", &grillex::PlateElement8::area, "Get plate element area [m²]")
        .def("centroid", &grillex::PlateElement8::centroid, "Get centroid position in global coordinates")
        .def("to_local", &grillex::PlateElement8::to_local,
             py::arg("global_vec"),
             "Transform vector from global to local coordinates")
        .def("to_global", &grillex::PlateElement8::to_global,
             py::arg("local_vec"),
             "Transform vector from local to global coordinates")
        .def("__repr__", [](const grillex::PlateElement8 &p) {
            return "<PlateElement8 id=" + std::to_string(p.id) +
                   " thickness=" + std::to_string(p.thickness) +
                   " area=" + std::to_string(p.area()) + ">";
        });

    // PlateElement9 class (9-node Lagrangian)
    py::class_<grillex::PlateElement9>(m, "PlateElement9",
        "9-node Lagrangian Mindlin plate element (MITC9 formulation).\n\n"
        "A higher-order plate element for bending analysis using Mindlin plate theory\n"
        "with biquadratic shape functions (Lagrangian family).\n\n"
        "Node numbering (natural coordinates):\n"
        "   4 (-1,+1) --- 7 (0,+1) --- 3 (+1,+1)\n"
        "       |                          |\n"
        "   8 (-1,0)      9 (0,0)       6 (+1,0)\n"
        "       |                          |\n"
        "   1 (-1,-1) --- 5 (0,-1) --- 2 (+1,-1)\n\n"
        "Properties:\n"
        "- thickness: Plate thickness [m]\n"
        "- material: Material properties (E, nu, rho)")
        .def(py::init<int, grillex::Node*, grillex::Node*, grillex::Node*,
                      grillex::Node*, grillex::Node*, grillex::Node*,
                      grillex::Node*, grillex::Node*, grillex::Node*,
                      double, grillex::Material*>(),
             py::arg("id"), py::arg("n1"), py::arg("n2"), py::arg("n3"),
             py::arg("n4"), py::arg("n5"), py::arg("n6"), py::arg("n7"),
             py::arg("n8"), py::arg("n9"), py::arg("thickness"), py::arg("material"),
             "Construct a 9-node plate element")
        .def_readwrite("id", &grillex::PlateElement9::id, "Element ID")
        .def_readonly("nodes", &grillex::PlateElement9::nodes, "Array of 9 nodes")
        .def_readwrite("thickness", &grillex::PlateElement9::thickness, "Plate thickness [m]")
        .def_readonly("material", &grillex::PlateElement9::material, "Material properties")
        .def_readonly("x_axis", &grillex::PlateElement9::x_axis, "Local x-axis")
        .def_readonly("y_axis", &grillex::PlateElement9::y_axis, "Local y-axis")
        .def_readonly("z_axis", &grillex::PlateElement9::z_axis, "Local z-axis (plate normal)")
        .def("num_dofs", &grillex::PlateElement9::num_dofs, "Get number of DOFs (always 54)")
        .def("has_warping", &grillex::PlateElement9::has_warping, "Check for warping DOF (always false)")
        .def("global_stiffness_matrix", &grillex::PlateElement9::global_stiffness_matrix,
             "Get 54x54 global stiffness matrix")
        .def("global_mass_matrix", &grillex::PlateElement9::global_mass_matrix,
             "Get 54x54 global mass matrix (lumped)")
        .def("area", &grillex::PlateElement9::area, "Get plate element area [m²]")
        .def("centroid", &grillex::PlateElement9::centroid, "Get centroid position in global coordinates")
        .def("to_local", &grillex::PlateElement9::to_local,
             py::arg("global_vec"),
             "Transform vector from global to local coordinates")
        .def("to_global", &grillex::PlateElement9::to_global,
             py::arg("local_vec"),
             "Transform vector from local to global coordinates")
        .def("__repr__", [](const grillex::PlateElement9 &p) {
            return "<PlateElement9 id=" + std::to_string(p.id) +
                   " thickness=" + std::to_string(p.thickness) +
                   " area=" + std::to_string(p.area()) + ">";
        });

    // ========================================================================
    // Phase 11: Error Handling & Diagnostics
    // ========================================================================

    // ErrorCode enum
    py::enum_<grillex::ErrorCode>(m, "ErrorCode",
        "Error codes for Grillex analysis failures")
        .value("OK", grillex::ErrorCode::OK, "No error")
        .value("UNCONSTRAINED_SYSTEM", grillex::ErrorCode::UNCONSTRAINED_SYSTEM,
               "System has unconstrained DOFs (rigid body modes)")
        .value("SINGULAR_MATRIX", grillex::ErrorCode::SINGULAR_MATRIX,
               "Stiffness matrix is singular")
        .value("INSUFFICIENT_CONSTRAINTS", grillex::ErrorCode::INSUFFICIENT_CONSTRAINTS,
               "Insufficient boundary conditions")
        .value("REDUNDANT_CONSTRAINTS", grillex::ErrorCode::REDUNDANT_CONSTRAINTS,
               "Redundant or conflicting constraints")
        .value("INVALID_ELEMENT", grillex::ErrorCode::INVALID_ELEMENT,
               "Invalid element definition")
        .value("INVALID_MATERIAL", grillex::ErrorCode::INVALID_MATERIAL,
               "Invalid or missing material")
        .value("INVALID_SECTION", grillex::ErrorCode::INVALID_SECTION,
               "Invalid or missing section")
        .value("INVALID_NODE_REFERENCE", grillex::ErrorCode::INVALID_NODE_REFERENCE,
               "Element references non-existent node")
        .value("INVALID_PROPERTY", grillex::ErrorCode::INVALID_PROPERTY,
               "Invalid property value")
        .value("INVALID_ELEMENT_STIFFNESS", grillex::ErrorCode::INVALID_ELEMENT_STIFFNESS,
               "Element stiffness matrix is invalid")
        .value("INVALID_LOAD_NODE", grillex::ErrorCode::INVALID_LOAD_NODE,
               "Load references non-existent node")
        .value("INVALID_LOAD_ELEMENT", grillex::ErrorCode::INVALID_LOAD_ELEMENT,
               "Load references non-existent element")
        .value("EMPTY_LOAD_CASE", grillex::ErrorCode::EMPTY_LOAD_CASE,
               "Load case has no loads")
        .value("INVALID_LOAD_COMBINATION", grillex::ErrorCode::INVALID_LOAD_COMBINATION,
               "Load combination references invalid load case")
        .value("EMPTY_MODEL", grillex::ErrorCode::EMPTY_MODEL,
               "Model has no elements")
        .value("NO_NODES", grillex::ErrorCode::NO_NODES,
               "Model has no nodes")
        .value("DISCONNECTED_MODEL", grillex::ErrorCode::DISCONNECTED_MODEL,
               "Model has disconnected parts")
        .value("NOT_ANALYZED", grillex::ErrorCode::NOT_ANALYZED,
               "Analysis not performed before querying results")
        .value("SOLVER_CONVERGENCE_FAILED", grillex::ErrorCode::SOLVER_CONVERGENCE_FAILED,
               "Solver failed to converge")
        .value("NUMERICAL_OVERFLOW", grillex::ErrorCode::NUMERICAL_OVERFLOW,
               "Numerical overflow during computation")
        .value("OUT_OF_MEMORY", grillex::ErrorCode::OUT_OF_MEMORY,
               "Out of memory")
        .value("UNKNOWN_ERROR", grillex::ErrorCode::UNKNOWN_ERROR,
               "Unknown error")
        .export_values();

    // GrillexError class
    py::class_<grillex::GrillexError>(m, "GrillexError",
        "Structured error information with machine-readable code and diagnostics")
        .def(py::init<>(), "Create OK (no error) status")
        .def(py::init<grillex::ErrorCode, const std::string&>(),
             py::arg("code"), py::arg("message"),
             "Create error with code and message")
        .def_readwrite("code", &grillex::GrillexError::code, "Error code")
        .def_readwrite("message", &grillex::GrillexError::message, "Error message")
        .def_readwrite("involved_dofs", &grillex::GrillexError::involved_dofs,
                      "Global DOF indices involved in the error")
        .def_readwrite("involved_elements", &grillex::GrillexError::involved_elements,
                      "Element IDs involved in the error")
        .def_readwrite("involved_nodes", &grillex::GrillexError::involved_nodes,
                      "Node IDs involved in the error")
        .def_readwrite("details", &grillex::GrillexError::details,
                      "Additional diagnostic details (key-value pairs)")
        .def_readwrite("suggestion", &grillex::GrillexError::suggestion,
                      "Suggested fix for the error")
        .def("is_ok", &grillex::GrillexError::is_ok, "Check if no error")
        .def("is_error", &grillex::GrillexError::is_error, "Check if error occurred")
        .def("code_string", &grillex::GrillexError::code_string,
             "Get string representation of error code")
        .def("to_string", &grillex::GrillexError::to_string,
             "Get formatted error string")
        .def_static("unconstrained", &grillex::GrillexError::unconstrained,
                   py::arg("dofs"), py::arg("nodes") = std::vector<int>{},
                   "Create unconstrained system error")
        .def_static("singular", &grillex::GrillexError::singular,
                   py::arg("details") = "",
                   "Create singular matrix error")
        .def_static("invalid_element", &grillex::GrillexError::invalid_element,
                   py::arg("element_id"), py::arg("reason"),
                   "Create invalid element error")
        .def_static("invalid_node", &grillex::GrillexError::invalid_node,
                   py::arg("node_id"), py::arg("context") = "",
                   "Create invalid node reference error")
        .def_static("empty_model", &grillex::GrillexError::empty_model,
                   "Create empty model error")
        .def_static("not_analyzed", &grillex::GrillexError::not_analyzed,
                   "Create not analyzed error")
        .def("__repr__", [](const grillex::GrillexError &e) {
            if (e.is_ok()) return std::string("<GrillexError OK>");
            return "<GrillexError " + e.code_string() + ": " + e.message + ">";
        })
        .def("__str__", &grillex::GrillexError::to_string)
        .def("__bool__", [](const grillex::GrillexError &e) {
            return e.is_error();  // True if error, False if OK
        });

    // WarningCode enum
    py::enum_<grillex::WarningCode>(m, "WarningCode",
        "Warning codes for questionable model configurations")
        .value("EXTREME_ASPECT_RATIO", grillex::WarningCode::EXTREME_ASPECT_RATIO,
               "Beam has extreme aspect ratio")
        .value("SMALL_ELEMENT", grillex::WarningCode::SMALL_ELEMENT,
               "Very short element")
        .value("LARGE_ELEMENT", grillex::WarningCode::LARGE_ELEMENT,
               "Very long element")
        .value("NON_COLLINEAR_WARPING", grillex::WarningCode::NON_COLLINEAR_WARPING,
               "Non-collinear beams share warping DOF")
        .value("STIFFNESS_CONTRAST", grillex::WarningCode::STIFFNESS_CONTRAST,
               "Large stiffness contrast between elements")
        .value("NEAR_SINGULARITY", grillex::WarningCode::NEAR_SINGULARITY,
               "Matrix is poorly conditioned")
        .value("VERY_STIFF_SPRING", grillex::WarningCode::VERY_STIFF_SPRING,
               "Very stiff spring may cause numerical issues")
        .value("VERY_SOFT_SPRING", grillex::WarningCode::VERY_SOFT_SPRING,
               "Very soft spring may not provide restraint")
        .value("NEAR_ZERO_PROPERTY", grillex::WarningCode::NEAR_ZERO_PROPERTY,
               "Near-zero property value")
        .value("POSSIBLE_UNIT_ERROR", grillex::WarningCode::POSSIBLE_UNIT_ERROR,
               "Property may be in wrong units")
        .value("INCONSISTENT_SECTION", grillex::WarningCode::INCONSISTENT_SECTION,
               "Section properties are inconsistent")
        .value("LARGE_LOAD", grillex::WarningCode::LARGE_LOAD,
               "Very large load magnitude")
        .value("LOAD_AT_FREE_NODE", grillex::WarningCode::LOAD_AT_FREE_NODE,
               "Load applied at unsupported node")
        .value("ACCELERATION_WITHOUT_MASS", grillex::WarningCode::ACCELERATION_WITHOUT_MASS,
               "Acceleration load may need point mass")
        .value("LARGE_DISPLACEMENT", grillex::WarningCode::LARGE_DISPLACEMENT,
               "Large displacement - linear analysis may be invalid")
        .value("HIGH_STRESS", grillex::WarningCode::HIGH_STRESS,
               "High stress detected")
        .value("SOLVER_REFINEMENT", grillex::WarningCode::SOLVER_REFINEMENT,
               "Solver used iterative refinement")
        .export_values();

    // WarningSeverity enum
    py::enum_<grillex::WarningSeverity>(m, "WarningSeverity",
        "Warning severity levels")
        .value("Low", grillex::WarningSeverity::Low, "Minor issue")
        .value("Medium", grillex::WarningSeverity::Medium, "Review recommended")
        .value("High", grillex::WarningSeverity::High, "Likely modeling error")
        .export_values();

    // GrillexWarning class
    py::class_<grillex::GrillexWarning>(m, "GrillexWarning",
        "Structured warning information for questionable models")
        .def(py::init<grillex::WarningCode, grillex::WarningSeverity, const std::string&>(),
             py::arg("code"), py::arg("severity"), py::arg("message"),
             "Create warning with code, severity, and message")
        .def_readwrite("code", &grillex::GrillexWarning::code, "Warning code")
        .def_readwrite("severity", &grillex::GrillexWarning::severity, "Severity level")
        .def_readwrite("message", &grillex::GrillexWarning::message, "Warning message")
        .def_readwrite("involved_elements", &grillex::GrillexWarning::involved_elements,
                      "Element IDs involved")
        .def_readwrite("involved_nodes", &grillex::GrillexWarning::involved_nodes,
                      "Node IDs involved")
        .def_readwrite("details", &grillex::GrillexWarning::details,
                      "Additional details (key-value pairs)")
        .def_readwrite("suggestion", &grillex::GrillexWarning::suggestion,
                      "Suggested fix")
        .def("code_string", &grillex::GrillexWarning::code_string,
             "Get string representation of warning code")
        .def("severity_string", &grillex::GrillexWarning::severity_string,
             "Get string representation of severity")
        .def("to_string", &grillex::GrillexWarning::to_string,
             "Get formatted warning string")
        .def_static("extreme_aspect_ratio", &grillex::GrillexWarning::extreme_aspect_ratio,
                   py::arg("element_id"), py::arg("ratio"),
                   "Create extreme aspect ratio warning")
        .def_static("small_element", &grillex::GrillexWarning::small_element,
                   py::arg("element_id"), py::arg("length"),
                   "Create small element warning")
        .def_static("stiffness_contrast", &grillex::GrillexWarning::stiffness_contrast,
                   py::arg("elem1"), py::arg("elem2"), py::arg("ratio"),
                   "Create stiffness contrast warning")
        .def_static("near_singularity", &grillex::GrillexWarning::near_singularity,
                   py::arg("condition_number"),
                   "Create near singularity warning")
        .def_static("large_displacement", &grillex::GrillexWarning::large_displacement,
                   py::arg("node_id"), py::arg("displacement"), py::arg("ratio"),
                   "Create large displacement warning")
        .def_static("near_zero_property", &grillex::GrillexWarning::near_zero_property,
                   py::arg("element_id"), py::arg("property_name"), py::arg("value"),
                   "Create near-zero property warning")
        .def("__repr__", [](const grillex::GrillexWarning &w) {
            return "<GrillexWarning [" + w.severity_string() + "] " +
                   w.code_string() + ": " + w.message + ">";
        })
        .def("__str__", &grillex::GrillexWarning::to_string);

    // WarningList class
    py::class_<grillex::WarningList>(m, "WarningList",
        "Collection of warnings from model validation")
        .def(py::init<>(), "Create empty warning list")
        .def_readwrite("warnings", &grillex::WarningList::warnings,
                      "List of warnings")
        .def("add", py::overload_cast<const grillex::GrillexWarning&>(
                 &grillex::WarningList::add),
             py::arg("warning"), "Add a warning to the list")
        .def("has_warnings", &grillex::WarningList::has_warnings,
             "Check if any warnings exist")
        .def("count", &grillex::WarningList::count,
             "Get total warning count")
        .def("count_by_severity", &grillex::WarningList::count_by_severity,
             py::arg("severity"), "Get count of warnings by severity")
        .def("get_by_min_severity", &grillex::WarningList::get_by_min_severity,
             py::arg("min_severity"),
             "Get warnings with given severity or higher")
        .def("clear", &grillex::WarningList::clear, "Clear all warnings")
        .def("summary", &grillex::WarningList::summary,
             "Get formatted summary string")
        .def("__len__", &grillex::WarningList::count)
        .def("__bool__", &grillex::WarningList::has_warnings)
        .def("__iter__", [](const grillex::WarningList &wl) {
            return py::make_iterator(wl.warnings.begin(), wl.warnings.end());
        }, py::keep_alive<0, 1>())
        .def("__repr__", [](const grillex::WarningList &wl) {
            return "<WarningList: " + wl.summary() + ">";
        });

    // ========================================================================
    // Phase 11 (Task 11.3): Singularity Diagnostics
    // ========================================================================

    // RigidBodyModeType enum
    py::enum_<grillex::RigidBodyModeType>(m, "RigidBodyModeType",
        "Type of rigid body mode detected in singularity analysis")
        .value("TranslationX", grillex::RigidBodyModeType::TranslationX,
               "Translation in X direction")
        .value("TranslationY", grillex::RigidBodyModeType::TranslationY,
               "Translation in Y direction")
        .value("TranslationZ", grillex::RigidBodyModeType::TranslationZ,
               "Translation in Z direction")
        .value("RotationX", grillex::RigidBodyModeType::RotationX,
               "Rotation about X axis")
        .value("RotationY", grillex::RigidBodyModeType::RotationY,
               "Rotation about Y axis")
        .value("RotationZ", grillex::RigidBodyModeType::RotationZ,
               "Rotation about Z axis")
        .value("Warping", grillex::RigidBodyModeType::Warping,
               "Warping mode")
        .value("Mixed", grillex::RigidBodyModeType::Mixed,
               "Combined mode (multiple DOF types)")
        .export_values();

    // RigidBodyModeInfo struct
    py::class_<grillex::RigidBodyModeInfo>(m, "RigidBodyModeInfo",
        "Information about a detected rigid body mode")
        .def(py::init<>())
        .def_readwrite("mode_number", &grillex::RigidBodyModeInfo::mode_number,
                      "Mode number (0-indexed)")
        .def_readwrite("eigenvalue", &grillex::RigidBodyModeInfo::eigenvalue,
                      "Eigenvalue (should be near zero for rigid body modes)")
        .def_readwrite("mode_type", &grillex::RigidBodyModeInfo::mode_type,
                      "Type of rigid body mode detected")
        .def_readwrite("involved_nodes", &grillex::RigidBodyModeInfo::involved_nodes,
                      "Node IDs with significant participation")
        .def_readwrite("involved_global_dofs", &grillex::RigidBodyModeInfo::involved_global_dofs,
                      "Global DOF indices with significant participation")
        .def_readwrite("description", &grillex::RigidBodyModeInfo::description,
                      "Description of the mode")
        .def_readwrite("suggested_fix", &grillex::RigidBodyModeInfo::suggested_fix,
                      "Suggested fix for this mode")
        .def("__repr__", [](const grillex::RigidBodyModeInfo &m) {
            return "<RigidBodyModeInfo mode=" + std::to_string(m.mode_number) +
                   " type=" + grillex::rigid_body_mode_type_to_string(m.mode_type) + ">";
        });

    // DOFParticipation struct
    py::class_<grillex::DOFParticipation>(m, "DOFParticipation",
        "DOF participation information in rigid body mode")
        .def(py::init<>())
        .def_readwrite("node_id", &grillex::DOFParticipation::node_id,
                      "Node ID")
        .def_readwrite("local_dof", &grillex::DOFParticipation::local_dof,
                      "Local DOF index (0=UX, 1=UY, 2=UZ, 3=RX, 4=RY, 5=RZ, 6=WARP)")
        .def_readwrite("global_dof", &grillex::DOFParticipation::global_dof,
                      "Global DOF index")
        .def_readwrite("participation", &grillex::DOFParticipation::participation,
                      "Participation magnitude (0 to 1, normalized)")
        .def_readwrite("element_id", &grillex::DOFParticipation::element_id,
                      "Element ID for warping DOFs (-1 for node-level DOFs)")
        .def("__repr__", [](const grillex::DOFParticipation &d) {
            std::string dof_names[] = {"UX", "UY", "UZ", "RX", "RY", "RZ", "WARP"};
            std::string dof_name = (d.local_dof >= 0 && d.local_dof <= 6) ?
                                   dof_names[d.local_dof] : "DOF" + std::to_string(d.local_dof);
            return "<DOFParticipation node=" + std::to_string(d.node_id) +
                   " " + dof_name + " participation=" +
                   std::to_string(static_cast<int>(d.participation * 100)) + "%>";
        });

    // SingularityDiagnostics struct
    py::class_<grillex::SingularityDiagnostics>(m, "SingularityDiagnostics",
        "Result of singularity diagnostics analysis.\n\n"
        "Contains detailed information about detected rigid body modes,\n"
        "unconstrained DOFs, and suggested fixes for making the system\n"
        "properly constrained.")
        .def(py::init<>())
        .def_readwrite("is_singular", &grillex::SingularityDiagnostics::is_singular,
                      "Whether the system is singular (has rigid body modes)")
        .def_readwrite("n_rigid_body_modes", &grillex::SingularityDiagnostics::n_rigid_body_modes,
                      "Number of rigid body modes detected")
        .def_readwrite("rigid_body_modes", &grillex::SingularityDiagnostics::rigid_body_modes,
                      "Detailed information about each rigid body mode")
        .def_readwrite("unconstrained_dofs", &grillex::SingularityDiagnostics::unconstrained_dofs,
                      "DOFs with highest participation in rigid body modes")
        .def_readwrite("summary_message", &grillex::SingularityDiagnostics::summary_message,
                      "Summary message for display")
        .def_readwrite("detailed_message", &grillex::SingularityDiagnostics::detailed_message,
                      "Detailed diagnostic message")
        .def_readwrite("suggested_fixes", &grillex::SingularityDiagnostics::suggested_fixes,
                      "Suggested fixes (one per rigid body mode)")
        .def_readwrite("nodes_needing_constraints", &grillex::SingularityDiagnostics::nodes_needing_constraints,
                      "Nodes that need additional constraints")
        .def_readwrite("dofs_to_constrain", &grillex::SingularityDiagnostics::dofs_to_constrain,
                      "DOFs that need to be constrained (as pairs of {node_id, local_dof})")
        .def("to_string", &grillex::SingularityDiagnostics::to_string,
             "Get formatted string representation")
        .def("to_json", &grillex::SingularityDiagnostics::to_json,
             "Get machine-readable JSON representation")
        .def("__repr__", [](const grillex::SingularityDiagnostics &d) {
            if (!d.is_singular) return std::string("<SingularityDiagnostics: System is well-constrained>");
            return "<SingularityDiagnostics: " + std::to_string(d.n_rigid_body_modes) +
                   " rigid body mode(s) detected>";
        })
        .def("__str__", &grillex::SingularityDiagnostics::to_string)
        .def("__bool__", [](const grillex::SingularityDiagnostics &d) {
            return d.is_singular;  // True if singular
        });

    // SingularityAnalyzerSettings struct
    py::class_<grillex::SingularityAnalyzerSettings>(m, "SingularityAnalyzerSettings",
        "Settings for singularity analysis.\n\n"
        "Controls eigenvalue threshold, number of modes to check,\n"
        "and participation threshold for DOF reporting.")
        .def(py::init<>())
        .def_readwrite("eigenvalue_threshold",
                      &grillex::SingularityAnalyzerSettings::eigenvalue_threshold,
                      "Eigenvalue threshold for rigid body mode detection (default 1e-8)")
        .def_readwrite("n_modes_to_check",
                      &grillex::SingularityAnalyzerSettings::n_modes_to_check,
                      "Number of lowest eigenvalues to check (default 10)")
        .def_readwrite("participation_threshold",
                      &grillex::SingularityAnalyzerSettings::participation_threshold,
                      "Minimum participation to include DOF in report (default 0.01)")
        .def_readwrite("max_dofs_to_report",
                      &grillex::SingularityAnalyzerSettings::max_dofs_to_report,
                      "Maximum number of unconstrained DOFs to report (default 20)")
        .def_readwrite("use_mass_matrix",
                      &grillex::SingularityAnalyzerSettings::use_mass_matrix,
                      "Use mass matrix for better mode identification (default true)");

    // SingularityAnalyzer class
    py::class_<grillex::SingularityAnalyzer>(m, "SingularityAnalyzer",
        "Singularity analyzer using eigenvalue decomposition.\n\n"
        "Analyzes structural systems for singularity by computing eigenvalues\n"
        "and identifying rigid body modes. This approach can identify:\n"
        "1. Which specific DOFs are unconstrained\n"
        "2. The type of rigid body motion (translation, rotation, warping)\n"
        "3. Suggested fixes based on mode analysis\n\n"
        "Usage:\n"
        "    analyzer = SingularityAnalyzer()\n"
        "    result = analyzer.analyze_with_mass(K, M, dof_handler)\n"
        "    if result.is_singular:\n"
        "        print(result.detailed_message)\n"
        "        for fix in result.suggested_fixes:\n"
        "            print(f'Suggestion: {fix}')")
        .def(py::init<>())
        .def("analyze", [](const grillex::SingularityAnalyzer& analyzer,
                           const Eigen::SparseMatrix<double>& K,
                           const grillex::DOFHandler& dof_handler,
                           const grillex::SingularityAnalyzerSettings& settings) {
            return analyzer.analyze(K, dof_handler, settings);
        },
             py::arg("K"), py::arg("dof_handler"),
             py::arg("settings") = grillex::SingularityAnalyzerSettings{},
             "Analyze stiffness matrix for singularity (without mass matrix)")
        .def("analyze_with_mass", [](const grillex::SingularityAnalyzer& analyzer,
                                     const Eigen::SparseMatrix<double>& K,
                                     const Eigen::SparseMatrix<double>& M,
                                     const grillex::DOFHandler& dof_handler,
                                     const grillex::SingularityAnalyzerSettings& settings) {
            return analyzer.analyze(K, M, dof_handler, settings);
        },
             py::arg("K"), py::arg("M"), py::arg("dof_handler"),
             py::arg("settings") = grillex::SingularityAnalyzerSettings{},
             "Analyze K and M matrices for singularity (preferred method)")
        .def("is_singular", &grillex::SingularityAnalyzer::is_singular,
             py::arg("K"),
             "Quick check for singularity (without detailed diagnostics)")
        .def("count_rigid_body_modes", &grillex::SingularityAnalyzer::count_rigid_body_modes,
             py::arg("K"), py::arg("threshold") = 1e-8,
             "Get the number of rigid body modes in the matrix");

    // ========================================================================
    // Phase 15: Nonlinear Solver
    // ========================================================================

    // NonlinearSolverResult struct
    py::class_<grillex::NonlinearSolverResult>(m, "NonlinearSolverResult",
        "Result from nonlinear spring solver.\n\n"
        "Contains solution displacements, convergence status, iteration count,\n"
        "and final spring states/forces for reporting.")
        .def(py::init<>())
        .def_readwrite("displacements", &grillex::NonlinearSolverResult::displacements,
             "Solution displacement vector [m, rad]")
        .def_readwrite("converged", &grillex::NonlinearSolverResult::converged,
             "True if solver converged")
        .def_readwrite("iterations", &grillex::NonlinearSolverResult::iterations,
             "Number of iterations performed")
        .def_readwrite("message", &grillex::NonlinearSolverResult::message,
             "Descriptive message (convergence info or error)")
        .def_readwrite("spring_states", &grillex::NonlinearSolverResult::spring_states,
             "Final spring states: list of (spring_id, active_state[6])")
        .def_readwrite("spring_forces", &grillex::NonlinearSolverResult::spring_forces,
             "Final spring forces: list of (spring_id, forces[6]) [kN or kN·m]")
        .def_readwrite("state_changes_per_iteration", &grillex::NonlinearSolverResult::state_changes_per_iteration,
             "History of state changes per iteration (for diagnostics)")
        .def("__repr__", [](const grillex::NonlinearSolverResult &r) {
            return "<NonlinearSolverResult converged=" +
                   std::string(r.converged ? "True" : "False") +
                   " iterations=" + std::to_string(r.iterations) + ">";
        });

    // NonlinearInitialState struct
    py::class_<grillex::NonlinearInitialState>(m, "NonlinearInitialState",
        "Initial state for starting nonlinear iteration.\n\n"
        "Used to start from a known state (e.g., static solution) rather than zero.\n"
        "Essential for static→dynamic load sequencing where the static (gravity)\n"
        "solution establishes the baseline contact pattern.")
        .def(py::init<>())
        .def_readwrite("displacement", &grillex::NonlinearInitialState::displacement,
             "Initial displacement vector (empty = start from zero)")
        .def_readwrite("spring_states", &grillex::NonlinearInitialState::spring_states,
             "Initial spring states: list of (spring_id, active_states[6])")
        .def("has_initial_state", &grillex::NonlinearInitialState::has_initial_state,
             "Check if initial state is provided")
        .def("__repr__", [](const grillex::NonlinearInitialState &s) {
            return "<NonlinearInitialState has_state=" +
                   std::string(s.has_initial_state() ? "True" : "False") + ">";
        });

    // NOTE: NonlinearSolverSettings is bound earlier (before Model) to support
    // default argument in Model.analyze_combination()

    // NonlinearSolver class
    py::class_<grillex::NonlinearSolver>(m, "NonlinearSolver",
        "Iterative solver for systems with nonlinear springs.\n\n"
        "Handles tension-only, compression-only, and gap springs through\n"
        "an iterative state-update algorithm. Each iteration:\n"
        "1. Assembles stiffness from currently active springs\n"
        "2. Computes gap closure forces for active gap springs\n"
        "3. Solves the linear system\n"
        "4. Updates spring states based on new displacements\n"
        "5. Checks for convergence (no state changes)\n\n"
        "For static→dynamic sequencing, use the initial_state parameter\n"
        "to start from a previous static solution.")
        .def(py::init<const grillex::NonlinearSolverSettings&>(),
             py::arg("settings") = grillex::NonlinearSolverSettings(),
             "Construct solver with settings")
        .def("solve", [](grillex::NonlinearSolver& solver,
                        const Eigen::SparseMatrix<double>& base_K,
                        const Eigen::VectorXd& F,
                        py::list springs_list,
                        const grillex::DOFHandler& dof_handler,
                        const grillex::NonlinearInitialState& initial_state) {
            // Convert py::list to std::vector<SpringElement*>
            std::vector<grillex::SpringElement*> springs;
            for (auto& item : springs_list) {
                springs.push_back(item.cast<grillex::SpringElement*>());
            }
            return solver.solve(base_K, F, springs, dof_handler, initial_state);
        },
             py::arg("base_K"),
             py::arg("F"),
             py::arg("springs"),
             py::arg("dof_handler"),
             py::arg("initial_state") = grillex::NonlinearInitialState(),
             "Solve system with nonlinear springs.\n\n"
             "Args:\n"
             "    base_K: Base stiffness matrix (beams, plates - excludes springs)\n"
             "    F: External force vector [kN]\n"
             "    springs: List of spring elements (states will be updated)\n"
             "    dof_handler: DOF handler for global DOF indexing\n"
             "    initial_state: Optional initial state from previous solve\n\n"
             "Returns:\n"
             "    NonlinearSolverResult with displacements and convergence info")
        .def("settings", &grillex::NonlinearSolver::settings,
             "Get current settings")
        .def("set_settings", &grillex::NonlinearSolver::set_settings,
             py::arg("settings"),
             "Update settings")
        .def("__repr__", [](const grillex::NonlinearSolver &s) {
            return "<NonlinearSolver max_iter=" +
                   std::to_string(s.settings().max_iterations) + ">";
        });

    // EigenvalueSolver class
    py::class_<grillex::EigenvalueSolver>(m, "EigenvalueSolver",
        "Eigenvalue solver for structural dynamics.\n\n"
        "Solves the generalized eigenvalue problem:\n"
        "    K * phi = omega^2 * M * phi\n\n"
        "Where:\n"
        "- K: Global stiffness matrix [kN/m]\n"
        "- M: Global mass matrix [mT]\n"
        "- omega: Natural circular frequency [rad/s]\n"
        "- phi: Mode shape (eigenvector)\n\n"
        "Example:\n"
        "    solver = EigenvalueSolver()\n"
        "    settings = EigensolverSettings()\n"
        "    settings.n_modes = 10\n"
        "    K_red, M_red, dof_map = solver.reduce_system(K, M, bc, dof_handler)\n"
        "    result = solver.solve(K_red, M_red, settings)")
        .def(py::init<>())
        .def("solve", &grillex::EigenvalueSolver::solve,
             py::arg("K"),
             py::arg("M"),
             py::arg("settings") = grillex::EigensolverSettings(),
             "Solve eigenvalue problem.\n\n"
             "Args:\n"
             "    K: Stiffness matrix (reduced, i.e., fixed DOFs eliminated)\n"
             "    M: Mass matrix (reduced, same size as K)\n"
             "    settings: Solver configuration\n\n"
             "Returns:\n"
             "    EigensolverResult with eigenvalues, frequencies, and mode shapes")
        .def("reduce_system", &grillex::EigenvalueSolver::reduce_system,
             py::arg("K"),
             py::arg("M"),
             py::arg("bc_handler"),
             py::arg("dof_handler"),
             "Reduce system by eliminating fixed DOFs.\n\n"
             "Args:\n"
             "    K: Full stiffness matrix\n"
             "    M: Full mass matrix\n"
             "    bc_handler: Boundary condition handler\n"
             "    dof_handler: DOF handler\n\n"
             "Returns:\n"
             "    Tuple of (K_reduced, M_reduced, dof_mapping)")
        .def_static("expand_mode_shape", &grillex::EigenvalueSolver::expand_mode_shape,
             py::arg("reduced_shape"),
             py::arg("dof_mapping"),
             py::arg("total_dofs"),
             "Expand reduced mode shape to full DOF vector")
        .def("compute_participation_factors", &grillex::EigenvalueSolver::compute_participation_factors,
             py::arg("result"),
             py::arg("M_reduced"),
             py::arg("dof_mapping"),
             py::arg("dof_handler"),
             py::arg("total_mass"),
             "Compute participation factors and effective modal mass for all modes.\n\n"
             "Args:\n"
             "    result: EigensolverResult to update (modes will be modified)\n"
             "    M_reduced: Reduced mass matrix as dense Eigen matrix\n"
             "    dof_mapping: Mapping from reduced DOF index to full DOF index\n"
             "    dof_handler: DOF handler for determining DOF types\n"
             "    total_mass: Total translational mass of structure [mT]\n\n"
             "Updates each mode with:\n"
             "- Participation factors (Γ = φᵀ × M × r)\n"
             "- Effective modal mass (Meff = Γ²)\n"
             "- Effective modal mass percentage\n"
             "Also updates result with cumulative mass percentages.")
        .def("__repr__", [](const grillex::EigenvalueSolver &) {
            return "<EigenvalueSolver>";
        });
}
