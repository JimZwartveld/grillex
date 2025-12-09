#pragma once

#include "grillex/node.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/local_axes.hpp"
#include <Eigen/Dense>
#include <vector>

namespace grillex {

/**
 * @brief Beam formulation type
 *
 * Determines which beam theory is used for stiffness and mass matrices.
 */
enum class BeamFormulation {
    EulerBernoulli,  ///< Classical beam theory (no shear deformation)
    Timoshenko       ///< Beam theory with shear deformation effects
};

/**
 * @brief End release configuration for beam element
 *
 * Defines which DOFs are released (force-free) at beam ends.
 * Released DOFs are internally condensed out using static condensation.
 */
struct EndRelease {
    // Translation releases at node i
    bool release_ux_i = false;  ///< Axial release at end i (sliding joint)
    bool release_uy_i = false;  ///< Shear y release at end i
    bool release_uz_i = false;  ///< Shear z release at end i

    // Rotation releases at node i
    bool release_rx_i = false;  ///< Torsion release at end i
    bool release_ry_i = false;  ///< Moment about y release at end i
    bool release_rz_i = false;  ///< Moment about z release at end i

    // Warping release at node i (for 14-DOF elements)
    bool release_warp_i = false;  ///< Warping release at end i (free to warp)

    // Translation releases at node j
    bool release_ux_j = false;  ///< Axial release at end j (sliding joint)
    bool release_uy_j = false;  ///< Shear y release at end j
    bool release_uz_j = false;  ///< Shear z release at end j

    // Rotation releases at node j
    bool release_rx_j = false;  ///< Torsion release at end j
    bool release_ry_j = false;  ///< Moment about y release at end j
    bool release_rz_j = false;  ///< Moment about z release at end j

    // Warping release at node j (for 14-DOF elements)
    bool release_warp_j = false;  ///< Warping release at end j (free to warp)

    /**
     * @brief Release both bending moments at end i (pin connection)
     */
    void release_moment_i() { release_ry_i = release_rz_i = true; }

    /**
     * @brief Release both bending moments at end j (pin connection)
     */
    void release_moment_j() { release_ry_j = release_rz_j = true; }

    /**
     * @brief Release all rotations at end i (true pin/ball joint)
     */
    void release_all_rotations_i() { release_rx_i = release_ry_i = release_rz_i = true; }

    /**
     * @brief Release all rotations at end j (true pin/ball joint)
     */
    void release_all_rotations_j() { release_rx_j = release_ry_j = release_rz_j = true; }

    /**
     * @brief Check if any releases are active
     */
    bool has_any_release() const;

    /**
     * @brief Get indices of released DOFs
     *
     * @param has_warping True if element has warping DOF (14-DOF), false for 12-DOF
     * @return std::vector<int> Indices of released DOFs (0-11 for 12-DOF, 0-13 for 14-DOF)
     */
    std::vector<int> get_released_indices(bool has_warping) const;
};

/**
 * @brief Beam element for structural analysis
 *
 * Implements a 3D Euler-Bernoulli beam element with 12 DOFs
 * (6 DOFs per node: 3 translations + 3 rotations).
 *
 * DOF ordering: [ui, vi, wi, θxi, θyi, θzi, uj, vj, wj, θxj, θyj, θzj]
 * where:
 * - u: translation along local x (axial)
 * - v: translation along local y
 * - w: translation along local z
 * - θx: rotation about local x (torsion)
 * - θy: rotation about local y (bending)
 * - θz: rotation about local z (bending)
 */
class BeamElement {
public:
    int id;              ///< Element ID
    Node* node_i;        ///< First node (start)
    Node* node_j;        ///< Second node (end)
    Material* material;  ///< Material properties
    Section* section;    ///< Section properties
    LocalAxes local_axes; ///< Local coordinate system
    double length;       ///< Element length [m]

    /// End offsets in local coordinates [m]
    Eigen::Vector3d offset_i = Eigen::Vector3d::Zero();
    Eigen::Vector3d offset_j = Eigen::Vector3d::Zero();

    /// End release configuration
    EndRelease releases;

    /**
     * @brief Construct a beam element
     *
     * @param id Element ID
     * @param node_i First node
     * @param node_j Second node
     * @param mat Material
     * @param sec Section
     * @param roll Roll angle about beam axis [radians], default 0.0
     */
    BeamElement(int id, Node* node_i, Node* node_j,
                Material* mat, Section* sec, double roll = 0.0);

    /**
     * @brief Compute 12x12 local stiffness matrix
     *
     * Returns the element stiffness matrix in local coordinates
     * using the specified beam formulation.
     *
     * @param formulation Beam formulation type (default: EulerBernoulli)
     * @return Eigen::Matrix<double, 12, 12> Local stiffness matrix [kN, m, rad]
     */
    Eigen::Matrix<double, 12, 12> local_stiffness_matrix(
        BeamFormulation formulation = BeamFormulation::EulerBernoulli) const;

    /**
     * @brief Compute 12x12 transformation matrix
     *
     * Returns the transformation matrix that relates local to global DOFs.
     * Block diagonal structure with 4 copies of the 3x3 rotation matrix.
     *
     * @return Eigen::Matrix<double, 12, 12> Transformation matrix
     */
    Eigen::Matrix<double, 12, 12> transformation_matrix() const;

    /**
     * @brief Compute 12x12 global stiffness matrix
     *
     * Transforms the local stiffness matrix to global coordinates:
     * K_global = T^T * K_local * T
     *
     * @return Eigen::Matrix<double, 12, 12> Global stiffness matrix [kN, m, rad]
     */
    Eigen::Matrix<double, 12, 12> global_stiffness_matrix() const;

    /**
     * @brief Compute 12x12 local consistent mass matrix
     *
     * Returns the consistent (not lumped) mass matrix in local coordinates.
     * Includes translational mass and rotary inertia contributions.
     *
     * @param formulation Beam formulation type (default: EulerBernoulli)
     * @return Eigen::Matrix<double, 12, 12> Local mass matrix [mT = metric tonne]
     */
    Eigen::Matrix<double, 12, 12> local_mass_matrix(
        BeamFormulation formulation = BeamFormulation::EulerBernoulli) const;

    /**
     * @brief Compute 12x12 global mass matrix
     *
     * Transforms the local mass matrix to global coordinates:
     * M_global = T^T * M_local * T
     *
     * @return Eigen::Matrix<double, 12, 12> Global mass matrix [mT]
     */
    Eigen::Matrix<double, 12, 12> global_mass_matrix() const;

    /**
     * @brief Set end offsets in local coordinates
     *
     * Offsets define rigid arms from nodes to beam ends.
     * Positive offset_i moves the beam end away from node_i along local axes.
     *
     * @param offset_i Offset at node i [m] in local coordinates [dx, dy, dz]
     * @param offset_j Offset at node j [m] in local coordinates [dx, dy, dz]
     */
    void set_offsets(const Eigen::Vector3d& offset_i, const Eigen::Vector3d& offset_j);

    /**
     * @brief Check if element has any offsets
     *
     * @return bool True if either offset is non-zero
     */
    bool has_offsets() const;

    /**
     * @brief Compute effective beam length accounting for offsets
     *
     * The effective length is the distance between the offset-adjusted
     * beam ends, not the node-to-node distance.
     *
     * @return double Effective beam length [m]
     */
    double effective_length() const;

    /**
     * @brief Compute 14x14 local stiffness matrix including warping DOF
     *
     * Returns the element stiffness matrix in local coordinates including
     * the 7th DOF (warping) for thin-walled sections.
     *
     * @param formulation Beam formulation type (default: EulerBernoulli)
     * @return Eigen::Matrix<double, 14, 14> Local stiffness matrix with warping [kN, m, rad]
     */
    Eigen::Matrix<double, 14, 14> local_stiffness_matrix_warping(
        BeamFormulation formulation = BeamFormulation::EulerBernoulli) const;

    /**
     * @brief Compute 14x14 local mass matrix including warping DOF
     *
     * Returns the consistent mass matrix in local coordinates including
     * the 7th DOF (warping) for thin-walled sections.
     *
     * @param formulation Beam formulation type (default: EulerBernoulli)
     * @return Eigen::Matrix<double, 14, 14> Local mass matrix with warping [mT]
     */
    Eigen::Matrix<double, 14, 14> local_mass_matrix_warping(
        BeamFormulation formulation = BeamFormulation::EulerBernoulli) const;

    /**
     * @brief Compute 14x14 transformation matrix for warping elements
     *
     * Returns the transformation matrix that relates local to global DOFs
     * including the warping DOF (which transforms as a scalar).
     *
     * @return Eigen::Matrix<double, 14, 14> Transformation matrix
     */
    Eigen::Matrix<double, 14, 14> transformation_matrix_warping() const;

    /**
     * @brief Compute 14x14 global stiffness matrix including warping DOF
     *
     * Transforms the local stiffness matrix to global coordinates:
     * K_global = T^T * K_local * T
     *
     * @param formulation Beam formulation type (default: EulerBernoulli)
     * @return Eigen::Matrix<double, 14, 14> Global stiffness matrix with warping [kN, m, rad]
     */
    Eigen::Matrix<double, 14, 14> global_stiffness_matrix_warping(
        BeamFormulation formulation = BeamFormulation::EulerBernoulli) const;

    /**
     * @brief Compute 14x14 global mass matrix including warping DOF
     *
     * Transforms the local mass matrix to global coordinates:
     * M_global = T^T * M_local * T
     *
     * @param formulation Beam formulation type (default: EulerBernoulli)
     * @return Eigen::Matrix<double, 14, 14> Global mass matrix with warping [mT]
     */
    Eigen::Matrix<double, 14, 14> global_mass_matrix_warping(
        BeamFormulation formulation = BeamFormulation::EulerBernoulli) const;

private:
    /**
     * @brief Compute element length from node positions
     */
    double compute_length() const;

    /**
     * @brief Compute 12x12 offset transformation matrix
     *
     * Relates beam end DOFs to node DOFs for rigid offsets.
     * For offset r at a node:
     *   u_beam = u_node + θ_node × r
     *   θ_beam = θ_node
     *
     * @return Eigen::Matrix<double, 12, 12> Offset transformation matrix
     */
    Eigen::Matrix<double, 12, 12> offset_transformation_matrix() const;

    /**
     * @brief Apply static condensation to remove released DOFs
     *
     * Uses static condensation to eliminate released DOFs from the element matrix:
     * K_condensed = K_rr - K_rc * K_cc^(-1) * K_cr
     *
     * Where r = retained DOFs, c = condensed (released) DOFs
     *
     * @param K Full element stiffness or mass matrix
     * @param released_indices Indices of DOFs to be released/condensed
     * @return Eigen::MatrixXd Condensed matrix (same size as input)
     */
    Eigen::MatrixXd apply_static_condensation(
        const Eigen::MatrixXd& K,
        const std::vector<int>& released_indices) const;
};

} // namespace grillex
