#pragma once

#include "grillex/node.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/local_axes.hpp"
#include <Eigen/Dense>

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
};

} // namespace grillex
