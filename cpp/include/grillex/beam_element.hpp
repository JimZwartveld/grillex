#pragma once

#include "grillex/node.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/local_axes.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>

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
 * @brief Configuration for beam element creation
 *
 * Defines which beam formulation and optional features to use.
 */
struct BeamConfig {
    BeamFormulation formulation = BeamFormulation::EulerBernoulli;  ///< Beam formulation type
    bool include_warping = false;          ///< Include warping DOF (7th DOF at each end)
    bool include_shear_deformation = false; ///< Alias for Timoshenko formulation

    /**
     * @brief Get the effective beam formulation
     *
     * If include_shear_deformation is true, returns Timoshenko regardless of formulation setting.
     *
     * @return BeamFormulation The effective formulation to use
     */
    BeamFormulation get_formulation() const {
        return include_shear_deformation ? BeamFormulation::Timoshenko : formulation;
    }
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
 * @brief Abstract base class for beam elements
 *
 * Provides polymorphic interface for beam elements with different formulations
 * (Euler-Bernoulli, Timoshenko) and DOF configurations (12 or 14 DOFs).
 */
class BeamElementBase {
public:
    /**
     * @brief Compute local stiffness matrix (polymorphic)
     *
     * @return Eigen::MatrixXd Local stiffness matrix (12x12 or 14x14)
     */
    virtual Eigen::MatrixXd compute_local_stiffness() const = 0;

    /**
     * @brief Compute local mass matrix (polymorphic)
     *
     * @return Eigen::MatrixXd Local mass matrix (12x12 or 14x14)
     */
    virtual Eigen::MatrixXd compute_local_mass() const = 0;

    /**
     * @brief Compute transformation matrix (polymorphic)
     *
     * @return Eigen::MatrixXd Transformation matrix (12x12 or 14x14)
     */
    virtual Eigen::MatrixXd compute_transformation() const = 0;

    /**
     * @brief Get number of DOFs for this element
     *
     * @return int Number of DOFs (12 for standard, 14 for warping)
     */
    virtual int num_dofs() const = 0;

    /**
     * @brief Get the beam formulation used
     *
     * @return BeamFormulation The formulation (EulerBernoulli or Timoshenko)
     */
    virtual BeamFormulation get_formulation() const = 0;

    /**
     * @brief Check if element includes warping DOF
     *
     * @return bool True if element has 14 DOFs (includes warping)
     */
    virtual bool has_warping() const = 0;

    virtual ~BeamElementBase() = default;
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
class BeamElement : public BeamElementBase {
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

    /// Beam configuration (formulation and features)
    BeamConfig config;

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
     * @brief Construct a beam element with configuration
     *
     * @param id Element ID
     * @param node_i First node
     * @param node_j Second node
     * @param mat Material
     * @param sec Section
     * @param config Beam configuration (formulation and features)
     * @param roll Roll angle about beam axis [radians], default 0.0
     */
    BeamElement(int id, Node* node_i, Node* node_j,
                Material* mat, Section* sec,
                const BeamConfig& config, double roll = 0.0);

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

    // Virtual method implementations from BeamElementBase

    /**
     * @brief Compute local stiffness matrix (polymorphic interface)
     *
     * Returns either 12x12 or 14x14 matrix depending on config.include_warping
     *
     * @return Eigen::MatrixXd Local stiffness matrix
     */
    Eigen::MatrixXd compute_local_stiffness() const override;

    /**
     * @brief Compute local mass matrix (polymorphic interface)
     *
     * Returns either 12x12 or 14x14 matrix depending on config.include_warping
     *
     * @return Eigen::MatrixXd Local mass matrix
     */
    Eigen::MatrixXd compute_local_mass() const override;

    /**
     * @brief Compute transformation matrix (polymorphic interface)
     *
     * Returns either 12x12 or 14x14 matrix depending on config.include_warping
     *
     * @return Eigen::MatrixXd Transformation matrix
     */
    Eigen::MatrixXd compute_transformation() const override;

    /**
     * @brief Get number of DOFs (polymorphic interface)
     *
     * @return int 12 or 14 depending on config.include_warping
     */
    int num_dofs() const override;

    /**
     * @brief Get beam formulation (polymorphic interface)
     *
     * @return BeamFormulation The effective formulation from config
     */
    BeamFormulation get_formulation() const override;

    /**
     * @brief Check if warping is enabled (polymorphic interface)
     *
     * @return bool True if config.include_warping is true
     */
    bool has_warping() const override;

    /**
     * @brief Get the direction vector of the beam (normalized)
     *
     * Returns a unit vector pointing from node_i to node_j.
     * Used for collinearity detection at shared nodes.
     *
     * @return Eigen::Vector3d Unit vector along beam axis
     */
    Eigen::Vector3d direction_vector() const;

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
     * @brief Compute 14x14 offset transformation matrix for warping elements
     *
     * Relates beam end DOFs to node DOFs for rigid offsets in warping elements.
     * The warping DOF is not affected by offsets (identity transformation).
     *
     * @return Eigen::Matrix<double, 14, 14> Offset transformation matrix
     */
    Eigen::Matrix<double, 14, 14> offset_transformation_matrix_warping() const;

private:
    /**
     * @brief Compute element length from node positions
     */
    double compute_length() const;

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

/**
 * @brief Factory function to create beam elements with different configurations
 *
 * Creates a beam element with the specified formulation and features.
 * Returns a unique_ptr to BeamElementBase for polymorphic usage.
 *
 * @param id Element ID
 * @param node_i First node
 * @param node_j Second node
 * @param mat Material
 * @param sec Section
 * @param config Beam configuration (formulation and features)
 * @param roll Roll angle about beam axis [radians], default 0.0
 * @return std::unique_ptr<BeamElementBase> Pointer to created beam element
 */
std::unique_ptr<BeamElementBase> create_beam_element(
    int id, Node* node_i, Node* node_j,
    Material* mat, Section* sec,
    const BeamConfig& config = BeamConfig{},
    double roll = 0.0);

/**
 * @brief Check if two elements sharing a node are collinear
 *
 * Two elements are considered collinear if their direction vectors are
 * (anti-)parallel within the specified angle tolerance. This is used to
 * determine warping DOF coupling at shared nodes.
 *
 * For collinear elements:
 * - Warping deformation is geometrically compatible
 * - Warping DOFs should be coupled (share same global DOF)
 *
 * For non-collinear elements:
 * - Warping deformation is geometrically incompatible
 * - Warping DOFs should be independent
 *
 * @param elem1 First beam element
 * @param elem2 Second beam element
 * @param shared_node_id ID of the node shared by both elements
 * @param angle_tolerance_deg Maximum angle (in degrees) for elements to be considered collinear.
 *                            Default is 5.0 degrees.
 * @return bool True if elements are collinear within tolerance
 */
bool are_elements_collinear(
    const BeamElement& elem1,
    const BeamElement& elem2,
    int shared_node_id,
    double angle_tolerance_deg = 5.0);

} // namespace grillex
