#ifndef GRILLEX_BOUNDARY_CONDITION_HPP
#define GRILLEX_BOUNDARY_CONDITION_HPP

#include <vector>
#include <Eigen/Sparse>
#include "grillex/dof_handler.hpp"

namespace grillex {

/**
 * @brief DOF index constants for clarity
 *
 * These indices correspond to the local DOF numbering at each node:
 * - UX, UY, UZ: Translations in local X, Y, Z directions
 * - RX, RY, RZ: Rotations about local X, Y, Z axes
 * - WARP: Warping DOF (7th DOF for thin-walled sections)
 */
enum DOFIndex {
    UX = 0,   ///< Translation in X direction (m)
    UY = 1,   ///< Translation in Y direction (m)
    UZ = 2,   ///< Translation in Z direction (m)
    RX = 3,   ///< Rotation about X axis (rad)
    RY = 4,   ///< Rotation about Y axis (rad)
    RZ = 5,   ///< Rotation about Z axis (rad)
    WARP = 6  ///< Warping displacement (m²)
};

/**
 * @brief Represents a fixed degree of freedom with optional prescribed value
 */
struct FixedDOF {
    int node_id;        ///< Node ID where DOF is fixed
    int local_dof;      ///< Local DOF index (0-5 for standard, 6 for warping)
    double value;       ///< Prescribed value (0.0 for standard fixity)

    /**
     * @brief Construct a fixed DOF
     * @param node Node ID
     * @param dof Local DOF index (0-6)
     * @param val Prescribed value (default 0.0)
     */
    FixedDOF(int node, int dof, double val = 0.0)
        : node_id(node), local_dof(dof), value(val) {}
};

/**
 * @brief Boundary Condition Handler
 *
 * Manages fixed DOFs and prescribed displacements for structural analysis.
 * Supports standard 6-DOF constraints (translations and rotations) plus
 * optional 7th DOF (warping) for thin-walled open sections.
 *
 * Common Support Types:
 * - Fixed support: All 6 DOFs restrained (use fix_node)
 * - Built-in with warping: All 7 DOFs restrained (use fix_node_with_warping)
 * - Pin support: Translations only (use pin_node)
 * - Fork support: Translations only, rotations and warping free (use fork_support)
 * - Roller support: Custom (use add_fixed_dof for specific DOFs)
 */
class BCHandler {
public:
    /**
     * @brief Construct an empty boundary condition handler
     */
    BCHandler() = default;

    /**
     * @brief Add a single fixed DOF with optional prescribed value
     * @param node_id Node ID where DOF is fixed
     * @param local_dof Local DOF index (0-6)
     * @param value Prescribed displacement/rotation (default 0.0)
     */
    void add_fixed_dof(int node_id, int local_dof, double value = 0.0);

    /**
     * @brief Fix all 6 standard DOFs at a node (full fixity, no warping)
     * @param node_id Node ID to fix
     *
     * Equivalent to a built-in support for standard beam theory.
     * Does NOT restrain warping DOF.
     */
    void fix_node(int node_id);

    /**
     * @brief Fix all 7 DOFs at a node (including warping)
     * @param node_id Node ID to fix
     *
     * Equivalent to a built-in support with warping restraint.
     * Use this for fixed ends of thin-walled beams where warping is prevented.
     */
    void fix_node_with_warping(int node_id);

    /**
     * @brief Fix only translations at a node (pin support)
     * @param node_id Node ID to pin
     *
     * Fixes UX, UY, UZ. Leaves rotations (RX, RY, RZ) and warping free.
     * Equivalent to a pinned support.
     */
    void pin_node(int node_id);

    /**
     * @brief Apply fork support (fix translations, free rotations and warping)
     * @param node_id Node ID for fork support
     *
     * Fixes UX, UY, UZ. Leaves rotations (RX, RY, RZ) and warping (WARP) free.
     * Equivalent to a fork support or simple support in warping torsion theory.
     */
    void fork_support(int node_id);

    /**
     * @brief Apply boundary conditions to system matrices using penalty method
     * @param K Global stiffness matrix
     * @param F Global force vector
     * @param dof_handler DOF handler for mapping node DOFs to global DOFs
     * @return Pair of (modified_K, modified_F)
     *
     * Uses penalty method:
     * For fixed DOF at global index i with prescribed value u_i:
     *   K(i,i) += penalty_factor
     *   F(i) = penalty_factor * u_i
     *
     * Where penalty_factor = 1e15 * max(K(i,i), 1.0)
     */
    std::pair<Eigen::SparseMatrix<double>, Eigen::VectorXd> apply_to_system(
        const Eigen::SparseMatrix<double>& K,
        const Eigen::VectorXd& F,
        const DOFHandler& dof_handler) const;

    /**
     * @brief Get list of global DOF indices that are fixed
     * @param dof_handler DOF handler for mapping
     * @return Vector of global DOF indices
     *
     * Useful for extracting reactions and identifying constrained DOFs.
     */
    std::vector<int> get_fixed_global_dofs(const DOFHandler& dof_handler) const;

    /**
     * @brief Get number of fixed DOFs
     * @return Number of fixed DOFs
     */
    int num_fixed_dofs() const { return static_cast<int>(fixed_dofs_.size()); }

    /**
     * @brief Clear all boundary conditions
     */
    void clear() { fixed_dofs_.clear(); }

    /**
     * @brief Get const reference to fixed DOFs list
     * @return Vector of FixedDOF structs
     */
    const std::vector<FixedDOF>& get_fixed_dofs() const { return fixed_dofs_; }

private:
    std::vector<FixedDOF> fixed_dofs_;  ///< List of all fixed DOFs
};

}  // namespace grillex

#endif  // GRILLEX_BOUNDARY_CONDITION_HPP
