#pragma once

#include "grillex/node.hpp"
#include "grillex/load_case.hpp"
#include <Eigen/Dense>
#include <array>

namespace grillex {

// Forward declaration
class DOFHandler;

/**
 * @brief Loading condition for spring elements
 *
 * Controls when a spring is active based on load case type.
 * Used for modeling cargo connections that behave differently
 * for static (set-down) vs dynamic (environmental) loads.
 *
 * - All: Spring is active for all load cases (default)
 * - Static: Spring only active for Permanent load cases (e.g., bearing pads)
 * - Dynamic: Spring only active for Variable/Environmental load cases (e.g., seafastening)
 */
enum class LoadingCondition {
    All = 0,      ///< Active for all load cases (default)
    Static = 1,   ///< Only active for Permanent load cases
    Dynamic = 2   ///< Only active for Variable/Environmental/Accidental load cases
};

/**
 * @brief Spring behavior type for nonlinear analysis
 *
 * Controls whether a spring DOF is always active or only active
 * under certain deformation conditions.
 *
 * - Linear: Always active (default, current behavior)
 * - TensionOnly: Active only when elongated (δ > gap)
 * - CompressionOnly: Active only when compressed (δ < -gap)
 *
 * When a gap is specified:
 * - TensionOnly: Spring activates when δ > gap (e.g., cable with slack)
 * - CompressionOnly: Spring activates when δ < -gap (e.g., contact with clearance)
 *
 * Force calculation for active springs:
 * - TensionOnly: F = k × (δ - gap)
 * - CompressionOnly: F = k × (δ + gap), where δ is negative
 */
enum class SpringBehavior {
    Linear = 0,         ///< Always active (default)
    TensionOnly = 1,    ///< Active only when elongated (δ > gap)
    CompressionOnly = 2 ///< Active only when compressed (δ < -gap)
};

/**
 * @brief Spring element connecting two nodes
 *
 * Represents a general 6-DOF spring element with independent stiffness
 * values for each translational and rotational DOF. The element connects
 * two nodes and provides stiffness coupling between them.
 *
 * Stiffness matrix structure (for each DOF pair):
 *   K = [+k  -k]
 *       [-k  +k]
 *
 * For uncoupled springs, each DOF is independent.
 *
 * The loading_condition property controls when the spring is active:
 * - LoadingCondition::All: Active for all load cases (default)
 * - LoadingCondition::Static: Only for Permanent load cases
 * - LoadingCondition::Dynamic: Only for Variable/Environmental/Accidental
 *
 * Usage:
 *   SpringElement spring(1, node_i, node_j);
 *   spring.kx = 1000.0;  // Axial stiffness [kN/m]
 *   spring.kz = 500.0;   // Vertical stiffness [kN/m]
 *   spring.loading_condition = LoadingCondition::Static;  // Only for gravity
 *   auto K = spring.global_stiffness_matrix();
 */
class SpringElement {
public:
    int id;           ///< Unique element ID
    Node* node_i;     ///< Start node
    Node* node_j;     ///< End node

    // Translational stiffness [kN/m]
    double kx = 0.0;  ///< Stiffness in x direction
    double ky = 0.0;  ///< Stiffness in y direction
    double kz = 0.0;  ///< Stiffness in z direction

    // Rotational stiffness [kN·m/rad]
    double krx = 0.0; ///< Rotational stiffness about x axis
    double kry = 0.0; ///< Rotational stiffness about y axis
    double krz = 0.0; ///< Rotational stiffness about z axis

    /// Loading condition controlling when this spring is active
    LoadingCondition loading_condition = LoadingCondition::All;

    // === Nonlinear spring behavior (Phase 15) ===

    /**
     * @brief Per-DOF behavior type
     *
     * Allows mixed behavior, e.g., compression-only in Z, linear in X/Y.
     * DOF indices: 0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ
     */
    std::array<SpringBehavior, 6> behavior = {{
        SpringBehavior::Linear, SpringBehavior::Linear, SpringBehavior::Linear,
        SpringBehavior::Linear, SpringBehavior::Linear, SpringBehavior::Linear
    }};

    /**
     * @brief Per-DOF gap values
     *
     * Gap must be overcome before spring activates.
     * Units: [m] for translation (0-2), [rad] for rotation (3-5)
     * DOF indices: 0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ
     */
    std::array<double, 6> gap = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    /**
     * @brief Per-DOF active state (updated during nonlinear iteration)
     *
     * true = spring DOF contributes to stiffness matrix
     * false = spring DOF is inactive (zero stiffness contribution)
     * Initially all true; updated by update_state() during iteration.
     */
    std::array<bool, 6> is_active = {{true, true, true, true, true, true}};

    /**
     * @brief Current deformation for each DOF: δ = u_j - u_i [m or rad]
     *
     * Updated by update_state() during nonlinear iteration.
     * Positive values indicate elongation/positive rotation.
     */
    std::array<double, 6> deformation = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    /**
     * @brief Construct a spring element
     * @param id Unique element ID
     * @param node_i Start node
     * @param node_j End node
     */
    SpringElement(int id, Node* node_i, Node* node_j);

    /**
     * @brief Get the number of DOFs for this element
     * @return Always 12 (6 DOFs per node)
     */
    int num_dofs() const { return 12; }

    /**
     * @brief Check if element has warping DOF
     * @return Always false (springs don't have warping)
     */
    bool has_warping() const { return false; }

    /**
     * @brief Get global stiffness matrix (12×12)
     * @return Stiffness matrix in global coordinates
     *
     * Matrix structure for each stiffness k:
     *   [+k  0  ... -k  0  ...]
     *   [0  +k ... 0  -k  ...]
     *   ...
     *   [-k  0 ... +k  0  ...]
     *   [0  -k ... 0  +k  ...]
     *
     * Spring stiffness is already in global coordinates (no transformation needed).
     */
    Eigen::Matrix<double, 12, 12> global_stiffness_matrix() const;

    /**
     * @brief Get global mass matrix (12×12)
     * @return Zero matrix (springs have no mass)
     *
     * Springs are assumed massless. Use PointMass for concentrated masses.
     */
    Eigen::Matrix<double, 12, 12> global_mass_matrix() const;

    /**
     * @brief Set all translational stiffnesses at once
     * @param kx_val Stiffness in x direction [kN/m]
     * @param ky_val Stiffness in y direction [kN/m]
     * @param kz_val Stiffness in z direction [kN/m]
     */
    void set_translational_stiffness(double kx_val, double ky_val, double kz_val);

    /**
     * @brief Set all rotational stiffnesses at once
     * @param krx_val Rotational stiffness about x axis [kN·m/rad]
     * @param kry_val Rotational stiffness about y axis [kN·m/rad]
     * @param krz_val Rotational stiffness about z axis [kN·m/rad]
     */
    void set_rotational_stiffness(double krx_val, double kry_val, double krz_val);

    /**
     * @brief Check if element has any non-zero stiffness
     * @return true if any stiffness component is non-zero
     */
    bool has_stiffness() const;

    /**
     * @brief Check if this spring is active for a given load case type
     * @param type The load case type to check against
     * @return true if this spring should contribute to the load case
     *
     * Based on loading_condition:
     * - All: Returns true for all load case types
     * - Static: Returns true only for Permanent load cases
     * - Dynamic: Returns true for Variable, Environmental, and Accidental
     */
    bool is_active_for_load_case(LoadCaseType type) const;

    // === Nonlinear spring methods (Phase 15) ===

    /**
     * @brief Update spring state based on current displacements
     *
     * Computes deformation for each DOF and updates is_active based on
     * behavior type and gap values. Should be called during each
     * nonlinear iteration.
     *
     * @param displacements Global displacement vector [m, rad]
     * @param dof_handler DOF handler for global DOF indexing
     */
    void update_state(const Eigen::VectorXd& displacements,
                      const DOFHandler& dof_handler);

    /**
     * @brief Check if any DOF state changed in last update_state() call
     * @return true if any is_active value changed
     */
    bool state_changed() const { return state_changed_; }

    /**
     * @brief Check if any DOF has a non-zero gap
     * @return true if any gap[i] > gap_tolerance_
     */
    bool has_gap() const;

    /**
     * @brief Check if any DOF has nonlinear behavior (not Linear)
     * @return true if any behavior[i] != SpringBehavior::Linear
     */
    bool is_nonlinear() const;

    /**
     * @brief Compute spring forces for each DOF [kN or kN·m]
     *
     * For active DOFs:
     * - Linear: F = k × δ
     * - TensionOnly: F = k × (δ - gap)
     * - CompressionOnly: F = k × (δ + gap), where δ is negative
     *
     * For inactive DOFs: F = 0
     *
     * @return Array of forces for each DOF
     */
    std::array<double, 6> compute_forces() const;

    /**
     * @brief Compute gap closure forces for solver RHS (12×1 element vector)
     *
     * For active gap springs, returns the force offset term that must be
     * added to the right-hand side of the equation. This accounts for the
     * gap in the force-displacement relationship.
     *
     * For TensionOnly with gap: F_gap = -k × gap (pulls nodes together)
     * For CompressionOnly with gap: F_gap = +k × gap (pushes nodes apart)
     *
     * @return 12×1 element force vector (node_i DOFs 0-5, node_j DOFs 6-11)
     */
    Eigen::Matrix<double, 12, 1> compute_gap_forces() const;

    /**
     * @brief Get stiffness matrix respecting current active state (12×12)
     *
     * Like global_stiffness_matrix() but only includes contributions from
     * DOFs where is_active[i] == true. Used during nonlinear iteration.
     *
     * @return Stiffness matrix with inactive DOFs zeroed out
     */
    Eigen::Matrix<double, 12, 12> current_stiffness_matrix() const;

    /**
     * @brief Set behavior for a specific DOF
     * @param dof DOF index (0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ)
     * @param b Behavior type
     */
    void set_behavior(int dof, SpringBehavior b);

    /**
     * @brief Set behavior for all DOFs at once
     * @param b Behavior type to apply to all DOFs
     */
    void set_all_behavior(SpringBehavior b);

    /**
     * @brief Set gap for a specific DOF
     * @param dof DOF index (0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ)
     * @param g Gap value [m for translation, rad for rotation]
     */
    void set_gap(int dof, double g);

    /**
     * @brief Set gap for all DOFs at once
     * @param g Gap value [m for translation, rad for rotation]
     */
    void set_all_gaps(double g);

    /**
     * @brief Get the gap tolerance used for state determination
     * @return Gap tolerance value [m]
     */
    double get_gap_tolerance() const { return gap_tolerance_; }

    /**
     * @brief Set the gap tolerance used for state determination
     * @param tol New tolerance value [m]
     */
    void set_gap_tolerance(double tol) { gap_tolerance_ = tol; }

private:
    /// Flag indicating if state changed in last update_state() call
    bool state_changed_ = false;

    /// Tolerance for gap comparisons to prevent numerical chattering [m]
    double gap_tolerance_ = 1e-10;
};

} // namespace grillex
