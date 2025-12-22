#pragma once

#include "grillex/solver.hpp"
#include "grillex/spring_element.hpp"
#include "grillex/dof_handler.hpp"
#include <Eigen/Sparse>
#include <vector>
#include <map>
#include <string>
#include <functional>
#include <array>

namespace grillex {

// Forward declarations
class SpringElement;
class DOFHandler;

/**
 * @brief Result from nonlinear spring solver
 *
 * Contains the solution displacements, convergence status, iteration count,
 * and final spring states/forces for reporting.
 */
struct NonlinearSolverResult {
    /// Solution displacement vector [m, rad]
    Eigen::VectorXd displacements;

    /// True if solver converged
    bool converged = false;

    /// Number of iterations performed
    int iterations = 0;

    /// Descriptive message (convergence info or error)
    std::string message;

    /// Final spring states: (spring_id, active_state[6])
    std::vector<std::pair<int, std::array<bool, 6>>> spring_states;

    /// Final spring forces: (spring_id, forces[6]) [kN or kN·m]
    std::vector<std::pair<int, std::array<double, 6>>> spring_forces;

    /// History of state changes per iteration (for diagnostics)
    std::vector<int> state_changes_per_iteration;
};

/**
 * @brief Initial state for starting nonlinear iteration
 *
 * Used to start from a known state (e.g., static solution) rather than zero.
 * This is essential for static→dynamic load sequencing where the static
 * (gravity) solution establishes the baseline contact pattern.
 */
struct NonlinearInitialState {
    /// Initial displacement vector (empty = start from zero)
    Eigen::VectorXd displacement;

    /// Initial spring states: (spring_id, active_states[6])
    /// If empty, all springs start as active
    std::vector<std::pair<int, std::array<bool, 6>>> spring_states;

    /// Check if initial state is provided
    bool has_initial_state() const {
        return displacement.size() > 0;
    }
};

/**
 * @brief Settings for nonlinear spring solver
 */
struct NonlinearSolverSettings {
    /// Maximum iterations before giving up
    int max_iterations = 50;

    /// Tolerance for spring activation threshold [m]
    /// Prevents chattering when deformation is very close to gap
    double gap_tolerance = 1e-10;

    /// Relative displacement tolerance for convergence check
    /// Used as secondary check: ||u_new - u_old|| / ||u_new|| < tol
    double displacement_tolerance = 1e-8;

    /// Allow solution where all nonlinear springs are inactive?
    /// If false, solver warns when this happens
    bool allow_all_springs_inactive = false;

    /// Enable oscillation detection and damping
    bool enable_oscillation_damping = true;

    /// Number of iterations to look back for oscillation detection
    int oscillation_history_size = 4;

    /// Damping factor when oscillation detected (0 < alpha < 1)
    /// u_damped = alpha * u_new + (1 - alpha) * u_old
    double oscillation_damping_factor = 0.5;

    /// Use partial stiffness (0.5*k) for oscillating springs?
    /// When true, springs that oscillate use half stiffness instead of 0 or k
    /// This can help convergence for marginally-active springs
    bool use_partial_stiffness = false;

    /// Hysteresis band width [m for translation, rad for rotation]
    /// Different thresholds for activation vs deactivation:
    /// - Activate when deformation > gap + hysteresis_band
    /// - Deactivate when deformation < gap - hysteresis_band
    /// Set to 0 for no hysteresis (default)
    double hysteresis_band = 0.0;

    /// Linear solver method to use
    LinearSolver::Method linear_method = LinearSolver::Method::SimplicialLDLT;

    /// Optional callback for iteration progress monitoring
    /// Parameters: (iteration, num_state_changes, displacement_norm)
    std::function<void(int, int, double)> progress_callback = nullptr;
};

/**
 * @brief Iterative solver for systems with nonlinear springs
 *
 * Handles tension-only, compression-only, and gap springs through
 * an iterative state-update algorithm. Each iteration:
 * 1. Assembles stiffness from currently active springs
 * 2. Computes gap closure forces for active gap springs
 * 3. Solves the linear system
 * 4. Updates spring states based on new displacements
 * 5. Checks for convergence (no state changes)
 *
 * **Static→Dynamic Sequencing**:
 * For proper cargo analysis, solve static loads first, then use that
 * solution as the initial state for dynamic combinations:
 *
 *   // Step 1: Solve static (gravity) to get baseline
 *   auto static_result = solver.solve(base_K, F_static, springs, dof_handler);
 *
 *   // Step 2: Create initial state from static solution
 *   NonlinearInitialState init;
 *   init.displacement = static_result.displacements;
 *   init.spring_states = static_result.spring_states;
 *
 *   // Step 3: Solve combined (static + dynamic) starting from static state
 *   auto combined_result = solver.solve(base_K, F_combined, springs, dof_handler, init);
 */
class NonlinearSolver {
public:
    /**
     * @brief Construct solver with settings
     * @param settings Solver configuration
     */
    explicit NonlinearSolver(const NonlinearSolverSettings& settings = {});

    /**
     * @brief Solve system with nonlinear springs
     *
     * @param base_K Base stiffness matrix (beams, plates - excludes springs)
     * @param F External force vector [kN]
     * @param springs Vector of spring elements (states will be updated)
     * @param dof_handler DOF handler for global DOF indexing
     * @param initial_state Optional initial state from previous solve (e.g., static)
     * @return Solver result with displacements and convergence info
     *
     * The springs vector is modified: after solve(), each spring's
     * is_active and deformation arrays contain the final converged state.
     *
     * **Initial State for Static→Dynamic Sequencing**:
     * When analyzing dynamic load combinations, provide the static solution
     * as initial_state. This ensures:
     * - Iteration starts from physically correct baseline (cargo settled)
     * - Gap springs that closed under static load start as active
     * - Springs that were inactive stay inactive unless dynamic loads change them
     */
    NonlinearSolverResult solve(
        const Eigen::SparseMatrix<double>& base_K,
        const Eigen::VectorXd& F,
        std::vector<SpringElement*>& springs,
        const DOFHandler& dof_handler,
        const NonlinearInitialState& initial_state = {});

    /// Get current settings
    const NonlinearSolverSettings& settings() const { return settings_; }

    /// Update settings
    void set_settings(const NonlinearSolverSettings& s) { settings_ = s; }

private:
    NonlinearSolverSettings settings_;
    LinearSolver linear_solver_;

    /**
     * @brief Assemble spring stiffness matrix from active springs
     *
     * Only includes contributions from springs where is_active[dof] == true.
     * Uses sparse triplet assembly for efficiency.
     */
    Eigen::SparseMatrix<double> assemble_spring_stiffness(
        const std::vector<SpringElement*>& springs,
        const DOFHandler& dof_handler,
        int total_dofs) const;

    /**
     * @brief Assemble gap closure forces for active gap springs
     *
     * For each active spring with gap > 0, computes the force offset
     * that accounts for the gap in the force-displacement relationship.
     */
    Eigen::VectorXd assemble_gap_forces(
        const std::vector<SpringElement*>& springs,
        const DOFHandler& dof_handler,
        int total_dofs) const;

    /**
     * @brief Check for oscillation in state history
     *
     * Detects if spring states are cycling between the same patterns,
     * indicating the solver is not converging.
     *
     * @param history State snapshots from recent iterations
     * @return true if oscillation detected
     */
    bool detect_oscillation(
        const std::vector<std::vector<bool>>& history) const;

    /**
     * @brief Encode current spring states as a flat boolean vector
     */
    std::vector<bool> encode_spring_states(
        const std::vector<SpringElement*>& springs) const;
};

} // namespace grillex
