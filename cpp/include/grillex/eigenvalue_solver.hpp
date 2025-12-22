#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <string>

namespace grillex {

// Forward declarations
class BCHandler;
class DOFHandler;

/**
 * @brief Eigenvalue solver method selection
 */
enum class EigensolverMethod {
    Dense,              ///< Eigen's GeneralizedSelfAdjointEigenSolver (all modes, small systems)
    SubspaceIteration,  ///< Iterative subspace method (selected modes, large systems)
    ShiftInvert         ///< Shift-and-invert with power iteration
};

/**
 * @brief Configuration settings for eigenvalue analysis
 *
 * Controls solver behavior, convergence criteria, and output options.
 *
 * Usage:
 *   EigensolverSettings settings;
 *   settings.n_modes = 10;           // Find first 10 modes
 *   settings.tolerance = 1e-8;        // Convergence tolerance
 *   settings.method = EigensolverMethod::SubspaceIteration;
 */
struct EigensolverSettings {
    /// Number of modes to compute (lowest frequencies first)
    int n_modes = 10;

    /// Frequency shift for shift-and-invert [rad/s]²
    /// Set to 0 to find lowest frequency modes
    /// Set to target frequency² to find modes near that frequency
    double shift = 0.0;

    /// Convergence tolerance for eigenvalue relative change
    double tolerance = 1e-8;

    /// Maximum iterations for iterative solvers
    int max_iterations = 100;

    /// Solver method to use
    EigensolverMethod method = EigensolverMethod::Dense;

    /// Whether to compute participation factors (requires DOF type info)
    bool compute_participation = true;

    /// Whether to mass-normalize mode shapes (φᵀMφ = 1)
    bool mass_normalize = true;

    /// Threshold for rigid body mode detection [rad/s]²
    /// Modes with eigenvalue < this are considered rigid body modes
    double rigid_body_threshold = 1e-6;
};

/**
 * @brief Result for a single vibration mode
 *
 * Contains eigenvalue, frequency, period, mode shape, and modal quantities.
 * All mode shapes are mass-normalized if settings.mass_normalize is true.
 */
struct ModeResult {
    /// Mode number (1-based indexing)
    int mode_number = 0;

    /// Eigenvalue λ = ω² [rad/s]²
    double eigenvalue = 0.0;

    /// Natural circular frequency ω = √λ [rad/s]
    double omega = 0.0;

    /// Natural frequency f = ω/(2π) [Hz]
    double frequency_hz = 0.0;

    /// Natural period T = 1/f [s]
    double period_s = 0.0;

    /// Mode shape vector (mass-normalized if settings.mass_normalize)
    /// Size equals number of free DOFs in the reduced system
    /// Use expand_mode_shape() to get full DOF vector
    Eigen::VectorXd mode_shape;

    /// Whether this is a rigid body mode (ω ≈ 0)
    bool is_rigid_body_mode = false;

    // ===== Participation factors =====
    // Γ = φᵀ × M × r, where r is influence vector

    /// Participation factor for X translation
    double participation_x = 0.0;

    /// Participation factor for Y translation
    double participation_y = 0.0;

    /// Participation factor for Z translation
    double participation_z = 0.0;

    /// Participation factor for rotation about X
    double participation_rx = 0.0;

    /// Participation factor for rotation about Y
    double participation_ry = 0.0;

    /// Participation factor for rotation about Z
    double participation_rz = 0.0;

    // ===== Effective modal mass =====
    // Meff = Γ² (for mass-normalized modes)

    /// Effective modal mass for X translation [mT]
    double effective_mass_x = 0.0;

    /// Effective modal mass for Y translation [mT]
    double effective_mass_y = 0.0;

    /// Effective modal mass for Z translation [mT]
    double effective_mass_z = 0.0;

    // ===== Effective modal mass percentage =====
    // Meff / M_total × 100%

    /// Effective modal mass percentage for X [%]
    double effective_mass_pct_x = 0.0;

    /// Effective modal mass percentage for Y [%]
    double effective_mass_pct_y = 0.0;

    /// Effective modal mass percentage for Z [%]
    double effective_mass_pct_z = 0.0;
};

/**
 * @brief Complete eigenvalue analysis result
 *
 * Contains all computed modes plus summary statistics.
 */
struct EigensolverResult {
    /// Whether solver converged successfully
    bool converged = false;

    /// Number of iterations (for iterative solvers)
    int iterations = 0;

    /// Status/error message
    std::string message;

    /// Computed modes, sorted by frequency (ascending)
    std::vector<ModeResult> modes;

    /// Number of rigid body modes detected
    int n_rigid_body_modes = 0;

    // ===== Total mass for percentage calculations =====

    /// Total translational mass in X direction [mT]
    double total_mass_x = 0.0;

    /// Total translational mass in Y direction [mT]
    double total_mass_y = 0.0;

    /// Total translational mass in Z direction [mT]
    double total_mass_z = 0.0;

    // ===== Cumulative effective mass percentages =====
    // cumulative[i] = sum(effective_mass_pct[0:i+1])

    /// Cumulative effective mass percentage for X [%]
    std::vector<double> cumulative_mass_pct_x;

    /// Cumulative effective mass percentage for Y [%]
    std::vector<double> cumulative_mass_pct_y;

    /// Cumulative effective mass percentage for Z [%]
    std::vector<double> cumulative_mass_pct_z;

    // ===== DOF mapping for mode shape expansion =====

    /// Mapping from reduced DOF index to full DOF index
    /// reduced_to_full[i] = global DOF index for reduced DOF i
    std::vector<int> reduced_to_full;

    /// Total number of DOFs (before reduction)
    int total_dofs = 0;

    /**
     * @brief Get mode by number (1-based)
     * @param mode_number Mode number (1 = first mode)
     * @return Pointer to ModeResult, or nullptr if not found
     */
    const ModeResult* get_mode(int mode_number) const {
        for (const auto& mode : modes) {
            if (mode.mode_number == mode_number) {
                return &mode;
            }
        }
        return nullptr;
    }

    /**
     * @brief Get vector of natural frequencies [Hz]
     * @return Vector of frequencies, sorted ascending
     */
    std::vector<double> get_frequencies() const {
        std::vector<double> freqs;
        freqs.reserve(modes.size());
        for (const auto& mode : modes) {
            freqs.push_back(mode.frequency_hz);
        }
        return freqs;
    }

    /**
     * @brief Get vector of natural periods [s]
     * @return Vector of periods
     */
    std::vector<double> get_periods() const {
        std::vector<double> periods;
        periods.reserve(modes.size());
        for (const auto& mode : modes) {
            periods.push_back(mode.period_s);
        }
        return periods;
    }

    /**
     * @brief Expand reduced mode shape to full DOF vector
     * @param reduced_shape Mode shape from ModeResult (size = n_free_dofs)
     * @return Full mode shape vector (size = total_dofs) with zeros at fixed DOFs
     */
    Eigen::VectorXd expand_mode_shape(const Eigen::VectorXd& reduced_shape) const {
        Eigen::VectorXd full_shape = Eigen::VectorXd::Zero(total_dofs);
        for (size_t i = 0; i < reduced_to_full.size(); ++i) {
            if (i < static_cast<size_t>(reduced_shape.size())) {
                full_shape(reduced_to_full[i]) = reduced_shape(i);
            }
        }
        return full_shape;
    }

    /**
     * @brief Get expanded mode shape for a specific mode
     * @param mode_number Mode number (1-based)
     * @return Full mode shape vector, or empty vector if mode not found
     */
    Eigen::VectorXd get_expanded_mode_shape(int mode_number) const {
        const ModeResult* mode = get_mode(mode_number);
        if (mode) {
            return expand_mode_shape(mode->mode_shape);
        }
        return Eigen::VectorXd();
    }
};

/**
 * @brief Eigenvalue solver for structural dynamics
 *
 * Solves the generalized eigenvalue problem:
 *   K × φ = ω² × M × φ
 *
 * Where:
 * - K: Global stiffness matrix [kN/m]
 * - M: Global mass matrix [mT]
 * - ω: Natural circular frequency [rad/s]
 * - φ: Mode shape (eigenvector)
 *
 * Supports multiple solver methods:
 * - Dense: Uses Eigen's GeneralizedSelfAdjointEigenSolver (all modes)
 * - SubspaceIteration: Iterative method for large systems (selected modes)
 * - ShiftInvert: Shift-and-invert for targeting specific frequencies
 *
 * Usage:
 *   EigenvalueSolver solver;
 *   EigensolverSettings settings;
 *   settings.n_modes = 10;
 *
 *   // Reduce system to eliminate fixed DOFs
 *   auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc_handler, dof_handler);
 *
 *   // Solve eigenvalue problem
 *   EigensolverResult result = solver.solve(K_red, M_red, settings);
 *
 *   // Expand mode shapes to full DOF vector
 *   Eigen::VectorXd mode1_full = result.get_expanded_mode_shape(1);
 */
class EigenvalueSolver {
public:
    /**
     * @brief Default constructor
     */
    EigenvalueSolver() = default;

    /**
     * @brief Solve eigenvalue problem
     * @param K Stiffness matrix (should be reduced, i.e., fixed DOFs eliminated)
     * @param M Mass matrix (should be reduced, same size as K)
     * @param settings Solver configuration
     * @return EigensolverResult with eigenvalues, frequencies, and mode shapes
     *
     * The input matrices should already have fixed DOFs eliminated.
     * Use reduce_system() to obtain reduced matrices.
     */
    EigensolverResult solve(
        const Eigen::SparseMatrix<double>& K,
        const Eigen::SparseMatrix<double>& M,
        const EigensolverSettings& settings = EigensolverSettings{}) const;

    /**
     * @brief Reduce system by eliminating fixed DOFs
     * @param K Full stiffness matrix
     * @param M Full mass matrix
     * @param bc_handler Boundary condition handler with fixed DOF info
     * @param dof_handler DOF handler for DOF indexing
     * @return Tuple of (K_reduced, M_reduced, dof_mapping)
     *         dof_mapping[i] = global DOF index for reduced DOF i
     *
     * Unlike static analysis which uses penalty method, eigenvalue analysis
     * requires true elimination to avoid spurious high-frequency modes.
     */
    std::tuple<Eigen::SparseMatrix<double>,
               Eigen::SparseMatrix<double>,
               std::vector<int>>
    reduce_system(
        const Eigen::SparseMatrix<double>& K,
        const Eigen::SparseMatrix<double>& M,
        const BCHandler& bc_handler,
        const DOFHandler& dof_handler) const;

    /**
     * @brief Expand reduced mode shape to full DOF vector
     * @param reduced_shape Mode shape in reduced coordinates
     * @param dof_mapping Mapping from reduced to full DOF indices
     * @param total_dofs Total number of DOFs in full system
     * @return Full mode shape with zeros at fixed DOFs
     */
    static Eigen::VectorXd expand_mode_shape(
        const Eigen::VectorXd& reduced_shape,
        const std::vector<int>& dof_mapping,
        int total_dofs);

    /**
     * @brief Compute participation factors and effective modal mass
     * @param result EigensolverResult to update (modes will be modified)
     * @param M_reduced Reduced mass matrix (same size as mode shapes)
     * @param dof_mapping Mapping from reduced DOF index to full DOF index
     * @param dof_handler DOF handler for determining DOF types
     * @param total_mass Total translational mass of structure [mT]
     *
     * Computes for each mode:
     * - Participation factors Γ = φᵀ × M × r for each direction
     * - Effective modal mass Meff = Γ² (for mass-normalized modes)
     * - Effective modal mass percentage
     * - Cumulative effective mass percentages
     *
     * Requires mode shapes to be mass-normalized (φᵀMφ = 1).
     */
    void compute_participation_factors(
        EigensolverResult& result,
        const Eigen::MatrixXd& M_reduced,
        const std::vector<int>& dof_mapping,
        const DOFHandler& dof_handler,
        double total_mass) const;

private:
    /**
     * @brief Solve using dense Eigen solver
     * @param K Reduced stiffness matrix
     * @param M Reduced mass matrix
     * @param settings Solver settings
     * @return EigensolverResult
     */
    EigensolverResult solve_dense(
        const Eigen::MatrixXd& K,
        const Eigen::MatrixXd& M,
        const EigensolverSettings& settings) const;

    /**
     * @brief Solve using subspace iteration
     * @param K Reduced stiffness matrix (sparse)
     * @param M Reduced mass matrix (sparse)
     * @param settings Solver settings
     * @return EigensolverResult
     */
    EigensolverResult solve_subspace_iteration(
        const Eigen::SparseMatrix<double>& K,
        const Eigen::SparseMatrix<double>& M,
        const EigensolverSettings& settings) const;

    /**
     * @brief Mass-normalize a set of eigenvectors
     * @param eigenvectors Matrix of eigenvectors (columns)
     * @param M Mass matrix
     * @return Mass-normalized eigenvectors (φᵀMφ = I)
     */
    Eigen::MatrixXd mass_normalize(
        const Eigen::MatrixXd& eigenvectors,
        const Eigen::MatrixXd& M) const;

    /**
     * @brief Compute frequencies and periods from eigenvalues
     * @param eigenvalue λ = ω² [rad/s]²
     * @param mode ModeResult to populate (omega, frequency_hz, period_s)
     * @param rigid_body_threshold Threshold for rigid body mode detection
     */
    void compute_frequencies(
        double eigenvalue,
        ModeResult& mode,
        double rigid_body_threshold) const;
};

} // namespace grillex
