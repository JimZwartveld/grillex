#include "grillex/eigenvalue_solver.hpp"
#include "grillex/boundary_condition.hpp"
#include "grillex/dof_handler.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SparseLU>
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace grillex {

EigensolverResult EigenvalueSolver::solve(
    const Eigen::SparseMatrix<double>& K,
    const Eigen::SparseMatrix<double>& M,
    const EigensolverSettings& settings) const
{
    // Validate inputs
    if (K.rows() != K.cols()) {
        EigensolverResult result;
        result.converged = false;
        result.message = "Stiffness matrix K is not square";
        return result;
    }

    if (M.rows() != M.cols()) {
        EigensolverResult result;
        result.converged = false;
        result.message = "Mass matrix M is not square";
        return result;
    }

    if (K.rows() != M.rows()) {
        EigensolverResult result;
        result.converged = false;
        result.message = "K and M matrices have different sizes";
        return result;
    }

    if (K.rows() == 0) {
        EigensolverResult result;
        result.converged = false;
        result.message = "Empty matrices (no free DOFs)";
        return result;
    }

    int n_dofs = static_cast<int>(K.rows());
    int n_modes_requested = std::min(settings.n_modes, n_dofs);

    if (n_modes_requested <= 0) {
        EigensolverResult result;
        result.converged = false;
        result.message = "n_modes must be positive";
        return result;
    }

    // Select solver method
    switch (settings.method) {
        case EigensolverMethod::Dense: {
            // Convert to dense and solve
            Eigen::MatrixXd K_dense = Eigen::MatrixXd(K);
            Eigen::MatrixXd M_dense = Eigen::MatrixXd(M);
            return solve_dense(K_dense, M_dense, settings);
        }

        case EigensolverMethod::SubspaceIteration:
        case EigensolverMethod::ShiftInvert:
            // Use subspace iteration for these methods
            return solve_subspace_iteration(K, M, settings);

        default: {
            EigensolverResult result;
            result.converged = false;
            result.message = "Unknown solver method";
            return result;
        }
    }
}

std::tuple<Eigen::SparseMatrix<double>,
           Eigen::SparseMatrix<double>,
           std::vector<int>>
EigenvalueSolver::reduce_system(
    const Eigen::SparseMatrix<double>& K,
    const Eigen::SparseMatrix<double>& M,
    const BCHandler& bc_handler,
    const DOFHandler& dof_handler) const
{
    int n_total = static_cast<int>(K.rows());

    // Get list of fixed DOFs from boundary condition handler
    std::vector<bool> is_fixed(n_total, false);
    int n_fixed = 0;

    // Mark fixed DOFs - iterate over vector of FixedDOF structs
    for (const auto& fixed_dof : bc_handler.get_fixed_dofs()) {
        int node_id = fixed_dof.node_id;
        int local_dof = fixed_dof.local_dof;
        int element_id = fixed_dof.element_id;

        int global_dof = -1;

        // Handle warping DOFs differently (element-specific)
        if (local_dof == 6 && element_id >= 0) {
            // Element-specific warping DOF
            global_dof = dof_handler.get_warping_dof(element_id, node_id);
        } else {
            // Standard DOFs (0-5) or node-level warping
            global_dof = dof_handler.get_global_dof(node_id, local_dof);
        }

        if (global_dof >= 0 && global_dof < n_total) {
            if (!is_fixed[global_dof]) {
                is_fixed[global_dof] = true;
                n_fixed++;
            }
        }
    }

    int n_free = n_total - n_fixed;

    if (n_free <= 0) {
        // All DOFs are fixed - return empty matrices
        return std::make_tuple(
            Eigen::SparseMatrix<double>(0, 0),
            Eigen::SparseMatrix<double>(0, 0),
            std::vector<int>()
        );
    }

    // Create mapping: reduced DOF index -> full DOF index
    std::vector<int> reduced_to_full;
    reduced_to_full.reserve(n_free);

    // And reverse mapping: full DOF index -> reduced DOF index (-1 if fixed)
    std::vector<int> full_to_reduced(n_total, -1);

    int reduced_idx = 0;
    for (int full_idx = 0; full_idx < n_total; ++full_idx) {
        if (!is_fixed[full_idx]) {
            reduced_to_full.push_back(full_idx);
            full_to_reduced[full_idx] = reduced_idx;
            reduced_idx++;
        }
    }

    // Build reduced matrices using triplet lists
    std::vector<Eigen::Triplet<double>> K_triplets;
    std::vector<Eigen::Triplet<double>> M_triplets;

    // Reserve space (estimate: sparse matrices have ~10 entries per row)
    K_triplets.reserve(n_free * 10);
    M_triplets.reserve(n_free * 10);

    // Extract submatrices for free DOFs
    for (int k = 0; k < K.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(K, k); it; ++it) {
            int row_full = static_cast<int>(it.row());
            int col_full = static_cast<int>(it.col());

            int row_red = full_to_reduced[row_full];
            int col_red = full_to_reduced[col_full];

            // Only include if both DOFs are free
            if (row_red >= 0 && col_red >= 0) {
                K_triplets.emplace_back(row_red, col_red, it.value());
            }
        }
    }

    for (int k = 0; k < M.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(M, k); it; ++it) {
            int row_full = static_cast<int>(it.row());
            int col_full = static_cast<int>(it.col());

            int row_red = full_to_reduced[row_full];
            int col_red = full_to_reduced[col_full];

            // Only include if both DOFs are free
            if (row_red >= 0 && col_red >= 0) {
                M_triplets.emplace_back(row_red, col_red, it.value());
            }
        }
    }

    // Build sparse matrices
    Eigen::SparseMatrix<double> K_reduced(n_free, n_free);
    Eigen::SparseMatrix<double> M_reduced(n_free, n_free);

    K_reduced.setFromTriplets(K_triplets.begin(), K_triplets.end());
    M_reduced.setFromTriplets(M_triplets.begin(), M_triplets.end());

    return std::make_tuple(K_reduced, M_reduced, reduced_to_full);
}

Eigen::VectorXd EigenvalueSolver::expand_mode_shape(
    const Eigen::VectorXd& reduced_shape,
    const std::vector<int>& dof_mapping,
    int total_dofs)
{
    Eigen::VectorXd full_shape = Eigen::VectorXd::Zero(total_dofs);

    for (size_t i = 0; i < dof_mapping.size(); ++i) {
        if (i < static_cast<size_t>(reduced_shape.size()) &&
            dof_mapping[i] >= 0 &&
            dof_mapping[i] < total_dofs) {
            full_shape(dof_mapping[i]) = reduced_shape(static_cast<int>(i));
        }
    }

    return full_shape;
}

EigensolverResult EigenvalueSolver::solve_dense(
    const Eigen::MatrixXd& K,
    const Eigen::MatrixXd& M,
    const EigensolverSettings& settings) const
{
    EigensolverResult result;
    result.iterations = 1;  // Dense solver is direct

    int n_dofs = static_cast<int>(K.rows());
    int n_modes_requested = std::min(settings.n_modes, n_dofs);

    // Check for symmetric matrices
    double K_symmetry_error = (K - K.transpose()).norm() / (K.norm() + 1e-14);
    double M_symmetry_error = (M - M.transpose()).norm() / (M.norm() + 1e-14);

    if (K_symmetry_error > 1e-10) {
        result.converged = false;
        result.message = "Stiffness matrix K is not symmetric (error: " +
                         std::to_string(K_symmetry_error) + ")";
        return result;
    }

    if (M_symmetry_error > 1e-10) {
        result.converged = false;
        result.message = "Mass matrix M is not symmetric (error: " +
                         std::to_string(M_symmetry_error) + ")";
        return result;
    }

    // Solve generalized eigenvalue problem: K * φ = λ * M * φ
    // Using Eigen's GeneralizedSelfAdjointEigenSolver
    // This requires both K and M to be symmetric and M positive definite
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver;

    try {
        // Compute with Cholesky decomposition of M (requires M > 0)
        solver.compute(K, M);

        if (solver.info() != Eigen::Success) {
            result.converged = false;
            result.message = "Eigenvalue decomposition failed. "
                           "Mass matrix M may not be positive definite.";
            return result;
        }
    } catch (const std::exception& e) {
        result.converged = false;
        result.message = std::string("Exception during eigenvalue solve: ") + e.what();
        return result;
    }

    // Get eigenvalues (sorted in ascending order by Eigen)
    Eigen::VectorXd eigenvalues = solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = solver.eigenvectors();

    // Mass-normalize eigenvectors if requested
    Eigen::MatrixXd eigenvectors_normalized;
    if (settings.mass_normalize) {
        eigenvectors_normalized = mass_normalize(eigenvectors, M);
    } else {
        eigenvectors_normalized = eigenvectors;
    }

    // Count rigid body modes and populate results
    int n_rigid_body = 0;

    for (int i = 0; i < n_modes_requested; ++i) {
        ModeResult mode;
        mode.mode_number = i + 1;
        mode.eigenvalue = eigenvalues(i);
        mode.mode_shape = eigenvectors_normalized.col(i);

        // Compute frequencies and detect rigid body modes
        compute_frequencies(eigenvalues(i), mode, settings.rigid_body_threshold);

        if (mode.is_rigid_body_mode) {
            n_rigid_body++;
        }

        result.modes.push_back(mode);
    }

    result.n_rigid_body_modes = n_rigid_body;
    result.converged = true;

    if (n_rigid_body > 0) {
        result.message = "Converged. Found " + std::to_string(n_rigid_body) +
                        " rigid body mode(s).";
    } else {
        result.message = "Converged successfully.";
    }

    return result;
}

EigensolverResult EigenvalueSolver::solve_subspace_iteration(
    const Eigen::SparseMatrix<double>& K,
    const Eigen::SparseMatrix<double>& M,
    const EigensolverSettings& settings) const
{
    EigensolverResult result;
    int n = static_cast<int>(K.rows());
    int n_modes_requested = std::min(settings.n_modes, n);

    // For very small systems, fall back to dense solver
    if (n < 50 || n_modes_requested >= n / 2) {
        Eigen::MatrixXd K_dense = Eigen::MatrixXd(K);
        Eigen::MatrixXd M_dense = Eigen::MatrixXd(M);
        return solve_dense(K_dense, M_dense, settings);
    }

    // Subspace size: oversample for robust convergence
    // Use at least 2*n_modes or n_modes+8, whichever is larger (but not more than n)
    int p = std::min(std::max(2 * n_modes_requested, n_modes_requested + 8), n);

    // Form shifted matrix: A = K - σ*M
    Eigen::SparseMatrix<double> A = K;
    if (std::abs(settings.shift) > 1e-14) {
        A = K - settings.shift * M;
    }

    // Factor A for repeated solves using sparse LU
    Eigen::SparseLU<Eigen::SparseMatrix<double>> lu_solver;
    lu_solver.compute(A);

    if (lu_solver.info() != Eigen::Success) {
        result.converged = false;
        result.message = "SparseLU factorization failed. System may be singular "
                        "or shift may be an eigenvalue.";
        return result;
    }

    // Initialize random subspace X (n × p)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);

    Eigen::MatrixXd X(n, p);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            X(i, j) = dis(gen);
        }
    }

    // Previous eigenvalues for convergence checking
    Eigen::VectorXd prev_eigenvalues = Eigen::VectorXd::Zero(p);
    bool converged = false;
    int iter = 0;

    // Convert M to dense for small projected problems
    Eigen::MatrixXd M_dense = Eigen::MatrixXd(M);

    for (iter = 0; iter < settings.max_iterations; ++iter) {
        // Compute M * X
        Eigen::MatrixXd MX = M_dense * X;

        // Solve A * Y = M * X  (i.e., (K - σM) * Y = M * X)
        Eigen::MatrixXd Y(n, p);
        for (int j = 0; j < p; ++j) {
            Y.col(j) = lu_solver.solve(MX.col(j));
        }

        // Project onto subspace: K_proj = Y^T * K * Y, M_proj = Y^T * M * Y
        Eigen::MatrixXd K_dense = Eigen::MatrixXd(K);
        Eigen::MatrixXd K_proj = Y.transpose() * K_dense * Y;
        Eigen::MatrixXd M_proj = Y.transpose() * M_dense * Y;

        // Solve reduced eigenvalue problem (dense, small: p × p)
        Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> eig;
        try {
            eig.compute(K_proj, M_proj);
            if (eig.info() != Eigen::Success) {
                result.converged = false;
                result.message = "Reduced eigenvalue problem failed at iteration " +
                                std::to_string(iter + 1);
                return result;
            }
        } catch (const std::exception& e) {
            result.converged = false;
            result.message = std::string("Exception in reduced eigenproblem: ") + e.what();
            return result;
        }

        // Update subspace: X_new = Y * Z (where Z are eigenvectors of reduced problem)
        Eigen::MatrixXd Z = eig.eigenvectors();
        X = Y * Z;

        // Mass-orthonormalize the updated subspace
        for (int j = 0; j < p; ++j) {
            Eigen::VectorXd phi = X.col(j);
            double modal_mass = phi.transpose() * M_dense * phi;
            if (modal_mass > 1e-14) {
                X.col(j) = phi / std::sqrt(modal_mass);
            }
        }

        // Check convergence: eigenvalue relative change < tolerance
        Eigen::VectorXd curr_eigenvalues = eig.eigenvalues();
        double max_rel_change = 0.0;

        for (int j = 0; j < n_modes_requested; ++j) {
            double curr = curr_eigenvalues(j);
            double prev = prev_eigenvalues(j);
            double rel_change = 0.0;

            if (std::abs(curr) > 1e-14) {
                rel_change = std::abs(curr - prev) / std::abs(curr);
            } else if (std::abs(prev) > 1e-14) {
                rel_change = 1.0;  // Changed from non-zero to zero
            }
            // else both are ~0, rel_change stays 0

            max_rel_change = std::max(max_rel_change, rel_change);
        }

        prev_eigenvalues = curr_eigenvalues;

        if (iter > 0 && max_rel_change < settings.tolerance) {
            converged = true;
            break;
        }
    }

    // Final eigenvalue extraction
    Eigen::MatrixXd MX = M_dense * X;
    Eigen::MatrixXd Y(n, p);
    for (int j = 0; j < p; ++j) {
        Y.col(j) = lu_solver.solve(MX.col(j));
    }

    Eigen::MatrixXd K_dense_final = Eigen::MatrixXd(K);
    Eigen::MatrixXd K_proj = Y.transpose() * K_dense_final * Y;
    Eigen::MatrixXd M_proj = Y.transpose() * M_dense * Y;

    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> eig_final;
    eig_final.compute(K_proj, M_proj);

    if (eig_final.info() != Eigen::Success) {
        result.converged = false;
        result.message = "Final eigenvalue extraction failed";
        return result;
    }

    Eigen::VectorXd eigenvalues = eig_final.eigenvalues();
    Eigen::MatrixXd Z = eig_final.eigenvectors();
    Eigen::MatrixXd eigenvectors = Y * Z;

    // Mass-normalize final eigenvectors
    if (settings.mass_normalize) {
        for (int j = 0; j < p; ++j) {
            Eigen::VectorXd phi = eigenvectors.col(j);
            double modal_mass = phi.transpose() * M_dense * phi;
            if (modal_mass > 1e-14) {
                eigenvectors.col(j) = phi / std::sqrt(modal_mass);
            }
        }
    }

    // Count rigid body modes and populate results
    int n_rigid_body = 0;

    for (int i = 0; i < n_modes_requested; ++i) {
        ModeResult mode;
        mode.mode_number = i + 1;
        mode.eigenvalue = eigenvalues(i);
        mode.mode_shape = eigenvectors.col(i);

        // Compute frequencies and detect rigid body modes
        compute_frequencies(eigenvalues(i), mode, settings.rigid_body_threshold);

        if (mode.is_rigid_body_mode) {
            n_rigid_body++;
        }

        result.modes.push_back(mode);
    }

    result.n_rigid_body_modes = n_rigid_body;
    result.iterations = iter + 1;
    result.converged = converged;

    if (converged) {
        if (n_rigid_body > 0) {
            result.message = "Converged in " + std::to_string(iter + 1) +
                            " iterations. Found " + std::to_string(n_rigid_body) +
                            " rigid body mode(s).";
        } else {
            result.message = "Converged in " + std::to_string(iter + 1) + " iterations.";
        }
    } else {
        result.message = "Did not converge within " +
                        std::to_string(settings.max_iterations) +
                        " iterations. Results may be inaccurate.";
    }

    return result;
}

Eigen::MatrixXd EigenvalueSolver::mass_normalize(
    const Eigen::MatrixXd& eigenvectors,
    const Eigen::MatrixXd& M) const
{
    int n_modes = static_cast<int>(eigenvectors.cols());
    Eigen::MatrixXd normalized = eigenvectors;

    for (int i = 0; i < n_modes; ++i) {
        Eigen::VectorXd phi = eigenvectors.col(i);

        // Compute modal mass: m_i = φᵀ × M × φ
        double modal_mass = phi.transpose() * M * phi;

        if (modal_mass > 1e-14) {
            // Normalize: φ_norm = φ / √(m_i) so that φᵀMφ = 1
            normalized.col(i) = phi / std::sqrt(modal_mass);
        }
        // If modal mass is zero/negative, keep original (shouldn't happen with valid M)
    }

    return normalized;
}

void EigenvalueSolver::compute_frequencies(
    double eigenvalue,
    ModeResult& mode,
    double rigid_body_threshold) const
{
    // Handle negative or near-zero eigenvalues (rigid body modes)
    if (eigenvalue < rigid_body_threshold) {
        mode.is_rigid_body_mode = true;
        mode.omega = 0.0;
        mode.frequency_hz = 0.0;
        mode.period_s = std::numeric_limits<double>::infinity();
    } else {
        mode.is_rigid_body_mode = false;

        // ω = √λ [rad/s]
        mode.omega = std::sqrt(eigenvalue);

        // f = ω / (2π) [Hz]
        mode.frequency_hz = mode.omega / (2.0 * M_PI);

        // T = 1/f [s]
        if (mode.frequency_hz > 1e-14) {
            mode.period_s = 1.0 / mode.frequency_hz;
        } else {
            mode.period_s = std::numeric_limits<double>::infinity();
        }
    }
}

void EigenvalueSolver::compute_participation_factors(
    EigensolverResult& result,
    const Eigen::MatrixXd& M_reduced,
    const std::vector<int>& dof_mapping,
    const DOFHandler& dof_handler,
    double total_mass) const
{
    int n_reduced = static_cast<int>(M_reduced.rows());

    // Build influence vectors for each translational direction
    // These vectors have 1.0 at translational DOFs in the given direction
    Eigen::VectorXd r_x = Eigen::VectorXd::Zero(n_reduced);
    Eigen::VectorXd r_y = Eigen::VectorXd::Zero(n_reduced);
    Eigen::VectorXd r_z = Eigen::VectorXd::Zero(n_reduced);

    // Map reduced DOF index to DOF type (0-5 for UX,UY,UZ,RX,RY,RZ)
    for (int i = 0; i < n_reduced; ++i) {
        int full_dof = dof_mapping[i];

        // Determine which DOF type this is by checking the remainder when divided by 6
        // (assuming standard 6-DOF per node without warping)
        // Local DOF: 0=UX, 1=UY, 2=UZ, 3=RX, 4=RY, 5=RZ
        int local_dof = full_dof % 6;

        if (local_dof == 0) {  // UX
            r_x(i) = 1.0;
        } else if (local_dof == 1) {  // UY
            r_y(i) = 1.0;
        } else if (local_dof == 2) {  // UZ
            r_z(i) = 1.0;
        }
    }

    // Compute total modal mass in each direction: M_total = rᵀ × M × r
    double M_total_x = r_x.transpose() * M_reduced * r_x;
    double M_total_y = r_y.transpose() * M_reduced * r_y;
    double M_total_z = r_z.transpose() * M_reduced * r_z;

    // Store total mass in result (use computed values if available, otherwise use input)
    result.total_mass_x = (M_total_x > 1e-14) ? M_total_x : total_mass;
    result.total_mass_y = (M_total_y > 1e-14) ? M_total_y : total_mass;
    result.total_mass_z = (M_total_z > 1e-14) ? M_total_z : total_mass;

    // Initialize cumulative mass vectors
    result.cumulative_mass_pct_x.clear();
    result.cumulative_mass_pct_y.clear();
    result.cumulative_mass_pct_z.clear();

    double cumulative_x = 0.0;
    double cumulative_y = 0.0;
    double cumulative_z = 0.0;

    // Compute participation factors for each mode
    for (auto& mode : result.modes) {
        const Eigen::VectorXd& phi = mode.mode_shape;

        // Check size compatibility
        if (phi.size() != n_reduced) {
            continue;  // Skip if mode shape size doesn't match
        }

        // Participation factors: Γ = φᵀ × M × r
        double gamma_x = phi.transpose() * M_reduced * r_x;
        double gamma_y = phi.transpose() * M_reduced * r_y;
        double gamma_z = phi.transpose() * M_reduced * r_z;

        mode.participation_x = gamma_x;
        mode.participation_y = gamma_y;
        mode.participation_z = gamma_z;

        // Effective modal mass: Meff = Γ² (for mass-normalized modes where φᵀMφ = 1)
        mode.effective_mass_x = gamma_x * gamma_x;
        mode.effective_mass_y = gamma_y * gamma_y;
        mode.effective_mass_z = gamma_z * gamma_z;

        // Effective modal mass percentage
        if (result.total_mass_x > 1e-14) {
            mode.effective_mass_pct_x = 100.0 * mode.effective_mass_x / result.total_mass_x;
        }
        if (result.total_mass_y > 1e-14) {
            mode.effective_mass_pct_y = 100.0 * mode.effective_mass_y / result.total_mass_y;
        }
        if (result.total_mass_z > 1e-14) {
            mode.effective_mass_pct_z = 100.0 * mode.effective_mass_z / result.total_mass_z;
        }

        // Update cumulative percentages
        cumulative_x += mode.effective_mass_pct_x;
        cumulative_y += mode.effective_mass_pct_y;
        cumulative_z += mode.effective_mass_pct_z;

        result.cumulative_mass_pct_x.push_back(cumulative_x);
        result.cumulative_mass_pct_y.push_back(cumulative_y);
        result.cumulative_mass_pct_z.push_back(cumulative_z);
    }
}

} // namespace grillex
