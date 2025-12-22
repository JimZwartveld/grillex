#include "grillex/eigenvalue_solver.hpp"
#include "grillex/boundary_condition.hpp"
#include "grillex/dof_handler.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
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
            // For now, fall back to dense for these methods
            // TODO: Implement subspace iteration in Task 16.4
            {
                Eigen::MatrixXd K_dense = Eigen::MatrixXd(K);
                Eigen::MatrixXd M_dense = Eigen::MatrixXd(M);
                return solve_dense(K_dense, M_dense, settings);
            }

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
    // TODO: Implement in Task 16.4
    // For now, convert to dense and use dense solver
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K);
    Eigen::MatrixXd M_dense = Eigen::MatrixXd(M);
    return solve_dense(K_dense, M_dense, settings);
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

} // namespace grillex
