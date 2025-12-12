#include "grillex/solver.hpp"
#include <cmath>
#include <sstream>

namespace grillex {

LinearSolver::LinearSolver(Method method)
    : method_(method), singular_(false), error_msg_(""), iterations_(0), error_(0.0) {}

Eigen::VectorXd LinearSolver::solve(const Eigen::SparseMatrix<double>& K,
                                     const Eigen::VectorXd& F) {
    // Reset error state
    singular_ = false;
    error_msg_ = "";
    iterations_ = 0;
    error_ = 0.0;

    // Check dimensions
    if (K.rows() != K.cols()) {
        singular_ = true;
        error_msg_ = "Stiffness matrix is not square";
        return Eigen::VectorXd::Zero(F.size());
    }

    if (K.rows() != F.size()) {
        singular_ = true;
        error_msg_ = "Stiffness matrix and force vector dimensions mismatch";
        return Eigen::VectorXd::Zero(F.size());
    }

    // Early singularity check
    if (check_singularity(K)) {
        singular_ = true;
        error_msg_ = "Stiffness matrix appears singular (rigid body modes without constraints)";
        return Eigen::VectorXd::Zero(F.size());
    }

    // Dispatch to appropriate solver
    switch (method_) {
        case Method::SparseLU:
            return solve_sparse_lu(K, F);
        case Method::SimplicialLDLT:
            return solve_simplicial_ldlt(K, F);
        case Method::ConjugateGradient:
            return solve_conjugate_gradient(K, F);
        default:
            singular_ = true;
            error_msg_ = "Unknown solver method";
            return Eigen::VectorXd::Zero(F.size());
    }
}

Eigen::VectorXd LinearSolver::solve_sparse_lu(const Eigen::SparseMatrix<double>& K,
                                               const Eigen::VectorXd& F) {
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(K);

    if (solver.info() != Eigen::Success) {
        singular_ = true;
        std::ostringstream oss;
        oss << "SparseLU decomposition failed (Eigen::ComputationInfo = " << solver.info() << ")";
        error_msg_ = oss.str();
        return Eigen::VectorXd::Zero(F.size());
    }

    Eigen::VectorXd u = solver.solve(F);

    if (solver.info() != Eigen::Success) {
        singular_ = true;
        std::ostringstream oss;
        oss << "SparseLU solve failed (Eigen::ComputationInfo = " << solver.info() << ")";
        error_msg_ = oss.str();
        return Eigen::VectorXd::Zero(F.size());
    }

    iterations_ = 1; // Direct solver
    return u;
}

Eigen::VectorXd LinearSolver::solve_simplicial_ldlt(const Eigen::SparseMatrix<double>& K,
                                                     const Eigen::VectorXd& F) {
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(K);

    if (solver.info() != Eigen::Success) {
        singular_ = true;
        std::ostringstream oss;
        oss << "SimplicialLDLT decomposition failed (Eigen::ComputationInfo = " << solver.info() << ")";

        // Provide more specific error messages
        if (solver.info() == Eigen::NumericalIssue) {
            oss << " - Matrix is not positive definite or has numerical issues";
        } else if (solver.info() == Eigen::NoConvergence) {
            oss << " - Solver did not converge";
        } else if (solver.info() == Eigen::InvalidInput) {
            oss << " - Invalid input matrix";
        }

        error_msg_ = oss.str();
        return Eigen::VectorXd::Zero(F.size());
    }

    Eigen::VectorXd u = solver.solve(F);

    if (solver.info() != Eigen::Success) {
        singular_ = true;
        std::ostringstream oss;
        oss << "SimplicialLDLT solve failed (Eigen::ComputationInfo = " << solver.info() << ")";
        error_msg_ = oss.str();
        return Eigen::VectorXd::Zero(F.size());
    }

    iterations_ = 1; // Direct solver
    return u;
}

Eigen::VectorXd LinearSolver::solve_conjugate_gradient(const Eigen::SparseMatrix<double>& K,
                                                        const Eigen::VectorXd& F) {
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> solver;
    solver.setMaxIterations(max_iterations_);
    solver.setTolerance(tolerance_);
    solver.compute(K);

    if (solver.info() != Eigen::Success) {
        singular_ = true;
        std::ostringstream oss;
        oss << "ConjugateGradient setup failed (Eigen::ComputationInfo = " << solver.info() << ")";
        error_msg_ = oss.str();
        return Eigen::VectorXd::Zero(F.size());
    }

    Eigen::VectorXd u = solver.solve(F);

    iterations_ = solver.iterations();
    error_ = solver.error();

    if (solver.info() != Eigen::Success) {
        singular_ = true;
        std::ostringstream oss;
        oss << "ConjugateGradient solve failed after " << iterations_
            << " iterations (error = " << error_ << ", Eigen::ComputationInfo = " << solver.info() << ")";
        error_msg_ = oss.str();
        return Eigen::VectorXd::Zero(F.size());
    }

    return u;
}

bool LinearSolver::check_singularity(const Eigen::SparseMatrix<double>& K) const {
    // Check for zero or near-zero diagonal entries
    // This is a heuristic check - the actual solver will catch true singularities

    double max_diag = 0.0;
    double min_nonzero_diag = std::numeric_limits<double>::max();
    int zero_diagonal_count = 0;

    // First pass: find statistics
    for (int k = 0; k < K.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(K, k); it; ++it) {
            if (it.row() == it.col()) {
                double val = std::abs(it.value());
                max_diag = std::max(max_diag, val);
                if (val > 0.0) {
                    min_nonzero_diag = std::min(min_nonzero_diag, val);
                } else {
                    zero_diagonal_count++;
                }
            }
        }
    }

    // If all diagonal entries are zero, definitely singular
    if (max_diag == 0.0) {
        return true;
    }

    // If there are zero diagonal entries, likely singular
    if (zero_diagonal_count > 0) {
        return true;
    }

    // Check for very small diagonal entries relative to typical scale
    // Use min_nonzero_diag as reference (not max_diag) to avoid issues with penalty method
    // If penalty BCs are applied, max_diag will be huge (1e15-1e21)
    // We want to check against the natural scale of the stiffness matrix
    if (min_nonzero_diag < 1e-10) {
        // Diagonal entries are suspiciously small (likely underflow or numerical issues)
        return true;
    }

    return false;
}

} // namespace grillex
