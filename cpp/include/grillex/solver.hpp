#pragma once

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/IterativeLinearSolvers>
#include <string>

namespace grillex {

/**
 * @brief Linear solver for finite element systems
 *
 * Solves linear systems of the form K * u = F where:
 * - K: Global stiffness matrix [kN/m] (sparse, symmetric, positive semi-definite)
 * - F: Global force vector [kN]
 * - u: Global displacement vector [m, rad, m²] (warping)
 *
 * Supports multiple solver methods:
 * - SparseLU: Direct solver, works for any sparse matrix (symmetric or not)
 * - SimplicialLDLT: Direct solver, optimized for symmetric positive definite matrices (default)
 * - ConjugateGradient: Iterative solver, memory-efficient for large systems
 *
 * Usage:
 *   LinearSolver solver(LinearSolver::Method::SimplicialLDLT);
 *   Eigen::VectorXd u = solver.solve(K, F);
 *   if (solver.is_singular()) {
 *       std::cerr << solver.get_error_message() << std::endl;
 *   }
 */
class LinearSolver {
public:
    /**
     * @brief Available solver methods
     */
    enum class Method {
        SparseLU,           ///< Direct solver (Eigen::SparseLU) - general sparse matrices
        SimplicialLDLT,     ///< Direct solver (Eigen::SimplicialLDLT) - symmetric matrices (default)
        ConjugateGradient   ///< Iterative solver (Eigen::ConjugateGradient) - large systems
    };

    /**
     * @brief Construct a linear solver with specified method
     * @param method Solver method to use (default: SimplicialLDLT)
     */
    explicit LinearSolver(Method method = Method::SimplicialLDLT);

    /**
     * @brief Solve the linear system K * u = F
     * @param K Global stiffness matrix (sparse, N×N)
     * @param F Global force vector (N×1)
     * @return Displacement vector u (N×1)
     *
     * If the system is singular, returns a zero vector and sets is_singular() to true.
     * Check is_singular() and get_error_message() after calling solve().
     */
    Eigen::VectorXd solve(const Eigen::SparseMatrix<double>& K,
                          const Eigen::VectorXd& F);

    /**
     * @brief Check if the last solve detected a singular system
     * @return true if system is singular (unbounded, rigid body modes without constraints)
     */
    bool is_singular() const { return singular_; }

    /**
     * @brief Get error message from last solve
     * @return Error message string (empty if no error)
     */
    std::string get_error_message() const { return error_msg_; }

    /**
     * @brief Get the solver method being used
     * @return Current solver method
     */
    Method get_method() const { return method_; }

    /**
     * @brief Set the solver method
     * @param method New solver method to use
     */
    void set_method(Method method) { method_ = method; }

    /**
     * @brief Get the number of iterations (for iterative solvers only)
     * @return Number of iterations, or 1 for direct solvers
     */
    int get_iterations() const { return iterations_; }

    /**
     * @brief Get the estimated error (for iterative solvers only)
     * @return Estimated error, or 0.0 for direct solvers
     */
    double get_error() const { return error_; }

    /**
     * @brief Set maximum iterations for iterative solvers
     * @param max_iter Maximum iterations (default: 1000)
     */
    void set_max_iterations(int max_iter) { max_iterations_ = max_iter; }

    /**
     * @brief Set tolerance for iterative solvers
     * @param tol Convergence tolerance (default: 1e-10)
     */
    void set_tolerance(double tol) { tolerance_ = tol; }

private:
    Method method_;                 ///< Current solver method
    bool singular_ = false;         ///< Flag for singular system detection
    std::string error_msg_;         ///< Error message from last solve
    int iterations_ = 0;            ///< Number of iterations (iterative solvers)
    double error_ = 0.0;            ///< Estimated error (iterative solvers)

    // Settings for iterative solvers
    int max_iterations_ = 1000;     ///< Maximum iterations for CG
    double tolerance_ = 1e-10;      ///< Convergence tolerance for CG

    /**
     * @brief Solve using SparseLU
     */
    Eigen::VectorXd solve_sparse_lu(const Eigen::SparseMatrix<double>& K,
                                     const Eigen::VectorXd& F);

    /**
     * @brief Solve using SimplicialLDLT
     */
    Eigen::VectorXd solve_simplicial_ldlt(const Eigen::SparseMatrix<double>& K,
                                           const Eigen::VectorXd& F);

    /**
     * @brief Solve using ConjugateGradient
     */
    Eigen::VectorXd solve_conjugate_gradient(const Eigen::SparseMatrix<double>& K,
                                              const Eigen::VectorXd& F);

    /**
     * @brief Detect if system is singular by checking for near-zero diagonal entries
     * @param K Stiffness matrix
     * @return true if likely singular
     */
    bool check_singularity(const Eigen::SparseMatrix<double>& K) const;
};

} // namespace grillex
