#pragma once

#include <Eigen/Sparse>
#include <vector>
#include "grillex/dof_handler.hpp"
#include "grillex/beam_element.hpp"
#include "grillex/point_mass.hpp"

namespace grillex {

/**
 * @brief Assembles global stiffness and mass matrices from element matrices
 *
 * Handles both 12-DOF and 14-DOF elements automatically using sparse matrix format.
 * Uses triplet list for efficient assembly of large sparse systems.
 */
class Assembler {
public:
    /**
     * @brief Construct an Assembler with a DOF handler
     * @param dof_handler Reference to DOFHandler containing global DOF numbering
     */
    explicit Assembler(DOFHandler& dof_handler);

    /**
     * @brief Assemble global stiffness matrix from element stiffness matrices
     * @param elements Vector of beam elements (can be mixed 12-DOF and 14-DOF)
     * @return Sparse global stiffness matrix (total_dofs × total_dofs)
     *
     * The assembled matrix is:
     * - Sparse (only non-zero entries stored)
     * - Symmetric (K_ij = K_ji)
     * - Positive semi-definite (with 6 zero eigenvalues for rigid body modes)
     */
    Eigen::SparseMatrix<double> assemble_stiffness(
        const std::vector<BeamElement*>& elements) const;

    /**
     * @brief Assemble global mass matrix from element mass matrices
     * @param elements Vector of beam elements (can be mixed 12-DOF and 14-DOF)
     * @return Sparse global mass matrix (total_dofs × total_dofs)
     *
     * The assembled matrix is:
     * - Sparse (only non-zero entries stored)
     * - Symmetric (M_ij = M_ji)
     * - Positive semi-definite
     */
    Eigen::SparseMatrix<double> assemble_mass(
        const std::vector<BeamElement*>& elements) const;

    /**
     * @brief Assemble global mass matrix including point masses
     * @param beam_elements Vector of beam elements
     * @param point_masses Vector of point mass elements
     * @return Sparse global mass matrix (total_dofs × total_dofs)
     *
     * Assembles mass contributions from both beam elements and point masses.
     * Point mass matrices (6×6) are assembled at the node's global DOFs.
     *
     * The assembled matrix is:
     * - Sparse (only non-zero entries stored)
     * - Symmetric (M_ij = M_ji)
     * - Positive semi-definite
     */
    Eigen::SparseMatrix<double> assemble_mass(
        const std::vector<BeamElement*>& beam_elements,
        const std::vector<PointMass*>& point_masses) const;

    /**
     * @brief Compute total translational mass from point masses and beam elements
     * @param beam_elements Vector of beam elements
     * @param point_masses Vector of point mass elements
     * @return Total mass [mT] (sum of all translational masses)
     *
     * Useful for computing effective modal mass percentages.
     */
    double compute_total_mass(
        const std::vector<BeamElement*>& beam_elements,
        const std::vector<PointMass*>& point_masses) const;

    /**
     * @brief Get the DOF handler used by this assembler
     * @return Reference to DOFHandler
     */
    const DOFHandler& get_dof_handler() const { return dof_handler_; }

private:
    /// Reference to DOF handler for global DOF numbering
    DOFHandler& dof_handler_;

    /**
     * @brief Add element matrix to triplet list
     * @param triplets Triplet list to append to
     * @param element_matrix Element stiffness or mass matrix (12×12 or 14×14)
     * @param loc_array Location array mapping element DOFs to global DOFs
     *
     * For each non-zero entry K_e(i,j), adds triplet (loc[i], loc[j], K_e(i,j))
     * Skips entries where either DOF is inactive (loc[i] < 0 or loc[j] < 0)
     */
    void add_element_matrix(
        std::vector<Eigen::Triplet<double>>& triplets,
        const Eigen::MatrixXd& element_matrix,
        const std::vector<int>& loc_array) const;
};

} // namespace grillex
