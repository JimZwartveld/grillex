#include "grillex/assembler.hpp"
#include <Eigen/Dense>

namespace grillex {

Assembler::Assembler(DOFHandler& dof_handler)
    : dof_handler_(dof_handler) {}

Eigen::SparseMatrix<double> Assembler::assemble_stiffness(
    const std::vector<BeamElement*>& elements) const {

    int total_dofs = dof_handler_.total_dofs();
    Eigen::SparseMatrix<double> K_global(total_dofs, total_dofs);

    // Use triplet list for efficient sparse matrix assembly
    std::vector<Eigen::Triplet<double>> triplets;

    // Reserve space to reduce reallocations
    // Estimate: each element contributes ~144 entries (12×12)
    // or ~196 entries (14×14), but sparse so use conservative estimate
    triplets.reserve(elements.size() * 150);

    // Loop over all elements and add their contributions
    for (const auto* elem : elements) {
        // Determine element size (12 or 14 DOFs)
        int n_dofs = elem->num_dofs();

        // Get global stiffness matrix for this element
        Eigen::MatrixXd K_elem;
        if (n_dofs == 12) {
            K_elem = elem->global_stiffness_matrix();
        } else if (n_dofs == 14) {
            K_elem = elem->global_stiffness_matrix_warping();
        } else {
            throw std::runtime_error("Unsupported element DOF count: " +
                                     std::to_string(n_dofs));
        }

        // Get location array (maps element DOFs to global DOFs)
        std::vector<int> loc_array = dof_handler_.get_location_array(*elem);

        // Add element matrix to global triplet list
        add_element_matrix(triplets, K_elem, loc_array);
    }

    // Build sparse matrix from triplets
    K_global.setFromTriplets(triplets.begin(), triplets.end());

    return K_global;
}

Eigen::SparseMatrix<double> Assembler::assemble_mass(
    const std::vector<BeamElement*>& elements) const {

    int total_dofs = dof_handler_.total_dofs();
    Eigen::SparseMatrix<double> M_global(total_dofs, total_dofs);

    // Use triplet list for efficient sparse matrix assembly
    std::vector<Eigen::Triplet<double>> triplets;

    // Reserve space to reduce reallocations
    triplets.reserve(elements.size() * 150);

    // Loop over all elements and add their contributions
    for (const auto* elem : elements) {
        // Determine element size (12 or 14 DOFs)
        int n_dofs = elem->num_dofs();

        // Get global mass matrix for this element
        Eigen::MatrixXd M_elem;
        if (n_dofs == 12) {
            M_elem = elem->global_mass_matrix();
        } else if (n_dofs == 14) {
            M_elem = elem->global_mass_matrix_warping();
        } else {
            throw std::runtime_error("Unsupported element DOF count: " +
                                     std::to_string(n_dofs));
        }

        // Get location array (maps element DOFs to global DOFs)
        std::vector<int> loc_array = dof_handler_.get_location_array(*elem);

        // Add element matrix to global triplet list
        add_element_matrix(triplets, M_elem, loc_array);
    }

    // Build sparse matrix from triplets
    M_global.setFromTriplets(triplets.begin(), triplets.end());

    return M_global;
}

void Assembler::add_element_matrix(
    std::vector<Eigen::Triplet<double>>& triplets,
    const Eigen::MatrixXd& element_matrix,
    const std::vector<int>& loc_array) const {

    int n_elem_dofs = element_matrix.rows();

    // Add all non-zero entries to triplet list
    for (int i = 0; i < n_elem_dofs; ++i) {
        int global_i = loc_array[i];

        // Skip if DOF is inactive (global index = -1)
        if (global_i < 0) continue;

        for (int j = 0; j < n_elem_dofs; ++j) {
            int global_j = loc_array[j];

            // Skip if DOF is inactive
            if (global_j < 0) continue;

            double value = element_matrix(i, j);

            // Add to triplet list (even if zero - setFromTriplets handles duplicates)
            // Note: setFromTriplets sums duplicate entries automatically
            triplets.emplace_back(global_i, global_j, value);
        }
    }
}

} // namespace grillex
