#include "grillex/assembler.hpp"
#include <Eigen/Dense>
#include <cmath>

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

Eigen::SparseMatrix<double> Assembler::assemble_mass(
    const std::vector<BeamElement*>& beam_elements,
    const std::vector<PointMass*>& point_masses) const {

    int total_dofs = dof_handler_.total_dofs();
    Eigen::SparseMatrix<double> M_global(total_dofs, total_dofs);

    // Use triplet list for efficient sparse matrix assembly
    std::vector<Eigen::Triplet<double>> triplets;

    // Reserve space for beam elements and point masses
    triplets.reserve(beam_elements.size() * 150 + point_masses.size() * 36);

    // 1. Assemble beam element mass matrices
    for (const auto* elem : beam_elements) {
        int n_dofs = elem->num_dofs();

        Eigen::MatrixXd M_elem;
        if (n_dofs == 12) {
            M_elem = elem->global_mass_matrix();
        } else if (n_dofs == 14) {
            M_elem = elem->global_mass_matrix_warping();
        } else {
            throw std::runtime_error("Unsupported element DOF count: " +
                                     std::to_string(n_dofs));
        }

        std::vector<int> loc_array = dof_handler_.get_location_array(*elem);
        add_element_matrix(triplets, M_elem, loc_array);
    }

    // 2. Assemble point mass matrices
    for (const auto* pm : point_masses) {
        if (!pm || !pm->node) continue;

        // Get the 6×6 mass matrix from the point mass
        Eigen::Matrix<double, 6, 6> M_pm = pm->mass_matrix();

        // Get global DOF indices for this node (6 DOFs: UX, UY, UZ, RX, RY, RZ)
        int node_id = pm->node->id;

        // Build location array for the 6 DOFs of this node
        std::vector<int> loc_array(6);
        for (int i = 0; i < 6; ++i) {
            loc_array[i] = dof_handler_.get_global_dof(node_id, i);
        }

        // Add point mass matrix entries to triplet list
        for (int i = 0; i < 6; ++i) {
            int global_i = loc_array[i];
            if (global_i < 0) continue;

            for (int j = 0; j < 6; ++j) {
                int global_j = loc_array[j];
                if (global_j < 0) continue;

                double value = M_pm(i, j);
                if (std::abs(value) > 1e-14) {
                    triplets.emplace_back(global_i, global_j, value);
                }
            }
        }
    }

    // Build sparse matrix from triplets
    M_global.setFromTriplets(triplets.begin(), triplets.end());

    return M_global;
}

double Assembler::compute_total_mass(
    const std::vector<BeamElement*>& beam_elements,
    const std::vector<PointMass*>& point_masses) const {

    double total_mass = 0.0;

    // Sum mass from beam elements
    for (const auto* elem : beam_elements) {
        if (!elem) continue;

        // Beam mass = ρ * A * L
        double rho = elem->material->rho;
        double A = elem->section->A;
        double L = elem->length;
        total_mass += rho * A * L;
    }

    // Sum mass from point masses
    for (const auto* pm : point_masses) {
        if (!pm) continue;
        total_mass += pm->mass;
    }

    return total_mass;
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
