#include "grillex/boundary_condition.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace grillex {

void BCHandler::add_fixed_dof(int node_id, int local_dof, double value) {
    // Validate local_dof index
    if (local_dof < 0 || local_dof > 6) {
        throw std::invalid_argument("local_dof must be in range [0, 6]");
    }

    // Check if this DOF is already fixed
    for (const auto& fixed : fixed_dofs_) {
        if (fixed.node_id == node_id && fixed.local_dof == local_dof &&
            fixed.element_id == -1) {
            // DOF already fixed - could update value or throw error
            // For now, we'll just skip to avoid duplicates
            return;
        }
    }

    fixed_dofs_.emplace_back(node_id, local_dof, value);
}

void BCHandler::add_fixed_warping_dof(int element_id, int node_id, double value) {
    // Check if this warping DOF is already fixed for this element
    for (const auto& fixed : fixed_dofs_) {
        if (fixed.node_id == node_id && fixed.local_dof == WARP &&
            fixed.element_id == element_id) {
            return;
        }
    }

    fixed_dofs_.emplace_back(node_id, WARP, element_id, value);
}

void BCHandler::fix_node(int node_id) {
    // Fix all 6 standard DOFs (not warping)
    for (int dof = 0; dof <= 5; ++dof) {
        add_fixed_dof(node_id, dof, 0.0);
    }
}

void BCHandler::fix_node_with_warping(int node_id) {
    // Fix all 7 DOFs including warping (for all elements)
    for (int dof = 0; dof <= 6; ++dof) {
        add_fixed_dof(node_id, dof, 0.0);
    }
}

void BCHandler::fix_node_with_warping(int node_id, int element_id) {
    // Fix all 6 standard DOFs
    for (int dof = 0; dof <= 5; ++dof) {
        add_fixed_dof(node_id, dof, 0.0);
    }
    // Fix warping for specific element
    add_fixed_warping_dof(element_id, node_id, 0.0);
}

void BCHandler::pin_node(int node_id) {
    // Fix only translations (UX, UY, UZ)
    // Leave rotations (RX, RY, RZ) and warping (WARP) free
    add_fixed_dof(node_id, UX, 0.0);
    add_fixed_dof(node_id, UY, 0.0);
    add_fixed_dof(node_id, UZ, 0.0);
}

void BCHandler::fork_support(int node_id) {
    // Fork support: fix translations only
    // Leave rotations and warping free
    add_fixed_dof(node_id, UX, 0.0);
    add_fixed_dof(node_id, UY, 0.0);
    add_fixed_dof(node_id, UZ, 0.0);
}

std::pair<Eigen::SparseMatrix<double>, Eigen::VectorXd> BCHandler::apply_to_system(
    const Eigen::SparseMatrix<double>& K,
    const Eigen::VectorXd& F,
    const DOFHandler& dof_handler) const {

    // Penalty method implementation
    // For each fixed DOF, we modify K and F to enforce the constraint

    // Create copies for modification
    Eigen::VectorXd F_modified = F;

    // First, convert sparse matrix to triplet format for modification
    int n = K.rows();
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(K.nonZeros() + fixed_dofs_.size());

    // Copy existing non-zero entries to triplet list
    for (int k = 0; k < K.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(K, k); it; ++it) {
            triplets.push_back(Eigen::Triplet<double>(it.row(), it.col(), it.value()));
        }
    }

    // Apply penalty method for each fixed DOF
    for (const auto& fixed : fixed_dofs_) {
        int global_dof = -1;

        // For warping DOF with element-specific fixity
        if (fixed.local_dof == WARP && fixed.element_id >= 0) {
            global_dof = dof_handler.get_warping_dof(fixed.element_id, fixed.node_id);
        }
        // For standard DOFs or node-level warping
        else {
            global_dof = dof_handler.get_global_dof(fixed.node_id, fixed.local_dof);
        }

        if (global_dof < 0) {
            // DOF is not active - skip
            continue;
        }

        // Get current diagonal value
        double K_ii = K.coeff(global_dof, global_dof);

        // Compute penalty factor
        // Use large penalty relative to stiffness magnitude
        double penalty = 1.0e15 * std::max(std::abs(K_ii), 1.0);

        // Add penalty to diagonal
        triplets.push_back(Eigen::Triplet<double>(global_dof, global_dof, penalty));

        // Modify force vector: F(i) = penalty * prescribed_value
        F_modified(global_dof) = penalty * fixed.value;
    }

    // Build sparse matrix from triplets (will sum duplicate entries)
    Eigen::SparseMatrix<double> K_modified(n, n);
    K_modified.setFromTriplets(triplets.begin(), triplets.end());

    return std::make_pair(K_modified, F_modified);
}

std::vector<int> BCHandler::get_fixed_global_dofs(const DOFHandler& dof_handler) const {
    std::vector<int> global_dofs;
    global_dofs.reserve(fixed_dofs_.size());

    for (const auto& fixed : fixed_dofs_) {
        int global_dof = -1;

        // For warping DOF with element-specific fixity
        if (fixed.local_dof == WARP && fixed.element_id >= 0) {
            global_dof = dof_handler.get_warping_dof(fixed.element_id, fixed.node_id);
        }
        // For standard DOFs or node-level warping
        else {
            global_dof = dof_handler.get_global_dof(fixed.node_id, fixed.local_dof);
        }

        if (global_dof >= 0) {
            global_dofs.push_back(global_dof);
        }
    }

    // Sort for convenience
    std::sort(global_dofs.begin(), global_dofs.end());

    return global_dofs;
}

}  // namespace grillex
