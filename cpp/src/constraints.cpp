#include "grillex/constraints.hpp"
#include <stdexcept>
#include <algorithm>

namespace grillex {

void ConstraintHandler::add_equality_constraint(int slave_node, int slave_dof,
                                                 int master_node, int master_dof) {
    // Validate DOF indices
    if (slave_dof < 0 || slave_dof > 6 || master_dof < 0 || master_dof > 6) {
        throw std::invalid_argument("Local DOF must be in range [0, 6]");
    }

    // Check that we're not constraining a DOF to itself
    if (slave_node == master_node && slave_dof == master_dof) {
        throw std::invalid_argument("Cannot constrain a DOF to itself");
    }

    equalities_.emplace_back(slave_node, slave_dof, master_node, master_dof);
}

void ConstraintHandler::add_rigid_link(int slave_node, int master_node,
                                        const Eigen::Vector3d& offset) {
    // Check that slave and master are different
    if (slave_node == master_node) {
        throw std::invalid_argument("Rigid link slave and master must be different nodes");
    }

    rigid_links_.emplace_back(slave_node, master_node, offset);
}

std::set<int> ConstraintHandler::get_slave_dofs(const DOFHandler& dof_handler) const {
    std::set<int> slaves;

    // Equality constraints: slave DOF is dependent
    for (const auto& eq : equalities_) {
        int slave_global = dof_handler.get_global_dof(eq.slave_node_id, eq.slave_local_dof);
        if (slave_global >= 0) {
            slaves.insert(slave_global);
        }
    }

    // Rigid links: all 6 DOFs of slave node are dependent
    for (const auto& rl : rigid_links_) {
        for (int dof = 0; dof < 6; ++dof) {
            int slave_global = dof_handler.get_global_dof(rl.slave_node_id, dof);
            if (slave_global >= 0) {
                slaves.insert(slave_global);
            }
        }
    }

    return slaves;
}

std::map<int, std::vector<std::pair<int, double>>> ConstraintHandler::build_constraint_map(
    const DOFHandler& dof_handler) const {

    std::map<int, std::vector<std::pair<int, double>>> constraint_map;

    // Process equality constraints
    for (const auto& eq : equalities_) {
        int slave_global = dof_handler.get_global_dof(eq.slave_node_id, eq.slave_local_dof);
        int master_global = dof_handler.get_global_dof(eq.master_node_id, eq.master_local_dof);

        if (slave_global >= 0 && master_global >= 0) {
            // slave = 1.0 * master
            constraint_map[slave_global].push_back({master_global, 1.0});
        }
    }

    // Process rigid links
    // Rigid link kinematics:
    // u_sx = u_mx + 0*θmx - rz*θmy + ry*θmz
    // u_sy = u_my + rz*θmx + 0*θmy - rx*θmz
    // u_sz = u_mz - ry*θmx + rx*θmy + 0*θmz
    // θ_sx = θ_mx
    // θ_sy = θ_my
    // θ_sz = θ_mz

    for (const auto& rl : rigid_links_) {
        // Get slave global DOFs
        int s_ux = dof_handler.get_global_dof(rl.slave_node_id, 0);  // UX
        int s_uy = dof_handler.get_global_dof(rl.slave_node_id, 1);  // UY
        int s_uz = dof_handler.get_global_dof(rl.slave_node_id, 2);  // UZ
        int s_rx = dof_handler.get_global_dof(rl.slave_node_id, 3);  // RX
        int s_ry = dof_handler.get_global_dof(rl.slave_node_id, 4);  // RY
        int s_rz = dof_handler.get_global_dof(rl.slave_node_id, 5);  // RZ

        // Get master global DOFs
        int m_ux = dof_handler.get_global_dof(rl.master_node_id, 0);  // UX
        int m_uy = dof_handler.get_global_dof(rl.master_node_id, 1);  // UY
        int m_uz = dof_handler.get_global_dof(rl.master_node_id, 2);  // UZ
        int m_rx = dof_handler.get_global_dof(rl.master_node_id, 3);  // RX
        int m_ry = dof_handler.get_global_dof(rl.master_node_id, 4);  // RY
        int m_rz = dof_handler.get_global_dof(rl.master_node_id, 5);  // RZ

        double rx = rl.offset.x();
        double ry = rl.offset.y();
        double rz = rl.offset.z();

        // Slave UX = Master UX - rz*θmy + ry*θmz
        if (s_ux >= 0) {
            if (m_ux >= 0) constraint_map[s_ux].push_back({m_ux, 1.0});
            if (m_ry >= 0) constraint_map[s_ux].push_back({m_ry, -rz});
            if (m_rz >= 0) constraint_map[s_ux].push_back({m_rz, ry});
        }

        // Slave UY = Master UY + rz*θmx - rx*θmz
        if (s_uy >= 0) {
            if (m_uy >= 0) constraint_map[s_uy].push_back({m_uy, 1.0});
            if (m_rx >= 0) constraint_map[s_uy].push_back({m_rx, rz});
            if (m_rz >= 0) constraint_map[s_uy].push_back({m_rz, -rx});
        }

        // Slave UZ = Master UZ - ry*θmx + rx*θmy
        if (s_uz >= 0) {
            if (m_uz >= 0) constraint_map[s_uz].push_back({m_uz, 1.0});
            if (m_rx >= 0) constraint_map[s_uz].push_back({m_rx, -ry});
            if (m_ry >= 0) constraint_map[s_uz].push_back({m_ry, rx});
        }

        // Slave rotations equal master rotations
        if (s_rx >= 0 && m_rx >= 0) {
            constraint_map[s_rx].push_back({m_rx, 1.0});
        }
        if (s_ry >= 0 && m_ry >= 0) {
            constraint_map[s_ry].push_back({m_ry, 1.0});
        }
        if (s_rz >= 0 && m_rz >= 0) {
            constraint_map[s_rz].push_back({m_rz, 1.0});
        }
    }

    return constraint_map;
}

int ConstraintHandler::num_slave_dofs(const DOFHandler& dof_handler) const {
    return static_cast<int>(get_slave_dofs(dof_handler).size());
}

Eigen::SparseMatrix<double> ConstraintHandler::build_transformation_matrix(
    const DOFHandler& dof_handler) const {

    int n_full = dof_handler.total_dofs();

    // Get slave DOFs
    std::set<int> slave_dofs = get_slave_dofs(dof_handler);
    int n_slaves = static_cast<int>(slave_dofs.size());
    int n_reduced = n_full - n_slaves;

    if (n_reduced <= 0) {
        throw std::runtime_error("All DOFs are constrained - no independent DOFs remain");
    }

    // Build mapping from full DOF index to reduced DOF index
    // Only independent (non-slave) DOFs get a reduced index
    std::map<int, int> full_to_reduced;
    int reduced_idx = 0;
    for (int full_idx = 0; full_idx < n_full; ++full_idx) {
        if (slave_dofs.find(full_idx) == slave_dofs.end()) {
            // This is an independent DOF
            full_to_reduced[full_idx] = reduced_idx++;
        }
    }

    // Build constraint map (slave DOF -> list of (master DOF, coefficient))
    auto constraint_map = build_constraint_map(dof_handler);

    // Build transformation matrix T (n_full x n_reduced)
    // u_full = T * u_reduced
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(n_full + slave_dofs.size() * 6);  // Conservative estimate

    for (int full_idx = 0; full_idx < n_full; ++full_idx) {
        if (slave_dofs.find(full_idx) == slave_dofs.end()) {
            // Independent DOF: T[full, reduced] = 1.0
            int reduced_col = full_to_reduced[full_idx];
            triplets.emplace_back(full_idx, reduced_col, 1.0);
        } else {
            // Slave DOF: look up its constraint equation
            auto it = constraint_map.find(full_idx);
            if (it != constraint_map.end()) {
                for (const auto& [master_full, coeff] : it->second) {
                    // Only add if master is independent
                    auto master_it = full_to_reduced.find(master_full);
                    if (master_it != full_to_reduced.end()) {
                        triplets.emplace_back(full_idx, master_it->second, coeff);
                    } else {
                        // Master is also a slave - need to handle chained constraints
                        // For now, throw an error as this requires more complex handling
                        throw std::runtime_error(
                            "Chained constraints not yet supported: DOF " +
                            std::to_string(master_full) + " is both master and slave");
                    }
                }
            }
        }
    }

    Eigen::SparseMatrix<double> T(n_full, n_reduced);
    T.setFromTriplets(triplets.begin(), triplets.end());

    return T;
}

ConstraintHandler::ReducedSystem ConstraintHandler::reduce_system(
    const Eigen::SparseMatrix<double>& K,
    const Eigen::VectorXd& F,
    const DOFHandler& dof_handler) const {

    ReducedSystem result;
    result.n_full = dof_handler.total_dofs();

    // Validate dimensions
    if (K.rows() != result.n_full || K.cols() != result.n_full) {
        throw std::invalid_argument("K matrix dimensions don't match DOF count");
    }
    if (F.size() != result.n_full) {
        throw std::invalid_argument("F vector size doesn't match DOF count");
    }

    // If no constraints, return identity transformation
    if (!has_constraints()) {
        result.T.resize(result.n_full, result.n_full);
        result.T.setIdentity();
        result.K_reduced = K;
        result.F_reduced = F;
        result.n_reduced = result.n_full;
        return result;
    }

    // Build transformation matrix
    result.T = build_transformation_matrix(dof_handler);
    result.n_reduced = static_cast<int>(result.T.cols());

    // Compute reduced system
    // K_reduced = T^T * K * T
    // F_reduced = T^T * F
    Eigen::SparseMatrix<double> Tt = result.T.transpose();
    result.K_reduced = Tt * K * result.T;
    result.F_reduced = Tt * F;

    return result;
}

Eigen::VectorXd ConstraintHandler::expand_displacements(
    const Eigen::VectorXd& u_reduced,
    const Eigen::SparseMatrix<double>& T) const {

    // u_full = T * u_reduced
    if (u_reduced.size() != T.cols()) {
        throw std::invalid_argument(
            "u_reduced size (" + std::to_string(u_reduced.size()) +
            ") doesn't match T columns (" + std::to_string(T.cols()) + ")");
    }

    return T * u_reduced;
}

}  // namespace grillex
