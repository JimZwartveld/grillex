#include "grillex/load_case.hpp"
#include "grillex/model.hpp"
#include "grillex/dof_handler.hpp"
#include "grillex/beam_element.hpp"

namespace grillex {

LoadCase::LoadCase(int id, const std::string& name, LoadCaseType type)
    : id_(id), name_(name), type_(type)
{
    // Initialize acceleration field to zero
    acceleration_.setZero();
    acceleration_ref_point_.setZero();
}

void LoadCase::add_nodal_load(int node_id, int local_dof, double value) {
    // Search for existing load on this node/DOF
    for (auto& load : nodal_loads_) {
        if (load.node_id == node_id && load.local_dof == local_dof) {
            // Accumulate
            load.value += value;
            return;
        }
    }

    // New load
    nodal_loads_.emplace_back(node_id, local_dof, value);
}

void LoadCase::add_line_load(int element_id,
                             const Eigen::Vector3d& w_start,
                             const Eigen::Vector3d& w_end) {
    line_loads_.emplace_back(element_id, w_start, w_end);
}

void LoadCase::set_acceleration_field(const Eigen::Vector<double, 6>& accel,
                                      const Eigen::Vector3d& ref_point) {
    acceleration_ = accel;
    acceleration_ref_point_ = ref_point;
}

void LoadCase::clear() {
    nodal_loads_.clear();
    line_loads_.clear();
    acceleration_.setZero();
    acceleration_ref_point_.setZero();
}

bool LoadCase::is_empty() const {
    return nodal_loads_.empty() &&
           line_loads_.empty() &&
           acceleration_.norm() < 1e-10;
}

Eigen::VectorXd LoadCase::assemble_load_vector(
    const Model& model,
    const DOFHandler& dof_handler) const
{
    int total_dofs = dof_handler.total_dofs();
    Eigen::VectorXd F = Eigen::VectorXd::Zero(total_dofs);

    // 1. Direct nodal loads
    for (const auto& load : nodal_loads_) {
        int global_dof = dof_handler.get_global_dof(load.node_id, load.local_dof);
        if (global_dof >= 0 && global_dof < total_dofs) {
            F(global_dof) += load.value;
        }
    }

    // 2. Equivalent nodal forces from line loads
    for (const auto& line_load : line_loads_) {
        // Get element
        BeamElement* elem = model.get_element(line_load.element_id);
        if (!elem) {
            continue;  // Skip invalid element IDs
        }

        // Compute equivalent nodal forces in global coordinates
        Eigen::Matrix<double, 12, 1> f_equiv = elem->equivalent_nodal_forces(
            line_load.w_start, line_load.w_end);

        // Get location array mapping local DOFs to global DOFs
        std::vector<int> location = dof_handler.get_location_array(*elem);

        // Add equivalent nodal forces to global load vector
        for (int i = 0; i < 12; i++) {
            int global_dof = location[i];
            if (global_dof >= 0 && global_dof < total_dofs) {
                F(global_dof) += f_equiv(i);
            }
        }
    }

    // 3. Inertial loads from acceleration field (Phase 5 Task 5.3)
    // TODO: Implement when element mass matrices are available
    // if (acceleration_.norm() > 1e-10) {
    //     // For each element, compute inertial forces
    //     // f_inertial = -M * a
    //     // where M is element mass matrix, a is nodal accelerations
    //     // ... (implementation in Phase 5)
    // }

    return F;
}

} // namespace grillex
