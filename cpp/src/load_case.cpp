#include "grillex/load_case.hpp"
#include "grillex/model.hpp"
#include "grillex/dof_handler.hpp"
#include "grillex/beam_element.hpp"

namespace grillex {

/**
 * @brief Compute acceleration at a point given an acceleration field
 *
 * For quasi-static analysis with rigid body kinematics:
 *   a(P) = a(ref) + α × r
 * where r = P - ref_point
 *
 * The full formula including centrifugal effects would be:
 *   a(P) = a(ref) + α × r + ω × (ω × r)
 * but for quasi-static analysis we ignore centrifugal terms.
 *
 * @param point Position where acceleration is computed [m]
 * @param accel 6-component acceleration field [ax, ay, az, αx, αy, αz]
 *              Linear accelerations in [m/s²], angular in [rad/s²]
 * @param ref_point Reference point for the acceleration field [m]
 * @return 6-component acceleration at the point [ax, ay, az, αx, αy, αz]
 *         The angular acceleration is the same everywhere in a rigid body
 */
static Eigen::Vector<double, 6> compute_acceleration_at_point(
    const Eigen::Vector3d& point,
    const Eigen::Vector<double, 6>& accel,
    const Eigen::Vector3d& ref_point)
{
    // Extract linear and angular accelerations
    Eigen::Vector3d a_linear = accel.head<3>();   // [ax, ay, az]
    Eigen::Vector3d alpha = accel.tail<3>();       // [αx, αy, αz]

    // Position vector from reference point to current point
    Eigen::Vector3d r = point - ref_point;

    // Compute tangential acceleration: α × r
    Eigen::Vector3d a_tangential = alpha.cross(r);

    // Total linear acceleration at point
    Eigen::Vector3d a_at_point = a_linear + a_tangential;

    // Build result: [ax, ay, az, αx, αy, αz]
    // Angular acceleration is the same everywhere in a rigid body
    Eigen::Vector<double, 6> result;
    result.head<3>() = a_at_point;
    result.tail<3>() = alpha;

    return result;
}

LoadCase::LoadCase(int id, const std::string& name, LoadCaseType type)
    : id_(id), name_(name), type_(type)
{
    // Initialize acceleration field to zero
    acceleration_.setZero();
    acceleration_ref_point_.setZero();
}

void LoadCase::add_nodal_load(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& force,
                              const Eigen::Vector3d& moment) {
    // Search for existing load at this position (within tolerance)
    const double tol = 1e-6;
    for (auto& load : nodal_loads_) {
        if ((load.position - position).norm() < tol) {
            // Accumulate forces and moments
            load.force += force;
            load.moment += moment;
            return;
        }
    }

    // New load
    nodal_loads_.emplace_back(position, force, moment);
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
        // Find node at load position
        Node* node = model.nodes.find_node(load.position.x(), load.position.y(), load.position.z());
        if (!node) {
            continue;  // Skip if node not found (shouldn't happen in well-formed model)
        }

        // Apply force components (DOFs 0, 1, 2 = UX, UY, UZ)
        for (int i = 0; i < 3; i++) {
            if (std::abs(load.force(i)) > 1e-10) {
                int global_dof = dof_handler.get_global_dof(node->id, i);
                if (global_dof >= 0 && global_dof < total_dofs) {
                    F(global_dof) += load.force(i);
                }
            }
        }

        // Apply moment components (DOFs 3, 4, 5 = RX, RY, RZ)
        for (int i = 0; i < 3; i++) {
            if (std::abs(load.moment(i)) > 1e-10) {
                int global_dof = dof_handler.get_global_dof(node->id, i + 3);
                if (global_dof >= 0 && global_dof < total_dofs) {
                    F(global_dof) += load.moment(i);
                }
            }
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

    // 3. Inertial loads from acceleration field
    // For quasi-static analysis: f_inertial = -M * a
    // where M is element mass matrix, a is nodal accelerations
    if (acceleration_.norm() > 1e-10) {
        for (const auto& elem_ptr : model.elements) {
            BeamElement* elem = elem_ptr.get();
            if (!elem) continue;

            // Get node positions (accounting for offsets if present)
            Eigen::Vector3d pos_i = elem->node_i->position();
            Eigen::Vector3d pos_j = elem->node_j->position();

            // For elements with offsets, compute acceleration at beam ends
            // The beam ends are offset from the nodes
            if (elem->has_offsets()) {
                Eigen::Vector3d offset_i_global = elem->local_axes.to_global(elem->offset_i);
                Eigen::Vector3d offset_j_global = elem->local_axes.to_global(elem->offset_j);
                pos_i = pos_i + offset_i_global;
                pos_j = pos_j + offset_j_global;
            }

            // Compute acceleration at element nodes
            Eigen::Vector<double, 6> a_i = compute_acceleration_at_point(
                pos_i, acceleration_, acceleration_ref_point_);
            Eigen::Vector<double, 6> a_j = compute_acceleration_at_point(
                pos_j, acceleration_, acceleration_ref_point_);

            // Stack into element acceleration vector (12x1)
            Eigen::Vector<double, 12> a_elem;
            a_elem.head<6>() = a_i;
            a_elem.tail<6>() = a_j;

            // Get element global mass matrix
            Eigen::Matrix<double, 12, 12> M = elem->global_mass_matrix();

            // Body forces from acceleration field: f = M * a
            // For quasi-static analysis, this represents the equivalent nodal forces
            // from body forces due to the acceleration field (e.g., gravity)
            // Note: For gravity (a = [0,0,-g]), this gives downward forces as expected
            Eigen::Vector<double, 12> f_inertial = M * a_elem;

            // Get location array mapping local DOFs to global DOFs
            std::vector<int> location = dof_handler.get_location_array(*elem);

            // Add inertial forces to global load vector
            for (int i = 0; i < 12; i++) {
                int global_dof = location[i];
                if (global_dof >= 0 && global_dof < total_dofs) {
                    F(global_dof) += f_inertial(i);
                }
            }
        }
    }

    return F;
}

// =============================================================================
// LoadCombination Implementation
// =============================================================================

LoadCombination::LoadCombination(int id, const std::string& name,
                                 double permanent_factor,
                                 double variable_factor,
                                 double environmental_factor,
                                 double accidental_factor)
    : id_(id), name_(name),
      permanent_factor_(permanent_factor),
      variable_factor_(variable_factor),
      environmental_factor_(environmental_factor),
      accidental_factor_(accidental_factor)
{
}

double LoadCombination::get_type_factor(LoadCaseType type) const {
    switch (type) {
        case LoadCaseType::Permanent:
            return permanent_factor_;
        case LoadCaseType::Variable:
            return variable_factor_;
        case LoadCaseType::Environmental:
            return environmental_factor_;
        case LoadCaseType::Accidental:
            return accidental_factor_;
        default:
            return 1.0;
    }
}

void LoadCombination::set_type_factor(LoadCaseType type, double factor) {
    switch (type) {
        case LoadCaseType::Permanent:
            permanent_factor_ = factor;
            break;
        case LoadCaseType::Variable:
            variable_factor_ = factor;
            break;
        case LoadCaseType::Environmental:
            environmental_factor_ = factor;
            break;
        case LoadCaseType::Accidental:
            accidental_factor_ = factor;
            break;
    }
}

void LoadCombination::add_load_case(LoadCase* load_case) {
    if (!load_case) return;

    // Check if already added
    for (const auto& term : terms_) {
        if (term.load_case == load_case) {
            return;  // Already exists, don't add duplicate
        }
    }

    // Get factor based on load case type
    double factor = get_type_factor(load_case->type());
    terms_.emplace_back(load_case, factor, false);  // Not explicit
}

void LoadCombination::add_load_case(LoadCase* load_case, double factor) {
    if (!load_case) return;

    // Check if already added - if so, update the factor
    for (auto& term : terms_) {
        if (term.load_case == load_case) {
            term.factor = factor;
            term.explicit_factor = true;
            return;
        }
    }

    // New load case with explicit factor
    terms_.emplace_back(load_case, factor, true);
}

bool LoadCombination::remove_load_case(LoadCase* load_case) {
    for (auto it = terms_.begin(); it != terms_.end(); ++it) {
        if (it->load_case == load_case) {
            terms_.erase(it);
            return true;
        }
    }
    return false;
}

void LoadCombination::clear() {
    terms_.clear();
}

Eigen::VectorXd LoadCombination::get_combined_displacements(
    const std::map<int, LoadCaseResult>& results) const
{
    if (terms_.empty()) {
        return Eigen::VectorXd();
    }

    // Find vector size from first available result
    int size = 0;
    for (const auto& term : terms_) {
        auto it = results.find(term.load_case->id());
        if (it != results.end() && it->second.success) {
            size = static_cast<int>(it->second.displacements.size());
            break;
        }
    }

    if (size == 0) {
        throw std::runtime_error("No valid load case results found for combination");
    }

    Eigen::VectorXd combined = Eigen::VectorXd::Zero(size);

    for (const auto& term : terms_) {
        auto it = results.find(term.load_case->id());
        if (it == results.end()) {
            throw std::runtime_error("Missing result for load case '" +
                                   term.load_case->name() + "' (ID: " +
                                   std::to_string(term.load_case->id()) + ")");
        }

        if (!it->second.success) {
            throw std::runtime_error("Load case '" + term.load_case->name() +
                                   "' analysis failed: " + it->second.error_message);
        }

        if (it->second.displacements.size() != size) {
            throw std::runtime_error("Inconsistent displacement vector size for load case '" +
                                   term.load_case->name() + "'");
        }

        combined += term.factor * it->second.displacements;
    }

    return combined;
}

Eigen::VectorXd LoadCombination::get_combined_reactions(
    const std::map<int, LoadCaseResult>& results) const
{
    if (terms_.empty()) {
        return Eigen::VectorXd();
    }

    // Find vector size from first available result
    int size = 0;
    for (const auto& term : terms_) {
        auto it = results.find(term.load_case->id());
        if (it != results.end() && it->second.success) {
            size = static_cast<int>(it->second.reactions.size());
            break;
        }
    }

    if (size == 0) {
        throw std::runtime_error("No valid load case results found for combination");
    }

    Eigen::VectorXd combined = Eigen::VectorXd::Zero(size);

    for (const auto& term : terms_) {
        auto it = results.find(term.load_case->id());
        if (it == results.end()) {
            throw std::runtime_error("Missing result for load case '" +
                                   term.load_case->name() + "' (ID: " +
                                   std::to_string(term.load_case->id()) + ")");
        }

        if (!it->second.success) {
            throw std::runtime_error("Load case '" + term.load_case->name() +
                                   "' analysis failed: " + it->second.error_message);
        }

        if (it->second.reactions.size() != size) {
            throw std::runtime_error("Inconsistent reaction vector size for load case '" +
                                   term.load_case->name() + "'");
        }

        combined += term.factor * it->second.reactions;
    }

    return combined;
}

} // namespace grillex
