#include "grillex/dof_handler.hpp"
#include <stdexcept>
#include <algorithm>

namespace grillex {

DOFHandler::DOFHandler()
    : total_dofs_(0)
    , has_warping_(false)
    , collinearity_tolerance_(5.0) {}

void DOFHandler::number_dofs(NodeRegistry& registry) {
    // Clear any existing numbering
    clear();

    int global_dof_counter = 0;

    // Iterate through all nodes in the registry
    for (const auto& node_ptr : registry.all_nodes()) {
        Node* node = node_ptr.get();

        // Number standard DOFs (0-5: UX, UY, UZ, RX, RY, RZ)
        for (int local_dof = 0; local_dof < 6; ++local_dof) {
            if (node->dof_active[local_dof]) {
                dof_map_[{node->id, local_dof}] = global_dof_counter++;
            }
        }

        // Number warping DOF if active (local_dof = 6)
        // In legacy mode, warping DOF is nodal (shared by all elements)
        if (node->has_warping_dof() && node->dof_active[6]) {
            dof_map_[{node->id, 6}] = global_dof_counter++;
            has_warping_ = true;
        }
    }

    total_dofs_ = global_dof_counter;
}

void DOFHandler::number_dofs_with_elements(
    NodeRegistry& registry,
    const std::vector<BeamElement*>& elements,
    double collinearity_tolerance_deg) {

    // Clear any existing numbering
    clear();
    collinearity_tolerance_ = collinearity_tolerance_deg;

    int global_dof_counter = 0;

    // Step 1: Number standard DOFs (0-5) for all nodes
    for (const auto& node_ptr : registry.all_nodes()) {
        Node* node = node_ptr.get();

        // Number standard DOFs (0-5: UX, UY, UZ, RX, RY, RZ)
        for (int local_dof = 0; local_dof < 6; ++local_dof) {
            if (node->dof_active[local_dof]) {
                dof_map_[{node->id, local_dof}] = global_dof_counter++;
            }
        }
    }

    // Step 2: Collect warping elements and build node->elements map
    std::map<int, std::vector<BeamElement*>> node_to_warping_elements;

    for (BeamElement* elem : elements) {
        if (elem->has_warping()) {
            // Check if nodes have warping DOF enabled
            if (elem->node_i->has_warping_dof() && elem->node_i->dof_active[6]) {
                node_to_warping_elements[elem->node_i->id].push_back(elem);
            }
            if (elem->node_j->has_warping_dof() && elem->node_j->dof_active[6]) {
                node_to_warping_elements[elem->node_j->id].push_back(elem);
            }
        }
    }

    if (node_to_warping_elements.empty()) {
        // No warping DOFs needed
        total_dofs_ = global_dof_counter;
        return;
    }

    has_warping_ = true;

    // Step 3: For each node with warping elements, identify collinear groups
    // and assign warping DOFs
    for (auto& [node_id, node_elements] : node_to_warping_elements) {
        // Get collinear groups at this node
        auto collinear_groups = identify_collinear_groups(
            node_id, node_elements, collinearity_tolerance_deg);

        // Assign warping DOFs for each group
        for (auto& group : collinear_groups) {
            if (group.empty()) continue;

            // All elements in a collinear group share the same warping DOF
            int master_dof = global_dof_counter++;

            WarpingCoupling coupling;
            coupling.master_dof = master_dof;

            for (BeamElement* elem : group) {
                // Determine if this is node_i or node_j of the element
                bool is_node_i = (elem->node_i->id == node_id);

                // Assign the warping DOF
                warping_dof_map_[{elem->id, node_id}] = master_dof;

                // Record the coupling info
                WarpingDOFInfo info;
                info.element_id = elem->id;
                info.node_id = node_id;
                info.is_node_i = is_node_i;
                info.global_dof = master_dof;
                coupling.coupled_dofs.push_back(info);
            }

            // Only record coupling if more than one element
            if (coupling.coupled_dofs.size() > 1) {
                warping_couplings_.push_back(coupling);
            }
        }
    }

    total_dofs_ = global_dof_counter;
}

std::vector<std::vector<BeamElement*>> DOFHandler::identify_collinear_groups(
    int node_id,
    const std::vector<BeamElement*>& node_elements,
    double tolerance_deg) const {

    std::vector<std::vector<BeamElement*>> groups;
    std::vector<bool> assigned(node_elements.size(), false);

    // Check for forced couplings
    auto forced_it = forced_couplings_.find(node_id);
    std::set<int> forced_set;
    if (forced_it != forced_couplings_.end()) {
        forced_set = forced_it->second;
    }

    // Check for released couplings
    auto released_it = released_couplings_.find(node_id);
    std::set<std::pair<int,int>> released_pairs;
    if (released_it != released_couplings_.end()) {
        released_pairs = released_it->second;
    }

    // Helper to check if two elements should NOT be coupled (released)
    auto is_released = [&](int elem1_id, int elem2_id) {
        auto p1 = std::make_pair(std::min(elem1_id, elem2_id),
                                 std::max(elem1_id, elem2_id));
        return released_pairs.find(p1) != released_pairs.end();
    };

    // Group elements
    for (size_t i = 0; i < node_elements.size(); ++i) {
        if (assigned[i]) continue;

        // Start a new group with element i
        std::vector<BeamElement*> group;
        group.push_back(node_elements[i]);
        assigned[i] = true;

        // Try to add other elements to this group
        for (size_t j = i + 1; j < node_elements.size(); ++j) {
            if (assigned[j]) continue;

            BeamElement* elem_i = node_elements[i];
            BeamElement* elem_j = node_elements[j];

            // Check if released by user
            if (is_released(elem_i->id, elem_j->id)) {
                continue;
            }

            // Check if forced by user
            bool both_forced = (forced_set.count(elem_i->id) > 0 &&
                               forced_set.count(elem_j->id) > 0);

            // Check collinearity (or forced coupling)
            bool collinear = both_forced ||
                            are_elements_collinear(*elem_i, *elem_j, node_id, tolerance_deg);

            // Also check transitivity: if elem_j is collinear with ANY element
            // already in the group, add it
            if (!collinear) {
                for (BeamElement* group_elem : group) {
                    if (group_elem == elem_i) continue;
                    if (is_released(group_elem->id, elem_j->id)) continue;

                    bool both_forced_with_group = (forced_set.count(group_elem->id) > 0 &&
                                                   forced_set.count(elem_j->id) > 0);
                    if (both_forced_with_group ||
                        are_elements_collinear(*group_elem, *elem_j, node_id, tolerance_deg)) {
                        collinear = true;
                        break;
                    }
                }
            }

            if (collinear) {
                group.push_back(elem_j);
                assigned[j] = true;
            }
        }

        groups.push_back(group);
    }

    return groups;
}

int DOFHandler::total_dofs() const {
    return total_dofs_;
}

int DOFHandler::get_global_dof(int node_id, int local_dof) const {
    auto key = std::make_pair(node_id, local_dof);
    auto it = dof_map_.find(key);

    if (it != dof_map_.end()) {
        return it->second;
    }

    return -1;  // DOF is not active or node doesn't exist
}

int DOFHandler::get_warping_dof(int element_id, int node_id) const {
    auto key = std::make_pair(element_id, node_id);
    auto it = warping_dof_map_.find(key);

    if (it != warping_dof_map_.end()) {
        return it->second;
    }

    // Fall back to nodal warping DOF (legacy mode)
    return get_global_dof(node_id, 6);
}

std::vector<int> DOFHandler::get_location_array(const BeamElementBase& elem) const {
    // Get the concrete beam element to access node pointers
    const BeamElement* beam_elem = dynamic_cast<const BeamElement*>(&elem);
    if (!beam_elem) {
        throw std::runtime_error("Element is not a BeamElement");
    }

    int n_dofs = elem.num_dofs();  // 12 or 14
    std::vector<int> loc(n_dofs);

    if (n_dofs == 12) {
        // Standard 12-DOF element: 6 DOFs per node (all nodal)
        for (int d = 0; d < 6; ++d) {
            loc[d] = get_global_dof(beam_elem->node_i->id, d);
            loc[6 + d] = get_global_dof(beam_elem->node_j->id, d);
        }
    } else {
        // 14-DOF element with warping
        // Standard DOFs (0-5) are nodal
        for (int d = 0; d < 6; ++d) {
            loc[d] = get_global_dof(beam_elem->node_i->id, d);
            loc[7 + d] = get_global_dof(beam_elem->node_j->id, d);
        }

        // Warping DOFs (6, 13) are element-specific
        loc[6] = get_warping_dof(beam_elem->id, beam_elem->node_i->id);
        loc[13] = get_warping_dof(beam_elem->id, beam_elem->node_j->id);
    }

    return loc;
}

bool DOFHandler::has_warping_dofs() const {
    return has_warping_;
}

void DOFHandler::clear() {
    total_dofs_ = 0;
    has_warping_ = false;
    dof_map_.clear();
    warping_dof_map_.clear();
    warping_couplings_.clear();
    // Note: forced_couplings_ and released_couplings_ are NOT cleared
    // as they represent user configuration that should persist
}

void DOFHandler::set_warping_continuous(int node_id, const std::vector<int>& element_ids) {
    for (int elem_id : element_ids) {
        forced_couplings_[node_id].insert(elem_id);
    }
}

void DOFHandler::release_warping_coupling(int node_id, int element1_id, int element2_id) {
    // Store as ordered pair for consistent lookup
    auto pair = std::make_pair(std::min(element1_id, element2_id),
                               std::max(element1_id, element2_id));
    released_couplings_[node_id].insert(pair);
}

const std::vector<WarpingCoupling>& DOFHandler::get_warping_couplings() const {
    return warping_couplings_;
}

// ===== Reverse lookup methods (for singularity diagnostics) =====

bool DOFHandler::is_warping_dof(int global_dof) const {
    // Check warping_dof_map_ for this global DOF
    for (const auto& [key, dof] : warping_dof_map_) {
        if (dof == global_dof) {
            return true;
        }
    }

    // Also check nodal warping DOFs in legacy mode
    for (const auto& [key, dof] : dof_map_) {
        if (dof == global_dof && key.second == 6) {
            return true;
        }
    }

    return false;
}

int DOFHandler::get_node_from_global_dof(int global_dof) const {
    // First check standard DOF map
    for (const auto& [key, dof] : dof_map_) {
        if (dof == global_dof) {
            return key.first;  // key is (node_id, local_dof)
        }
    }

    // Check warping DOF map
    for (const auto& [key, dof] : warping_dof_map_) {
        if (dof == global_dof) {
            return key.second;  // key is (element_id, node_id)
        }
    }

    return -1;  // Not found
}

int DOFHandler::get_local_dof_from_global_dof(int global_dof) const {
    // First check standard DOF map
    for (const auto& [key, dof] : dof_map_) {
        if (dof == global_dof) {
            return key.second;  // key is (node_id, local_dof)
        }
    }

    // Check warping DOF map (warping is always local_dof = 6)
    for (const auto& [key, dof] : warping_dof_map_) {
        if (dof == global_dof) {
            return 6;  // Warping DOF
        }
    }

    return -1;  // Not found
}

int DOFHandler::get_element_from_warping_dof(int global_dof) const {
    // Only check warping DOF map
    for (const auto& [key, dof] : warping_dof_map_) {
        if (dof == global_dof) {
            return key.first;  // key is (element_id, node_id)
        }
    }

    return -1;  // Not a warping DOF or not found
}

} // namespace grillex
