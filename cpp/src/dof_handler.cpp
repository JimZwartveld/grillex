#include "grillex/dof_handler.hpp"
#include <stdexcept>

namespace grillex {

DOFHandler::DOFHandler() : total_dofs_(0), has_warping_(false) {}

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
        if (node->has_warping_dof() && node->dof_active[6]) {
            dof_map_[{node->id, 6}] = global_dof_counter++;
            has_warping_ = true;
        }
    }

    total_dofs_ = global_dof_counter;
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

std::vector<int> DOFHandler::get_location_array(const BeamElementBase& elem) const {
    // Get the concrete beam element to access node pointers
    const BeamElement* beam_elem = dynamic_cast<const BeamElement*>(&elem);
    if (!beam_elem) {
        throw std::runtime_error("Element is not a BeamElement");
    }

    int n_dofs = elem.num_dofs();  // 12 or 14
    int dofs_per_node = n_dofs / 2;  // 6 or 7
    std::vector<int> loc(n_dofs);

    // Node i DOFs
    for (int d = 0; d < dofs_per_node; ++d) {
        loc[d] = get_global_dof(beam_elem->node_i->id, d);
    }

    // Node j DOFs
    for (int d = 0; d < dofs_per_node; ++d) {
        loc[dofs_per_node + d] = get_global_dof(beam_elem->node_j->id, d);
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
}

} // namespace grillex
