#include "grillex/node.hpp"

namespace grillex {

Node::Node(int id, double x, double y, double z)
    : id(id), x(x), y(y), z(z) {
    // dof_active and global_dof_numbers are initialized with default values in the header
}

Eigen::Vector3d Node::position() const {
    return Eigen::Vector3d(x, y, z);
}

void Node::enable_warping_dof() {
    dof_active[6] = true;
}

bool Node::has_warping_dof() const {
    return dof_active[6];
}

int Node::num_active_dofs() const {
    int count = 0;
    for (bool active : dof_active) {
        if (active) count++;
    }
    return count;
}

} // namespace grillex
