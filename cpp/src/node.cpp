#include "grillex/node.hpp"

namespace grillex {

Node::Node(int id, double x, double y, double z)
    : id(id), x(x), y(y), z(z) {
    // dof_active and global_dof_numbers are initialized with default values in the header
}

Eigen::Vector3d Node::position() const {
    return Eigen::Vector3d(x, y, z);
}

} // namespace grillex
