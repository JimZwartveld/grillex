#include "grillex/node_registry.hpp"
#include <cmath>

namespace grillex {

NodeRegistry::NodeRegistry(double tolerance)
    : tolerance_(tolerance), next_id_(1) {
}

Node* NodeRegistry::get_or_create_node(double x, double y, double z) {
    // Search for existing node within tolerance
    for (const auto& node : nodes_) {
        double dx = node->x - x;
        double dy = node->y - y;
        double dz = node->z - z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < tolerance_) {
            // Found existing node within tolerance
            return node.get();
        }
    }

    // No existing node found, create a new one
    auto new_node = std::make_unique<Node>(next_id_++, x, y, z);
    Node* node_ptr = new_node.get();
    nodes_.push_back(std::move(new_node));
    return node_ptr;
}

Node* NodeRegistry::create_node(double x, double y, double z) {
    // Always create a new node without checking for existing nodes
    auto new_node = std::make_unique<Node>(next_id_++, x, y, z);
    Node* node_ptr = new_node.get();
    nodes_.push_back(std::move(new_node));
    return node_ptr;
}

Node* NodeRegistry::find_node(double x, double y, double z) const {
    // Search for existing node within tolerance
    for (const auto& node : nodes_) {
        double dx = node->x - x;
        double dy = node->y - y;
        double dz = node->z - z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < tolerance_) {
            return node.get();
        }
    }
    return nullptr;
}

Node* NodeRegistry::get_node_by_id(int id) const {
    for (const auto& node : nodes_) {
        if (node->id == id) {
            return node.get();
        }
    }
    return nullptr;
}

const std::vector<std::unique_ptr<Node>>& NodeRegistry::all_nodes() const {
    return nodes_;
}

void NodeRegistry::set_tolerance(double tol) {
    tolerance_ = tol;
}

double NodeRegistry::get_tolerance() const {
    return tolerance_;
}

} // namespace grillex
