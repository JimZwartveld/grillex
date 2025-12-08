#pragma once

#include "grillex/node.hpp"
#include <vector>
#include <memory>

namespace grillex {

/**
 * @brief Registry that manages nodes and handles node merging based on tolerance
 *
 * The NodeRegistry maintains a collection of nodes and automatically merges
 * nodes that are within a specified geometric tolerance. This helps prevent
 * duplicate nodes at nearly identical positions.
 *
 * Tolerance is specified in meters [m].
 */
class NodeRegistry {
public:
    /**
     * @brief Construct a new Node Registry
     *
     * @param tolerance Distance tolerance for merging nodes [m]. Default: 1e-6 m
     */
    NodeRegistry(double tolerance = 1e-6);

    /**
     * @brief Get existing node or create new one at specified position
     *
     * If a node exists within tolerance of (x, y, z), returns that node.
     * Otherwise, creates a new node with the next available ID.
     *
     * @param x X-coordinate [m]
     * @param y Y-coordinate [m]
     * @param z Z-coordinate [m]
     * @return Node* Pointer to existing or newly created node
     */
    Node* get_or_create_node(double x, double y, double z);

    /**
     * @brief Get node by its ID
     *
     * @param id Node ID to find
     * @return Node* Pointer to node if found, nullptr otherwise
     */
    Node* get_node_by_id(int id);

    /**
     * @brief Get all nodes in the registry
     *
     * @return const std::vector<std::unique_ptr<Node>>& Vector of all nodes
     */
    const std::vector<std::unique_ptr<Node>>& all_nodes() const;

    /**
     * @brief Set the distance tolerance for node merging
     *
     * @param tol New tolerance value [m]
     */
    void set_tolerance(double tol);

    /**
     * @brief Get the current distance tolerance
     *
     * @return double Current tolerance [m]
     */
    double get_tolerance() const;

private:
    double tolerance_;  ///< Distance tolerance for node merging [m]
    std::vector<std::unique_ptr<Node>> nodes_;  ///< Storage for all nodes
    int next_id_ = 1;  ///< Next node ID to assign (starts at 1)
};

} // namespace grillex
