#pragma once

#include <array>
#include <Eigen/Dense>

namespace grillex {

/**
 * @brief Represents a node (point) in the structural model
 *
 * A Node represents a point in 3D space with up to 6 degrees of freedom:
 * - 3 translational: UX, UY, UZ (displacements in x, y, z)
 * - 3 rotational: RX, RY, RZ (rotations about x, y, z axes)
 *
 * Coordinates are in meters [m].
 */
class Node {
public:
    int id;           ///< Unique node identifier
    double x, y, z;   ///< Nodal coordinates [m]

    /// DOF activation flags (true = active, false = inactive)
    /// Order: [UX, UY, UZ, RX, RY, RZ]
    std::array<bool, 6> dof_active = {true, true, true, true, true, true};

    /// Global DOF numbers assigned during assembly (-1 = not assigned)
    /// Order: [UX, UY, UZ, RX, RY, RZ]
    std::array<int, 6> global_dof_numbers = {-1, -1, -1, -1, -1, -1};

    /**
     * @brief Construct a new Node
     *
     * @param id Unique node identifier
     * @param x X-coordinate [m]
     * @param y Y-coordinate [m]
     * @param z Z-coordinate [m]
     */
    Node(int id, double x, double y, double z);

    /**
     * @brief Get the node position as an Eigen vector
     *
     * @return Eigen::Vector3d Position vector [x, y, z] in meters
     */
    Eigen::Vector3d position() const;
};

} // namespace grillex
