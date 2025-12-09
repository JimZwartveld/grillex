#pragma once

#include <array>
#include <Eigen/Dense>

namespace grillex {

/**
 * @brief Represents a node (point) in the structural model
 *
 * A Node represents a point in 3D space with up to 7 degrees of freedom:
 * - 3 translational: UX, UY, UZ (displacements in x, y, z)
 * - 3 rotational: RX, RY, RZ (rotations about x, y, z axes)
 * - 1 warping: WARP (warping displacement for thin-walled sections, optional)
 *
 * Coordinates are in meters [m].
 * The 7th DOF (warping) is disabled by default and can be enabled via enable_warping_dof().
 */
class Node {
public:
    int id;           ///< Unique node identifier
    double x, y, z;   ///< Nodal coordinates [m]

    /// DOF activation flags (true = active, false = inactive)
    /// Order: [UX, UY, UZ, RX, RY, RZ, WARP]
    std::array<bool, 7> dof_active = {true, true, true, true, true, true, false};

    /// Global DOF numbers assigned during assembly (-1 = not assigned)
    /// Order: [UX, UY, UZ, RX, RY, RZ, WARP]
    std::array<int, 7> global_dof_numbers = {-1, -1, -1, -1, -1, -1, -1};

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

    /**
     * @brief Enable the warping DOF (7th DOF) for this node
     *
     * Activates the warping degree of freedom for thin-walled section analysis.
     * This should be called for nodes connected to elements with warping capability.
     */
    void enable_warping_dof();

    /**
     * @brief Check if warping DOF is enabled
     *
     * @return bool True if warping DOF is active
     */
    bool has_warping_dof() const;

    /**
     * @brief Get number of active DOFs at this node
     *
     * @return int Number of active DOFs (6 or 7)
     */
    int num_active_dofs() const;
};

} // namespace grillex
