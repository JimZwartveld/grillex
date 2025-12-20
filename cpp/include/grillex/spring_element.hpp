#pragma once

#include "grillex/node.hpp"
#include <Eigen/Dense>

namespace grillex {

/**
 * @brief Spring element connecting two nodes
 *
 * Represents a general 6-DOF spring element with independent stiffness
 * values for each translational and rotational DOF. The element connects
 * two nodes and provides stiffness coupling between them.
 *
 * Stiffness matrix structure (for each DOF pair):
 *   K = [+k  -k]
 *       [-k  +k]
 *
 * For uncoupled springs, each DOF is independent.
 *
 * Usage:
 *   SpringElement spring(1, node_i, node_j);
 *   spring.kx = 1000.0;  // Axial stiffness [kN/m]
 *   spring.kz = 500.0;   // Vertical stiffness [kN/m]
 *   auto K = spring.global_stiffness_matrix();
 */
class SpringElement {
public:
    int id;           ///< Unique element ID
    Node* node_i;     ///< Start node
    Node* node_j;     ///< End node

    // Translational stiffness [kN/m]
    double kx = 0.0;  ///< Stiffness in x direction
    double ky = 0.0;  ///< Stiffness in y direction
    double kz = 0.0;  ///< Stiffness in z direction

    // Rotational stiffness [kN·m/rad]
    double krx = 0.0; ///< Rotational stiffness about x axis
    double kry = 0.0; ///< Rotational stiffness about y axis
    double krz = 0.0; ///< Rotational stiffness about z axis

    /**
     * @brief Construct a spring element
     * @param id Unique element ID
     * @param node_i Start node
     * @param node_j End node
     */
    SpringElement(int id, Node* node_i, Node* node_j);

    /**
     * @brief Get the number of DOFs for this element
     * @return Always 12 (6 DOFs per node)
     */
    int num_dofs() const { return 12; }

    /**
     * @brief Check if element has warping DOF
     * @return Always false (springs don't have warping)
     */
    bool has_warping() const { return false; }

    /**
     * @brief Get global stiffness matrix (12×12)
     * @return Stiffness matrix in global coordinates
     *
     * Matrix structure for each stiffness k:
     *   [+k  0  ... -k  0  ...]
     *   [0  +k ... 0  -k  ...]
     *   ...
     *   [-k  0 ... +k  0  ...]
     *   [0  -k ... 0  +k  ...]
     *
     * Spring stiffness is already in global coordinates (no transformation needed).
     */
    Eigen::Matrix<double, 12, 12> global_stiffness_matrix() const;

    /**
     * @brief Get global mass matrix (12×12)
     * @return Zero matrix (springs have no mass)
     *
     * Springs are assumed massless. Use PointMass for concentrated masses.
     */
    Eigen::Matrix<double, 12, 12> global_mass_matrix() const;

    /**
     * @brief Set all translational stiffnesses at once
     * @param kx_val Stiffness in x direction [kN/m]
     * @param ky_val Stiffness in y direction [kN/m]
     * @param kz_val Stiffness in z direction [kN/m]
     */
    void set_translational_stiffness(double kx_val, double ky_val, double kz_val);

    /**
     * @brief Set all rotational stiffnesses at once
     * @param krx_val Rotational stiffness about x axis [kN·m/rad]
     * @param kry_val Rotational stiffness about y axis [kN·m/rad]
     * @param krz_val Rotational stiffness about z axis [kN·m/rad]
     */
    void set_rotational_stiffness(double krx_val, double kry_val, double krz_val);

    /**
     * @brief Check if element has any non-zero stiffness
     * @return true if any stiffness component is non-zero
     */
    bool has_stiffness() const;
};

} // namespace grillex
