#pragma once

#include "grillex/node.hpp"
#include <Eigen/Dense>

namespace grillex {

/**
 * @brief Point mass element at a single node
 *
 * Represents a concentrated mass and rotational inertia at a node.
 * The mass matrix is a 6×6 matrix with translational mass on the
 * first 3 diagonal entries and the full inertia tensor in the
 * rotational DOFs (including off-diagonal products of inertia).
 *
 * Mass matrix structure:
 *   M = [m  0  0   0    0    0  ]
 *       [0  m  0   0    0    0  ]
 *       [0  0  m   0    0    0  ]
 *       [0  0  0  Ixx  Ixy  Ixz]
 *       [0  0  0  Ixy  Iyy  Iyz]
 *       [0  0  0  Ixz  Iyz  Izz]
 *
 * Usage:
 *   PointMass pm(1, node);
 *   pm.mass = 10.0;  // 10 mT
 *   pm.Ixx = 5.0;    // kN·m·s²
 *   auto M = pm.mass_matrix();
 */
class PointMass {
public:
    int id;         ///< Unique element ID
    Node* node;     ///< Associated node

    // Translational mass [mT]
    double mass = 0.0;

    // Moments of inertia [mT·m² = kN·m·s²]
    double Ixx = 0.0;  ///< Moment of inertia about x axis
    double Iyy = 0.0;  ///< Moment of inertia about y axis
    double Izz = 0.0;  ///< Moment of inertia about z axis

    // Products of inertia [mT·m² = kN·m·s²]
    double Ixy = 0.0;  ///< Product of inertia xy
    double Ixz = 0.0;  ///< Product of inertia xz
    double Iyz = 0.0;  ///< Product of inertia yz

    /**
     * @brief Construct a point mass
     * @param id Unique element ID
     * @param node Node where mass is located
     */
    PointMass(int id, Node* node);

    /**
     * @brief Get the number of DOFs for this element
     * @return Always 6 (single node)
     */
    int num_dofs() const { return 6; }

    /**
     * @brief Check if element has warping DOF
     * @return Always false (point masses don't have warping)
     */
    bool has_warping() const { return false; }

    /**
     * @brief Get mass matrix (6×6)
     * @return Mass matrix including full inertia tensor
     *
     * Returns symmetric positive semi-definite matrix.
     * Off-diagonal inertia terms (Ixy, Ixz, Iyz) allow modeling
     * of asymmetric mass distributions.
     */
    Eigen::Matrix<double, 6, 6> mass_matrix() const;

    /**
     * @brief Set translational mass
     * @param m Mass value [mT]
     */
    void set_mass(double m) { mass = m; }

    /**
     * @brief Set diagonal moments of inertia
     * @param ixx Moment about x [mT·m²]
     * @param iyy Moment about y [mT·m²]
     * @param izz Moment about z [mT·m²]
     */
    void set_inertia(double ixx, double iyy, double izz);

    /**
     * @brief Set products of inertia (off-diagonal terms)
     * @param ixy Product of inertia xy [mT·m²]
     * @param ixz Product of inertia xz [mT·m²]
     * @param iyz Product of inertia yz [mT·m²]
     */
    void set_products_of_inertia(double ixy, double ixz, double iyz);

    /**
     * @brief Set full inertia tensor at once
     * @param ixx Moment about x [mT·m²]
     * @param iyy Moment about y [mT·m²]
     * @param izz Moment about z [mT·m²]
     * @param ixy Product of inertia xy [mT·m²]
     * @param ixz Product of inertia xz [mT·m²]
     * @param iyz Product of inertia yz [mT·m²]
     */
    void set_full_inertia(double ixx, double iyy, double izz,
                          double ixy, double ixz, double iyz);

    /**
     * @brief Check if mass matrix is physically valid
     * @return true if mass >= 0 and inertia tensor is positive semi-definite
     *
     * A valid inertia tensor must satisfy:
     * - Ixx, Iyy, Izz >= 0
     * - Triangle inequality: Ixx + Iyy >= Izz, etc.
     * - Determinant >= 0
     */
    bool is_valid() const;

    /**
     * @brief Get total mass
     * @return Translational mass value [mT]
     */
    double get_total_mass() const { return mass; }
};

} // namespace grillex
