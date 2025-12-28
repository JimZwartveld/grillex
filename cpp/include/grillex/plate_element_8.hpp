#pragma once

#include "grillex/node.hpp"
#include "grillex/material.hpp"
#include <Eigen/Dense>
#include <array>

namespace grillex {

/**
 * @brief 8-node serendipity Mindlin plate element (MITC8)
 *
 * Implements an 8-node plate element using Mindlin plate theory with
 * Mixed Interpolation of Tensorial Components (MITC8) to avoid shear locking.
 *
 * The element has 48 DOFs (6 per node), but only uses 3 DOFs per node for
 * bending behavior: w (transverse displacement), θx (rotation about x),
 * and θy (rotation about y).
 *
 * Node numbering (natural coordinates):
 *   4 (-1,+1) --- 7 (0,+1) --- 3 (+1,+1)
 *       |                          |
 *   8 (-1,0)                    6 (+1,0)
 *       |                          |
 *   1 (-1,-1) --- 5 (0,-1) --- 2 (+1,-1)
 *
 * Shape functions are serendipity (8-node) quadratic polynomials.
 *
 * Local coordinate system:
 * - x_local: From node 1 toward node 2
 * - z_local: Normal to plate (outward)
 * - y_local: Completes right-hand system
 */
class PlateElement8 {
public:
    int id;                          ///< Unique element ID
    std::array<Node*, 8> nodes;      ///< 8 nodes (4 corners + 4 midside)
    double thickness;                ///< Plate thickness [m]
    Material* material;              ///< Material properties

    // Local coordinate system (computed from node positions)
    Eigen::Vector3d x_axis;          ///< Local x-axis
    Eigen::Vector3d y_axis;          ///< Local y-axis
    Eigen::Vector3d z_axis;          ///< Local z-axis (plate normal)
    Eigen::Matrix3d rotation_matrix; ///< Rotation matrix (global to local)

    /**
     * @brief Construct an 8-node plate element
     * @param id Unique element ID
     * @param n1-n4 Corner nodes (counter-clockwise from ξ=-1, η=-1)
     * @param n5-n8 Midside nodes (n5 between n1-n2, n6 between n2-n3, etc.)
     * @param thickness Plate thickness [m]
     * @param material Material properties
     */
    PlateElement8(int id, Node* n1, Node* n2, Node* n3, Node* n4,
                  Node* n5, Node* n6, Node* n7, Node* n8,
                  double thickness, Material* material);

    /**
     * @brief Get the number of DOFs for this element
     * @return Always 48 (6 DOFs per node × 8 nodes)
     */
    int num_dofs() const { return 48; }

    /**
     * @brief Check if element has warping DOF
     * @return Always false (plates don't have warping)
     */
    bool has_warping() const { return false; }

    /**
     * @brief Get global stiffness matrix (48×48)
     * @return Stiffness matrix in global coordinates
     *
     * Uses 3×3 Gauss quadrature for integration.
     */
    Eigen::Matrix<double, 48, 48> global_stiffness_matrix() const;

    /**
     * @brief Get global mass matrix (48×48)
     * @return Consistent mass matrix in global coordinates
     */
    Eigen::Matrix<double, 48, 48> global_mass_matrix() const;

    /**
     * @brief Get element area
     * @return Area in [m²]
     */
    double area() const;

    /**
     * @brief Get centroid position
     * @return Centroid coordinates in global system [m]
     */
    Eigen::Vector3d centroid() const;

    /**
     * @brief Transform a vector from global to local coordinates
     */
    Eigen::Vector3d to_local(const Eigen::Vector3d& global) const;

    /**
     * @brief Transform a vector from local to global coordinates
     */
    Eigen::Vector3d to_global(const Eigen::Vector3d& local) const;

private:
    void compute_local_axes();
    void build_rotation_matrix();

    /**
     * @brief Get 8-node serendipity shape functions at natural coordinates
     * @param xi Natural coordinate ξ ∈ [-1, 1]
     * @param eta Natural coordinate η ∈ [-1, 1]
     * @return Shape functions N = [N1, N2, ..., N8]
     */
    Eigen::Matrix<double, 8, 1> shape_functions(double xi, double eta) const;

    /**
     * @brief Get shape function derivatives in natural coordinates
     * @param xi Natural coordinate ξ
     * @param eta Natural coordinate η
     * @return 2×8 matrix [dN/dξ; dN/dη]
     */
    Eigen::Matrix<double, 2, 8> shape_function_derivatives(double xi, double eta) const;

    /**
     * @brief Compute Jacobian matrix at natural coordinates
     */
    Eigen::Matrix2d jacobian(double xi, double eta) const;

    /**
     * @brief Get local coordinates of nodes (in plate plane)
     * @return 8×2 matrix with local (x,y) coordinates of each node
     */
    Eigen::Matrix<double, 8, 2> local_node_coords() const;

    /**
     * @brief Compute bending B-matrix at natural coordinates
     * @return 3×24 B-matrix (3 curvatures × 24 bending DOFs)
     */
    Eigen::Matrix<double, 3, 24> bending_B_matrix(double xi, double eta) const;

    /**
     * @brief Compute shear B-matrix using MITC8 formulation
     * @return 2×24 B-matrix (2 shear strains × 24 bending DOFs)
     */
    Eigen::Matrix<double, 2, 24> shear_B_matrix(double xi, double eta) const;

    /**
     * @brief Get bending stiffness matrix D_b
     */
    Eigen::Matrix3d bending_D_matrix() const;

    /**
     * @brief Get shear stiffness matrix D_s
     */
    Eigen::Matrix2d shear_D_matrix() const;

    /**
     * @brief Build local stiffness matrix (24×24 for bending DOFs only)
     */
    Eigen::Matrix<double, 24, 24> local_stiffness_matrix() const;

    /**
     * @brief Expand 24×24 bending matrix to 48×48 full DOF matrix
     */
    Eigen::Matrix<double, 48, 48> expand_to_full_dofs(const Eigen::Matrix<double, 24, 24>& K_bend) const;

    /**
     * @brief Build transformation matrix for global to local DOFs
     */
    Eigen::Matrix<double, 48, 48> transformation_matrix() const;

    // 3×3 Gauss quadrature points and weights
    static const double gauss_pts_3[3];
    static const double gauss_wts_3[3];
};

} // namespace grillex
