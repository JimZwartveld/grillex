#pragma once

#include "grillex/node.hpp"
#include "grillex/material.hpp"
#include <Eigen/Dense>
#include <array>

namespace grillex {

/**
 * @brief 9-node Lagrangian Mindlin plate element (MITC9)
 *
 * Implements a 9-node plate element using Mindlin plate theory with
 * Mixed Interpolation of Tensorial Components (MITC9) to avoid shear locking.
 *
 * The element has 54 DOFs (6 per node), but only uses 3 DOFs per node for
 * bending behavior: w (transverse displacement), θx (rotation about x),
 * and θy (rotation about y).
 *
 * Node numbering (natural coordinates):
 *   4 (-1,+1) --- 7 (0,+1) --- 3 (+1,+1)
 *       |                          |
 *   8 (-1,0)      9 (0,0)       6 (+1,0)
 *       |                          |
 *   1 (-1,-1) --- 5 (0,-1) --- 2 (+1,-1)
 *
 * Shape functions are Lagrangian (9-node) biquadratic polynomials.
 *
 * Local coordinate system:
 * - x_local: From node 1 toward node 2
 * - z_local: Normal to plate (outward)
 * - y_local: Completes right-hand system
 */
class PlateElement9 {
public:
    int id;                          ///< Unique element ID
    std::array<Node*, 9> nodes;      ///< 9 nodes (4 corners + 4 midside + 1 center)
    double thickness;                ///< Plate thickness [m]
    Material* material;              ///< Material properties

    // Local coordinate system (computed from node positions)
    Eigen::Vector3d x_axis;          ///< Local x-axis
    Eigen::Vector3d y_axis;          ///< Local y-axis
    Eigen::Vector3d z_axis;          ///< Local z-axis (plate normal)
    Eigen::Matrix3d rotation_matrix; ///< Rotation matrix (global to local)

    /**
     * @brief Construct a 9-node plate element
     * @param id Unique element ID
     * @param n1-n4 Corner nodes (counter-clockwise from ξ=-1, η=-1)
     * @param n5-n8 Midside nodes (n5 between n1-n2, n6 between n2-n3, etc.)
     * @param n9 Center node
     * @param thickness Plate thickness [m]
     * @param material Material properties
     */
    PlateElement9(int id, Node* n1, Node* n2, Node* n3, Node* n4,
                  Node* n5, Node* n6, Node* n7, Node* n8, Node* n9,
                  double thickness, Material* material);

    /**
     * @brief Get the number of DOFs for this element
     * @return Always 54 (6 DOFs per node × 9 nodes)
     */
    int num_dofs() const { return 54; }

    /**
     * @brief Check if element has warping DOF
     * @return Always false (plates don't have warping)
     */
    bool has_warping() const { return false; }

    /**
     * @brief Get global stiffness matrix (54×54)
     * @return Stiffness matrix in global coordinates
     *
     * Uses 3×3 Gauss quadrature for integration.
     */
    Eigen::Matrix<double, 54, 54> global_stiffness_matrix() const;

    /**
     * @brief Get global mass matrix (54×54)
     * @return Consistent mass matrix in global coordinates
     */
    Eigen::Matrix<double, 54, 54> global_mass_matrix() const;

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
     * @brief Get 9-node Lagrangian shape functions at natural coordinates
     * @param xi Natural coordinate ξ ∈ [-1, 1]
     * @param eta Natural coordinate η ∈ [-1, 1]
     * @return Shape functions N = [N1, N2, ..., N9]
     */
    Eigen::Matrix<double, 9, 1> shape_functions(double xi, double eta) const;

    /**
     * @brief Get shape function derivatives in natural coordinates
     * @param xi Natural coordinate ξ
     * @param eta Natural coordinate η
     * @return 2×9 matrix [dN/dξ; dN/dη]
     */
    Eigen::Matrix<double, 2, 9> shape_function_derivatives(double xi, double eta) const;

    /**
     * @brief Compute Jacobian matrix at natural coordinates
     */
    Eigen::Matrix2d jacobian(double xi, double eta) const;

    /**
     * @brief Get local coordinates of nodes (in plate plane)
     * @return 9×2 matrix with local (x,y) coordinates of each node
     */
    Eigen::Matrix<double, 9, 2> local_node_coords() const;

    /**
     * @brief Compute bending B-matrix at natural coordinates
     * @return 3×27 B-matrix (3 curvatures × 27 bending DOFs)
     */
    Eigen::Matrix<double, 3, 27> bending_B_matrix(double xi, double eta) const;

    /**
     * @brief Compute shear B-matrix using MITC9 formulation
     * @return 2×27 B-matrix (2 shear strains × 27 bending DOFs)
     */
    Eigen::Matrix<double, 2, 27> shear_B_matrix(double xi, double eta) const;

    /**
     * @brief Get bending stiffness matrix D_b
     */
    Eigen::Matrix3d bending_D_matrix() const;

    /**
     * @brief Get shear stiffness matrix D_s
     */
    Eigen::Matrix2d shear_D_matrix() const;

    /**
     * @brief Build local stiffness matrix (27×27 for bending DOFs only)
     */
    Eigen::Matrix<double, 27, 27> local_stiffness_matrix() const;

    /**
     * @brief Expand 27×27 bending matrix to 54×54 full DOF matrix
     */
    Eigen::Matrix<double, 54, 54> expand_to_full_dofs(const Eigen::Matrix<double, 27, 27>& K_bend) const;

    /**
     * @brief Build transformation matrix for global to local DOFs
     */
    Eigen::Matrix<double, 54, 54> transformation_matrix() const;

    // 3×3 Gauss quadrature points and weights
    static const double gauss_pts_3[3];
    static const double gauss_wts_3[3];
};

} // namespace grillex
