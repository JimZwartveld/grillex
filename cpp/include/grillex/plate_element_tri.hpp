#pragma once

#include "grillex/node.hpp"
#include "grillex/material.hpp"
#include <Eigen/Dense>
#include <array>

namespace grillex {

/**
 * @brief 3-node Discrete Kirchhoff Triangle (DKT) plate element
 *
 * Implements a 3-node triangular plate element using the Discrete Kirchhoff
 * Triangle formulation for thin plate bending analysis.
 *
 * The DKT element is based on Kirchhoff plate theory (no transverse shear
 * deformation) but uses a discrete approach to satisfy the Kirchhoff
 * constraint at specific points rather than throughout the element.
 *
 * The element has 18 DOFs (6 per node), but only uses 3 DOFs per node for
 * bending behavior: w (transverse displacement), θx (rotation about x),
 * and θy (rotation about y).
 *
 * Node numbering:
 *        3
 *       /\
 *      /  \
 *     /    \
 *    /      \
 *   1--------2
 *
 * Uses area coordinates (L1, L2, L3) for integration.
 *
 * Local coordinate system:
 * - x_local: From node 1 toward node 2
 * - z_local: Normal to plate (outward)
 * - y_local: Completes right-hand system
 *
 * Reference:
 * Batoz, J.L., Bathe, K.J., and Ho, L.W., "A Study of Three-Node
 * Triangular Plate Bending Elements", Int. J. Num. Meth. Eng., 1980.
 */
class PlateElementTri {
public:
    int id;                          ///< Unique element ID
    std::array<Node*, 3> nodes;      ///< 3 corner nodes
    double thickness;                ///< Plate thickness [m]
    Material* material;              ///< Material properties

    // Local coordinate system (computed from node positions)
    Eigen::Vector3d x_axis;          ///< Local x-axis (from node 1 to node 2)
    Eigen::Vector3d y_axis;          ///< Local y-axis
    Eigen::Vector3d z_axis;          ///< Local z-axis (plate normal)
    Eigen::Matrix3d rotation_matrix; ///< Rotation matrix (global to local)

    /**
     * @brief Construct a triangular plate element
     * @param id Unique element ID
     * @param n1 Node 1 (first corner)
     * @param n2 Node 2 (second corner)
     * @param n3 Node 3 (third corner)
     * @param thickness Plate thickness [m]
     * @param material Material properties
     */
    PlateElementTri(int id, Node* n1, Node* n2, Node* n3,
                    double thickness, Material* material);

    /**
     * @brief Get the number of DOFs for this element
     * @return Always 18 (6 DOFs per node × 3 nodes)
     */
    int num_dofs() const { return 18; }

    /**
     * @brief Check if element has warping DOF
     * @return Always false (plates don't have warping)
     */
    bool has_warping() const { return false; }

    /**
     * @brief Get global stiffness matrix (18×18)
     * @return Stiffness matrix in global coordinates
     *
     * Uses 3-point Gauss integration over the triangle.
     */
    Eigen::Matrix<double, 18, 18> global_stiffness_matrix() const;

    /**
     * @brief Get global mass matrix (18×18)
     * @return Lumped mass matrix in global coordinates
     */
    Eigen::Matrix<double, 18, 18> global_mass_matrix() const;

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
     * @brief Get local coordinates of nodes (in plate plane)
     * @return 3×2 matrix with local (x,y) coordinates of each node
     */
    Eigen::Matrix<double, 3, 2> local_node_coords() const;

    /**
     * @brief Compute edge lengths and geometric parameters
     */
    void compute_geometry_params();

    // Edge lengths
    double L12_, L23_, L31_;  // Edge lengths

    // Edge direction cosines (precomputed)
    double c12_, s12_;  // cos and sin for edge 1-2
    double c23_, s23_;  // cos and sin for edge 2-3
    double c31_, s31_;  // cos and sin for edge 3-1

    /**
     * @brief Get DKT shape functions Hx at given area coordinates
     * @param L1, L2, L3 Area coordinates (L1 + L2 + L3 = 1)
     * @return 9-element vector of Hx shape functions
     */
    Eigen::Matrix<double, 9, 1> Hx_functions(double L1, double L2, double L3) const;

    /**
     * @brief Get DKT shape functions Hy at given area coordinates
     * @param L1, L2, L3 Area coordinates
     * @return 9-element vector of Hy shape functions
     */
    Eigen::Matrix<double, 9, 1> Hy_functions(double L1, double L2, double L3) const;

    /**
     * @brief Get derivatives of Hx with respect to area coordinates
     * @return 3×9 matrix [dHx/dL1; dHx/dL2; dHx/dL3]
     */
    Eigen::Matrix<double, 3, 9> dHx_dL(double L1, double L2, double L3) const;

    /**
     * @brief Get derivatives of Hy with respect to area coordinates
     * @return 3×9 matrix [dHy/dL1; dHy/dL2; dHy/dL3]
     */
    Eigen::Matrix<double, 3, 9> dHy_dL(double L1, double L2, double L3) const;

    /**
     * @brief Compute bending B-matrix at given area coordinates
     * @param L1, L2, L3 Area coordinates
     * @return 3×9 B-matrix relating curvatures to nodal DOFs
     */
    Eigen::Matrix<double, 3, 9> bending_B_matrix(double L1, double L2, double L3) const;

    /**
     * @brief Get bending stiffness matrix D_b
     */
    Eigen::Matrix3d bending_D_matrix() const;

    /**
     * @brief Build local stiffness matrix (9×9 for bending DOFs only)
     */
    Eigen::Matrix<double, 9, 9> local_stiffness_matrix() const;

    /**
     * @brief Expand 9×9 bending matrix to 18×18 full DOF matrix
     */
    Eigen::Matrix<double, 18, 18> expand_to_full_dofs(const Eigen::Matrix<double, 9, 9>& K_bend) const;

    /**
     * @brief Build transformation matrix for global to local DOFs
     */
    Eigen::Matrix<double, 18, 18> transformation_matrix() const;

    // 3-point Gauss quadrature for triangles (area coordinates and weights)
    static const double gauss_L1_3pt[3];
    static const double gauss_L2_3pt[3];
    static const double gauss_wt_3pt;
};

} // namespace grillex
