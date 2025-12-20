#pragma once

#include "grillex/node.hpp"
#include "grillex/material.hpp"
#include <Eigen/Dense>
#include <array>

namespace grillex {

/**
 * @brief 4-node Mindlin plate element (MITC4)
 *
 * Implements a 4-node plate element using Mindlin plate theory with
 * Mixed Interpolation of Tensorial Components (MITC4) to avoid shear locking.
 *
 * The element has 24 DOFs (6 per node), but only uses 3 DOFs per node for
 * bending behavior: w (transverse displacement), θx (rotation about x),
 * and θy (rotation about y). The remaining DOFs (u, v, θz) are included
 * for compatibility with the 6-DOF node framework.
 *
 * Node numbering (natural coordinates):
 *   4 (-1,+1) -------- 3 (+1,+1)
 *       |                  |
 *       |     (0,0)        |
 *       |                  |
 *   1 (-1,-1) -------- 2 (+1,-1)
 *
 * Local coordinate system:
 * - x_local: From node 1 toward node 2
 * - z_local: Normal to plate (outward)
 * - y_local: Completes right-hand system
 *
 * Sign convention:
 * - Positive w: in positive z_local direction
 * - Positive θx: right-hand rotation about x_local (causes positive w at +y)
 * - Positive θy: right-hand rotation about y_local (causes negative w at +x)
 */
class PlateElement {
public:
    int id;                          ///< Unique element ID
    std::array<Node*, 4> nodes;      ///< 4 corner nodes
    double thickness;                ///< Plate thickness [m]
    Material* material;              ///< Material properties

    // Local coordinate system (computed from node positions)
    Eigen::Vector3d x_axis;          ///< Local x-axis (from node 1 to node 2)
    Eigen::Vector3d y_axis;          ///< Local y-axis
    Eigen::Vector3d z_axis;          ///< Local z-axis (plate normal)
    Eigen::Matrix3d rotation_matrix; ///< Rotation matrix (global to local)

    /**
     * @brief Construct a plate element
     * @param id Unique element ID
     * @param n1 Node 1 (corner at ξ=-1, η=-1)
     * @param n2 Node 2 (corner at ξ=+1, η=-1)
     * @param n3 Node 3 (corner at ξ=+1, η=+1)
     * @param n4 Node 4 (corner at ξ=-1, η=+1)
     * @param thickness Plate thickness [m]
     * @param material Material properties
     */
    PlateElement(int id, Node* n1, Node* n2, Node* n3, Node* n4,
                 double thickness, Material* material);

    /**
     * @brief Get the number of DOFs for this element
     * @return Always 24 (6 DOFs per node × 4 nodes)
     */
    int num_dofs() const { return 24; }

    /**
     * @brief Check if element has warping DOF
     * @return Always false (plates don't have warping)
     */
    bool has_warping() const { return false; }

    /**
     * @brief Get global stiffness matrix (24×24)
     * @return Stiffness matrix in global coordinates
     *
     * The stiffness matrix combines:
     * - Bending stiffness (from curvatures)
     * - Transverse shear stiffness (MITC4 interpolation)
     *
     * Uses 2×2 Gauss quadrature for bending and MITC4 for shear.
     */
    Eigen::Matrix<double, 24, 24> global_stiffness_matrix() const;

    /**
     * @brief Get global mass matrix (24×24)
     * @return Consistent mass matrix in global coordinates
     *
     * Uses lumped mass approximation with:
     * - Translational mass: ρ × t × A / 4 per node
     * - Rotational inertia: ρ × t³/12 × A / 4 per node
     */
    Eigen::Matrix<double, 24, 24> global_mass_matrix() const;

    /**
     * @brief Get element area
     * @return Area in [m²]
     *
     * Computes area from node positions using the cross product.
     */
    double area() const;

    /**
     * @brief Get centroid position
     * @return Centroid coordinates in global system [m]
     */
    Eigen::Vector3d centroid() const;

    /**
     * @brief Transform a vector from global to local coordinates
     * @param global Vector in global coordinates
     * @return Vector in local coordinates
     */
    Eigen::Vector3d to_local(const Eigen::Vector3d& global) const;

    /**
     * @brief Transform a vector from local to global coordinates
     * @param local Vector in local coordinates
     * @return Vector in global coordinates
     */
    Eigen::Vector3d to_global(const Eigen::Vector3d& local) const;

private:
    /**
     * @brief Compute local coordinate system from node positions
     *
     * The local axes are computed as:
     * - x_axis: (node2 - node1) normalized
     * - z_axis: (node2-node1) × (node4-node1) normalized (plate normal)
     * - y_axis: z_axis × x_axis (completes right-hand system)
     */
    void compute_local_axes();

    /**
     * @brief Build 3×3 rotation matrix from local axes
     *
     * R = [x_axis | y_axis | z_axis]
     * Transforms from global to local: v_local = R^T * v_global
     */
    void build_rotation_matrix();

    /**
     * @brief Get shape functions at natural coordinates
     * @param xi Natural coordinate ξ ∈ [-1, 1]
     * @param eta Natural coordinate η ∈ [-1, 1]
     * @return Shape functions N = [N1, N2, N3, N4]
     */
    Eigen::Vector4d shape_functions(double xi, double eta) const;

    /**
     * @brief Get shape function derivatives in natural coordinates
     * @param xi Natural coordinate ξ
     * @param eta Natural coordinate η
     * @return 2×4 matrix [dN/dξ; dN/dη]
     */
    Eigen::Matrix<double, 2, 4> shape_function_derivatives(double xi, double eta) const;

    /**
     * @brief Compute Jacobian matrix at natural coordinates
     * @param xi Natural coordinate ξ
     * @param eta Natural coordinate η
     * @return 2×2 Jacobian matrix J = [∂x/∂ξ, ∂y/∂ξ; ∂x/∂η, ∂y/∂η]
     */
    Eigen::Matrix2d jacobian(double xi, double eta) const;

    /**
     * @brief Get local coordinates of nodes (in plate plane)
     * @return 4×2 matrix with local (x,y) coordinates of each node
     */
    Eigen::Matrix<double, 4, 2> local_node_coords() const;

    /**
     * @brief Compute bending B-matrix at natural coordinates
     * @param xi Natural coordinate ξ
     * @param eta Natural coordinate η
     * @return 3×12 B-matrix relating curvatures to nodal rotations
     *
     * Curvature vector: κ = [κxx, κyy, 2κxy]^T
     * κ = Bb * [θx1, θy1, θx2, θy2, θx3, θy3, θx4, θy4]^T
     */
    Eigen::Matrix<double, 3, 12> bending_B_matrix(double xi, double eta) const;

    /**
     * @brief Compute MITC4 shear B-matrix at natural coordinates
     * @param xi Natural coordinate ξ
     * @param eta Natural coordinate η
     * @return 2×12 B-matrix relating shear strains to nodal DOFs
     *
     * Uses MITC4 formulation for locking-free shear interpolation.
     * Shear strain: γ = [γxz, γyz]^T
     */
    Eigen::Matrix<double, 2, 12> shear_B_matrix_mitc4(double xi, double eta) const;

    /**
     * @brief Get bending stiffness matrix D_b
     * @return 3×3 bending constitutive matrix
     *
     * D_b = (E*t³)/(12*(1-ν²)) * [1   ν   0  ]
     *                            [ν   1   0  ]
     *                            [0   0  (1-ν)/2]
     */
    Eigen::Matrix3d bending_D_matrix() const;

    /**
     * @brief Get shear stiffness matrix D_s
     * @return 2×2 shear constitutive matrix
     *
     * D_s = κ*G*t * [1  0]
     *               [0  1]
     * where κ = 5/6 (shear correction factor)
     */
    Eigen::Matrix2d shear_D_matrix() const;

    /**
     * @brief Build local stiffness matrix (12×12 for bending DOFs only)
     * @return Local stiffness matrix for [w1,θx1,θy1, w2,θx2,θy2, w3,θx3,θy3, w4,θx4,θy4]
     */
    Eigen::Matrix<double, 12, 12> local_stiffness_matrix() const;

    /**
     * @brief Expand 12×12 bending matrix to 24×24 full DOF matrix
     * @param K_bend 12×12 bending stiffness matrix
     * @return 24×24 stiffness matrix with zeros for in-plane DOFs
     */
    Eigen::Matrix<double, 24, 24> expand_to_full_dofs(const Eigen::Matrix<double, 12, 12>& K_bend) const;

    /**
     * @brief Build transformation matrix for global to local DOFs
     * @return 24×24 transformation matrix T
     *
     * K_global = T^T * K_local * T
     */
    Eigen::Matrix<double, 24, 24> transformation_matrix() const;
};

} // namespace grillex
