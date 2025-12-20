#include "grillex/plate_element.hpp"
#include <cmath>
#include <stdexcept>

namespace grillex {

PlateElement::PlateElement(int id, Node* n1, Node* n2, Node* n3, Node* n4,
                           double thickness, Material* material)
    : id(id), nodes{n1, n2, n3, n4}, thickness(thickness), material(material) {

    if (!n1 || !n2 || !n3 || !n4) {
        throw std::invalid_argument("PlateElement: All nodes must be non-null");
    }
    if (thickness <= 0) {
        throw std::invalid_argument("PlateElement: Thickness must be positive");
    }
    if (!material) {
        throw std::invalid_argument("PlateElement: Material must be non-null");
    }

    compute_local_axes();
    build_rotation_matrix();
}

void PlateElement::compute_local_axes() {
    // Get node positions
    Eigen::Vector3d p1(nodes[0]->x, nodes[0]->y, nodes[0]->z);
    Eigen::Vector3d p2(nodes[1]->x, nodes[1]->y, nodes[1]->z);
    Eigen::Vector3d p3(nodes[2]->x, nodes[2]->y, nodes[2]->z);
    Eigen::Vector3d p4(nodes[3]->x, nodes[3]->y, nodes[3]->z);

    // x-axis: from node 1 to node 2
    Eigen::Vector3d v12 = p2 - p1;
    Eigen::Vector3d v14 = p4 - p1;

    // z-axis: normal to plate (using cross product)
    z_axis = v12.cross(v14);
    double norm_z = z_axis.norm();
    if (norm_z < 1e-10) {
        throw std::runtime_error("PlateElement: Degenerate element (nodes are collinear)");
    }
    z_axis /= norm_z;

    // x-axis: along edge 1-2, normalized
    x_axis = v12.normalized();

    // y-axis: completes right-hand system
    y_axis = z_axis.cross(x_axis);
}

void PlateElement::build_rotation_matrix() {
    // Rotation matrix has local axes as columns
    // R transforms from global to local: v_local = R^T * v_global
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
}

Eigen::Vector3d PlateElement::to_local(const Eigen::Vector3d& global) const {
    return rotation_matrix.transpose() * global;
}

Eigen::Vector3d PlateElement::to_global(const Eigen::Vector3d& local) const {
    return rotation_matrix * local;
}

Eigen::Vector4d PlateElement::shape_functions(double xi, double eta) const {
    // Bilinear shape functions for 4-node quadrilateral
    Eigen::Vector4d N;
    N(0) = 0.25 * (1.0 - xi) * (1.0 - eta);  // N1 at (-1,-1)
    N(1) = 0.25 * (1.0 + xi) * (1.0 - eta);  // N2 at (+1,-1)
    N(2) = 0.25 * (1.0 + xi) * (1.0 + eta);  // N3 at (+1,+1)
    N(3) = 0.25 * (1.0 - xi) * (1.0 + eta);  // N4 at (-1,+1)
    return N;
}

Eigen::Matrix<double, 2, 4> PlateElement::shape_function_derivatives(double xi, double eta) const {
    // Derivatives of shape functions with respect to natural coordinates
    Eigen::Matrix<double, 2, 4> dN;

    // dN/dxi
    dN(0, 0) = -0.25 * (1.0 - eta);
    dN(0, 1) =  0.25 * (1.0 - eta);
    dN(0, 2) =  0.25 * (1.0 + eta);
    dN(0, 3) = -0.25 * (1.0 + eta);

    // dN/deta
    dN(1, 0) = -0.25 * (1.0 - xi);
    dN(1, 1) = -0.25 * (1.0 + xi);
    dN(1, 2) =  0.25 * (1.0 + xi);
    dN(1, 3) =  0.25 * (1.0 - xi);

    return dN;
}

Eigen::Matrix<double, 4, 2> PlateElement::local_node_coords() const {
    // Get node positions in local coordinates
    Eigen::Vector3d p1(nodes[0]->x, nodes[0]->y, nodes[0]->z);

    Eigen::Matrix<double, 4, 2> coords;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d pi(nodes[i]->x, nodes[i]->y, nodes[i]->z);
        Eigen::Vector3d local = to_local(pi - p1);
        coords(i, 0) = local(0);  // x_local
        coords(i, 1) = local(1);  // y_local
    }

    return coords;
}

Eigen::Matrix2d PlateElement::jacobian(double xi, double eta) const {
    auto dN = shape_function_derivatives(xi, eta);
    auto xy = local_node_coords();

    // J = dN * xy
    // J(0,0) = dx/dxi, J(0,1) = dy/dxi
    // J(1,0) = dx/deta, J(1,1) = dy/deta
    Eigen::Matrix2d J = dN * xy;

    return J;
}

double PlateElement::area() const {
    // Compute area using 2x2 Gauss integration of Jacobian determinant
    double A = 0.0;
    const double gp = 1.0 / std::sqrt(3.0);
    double gauss_pts[4][2] = {{-gp, -gp}, {gp, -gp}, {gp, gp}, {-gp, gp}};

    for (int i = 0; i < 4; ++i) {
        double xi = gauss_pts[i][0];
        double eta = gauss_pts[i][1];
        Eigen::Matrix2d J = jacobian(xi, eta);
        A += J.determinant();  // weight = 1 for each Gauss point
    }

    return A;
}

Eigen::Vector3d PlateElement::centroid() const {
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (int i = 0; i < 4; ++i) {
        c(0) += nodes[i]->x;
        c(1) += nodes[i]->y;
        c(2) += nodes[i]->z;
    }
    return c / 4.0;
}

Eigen::Matrix3d PlateElement::bending_D_matrix() const {
    // Bending constitutive matrix
    // D_b = (E*t³)/(12*(1-ν²)) * [1   ν   0     ]
    //                            [ν   1   0     ]
    //                            [0   0  (1-ν)/2]
    double E = material->E;
    double nu = material->nu;
    double t = thickness;

    double factor = E * t * t * t / (12.0 * (1.0 - nu * nu));

    Eigen::Matrix3d D;
    D << 1.0,   nu,    0.0,
         nu,    1.0,   0.0,
         0.0,   0.0,   (1.0 - nu) / 2.0;

    return factor * D;
}

Eigen::Matrix2d PlateElement::shear_D_matrix() const {
    // Shear constitutive matrix
    // D_s = κ*G*t * [1  0]
    //               [0  1]
    double G = material->G;
    double t = thickness;
    double kappa = 5.0 / 6.0;  // Shear correction factor

    return kappa * G * t * Eigen::Matrix2d::Identity();
}

Eigen::Matrix<double, 3, 12> PlateElement::bending_B_matrix(double xi, double eta) const {
    // Bending B-matrix relates curvatures to nodal rotations
    // Curvature: κ = [κxx, κyy, 2κxy]^T
    //
    // For Mindlin plate: κxx = -∂θy/∂x, κyy = ∂θx/∂y, κxy = 0.5*(∂θx/∂x - ∂θy/∂y)
    //
    // DOFs per node: [w, θx, θy]
    // B_b relates [w1,θx1,θy1, w2,θx2,θy2, w3,θx3,θy3, w4,θx4,θy4] to [κxx, κyy, 2κxy]

    auto dN = shape_function_derivatives(xi, eta);
    Eigen::Matrix2d J = jacobian(xi, eta);
    Eigen::Matrix2d Jinv = J.inverse();

    // Transform derivatives to physical coordinates
    // dN_dx = Jinv * dN
    Eigen::Matrix<double, 2, 4> dN_dx;
    dN_dx.row(0) = Jinv(0, 0) * dN.row(0) + Jinv(0, 1) * dN.row(1);  // dN/dx
    dN_dx.row(1) = Jinv(1, 0) * dN.row(0) + Jinv(1, 1) * dN.row(1);  // dN/dy

    // Build B-matrix (3 rows x 12 columns)
    // For each node i: DOFs are [w_i, θx_i, θy_i]
    Eigen::Matrix<double, 3, 12> B = Eigen::Matrix<double, 3, 12>::Zero();

    for (int i = 0; i < 4; ++i) {
        double dNi_dx = dN_dx(0, i);
        double dNi_dy = dN_dx(1, i);

        int col_w = 3 * i;
        int col_thx = 3 * i + 1;
        int col_thy = 3 * i + 2;

        // κxx = -∂θy/∂x
        B(0, col_thy) = -dNi_dx;

        // κyy = ∂θx/∂y
        B(1, col_thx) = dNi_dy;

        // 2κxy = ∂θx/∂x - ∂θy/∂y
        B(2, col_thx) = dNi_dx;
        B(2, col_thy) = -dNi_dy;
    }

    return B;
}

Eigen::Matrix<double, 2, 12> PlateElement::shear_B_matrix_mitc4(double xi, double eta) const {
    // MITC4 shear B-matrix for locking-free shear interpolation
    //
    // The MITC4 formulation samples shear strains at tying points on element edges:
    // - γxz is sampled at points A (0,-1) and B (0,+1) on edges 1-2 and 3-4
    // - γyz is sampled at points C (-1,0) and D (+1,0) on edges 1-4 and 2-3
    //
    // Then interpolates linearly:
    // - γxz = 0.5*(1-η)*γxz_A + 0.5*(1+η)*γxz_B
    // - γyz = 0.5*(1-ξ)*γyz_C + 0.5*(1+ξ)*γyz_D

    auto xy = local_node_coords();

    // Get node coordinates
    double x1 = xy(0, 0), y1 = xy(0, 1);
    double x2 = xy(1, 0), y2 = xy(1, 1);
    double x3 = xy(2, 0), y3 = xy(2, 1);
    double x4 = xy(3, 0), y4 = xy(3, 1);

    // Compute edge vectors
    double dx12 = x2 - x1, dy12 = y2 - y1;
    double dx23 = x3 - x2, dy23 = y3 - y2;
    double dx34 = x4 - x3, dy34 = y4 - y3;
    double dx41 = x1 - x4, dy41 = y1 - y4;

    // Shape functions at tying points
    // Point A: (0, -1) - midpoint of edge 1-2
    // Point B: (0, +1) - midpoint of edge 3-4
    // Point C: (-1, 0) - midpoint of edge 1-4
    // Point D: (+1, 0) - midpoint of edge 2-3

    // At point A (ξ=0, η=-1): N = [0.5, 0.5, 0, 0]
    // At point B (ξ=0, η=+1): N = [0, 0, 0.5, 0.5]
    // At point C (ξ=-1, η=0): N = [0.5, 0, 0, 0.5]
    // At point D (ξ=+1, η=0): N = [0, 0.5, 0.5, 0]

    // Jacobians at tying points
    Eigen::Matrix2d J_A = jacobian(0.0, -1.0);
    Eigen::Matrix2d J_B = jacobian(0.0, +1.0);
    Eigen::Matrix2d J_C = jacobian(-1.0, 0.0);
    Eigen::Matrix2d J_D = jacobian(+1.0, 0.0);

    double detJ_A = J_A.determinant();
    double detJ_B = J_B.determinant();
    double detJ_C = J_C.determinant();
    double detJ_D = J_D.determinant();

    // For MITC4, we compute the covariant shear strains at tying points
    // and then transform to Cartesian coordinates

    // The covariant shear strains are:
    // e_ξz = ∂w/∂ξ - (∂x/∂ξ * θy - ∂y/∂ξ * θx)  [actually involves J]
    // e_ηz = ∂w/∂η - (∂x/∂η * θy - ∂y/∂η * θx)

    // Build B-matrix directly using assumed strain approach
    // This is a simplified implementation for a rectangular-ish element

    Eigen::Matrix<double, 2, 12> B = Eigen::Matrix<double, 2, 12>::Zero();

    // Shape function derivatives at current point
    auto dN = shape_function_derivatives(xi, eta);
    Eigen::Matrix2d J = jacobian(xi, eta);
    Eigen::Matrix2d Jinv = J.inverse();

    // Shape functions at current point
    Eigen::Vector4d N = shape_functions(xi, eta);

    // Derivatives in physical coordinates
    Eigen::Matrix<double, 2, 4> dN_dx;
    dN_dx.row(0) = Jinv(0, 0) * dN.row(0) + Jinv(0, 1) * dN.row(1);  // dN/dx
    dN_dx.row(1) = Jinv(1, 0) * dN.row(0) + Jinv(1, 1) * dN.row(1);  // dN/dy

    // MITC4: Use assumed shear strain interpolation
    // γxz is interpolated along η direction (constant along ξ at each tying line)
    // γyz is interpolated along ξ direction (constant along η at each tying line)

    // For edge-based interpolation of shear strains:
    // Evaluate B-matrix components using MITC4 tying scheme

    // Simplified approach: Sample and interpolate
    // B_xz at point A (edge 1-2) and B (edge 3-4)
    // B_yz at point C (edge 1-4) and D (edge 2-3)

    // At point A (0,-1): Uses nodes 1 and 2
    Eigen::Matrix<double, 2, 4> dN_A = shape_function_derivatives(0.0, -1.0);
    Eigen::Matrix2d Jinv_A = J_A.inverse();
    Eigen::Matrix<double, 2, 4> dN_dx_A;
    dN_dx_A.row(0) = Jinv_A(0, 0) * dN_A.row(0) + Jinv_A(0, 1) * dN_A.row(1);
    dN_dx_A.row(1) = Jinv_A(1, 0) * dN_A.row(0) + Jinv_A(1, 1) * dN_A.row(1);
    Eigen::Vector4d N_A = shape_functions(0.0, -1.0);

    // At point B (0,+1): Uses nodes 3 and 4
    Eigen::Matrix<double, 2, 4> dN_B = shape_function_derivatives(0.0, +1.0);
    Eigen::Matrix2d Jinv_B = J_B.inverse();
    Eigen::Matrix<double, 2, 4> dN_dx_B;
    dN_dx_B.row(0) = Jinv_B(0, 0) * dN_B.row(0) + Jinv_B(0, 1) * dN_B.row(1);
    dN_dx_B.row(1) = Jinv_B(1, 0) * dN_B.row(0) + Jinv_B(1, 1) * dN_B.row(1);
    Eigen::Vector4d N_B = shape_functions(0.0, +1.0);

    // At point C (-1,0): Uses nodes 1 and 4
    Eigen::Matrix<double, 2, 4> dN_C = shape_function_derivatives(-1.0, 0.0);
    Eigen::Matrix2d Jinv_C = J_C.inverse();
    Eigen::Matrix<double, 2, 4> dN_dx_C;
    dN_dx_C.row(0) = Jinv_C(0, 0) * dN_C.row(0) + Jinv_C(0, 1) * dN_C.row(1);
    dN_dx_C.row(1) = Jinv_C(1, 0) * dN_C.row(0) + Jinv_C(1, 1) * dN_C.row(1);
    Eigen::Vector4d N_C = shape_functions(-1.0, 0.0);

    // At point D (+1,0): Uses nodes 2 and 3
    Eigen::Matrix<double, 2, 4> dN_D = shape_function_derivatives(+1.0, 0.0);
    Eigen::Matrix2d Jinv_D = J_D.inverse();
    Eigen::Matrix<double, 2, 4> dN_dx_D;
    dN_dx_D.row(0) = Jinv_D(0, 0) * dN_D.row(0) + Jinv_D(0, 1) * dN_D.row(1);
    dN_dx_D.row(1) = Jinv_D(1, 0) * dN_D.row(0) + Jinv_D(1, 1) * dN_D.row(1);
    Eigen::Vector4d N_D = shape_functions(+1.0, 0.0);

    // Interpolation factors
    double alpha_AB = 0.5 * (1.0 + eta);  // Interpolates from A (eta=-1) to B (eta=+1)
    double alpha_CD = 0.5 * (1.0 + xi);   // Interpolates from C (xi=-1) to D (xi=+1)

    // Build B-matrix using MITC4 interpolation
    for (int i = 0; i < 4; ++i) {
        int col_w = 3 * i;
        int col_thx = 3 * i + 1;
        int col_thy = 3 * i + 2;

        // γxz = ∂w/∂x + θy (interpolated from A and B)
        double dN_dx_i_A = dN_dx_A(0, i);
        double dN_dx_i_B = dN_dx_B(0, i);
        double N_i_A = N_A(i);
        double N_i_B = N_B(i);

        double Bxz_w = (1.0 - alpha_AB) * dN_dx_i_A + alpha_AB * dN_dx_i_B;
        double Bxz_thy = (1.0 - alpha_AB) * N_i_A + alpha_AB * N_i_B;

        B(0, col_w) = Bxz_w;
        B(0, col_thy) = Bxz_thy;

        // γyz = ∂w/∂y - θx (interpolated from C and D)
        double dN_dy_i_C = dN_dx_C(1, i);
        double dN_dy_i_D = dN_dx_D(1, i);
        double N_i_C = N_C(i);
        double N_i_D = N_D(i);

        double Byz_w = (1.0 - alpha_CD) * dN_dy_i_C + alpha_CD * dN_dy_i_D;
        double Byz_thx = -((1.0 - alpha_CD) * N_i_C + alpha_CD * N_i_D);

        B(1, col_w) = Byz_w;
        B(1, col_thx) = Byz_thx;
    }

    return B;
}

Eigen::Matrix<double, 12, 12> PlateElement::local_stiffness_matrix() const {
    // Compute stiffness matrix using 2x2 Gauss quadrature
    Eigen::Matrix<double, 12, 12> K = Eigen::Matrix<double, 12, 12>::Zero();

    // Gauss points and weights for 2x2 quadrature
    const double gp = 1.0 / std::sqrt(3.0);
    double gauss_pts[4][2] = {{-gp, -gp}, {gp, -gp}, {gp, gp}, {-gp, gp}};
    double weight = 1.0;  // Weight for 2x2 Gauss

    Eigen::Matrix3d D_b = bending_D_matrix();
    Eigen::Matrix2d D_s = shear_D_matrix();

    for (int i = 0; i < 4; ++i) {
        double xi = gauss_pts[i][0];
        double eta = gauss_pts[i][1];

        Eigen::Matrix2d J = jacobian(xi, eta);
        double detJ = J.determinant();

        if (detJ <= 0) {
            throw std::runtime_error("PlateElement: Negative Jacobian determinant (distorted element)");
        }

        // Bending contribution
        Eigen::Matrix<double, 3, 12> B_b = bending_B_matrix(xi, eta);
        K += weight * detJ * B_b.transpose() * D_b * B_b;

        // Shear contribution (MITC4)
        Eigen::Matrix<double, 2, 12> B_s = shear_B_matrix_mitc4(xi, eta);
        K += weight * detJ * B_s.transpose() * D_s * B_s;
    }

    return K;
}

Eigen::Matrix<double, 24, 24> PlateElement::expand_to_full_dofs(
    const Eigen::Matrix<double, 12, 12>& K_bend) const {
    // Expand 12x12 bending matrix to 24x24 full DOF matrix
    // Bending DOFs [w, θx, θy] map to full DOFs [u, v, w, θx, θy, θz]
    // Indices: w->2, θx->3, θy->4

    Eigen::Matrix<double, 24, 24> K_full = Eigen::Matrix<double, 24, 24>::Zero();

    // Mapping from bending DOFs to full DOFs
    // For node i: bending DOFs [3*i, 3*i+1, 3*i+2] -> full DOFs [6*i+2, 6*i+3, 6*i+4]
    for (int node_i = 0; node_i < 4; ++node_i) {
        for (int node_j = 0; node_j < 4; ++node_j) {
            // DOF indices for bending matrix
            int bi_w = 3 * node_i;
            int bi_thx = 3 * node_i + 1;
            int bi_thy = 3 * node_i + 2;

            int bj_w = 3 * node_j;
            int bj_thx = 3 * node_j + 1;
            int bj_thy = 3 * node_j + 2;

            // DOF indices for full matrix
            int fi_w = 6 * node_i + 2;    // w -> UZ
            int fi_thx = 6 * node_i + 3;  // θx -> RX
            int fi_thy = 6 * node_i + 4;  // θy -> RY

            int fj_w = 6 * node_j + 2;
            int fj_thx = 6 * node_j + 3;
            int fj_thy = 6 * node_j + 4;

            // Copy 3x3 submatrix
            K_full(fi_w, fj_w) = K_bend(bi_w, bj_w);
            K_full(fi_w, fj_thx) = K_bend(bi_w, bj_thx);
            K_full(fi_w, fj_thy) = K_bend(bi_w, bj_thy);

            K_full(fi_thx, fj_w) = K_bend(bi_thx, bj_w);
            K_full(fi_thx, fj_thx) = K_bend(bi_thx, bj_thx);
            K_full(fi_thx, fj_thy) = K_bend(bi_thx, bj_thy);

            K_full(fi_thy, fj_w) = K_bend(bi_thy, bj_w);
            K_full(fi_thy, fj_thx) = K_bend(bi_thy, bj_thx);
            K_full(fi_thy, fj_thy) = K_bend(bi_thy, bj_thy);
        }
    }

    return K_full;
}

Eigen::Matrix<double, 24, 24> PlateElement::transformation_matrix() const {
    // Build transformation matrix for 4 nodes, 6 DOFs each
    // T transforms from global to local coordinates
    // For each node: [u, v, w, θx, θy, θz]_local = T_node * [u, v, w, θx, θy, θz]_global

    Eigen::Matrix<double, 24, 24> T = Eigen::Matrix<double, 24, 24>::Zero();

    // R^T transforms from global to local
    Eigen::Matrix3d R_T = rotation_matrix.transpose();

    // Apply to each node (4 nodes, 2 blocks of 3 DOFs each)
    for (int i = 0; i < 4; ++i) {
        // Translations [u, v, w]
        T.block<3, 3>(6 * i, 6 * i) = R_T;
        // Rotations [θx, θy, θz]
        T.block<3, 3>(6 * i + 3, 6 * i + 3) = R_T;
    }

    return T;
}

Eigen::Matrix<double, 24, 24> PlateElement::global_stiffness_matrix() const {
    // Compute local stiffness matrix (12x12 for bending DOFs)
    Eigen::Matrix<double, 12, 12> K_local = local_stiffness_matrix();

    // Expand to 24x24 for full DOFs
    Eigen::Matrix<double, 24, 24> K_local_full = expand_to_full_dofs(K_local);

    // Transform to global coordinates: K_global = T^T * K_local * T
    Eigen::Matrix<double, 24, 24> T = transformation_matrix();

    return T.transpose() * K_local_full * T;
}

Eigen::Matrix<double, 24, 24> PlateElement::global_mass_matrix() const {
    // Lumped mass matrix
    // Each node gets 1/4 of total mass
    double rho = material->rho;
    double t = thickness;
    double A = area();

    double mass_per_node = rho * t * A / 4.0;
    double inertia_per_node = rho * t * t * t / 12.0 * A / 4.0;

    Eigen::Matrix<double, 24, 24> M = Eigen::Matrix<double, 24, 24>::Zero();

    for (int i = 0; i < 4; ++i) {
        // Translational mass
        M(6 * i + 0, 6 * i + 0) = mass_per_node;  // u
        M(6 * i + 1, 6 * i + 1) = mass_per_node;  // v
        M(6 * i + 2, 6 * i + 2) = mass_per_node;  // w

        // Rotational inertia
        M(6 * i + 3, 6 * i + 3) = inertia_per_node;  // θx
        M(6 * i + 4, 6 * i + 4) = inertia_per_node;  // θy
        M(6 * i + 5, 6 * i + 5) = inertia_per_node;  // θz
    }

    return M;
}

} // namespace grillex
