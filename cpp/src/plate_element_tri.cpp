#include "grillex/plate_element_tri.hpp"
#include <cmath>
#include <stdexcept>

namespace grillex {

// 3-point Gauss quadrature for triangles
// Integration points in area coordinates
const double PlateElementTri::gauss_L1_3pt[3] = {0.5, 0.5, 0.0};
const double PlateElementTri::gauss_L2_3pt[3] = {0.0, 0.5, 0.5};
// L3 = 1 - L1 - L2
const double PlateElementTri::gauss_wt_3pt = 1.0 / 3.0;  // Weight for each point

PlateElementTri::PlateElementTri(int id, Node* n1, Node* n2, Node* n3,
                                 double thickness, Material* material)
    : id(id), nodes{n1, n2, n3}, thickness(thickness), material(material) {

    if (!n1 || !n2 || !n3) {
        throw std::invalid_argument("PlateElementTri: All nodes must be non-null");
    }
    if (thickness <= 0) {
        throw std::invalid_argument("PlateElementTri: Thickness must be positive");
    }
    if (!material) {
        throw std::invalid_argument("PlateElementTri: Material must be non-null");
    }

    compute_local_axes();
    build_rotation_matrix();
    compute_geometry_params();
}

void PlateElementTri::compute_local_axes() {
    Eigen::Vector3d p1(nodes[0]->x, nodes[0]->y, nodes[0]->z);
    Eigen::Vector3d p2(nodes[1]->x, nodes[1]->y, nodes[1]->z);
    Eigen::Vector3d p3(nodes[2]->x, nodes[2]->y, nodes[2]->z);

    Eigen::Vector3d v12 = p2 - p1;
    Eigen::Vector3d v13 = p3 - p1;

    // z-axis: normal to plate
    z_axis = v12.cross(v13);
    double norm_z = z_axis.norm();
    if (norm_z < 1e-10) {
        throw std::runtime_error("PlateElementTri: Degenerate element (nodes are collinear)");
    }
    z_axis /= norm_z;

    // x-axis: along edge 1-2
    x_axis = v12.normalized();

    // y-axis: completes right-hand system
    y_axis = z_axis.cross(x_axis);
}

void PlateElementTri::build_rotation_matrix() {
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
}

Eigen::Vector3d PlateElementTri::to_local(const Eigen::Vector3d& global) const {
    return rotation_matrix.transpose() * global;
}

Eigen::Vector3d PlateElementTri::to_global(const Eigen::Vector3d& local) const {
    return rotation_matrix * local;
}

Eigen::Matrix<double, 3, 2> PlateElementTri::local_node_coords() const {
    Eigen::Vector3d p1(nodes[0]->x, nodes[0]->y, nodes[0]->z);

    Eigen::Matrix<double, 3, 2> coords;
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d pi(nodes[i]->x, nodes[i]->y, nodes[i]->z);
        Eigen::Vector3d local = to_local(pi - p1);
        coords(i, 0) = local(0);
        coords(i, 1) = local(1);
    }

    return coords;
}

void PlateElementTri::compute_geometry_params() {
    auto xy = local_node_coords();

    double x1 = xy(0, 0), y1 = xy(0, 1);
    double x2 = xy(1, 0), y2 = xy(1, 1);
    double x3 = xy(2, 0), y3 = xy(2, 1);

    // Edge vectors
    double dx12 = x2 - x1, dy12 = y2 - y1;
    double dx23 = x3 - x2, dy23 = y3 - y2;
    double dx31 = x1 - x3, dy31 = y1 - y3;

    // Edge lengths
    L12_ = std::sqrt(dx12 * dx12 + dy12 * dy12);
    L23_ = std::sqrt(dx23 * dx23 + dy23 * dy23);
    L31_ = std::sqrt(dx31 * dx31 + dy31 * dy31);

    // Direction cosines
    c12_ = dx12 / L12_;
    s12_ = dy12 / L12_;
    c23_ = dx23 / L23_;
    s23_ = dy23 / L23_;
    c31_ = dx31 / L31_;
    s31_ = dy31 / L31_;
}

double PlateElementTri::area() const {
    Eigen::Vector3d p1(nodes[0]->x, nodes[0]->y, nodes[0]->z);
    Eigen::Vector3d p2(nodes[1]->x, nodes[1]->y, nodes[1]->z);
    Eigen::Vector3d p3(nodes[2]->x, nodes[2]->y, nodes[2]->z);

    Eigen::Vector3d v12 = p2 - p1;
    Eigen::Vector3d v13 = p3 - p1;

    return 0.5 * v12.cross(v13).norm();
}

Eigen::Vector3d PlateElementTri::centroid() const {
    return Eigen::Vector3d(
        (nodes[0]->x + nodes[1]->x + nodes[2]->x) / 3.0,
        (nodes[0]->y + nodes[1]->y + nodes[2]->y) / 3.0,
        (nodes[0]->z + nodes[1]->z + nodes[2]->z) / 3.0
    );
}

Eigen::Matrix3d PlateElementTri::bending_D_matrix() const {
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

// DKT shape function parameters
// Following Batoz, Bathe, Ho (1980) notation

Eigen::Matrix<double, 9, 1> PlateElementTri::Hx_functions(double L1, double L2, double L3) const {
    // Hx shape functions for rotation βx = -∂w/∂y
    // DOF ordering: [w1, θx1, θy1, w2, θx2, θy2, w3, θx3, θy3]

    Eigen::Matrix<double, 9, 1> Hx;

    // Auxiliary parameters
    double a4 = -6.0 / L12_;
    double a5 = -6.0 / L23_;
    double a6 = -6.0 / L31_;

    double b4 = 3.0 * s12_ / L12_;
    double b5 = 3.0 * s23_ / L23_;
    double b6 = 3.0 * s31_ / L31_;

    double c4 = 3.0 * c12_ / L12_;
    double c5 = 3.0 * c23_ / L23_;
    double c6 = 3.0 * c31_ / L31_;

    double d4 = -1.5 * s12_ * s12_ / L12_;
    double d5 = -1.5 * s23_ * s23_ / L23_;
    double d6 = -1.5 * s31_ * s31_ / L31_;

    double e4 = -1.5 * s12_ * c12_ / L12_;
    double e5 = -1.5 * s23_ * c23_ / L23_;
    double e6 = -1.5 * s31_ * c31_ / L31_;

    // P functions (quadratic terms)
    double P4 = (1.0 - 2.0 * L1) * (a4 * L2 + a6 * L3) - (a4 - a6) * L2 * L3;
    double P5 = (1.0 - 2.0 * L2) * (a5 * L3 + a4 * L1) - (a5 - a4) * L3 * L1;
    double P6 = (1.0 - 2.0 * L3) * (a6 * L1 + a5 * L2) - (a6 - a5) * L1 * L2;

    // Q functions
    double Q4 = (1.0 - 2.0 * L1) * (b4 * L2 - b6 * L3) - (b4 + b6) * L2 * L3;
    double Q5 = (1.0 - 2.0 * L2) * (b5 * L3 - b4 * L1) - (b5 + b4) * L3 * L1;
    double Q6 = (1.0 - 2.0 * L3) * (b6 * L1 - b5 * L2) - (b6 + b5) * L1 * L2;

    // T functions
    double T4 = (1.0 - 2.0 * L1) * (c4 * L2 - c6 * L3) - (c4 + c6) * L2 * L3;
    double T5 = (1.0 - 2.0 * L2) * (c5 * L3 - c4 * L1) - (c5 + c4) * L3 * L1;
    double T6 = (1.0 - 2.0 * L3) * (c6 * L1 - c5 * L2) - (c6 + c5) * L1 * L2;

    // R functions
    double R4 = (1.0 - 2.0 * L1) * (d4 * L2 + d6 * L3) - (d4 - d6) * L2 * L3;
    double R5 = (1.0 - 2.0 * L2) * (d5 * L3 + d4 * L1) - (d5 - d4) * L3 * L1;
    double R6 = (1.0 - 2.0 * L3) * (d6 * L1 + d5 * L2) - (d6 - d5) * L1 * L2;

    // S functions
    double S4 = (1.0 - 2.0 * L1) * (e4 * L2 + e6 * L3) - (e4 - e6) * L2 * L3;
    double S5 = (1.0 - 2.0 * L2) * (e5 * L3 + e4 * L1) - (e5 - e4) * L3 * L1;
    double S6 = (1.0 - 2.0 * L3) * (e6 * L1 + e5 * L2) - (e6 - e5) * L1 * L2;

    // Hx shape functions
    // Node 1
    Hx(0) = P6 - P4;                                    // w1
    Hx(1) = L1 + Q6 - Q4 + R6 - R4;                    // θx1
    Hx(2) = T6 - T4 + S6 - S4;                         // θy1

    // Node 2
    Hx(3) = P4 - P5;                                    // w2
    Hx(4) = Q4 - Q5 + R4 - R5;                         // θx2
    Hx(5) = L2 + T4 - T5 + S4 - S5;                    // θy2

    // Node 3
    Hx(6) = P5 - P6;                                    // w3
    Hx(7) = Q5 - Q6 + R5 - R6;                         // θx3
    Hx(8) = T5 - T6 + S5 - S6;                         // θy3

    return Hx;
}

Eigen::Matrix<double, 9, 1> PlateElementTri::Hy_functions(double L1, double L2, double L3) const {
    // Hy shape functions for rotation βy = ∂w/∂x
    Eigen::Matrix<double, 9, 1> Hy;

    // Same auxiliary parameters
    double a4 = -6.0 / L12_;
    double a5 = -6.0 / L23_;
    double a6 = -6.0 / L31_;

    double b4 = 3.0 * s12_ / L12_;
    double b5 = 3.0 * s23_ / L23_;
    double b6 = 3.0 * s31_ / L31_;

    double c4 = 3.0 * c12_ / L12_;
    double c5 = 3.0 * c23_ / L23_;
    double c6 = 3.0 * c31_ / L31_;

    double d4 = -1.5 * c12_ * c12_ / L12_;
    double d5 = -1.5 * c23_ * c23_ / L23_;
    double d6 = -1.5 * c31_ * c31_ / L31_;

    double e4 = -1.5 * s12_ * c12_ / L12_;
    double e5 = -1.5 * s23_ * c23_ / L23_;
    double e6 = -1.5 * s31_ * c31_ / L31_;

    // P, Q, T, R, S functions (same structure as Hx)
    double P4 = (1.0 - 2.0 * L1) * (a4 * L2 + a6 * L3) - (a4 - a6) * L2 * L3;
    double P5 = (1.0 - 2.0 * L2) * (a5 * L3 + a4 * L1) - (a5 - a4) * L3 * L1;
    double P6 = (1.0 - 2.0 * L3) * (a6 * L1 + a5 * L2) - (a6 - a5) * L1 * L2;

    double Q4 = (1.0 - 2.0 * L1) * (b4 * L2 - b6 * L3) - (b4 + b6) * L2 * L3;
    double Q5 = (1.0 - 2.0 * L2) * (b5 * L3 - b4 * L1) - (b5 + b4) * L3 * L1;
    double Q6 = (1.0 - 2.0 * L3) * (b6 * L1 - b5 * L2) - (b6 + b5) * L1 * L2;

    double T4 = (1.0 - 2.0 * L1) * (c4 * L2 - c6 * L3) - (c4 + c6) * L2 * L3;
    double T5 = (1.0 - 2.0 * L2) * (c5 * L3 - c4 * L1) - (c5 + c4) * L3 * L1;
    double T6 = (1.0 - 2.0 * L3) * (c6 * L1 - c5 * L2) - (c6 + c5) * L1 * L2;

    double R4 = (1.0 - 2.0 * L1) * (d4 * L2 + d6 * L3) - (d4 - d6) * L2 * L3;
    double R5 = (1.0 - 2.0 * L2) * (d5 * L3 + d4 * L1) - (d5 - d4) * L3 * L1;
    double R6 = (1.0 - 2.0 * L3) * (d6 * L1 + d5 * L2) - (d6 - d5) * L1 * L2;

    double S4 = (1.0 - 2.0 * L1) * (e4 * L2 + e6 * L3) - (e4 - e6) * L2 * L3;
    double S5 = (1.0 - 2.0 * L2) * (e5 * L3 + e4 * L1) - (e5 - e4) * L3 * L1;
    double S6 = (1.0 - 2.0 * L3) * (e6 * L1 + e5 * L2) - (e6 - e5) * L1 * L2;

    // Hy shape functions
    // Node 1
    Hy(0) = -(P6 - P4);                                 // w1
    Hy(1) = -(Q6 - Q4) - (S6 - S4);                    // θx1
    Hy(2) = L1 - (T6 - T4) - (R6 - R4);                // θy1

    // Node 2
    Hy(3) = -(P4 - P5);                                 // w2
    Hy(4) = -(Q4 - Q5) - (S4 - S5);                    // θx2
    Hy(5) = -(T4 - T5) - (R4 - R5);                    // θy2

    // Node 3
    Hy(6) = -(P5 - P6);                                 // w3
    Hy(7) = L3 - (Q5 - Q6) - (S5 - S6);                // θx3
    Hy(8) = -(T5 - T6) - (R5 - R6);                    // θy3

    return Hy;
}

Eigen::Matrix<double, 3, 9> PlateElementTri::bending_B_matrix(double L1, double L2, double L3) const {
    // B-matrix relates curvatures to nodal DOFs
    // κ = [κxx, κyy, 2κxy]^T = B * u
    // u = [w1, θx1, θy1, w2, θx2, θy2, w3, θx3, θy3]^T

    auto xy = local_node_coords();
    double x1 = xy(0, 0), y1 = xy(0, 1);
    double x2 = xy(1, 0), y2 = xy(1, 1);
    double x3 = xy(2, 0), y3 = xy(2, 1);

    double A = area();

    // Derivatives of area coordinates with respect to x and y
    double dL1_dx = (y2 - y3) / (2.0 * A);
    double dL2_dx = (y3 - y1) / (2.0 * A);
    double dL3_dx = (y1 - y2) / (2.0 * A);

    double dL1_dy = (x3 - x2) / (2.0 * A);
    double dL2_dy = (x1 - x3) / (2.0 * A);
    double dL3_dy = (x2 - x1) / (2.0 * A);

    // Compute Hx and Hy and their derivatives
    // Use numerical differentiation for simplicity
    const double eps = 1e-8;

    Eigen::Matrix<double, 9, 1> Hx = Hx_functions(L1, L2, L3);
    Eigen::Matrix<double, 9, 1> Hy = Hy_functions(L1, L2, L3);

    // dHx/dL1, dHx/dL2, dHx/dL3
    Eigen::Matrix<double, 9, 1> dHx_dL1 = (Hx_functions(L1 + eps, L2, L3 - eps) - Hx_functions(L1 - eps, L2, L3 + eps)) / (2.0 * eps);
    Eigen::Matrix<double, 9, 1> dHx_dL2 = (Hx_functions(L1, L2 + eps, L3 - eps) - Hx_functions(L1, L2 - eps, L3 + eps)) / (2.0 * eps);

    Eigen::Matrix<double, 9, 1> dHy_dL1 = (Hy_functions(L1 + eps, L2, L3 - eps) - Hy_functions(L1 - eps, L2, L3 + eps)) / (2.0 * eps);
    Eigen::Matrix<double, 9, 1> dHy_dL2 = (Hy_functions(L1, L2 + eps, L3 - eps) - Hy_functions(L1, L2 - eps, L3 + eps)) / (2.0 * eps);

    // dHx/dx = dHx/dL1 * dL1/dx + dHx/dL2 * dL2/dx + dHx/dL3 * dL3/dx
    // Since L3 = 1 - L1 - L2, we have dL3/dx = -dL1/dx - dL2/dx
    Eigen::Matrix<double, 9, 1> dHx_dx = dHx_dL1 * dL1_dx + dHx_dL2 * dL2_dx;
    Eigen::Matrix<double, 9, 1> dHx_dy = dHx_dL1 * dL1_dy + dHx_dL2 * dL2_dy;
    Eigen::Matrix<double, 9, 1> dHy_dx = dHy_dL1 * dL1_dx + dHy_dL2 * dL2_dx;
    Eigen::Matrix<double, 9, 1> dHy_dy = dHy_dL1 * dL1_dy + dHy_dL2 * dL2_dy;

    // B-matrix
    // κxx = -∂βy/∂x = -dHy/dx
    // κyy = ∂βx/∂y = dHx/dy
    // 2κxy = ∂βx/∂x - ∂βy/∂y = dHx/dx - dHy/dy

    Eigen::Matrix<double, 3, 9> B;
    B.row(0) = -dHy_dx.transpose();           // κxx
    B.row(1) = dHx_dy.transpose();            // κyy
    B.row(2) = dHx_dx.transpose() - dHy_dy.transpose();  // 2κxy

    return B;
}

Eigen::Matrix<double, 9, 9> PlateElementTri::local_stiffness_matrix() const {
    Eigen::Matrix<double, 9, 9> K = Eigen::Matrix<double, 9, 9>::Zero();

    Eigen::Matrix3d D = bending_D_matrix();
    double A = area();

    // 3-point Gauss quadrature over triangle
    for (int i = 0; i < 3; ++i) {
        double L1 = gauss_L1_3pt[i];
        double L2 = gauss_L2_3pt[i];
        double L3 = 1.0 - L1 - L2;

        Eigen::Matrix<double, 3, 9> B = bending_B_matrix(L1, L2, L3);

        // Weight = (1/3) * 2 * A (triangle area factor)
        double w = gauss_wt_3pt * 2.0 * A;

        K += w * B.transpose() * D * B;
    }

    return K;
}

Eigen::Matrix<double, 18, 18> PlateElementTri::expand_to_full_dofs(
    const Eigen::Matrix<double, 9, 9>& K_bend) const {

    Eigen::Matrix<double, 18, 18> K_full = Eigen::Matrix<double, 18, 18>::Zero();

    // Mapping from bending DOFs to full DOFs
    // Bending DOFs [w, θx, θy] -> Full DOFs [UZ, RX, RY] = [2, 3, 4]
    for (int node_i = 0; node_i < 3; ++node_i) {
        for (int node_j = 0; node_j < 3; ++node_j) {
            int bi_w = 3 * node_i;
            int bi_thx = 3 * node_i + 1;
            int bi_thy = 3 * node_i + 2;

            int bj_w = 3 * node_j;
            int bj_thx = 3 * node_j + 1;
            int bj_thy = 3 * node_j + 2;

            int fi_w = 6 * node_i + 2;
            int fi_thx = 6 * node_i + 3;
            int fi_thy = 6 * node_i + 4;

            int fj_w = 6 * node_j + 2;
            int fj_thx = 6 * node_j + 3;
            int fj_thy = 6 * node_j + 4;

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

Eigen::Matrix<double, 18, 18> PlateElementTri::transformation_matrix() const {
    Eigen::Matrix<double, 18, 18> T = Eigen::Matrix<double, 18, 18>::Zero();
    Eigen::Matrix3d R_T = rotation_matrix.transpose();

    for (int i = 0; i < 3; ++i) {
        // Translations
        T.block<3, 3>(6 * i, 6 * i) = R_T;
        // Rotations
        T.block<3, 3>(6 * i + 3, 6 * i + 3) = R_T;
    }

    return T;
}

Eigen::Matrix<double, 18, 18> PlateElementTri::global_stiffness_matrix() const {
    Eigen::Matrix<double, 9, 9> K_local = local_stiffness_matrix();
    Eigen::Matrix<double, 18, 18> K_local_full = expand_to_full_dofs(K_local);
    Eigen::Matrix<double, 18, 18> T = transformation_matrix();

    return T.transpose() * K_local_full * T;
}

Eigen::Matrix<double, 18, 18> PlateElementTri::global_mass_matrix() const {
    double rho = material->rho;
    double t = thickness;
    double A = area();

    double mass_per_node = rho * t * A / 3.0;
    double inertia_per_node = rho * t * t * t / 12.0 * A / 3.0;

    Eigen::Matrix<double, 18, 18> M = Eigen::Matrix<double, 18, 18>::Zero();

    for (int i = 0; i < 3; ++i) {
        // Translational mass
        M(6 * i + 0, 6 * i + 0) = mass_per_node;
        M(6 * i + 1, 6 * i + 1) = mass_per_node;
        M(6 * i + 2, 6 * i + 2) = mass_per_node;

        // Rotational inertia
        M(6 * i + 3, 6 * i + 3) = inertia_per_node;
        M(6 * i + 4, 6 * i + 4) = inertia_per_node;
        M(6 * i + 5, 6 * i + 5) = inertia_per_node;
    }

    return M;
}

} // namespace grillex
