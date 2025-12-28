#include "grillex/plate_element_8.hpp"
#include <cmath>
#include <stdexcept>

namespace grillex {

// 3×3 Gauss quadrature points and weights
const double PlateElement8::gauss_pts_3[3] = {-std::sqrt(0.6), 0.0, std::sqrt(0.6)};
const double PlateElement8::gauss_wts_3[3] = {5.0/9.0, 8.0/9.0, 5.0/9.0};

PlateElement8::PlateElement8(int id, Node* n1, Node* n2, Node* n3, Node* n4,
                             Node* n5, Node* n6, Node* n7, Node* n8,
                             double thickness, Material* material)
    : id(id), nodes{n1, n2, n3, n4, n5, n6, n7, n8},
      thickness(thickness), material(material) {

    for (int i = 0; i < 8; ++i) {
        if (!nodes[i]) {
            throw std::invalid_argument("PlateElement8: All nodes must be non-null");
        }
    }
    if (thickness <= 0) {
        throw std::invalid_argument("PlateElement8: Thickness must be positive");
    }
    if (!material) {
        throw std::invalid_argument("PlateElement8: Material must be non-null");
    }

    compute_local_axes();
    build_rotation_matrix();
}

void PlateElement8::compute_local_axes() {
    // Get corner node positions
    Eigen::Vector3d p1(nodes[0]->x, nodes[0]->y, nodes[0]->z);
    Eigen::Vector3d p2(nodes[1]->x, nodes[1]->y, nodes[1]->z);
    Eigen::Vector3d p4(nodes[3]->x, nodes[3]->y, nodes[3]->z);

    // x-axis: from node 1 to node 2
    Eigen::Vector3d v12 = p2 - p1;
    Eigen::Vector3d v14 = p4 - p1;

    // z-axis: normal to plate
    z_axis = v12.cross(v14);
    double norm_z = z_axis.norm();
    if (norm_z < 1e-10) {
        throw std::runtime_error("PlateElement8: Degenerate element (nodes are collinear)");
    }
    z_axis /= norm_z;

    // x-axis: along edge 1-2, normalized
    x_axis = v12.normalized();

    // y-axis: completes right-hand system
    y_axis = z_axis.cross(x_axis);
}

void PlateElement8::build_rotation_matrix() {
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
}

Eigen::Vector3d PlateElement8::to_local(const Eigen::Vector3d& global) const {
    return rotation_matrix.transpose() * global;
}

Eigen::Vector3d PlateElement8::to_global(const Eigen::Vector3d& local) const {
    return rotation_matrix * local;
}

Eigen::Matrix<double, 8, 1> PlateElement8::shape_functions(double xi, double eta) const {
    // 8-node serendipity shape functions
    Eigen::Matrix<double, 8, 1> N;

    // Corner nodes (1-4)
    N(0) = 0.25 * (1.0 - xi) * (1.0 - eta) * (-xi - eta - 1.0);  // N1 at (-1,-1)
    N(1) = 0.25 * (1.0 + xi) * (1.0 - eta) * (xi - eta - 1.0);   // N2 at (+1,-1)
    N(2) = 0.25 * (1.0 + xi) * (1.0 + eta) * (xi + eta - 1.0);   // N3 at (+1,+1)
    N(3) = 0.25 * (1.0 - xi) * (1.0 + eta) * (-xi + eta - 1.0);  // N4 at (-1,+1)

    // Midside nodes (5-8)
    N(4) = 0.5 * (1.0 - xi * xi) * (1.0 - eta);  // N5 at (0,-1)
    N(5) = 0.5 * (1.0 + xi) * (1.0 - eta * eta); // N6 at (+1,0)
    N(6) = 0.5 * (1.0 - xi * xi) * (1.0 + eta);  // N7 at (0,+1)
    N(7) = 0.5 * (1.0 - xi) * (1.0 - eta * eta); // N8 at (-1,0)

    return N;
}

Eigen::Matrix<double, 2, 8> PlateElement8::shape_function_derivatives(double xi, double eta) const {
    // Derivatives of 8-node serendipity shape functions
    Eigen::Matrix<double, 2, 8> dN;

    // dN/dxi for corner nodes
    dN(0, 0) = 0.25 * (1.0 - eta) * (2.0 * xi + eta);
    dN(0, 1) = 0.25 * (1.0 - eta) * (2.0 * xi - eta);
    dN(0, 2) = 0.25 * (1.0 + eta) * (2.0 * xi + eta);
    dN(0, 3) = 0.25 * (1.0 + eta) * (2.0 * xi - eta);

    // dN/dxi for midside nodes
    dN(0, 4) = -xi * (1.0 - eta);
    dN(0, 5) = 0.5 * (1.0 - eta * eta);
    dN(0, 6) = -xi * (1.0 + eta);
    dN(0, 7) = -0.5 * (1.0 - eta * eta);

    // dN/deta for corner nodes
    dN(1, 0) = 0.25 * (1.0 - xi) * (xi + 2.0 * eta);
    dN(1, 1) = 0.25 * (1.0 + xi) * (-xi + 2.0 * eta);
    dN(1, 2) = 0.25 * (1.0 + xi) * (xi + 2.0 * eta);
    dN(1, 3) = 0.25 * (1.0 - xi) * (-xi + 2.0 * eta);

    // dN/deta for midside nodes
    dN(1, 4) = -0.5 * (1.0 - xi * xi);
    dN(1, 5) = -eta * (1.0 + xi);
    dN(1, 6) = 0.5 * (1.0 - xi * xi);
    dN(1, 7) = -eta * (1.0 - xi);

    return dN;
}

Eigen::Matrix<double, 8, 2> PlateElement8::local_node_coords() const {
    Eigen::Vector3d p1(nodes[0]->x, nodes[0]->y, nodes[0]->z);

    Eigen::Matrix<double, 8, 2> coords;
    for (int i = 0; i < 8; ++i) {
        Eigen::Vector3d pi(nodes[i]->x, nodes[i]->y, nodes[i]->z);
        Eigen::Vector3d local = to_local(pi - p1);
        coords(i, 0) = local(0);
        coords(i, 1) = local(1);
    }

    return coords;
}

Eigen::Matrix2d PlateElement8::jacobian(double xi, double eta) const {
    auto dN = shape_function_derivatives(xi, eta);
    auto xy = local_node_coords();
    return dN * xy;
}

double PlateElement8::area() const {
    double A = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double xi = gauss_pts_3[i];
            double eta = gauss_pts_3[j];
            double w = gauss_wts_3[i] * gauss_wts_3[j];
            Eigen::Matrix2d J = jacobian(xi, eta);
            A += w * J.determinant();
        }
    }
    return A;
}

Eigen::Vector3d PlateElement8::centroid() const {
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (int i = 0; i < 8; ++i) {
        c(0) += nodes[i]->x;
        c(1) += nodes[i]->y;
        c(2) += nodes[i]->z;
    }
    return c / 8.0;
}

Eigen::Matrix3d PlateElement8::bending_D_matrix() const {
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

Eigen::Matrix2d PlateElement8::shear_D_matrix() const {
    double G = material->G;
    double t = thickness;
    double kappa = 5.0 / 6.0;

    return kappa * G * t * Eigen::Matrix2d::Identity();
}

Eigen::Matrix<double, 3, 24> PlateElement8::bending_B_matrix(double xi, double eta) const {
    auto dN = shape_function_derivatives(xi, eta);
    Eigen::Matrix2d J = jacobian(xi, eta);
    Eigen::Matrix2d Jinv = J.inverse();

    // Transform derivatives to physical coordinates
    Eigen::Matrix<double, 2, 8> dN_dx;
    dN_dx.row(0) = Jinv(0, 0) * dN.row(0) + Jinv(0, 1) * dN.row(1);
    dN_dx.row(1) = Jinv(1, 0) * dN.row(0) + Jinv(1, 1) * dN.row(1);

    // Build B-matrix (3 rows x 24 columns)
    // DOFs per node: [w, θx, θy]
    Eigen::Matrix<double, 3, 24> B = Eigen::Matrix<double, 3, 24>::Zero();

    for (int i = 0; i < 8; ++i) {
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

Eigen::Matrix<double, 2, 24> PlateElement8::shear_B_matrix(double xi, double eta) const {
    // Simplified shear B-matrix for 8-node element
    // Using assumed strain approach with selective reduced integration
    auto dN = shape_function_derivatives(xi, eta);
    Eigen::Matrix2d J = jacobian(xi, eta);
    Eigen::Matrix2d Jinv = J.inverse();

    Eigen::Vector<double, 8> N = shape_functions(xi, eta);

    // Transform derivatives to physical coordinates
    Eigen::Matrix<double, 2, 8> dN_dx;
    dN_dx.row(0) = Jinv(0, 0) * dN.row(0) + Jinv(0, 1) * dN.row(1);
    dN_dx.row(1) = Jinv(1, 0) * dN.row(0) + Jinv(1, 1) * dN.row(1);

    Eigen::Matrix<double, 2, 24> B = Eigen::Matrix<double, 2, 24>::Zero();

    for (int i = 0; i < 8; ++i) {
        int col_w = 3 * i;
        int col_thx = 3 * i + 1;
        int col_thy = 3 * i + 2;

        // γxz = ∂w/∂x + θy
        B(0, col_w) = dN_dx(0, i);
        B(0, col_thy) = N(i);

        // γyz = ∂w/∂y - θx
        B(1, col_w) = dN_dx(1, i);
        B(1, col_thx) = -N(i);
    }

    return B;
}

Eigen::Matrix<double, 24, 24> PlateElement8::local_stiffness_matrix() const {
    Eigen::Matrix<double, 24, 24> K = Eigen::Matrix<double, 24, 24>::Zero();

    Eigen::Matrix3d D_b = bending_D_matrix();
    Eigen::Matrix2d D_s = shear_D_matrix();

    // 3×3 Gauss quadrature
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double xi = gauss_pts_3[i];
            double eta = gauss_pts_3[j];
            double w = gauss_wts_3[i] * gauss_wts_3[j];

            Eigen::Matrix2d J = jacobian(xi, eta);
            double detJ = J.determinant();

            if (detJ <= 0) {
                throw std::runtime_error("PlateElement8: Negative Jacobian determinant");
            }

            // Bending contribution
            Eigen::Matrix<double, 3, 24> B_b = bending_B_matrix(xi, eta);
            K += w * detJ * B_b.transpose() * D_b * B_b;

            // Shear contribution
            Eigen::Matrix<double, 2, 24> B_s = shear_B_matrix(xi, eta);
            K += w * detJ * B_s.transpose() * D_s * B_s;
        }
    }

    return K;
}

Eigen::Matrix<double, 48, 48> PlateElement8::expand_to_full_dofs(
    const Eigen::Matrix<double, 24, 24>& K_bend) const {

    Eigen::Matrix<double, 48, 48> K_full = Eigen::Matrix<double, 48, 48>::Zero();

    for (int node_i = 0; node_i < 8; ++node_i) {
        for (int node_j = 0; node_j < 8; ++node_j) {
            // Bending DOF indices
            int bi_w = 3 * node_i;
            int bi_thx = 3 * node_i + 1;
            int bi_thy = 3 * node_i + 2;

            int bj_w = 3 * node_j;
            int bj_thx = 3 * node_j + 1;
            int bj_thy = 3 * node_j + 2;

            // Full DOF indices
            int fi_w = 6 * node_i + 2;
            int fi_thx = 6 * node_i + 3;
            int fi_thy = 6 * node_i + 4;

            int fj_w = 6 * node_j + 2;
            int fj_thx = 6 * node_j + 3;
            int fj_thy = 6 * node_j + 4;

            // Copy 3×3 submatrix
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

Eigen::Matrix<double, 48, 48> PlateElement8::transformation_matrix() const {
    Eigen::Matrix<double, 48, 48> T = Eigen::Matrix<double, 48, 48>::Zero();
    Eigen::Matrix3d R_T = rotation_matrix.transpose();

    for (int i = 0; i < 8; ++i) {
        // Translations
        T.block<3, 3>(6 * i, 6 * i) = R_T;
        // Rotations
        T.block<3, 3>(6 * i + 3, 6 * i + 3) = R_T;
    }

    return T;
}

Eigen::Matrix<double, 48, 48> PlateElement8::global_stiffness_matrix() const {
    Eigen::Matrix<double, 24, 24> K_local = local_stiffness_matrix();
    Eigen::Matrix<double, 48, 48> K_local_full = expand_to_full_dofs(K_local);
    Eigen::Matrix<double, 48, 48> T = transformation_matrix();

    return T.transpose() * K_local_full * T;
}

Eigen::Matrix<double, 48, 48> PlateElement8::global_mass_matrix() const {
    double rho = material->rho;
    double t = thickness;
    double A = area();

    double mass_per_node = rho * t * A / 8.0;
    double inertia_per_node = rho * t * t * t / 12.0 * A / 8.0;

    Eigen::Matrix<double, 48, 48> M = Eigen::Matrix<double, 48, 48>::Zero();

    for (int i = 0; i < 8; ++i) {
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
