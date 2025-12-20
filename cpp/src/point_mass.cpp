#include "grillex/point_mass.hpp"
#include <cmath>

namespace grillex {

PointMass::PointMass(int id, Node* node)
    : id(id), node(node) {}

Eigen::Matrix<double, 6, 6> PointMass::mass_matrix() const {
    Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();

    // Translational mass (diagonal)
    M(0, 0) = mass;
    M(1, 1) = mass;
    M(2, 2) = mass;

    // Rotational inertia tensor (symmetric)
    M(3, 3) = Ixx;
    M(3, 4) = Ixy;
    M(3, 5) = Ixz;

    M(4, 3) = Ixy;  // Symmetric
    M(4, 4) = Iyy;
    M(4, 5) = Iyz;

    M(5, 3) = Ixz;  // Symmetric
    M(5, 4) = Iyz;
    M(5, 5) = Izz;

    return M;
}

void PointMass::set_inertia(double ixx, double iyy, double izz) {
    Ixx = ixx;
    Iyy = iyy;
    Izz = izz;
}

void PointMass::set_products_of_inertia(double ixy, double ixz, double iyz) {
    Ixy = ixy;
    Ixz = ixz;
    Iyz = iyz;
}

void PointMass::set_full_inertia(double ixx, double iyy, double izz,
                                  double ixy, double ixz, double iyz) {
    Ixx = ixx;
    Iyy = iyy;
    Izz = izz;
    Ixy = ixy;
    Ixz = ixz;
    Iyz = iyz;
}

bool PointMass::is_valid() const {
    // Mass must be non-negative
    if (mass < 0) return false;

    // Diagonal moments must be non-negative
    if (Ixx < 0 || Iyy < 0 || Izz < 0) return false;

    // For a valid inertia tensor, we need to check positive semi-definiteness
    // A 3x3 symmetric matrix is PSD if all principal minors are >= 0

    // 1x1 minors (already checked above)

    // 2x2 minors
    double det_xy = Ixx * Iyy - Ixy * Ixy;
    double det_xz = Ixx * Izz - Ixz * Ixz;
    double det_yz = Iyy * Izz - Iyz * Iyz;

    if (det_xy < -1e-10 || det_xz < -1e-10 || det_yz < -1e-10) return false;

    // 3x3 determinant (full inertia tensor)
    double det = Ixx * (Iyy * Izz - Iyz * Iyz)
               - Ixy * (Ixy * Izz - Iyz * Ixz)
               + Ixz * (Ixy * Iyz - Iyy * Ixz);

    if (det < -1e-10) return false;

    // Triangle inequality for physical inertias
    // For a rigid body: Ixx + Iyy >= Izz, etc.
    // But we allow point masses that may not satisfy this strictly
    // (e.g., pure rotation about one axis)

    return true;
}

} // namespace grillex
