#include "grillex/beam_element.hpp"
#include <cmath>

namespace grillex {

BeamElement::BeamElement(int id, Node* node_i, Node* node_j,
                         Material* mat, Section* sec, double roll)
    : id(id), node_i(node_i), node_j(node_j), material(mat), section(sec),
      local_axes(node_i->position(), node_j->position(), roll) {
    length = compute_length();
}

double BeamElement::compute_length() const {
    Eigen::Vector3d diff = node_j->position() - node_i->position();
    return diff.norm();
}

Eigen::Matrix<double, 12, 12> BeamElement::local_stiffness_matrix() const {
    Eigen::Matrix<double, 12, 12> K = Eigen::Matrix<double, 12, 12>::Zero();

    // Extract properties
    double E = material->E;
    double G = material->G;
    double A = section->A;
    double Iy = section->Iy;
    double Iz = section->Iz;
    double J = section->J;
    double L = length;

    // Common terms
    double L2 = L * L;
    double L3 = L2 * L;

    // Axial stiffness terms (DOFs 0, 6: u_i, u_j)
    double k_axial = E * A / L;
    K(0, 0) = k_axial;
    K(0, 6) = -k_axial;
    K(6, 0) = -k_axial;
    K(6, 6) = k_axial;

    // Bending about z-axis (in-plane y, DOFs 1, 5, 7, 11: v_i, θz_i, v_j, θz_j)
    double k_bend_z = 12.0 * E * Iz / L3;
    double k_bend_z2 = 6.0 * E * Iz / L2;
    double k_bend_z3 = 4.0 * E * Iz / L;
    double k_bend_z4 = 2.0 * E * Iz / L;

    K(1, 1) = k_bend_z;       // v_i, v_i
    K(1, 5) = k_bend_z2;      // v_i, θz_i
    K(1, 7) = -k_bend_z;      // v_i, v_j
    K(1, 11) = k_bend_z2;     // v_i, θz_j

    K(5, 1) = k_bend_z2;      // θz_i, v_i
    K(5, 5) = k_bend_z3;      // θz_i, θz_i
    K(5, 7) = -k_bend_z2;     // θz_i, v_j
    K(5, 11) = k_bend_z4;     // θz_i, θz_j

    K(7, 1) = -k_bend_z;      // v_j, v_i
    K(7, 5) = -k_bend_z2;     // v_j, θz_i
    K(7, 7) = k_bend_z;       // v_j, v_j
    K(7, 11) = -k_bend_z2;    // v_j, θz_j

    K(11, 1) = k_bend_z2;     // θz_j, v_i
    K(11, 5) = k_bend_z4;     // θz_j, θz_i
    K(11, 7) = -k_bend_z2;    // θz_j, v_j
    K(11, 11) = k_bend_z3;    // θz_j, θz_j

    // Bending about y-axis (out-of-plane w, DOFs 2, 4, 8, 10: w_i, θy_i, w_j, θy_j)
    double k_bend_y = 12.0 * E * Iy / L3;
    double k_bend_y2 = 6.0 * E * Iy / L2;
    double k_bend_y3 = 4.0 * E * Iy / L;
    double k_bend_y4 = 2.0 * E * Iy / L;

    K(2, 2) = k_bend_y;       // w_i, w_i
    K(2, 4) = -k_bend_y2;     // w_i, θy_i (note sign convention)
    K(2, 8) = -k_bend_y;      // w_i, w_j
    K(2, 10) = -k_bend_y2;    // w_i, θy_j

    K(4, 2) = -k_bend_y2;     // θy_i, w_i
    K(4, 4) = k_bend_y3;      // θy_i, θy_i
    K(4, 8) = k_bend_y2;      // θy_i, w_j
    K(4, 10) = k_bend_y4;     // θy_i, θy_j

    K(8, 2) = -k_bend_y;      // w_j, w_i
    K(8, 4) = k_bend_y2;      // w_j, θy_i
    K(8, 8) = k_bend_y;       // w_j, w_j
    K(8, 10) = k_bend_y2;     // w_j, θy_j

    K(10, 2) = -k_bend_y2;    // θy_j, w_i
    K(10, 4) = k_bend_y4;     // θy_j, θy_i
    K(10, 8) = k_bend_y2;     // θy_j, w_j
    K(10, 10) = k_bend_y3;    // θy_j, θy_j

    // Torsion (DOFs 3, 9: θx_i, θx_j)
    double k_torsion = G * J / L;
    K(3, 3) = k_torsion;
    K(3, 9) = -k_torsion;
    K(9, 3) = -k_torsion;
    K(9, 9) = k_torsion;

    return K;
}

Eigen::Matrix<double, 12, 12> BeamElement::transformation_matrix() const {
    Eigen::Matrix<double, 12, 12> T = Eigen::Matrix<double, 12, 12>::Zero();

    // Get the 3x3 rotation matrix from local axes
    Eigen::Matrix3d R = local_axes.rotation_matrix;

    // Block diagonal: 4 copies of R (for translations and rotations at both ends)
    T.block<3, 3>(0, 0) = R;   // Node i translations
    T.block<3, 3>(3, 3) = R;   // Node i rotations
    T.block<3, 3>(6, 6) = R;   // Node j translations
    T.block<3, 3>(9, 9) = R;   // Node j rotations

    return T;
}

Eigen::Matrix<double, 12, 12> BeamElement::global_stiffness_matrix() const {
    Eigen::Matrix<double, 12, 12> K_local = local_stiffness_matrix();
    Eigen::Matrix<double, 12, 12> T = transformation_matrix();

    // K_global = T^T * K_local * T
    return T.transpose() * K_local * T;
}

Eigen::Matrix<double, 12, 12> BeamElement::local_mass_matrix() const {
    Eigen::Matrix<double, 12, 12> M = Eigen::Matrix<double, 12, 12>::Zero();

    // Extract properties
    double rho = material->rho;  // Density [mT/m³]
    double A = section->A;        // Area [m²]
    double Iy = section->Iy;      // Second moment about y [m⁴]
    double Iz = section->Iz;      // Second moment about z [m⁴]
    double J = section->J;        // Torsional constant [m⁴]
    double L = length;            // Length [m]

    // Total mass
    double mass = rho * A * L;

    // Common coefficients for consistent mass matrix
    double m_trans = mass / 420.0;  // Translational mass coefficient
    double m_rot = rho * L / 420.0; // Rotational inertia coefficient

    // Axial mass (DOFs 0, 6: u_i, u_j)
    M(0, 0) = 140.0 * m_trans;
    M(0, 6) = 70.0 * m_trans;
    M(6, 0) = 70.0 * m_trans;
    M(6, 6) = 140.0 * m_trans;

    // Bending in y-direction (DOFs 1, 5, 7, 11: v_i, θz_i, v_j, θz_j)
    // Includes translational and rotary inertia (Iz)
    M(1, 1) = 156.0 * m_trans;
    M(1, 5) = 22.0 * L * m_trans;
    M(1, 7) = 54.0 * m_trans;
    M(1, 11) = -13.0 * L * m_trans;

    M(5, 1) = 22.0 * L * m_trans;
    M(5, 5) = 4.0 * L * L * m_trans + 140.0 * Iz * m_rot;
    M(5, 7) = 13.0 * L * m_trans;
    M(5, 11) = -3.0 * L * L * m_trans + 70.0 * Iz * m_rot;

    M(7, 1) = 54.0 * m_trans;
    M(7, 5) = 13.0 * L * m_trans;
    M(7, 7) = 156.0 * m_trans;
    M(7, 11) = -22.0 * L * m_trans;

    M(11, 1) = -13.0 * L * m_trans;
    M(11, 5) = -3.0 * L * L * m_trans + 70.0 * Iz * m_rot;
    M(11, 7) = -22.0 * L * m_trans;
    M(11, 11) = 4.0 * L * L * m_trans + 140.0 * Iz * m_rot;

    // Bending in z-direction (DOFs 2, 4, 8, 10: w_i, θy_i, w_j, θy_j)
    // Includes translational and rotary inertia (Iy)
    M(2, 2) = 156.0 * m_trans;
    M(2, 4) = -22.0 * L * m_trans;
    M(2, 8) = 54.0 * m_trans;
    M(2, 10) = 13.0 * L * m_trans;

    M(4, 2) = -22.0 * L * m_trans;
    M(4, 4) = 4.0 * L * L * m_trans + 140.0 * Iy * m_rot;
    M(4, 8) = -13.0 * L * m_trans;
    M(4, 10) = -3.0 * L * L * m_trans + 70.0 * Iy * m_rot;

    M(8, 2) = 54.0 * m_trans;
    M(8, 4) = -13.0 * L * m_trans;
    M(8, 8) = 156.0 * m_trans;
    M(8, 10) = 22.0 * L * m_trans;

    M(10, 2) = 13.0 * L * m_trans;
    M(10, 4) = -3.0 * L * L * m_trans + 70.0 * Iy * m_rot;
    M(10, 8) = 22.0 * L * m_trans;
    M(10, 10) = 4.0 * L * L * m_trans + 140.0 * Iy * m_rot;

    // Torsional inertia (DOFs 3, 9: θx_i, θx_j)
    double m_torsion = rho * J * L / 6.0;
    M(3, 3) = 2.0 * m_torsion;
    M(3, 9) = m_torsion;
    M(9, 3) = m_torsion;
    M(9, 9) = 2.0 * m_torsion;

    return M;
}

Eigen::Matrix<double, 12, 12> BeamElement::global_mass_matrix() const {
    Eigen::Matrix<double, 12, 12> M_local = local_mass_matrix();
    Eigen::Matrix<double, 12, 12> T = transformation_matrix();

    // M_global = T^T * M_local * T
    return T.transpose() * M_local * T;
}

} // namespace grillex
