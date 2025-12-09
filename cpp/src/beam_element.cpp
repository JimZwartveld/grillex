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

Eigen::Matrix<double, 12, 12> BeamElement::local_stiffness_matrix(
    BeamFormulation formulation) const {
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

    // Compute shear deformation factors for Timoshenko beams
    double phi_y = 0.0;  // Shear deformation factor for bending about y-axis
    double phi_z = 0.0;  // Shear deformation factor for bending about z-axis

    if (formulation == BeamFormulation::Timoshenko) {
        // φ_y = 12EI_y / (κA_sy * G * L²)
        // φ_z = 12EI_z / (κA_sz * G * L²)
        // Using κ = 5/6 for rectangular sections (conservative approximation)
        double kappa = 5.0 / 6.0;

        // Get shear areas (if not set, default to A with shear correction)
        double Asy = section->Asy > 0 ? section->Asy : kappa * A;
        double Asz = section->Asz > 0 ? section->Asz : kappa * A;

        phi_y = 12.0 * E * Iy / (Asy * G * L2);
        phi_z = 12.0 * E * Iz / (Asz * G * L2);
    }

    // Bending about z-axis (in-plane y, DOFs 1, 5, 7, 11: v_i, θz_i, v_j, θz_j)
    // Standard Euler-Bernoulli terms, modified for Timoshenko
    double k_bend_z = 12.0 * E * Iz / (L3 * (1.0 + phi_z));
    double k_bend_z2 = 6.0 * E * Iz / (L2 * (1.0 + phi_z));
    double k_bend_z3 = (4.0 + phi_z) * E * Iz / (L * (1.0 + phi_z));
    double k_bend_z4 = (2.0 - phi_z) * E * Iz / (L * (1.0 + phi_z));

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
    // Standard Euler-Bernoulli terms, modified for Timoshenko
    double k_bend_y = 12.0 * E * Iy / (L3 * (1.0 + phi_y));
    double k_bend_y2 = 6.0 * E * Iy / (L2 * (1.0 + phi_y));
    double k_bend_y3 = (4.0 + phi_y) * E * Iy / (L * (1.0 + phi_y));
    double k_bend_y4 = (2.0 - phi_y) * E * Iy / (L * (1.0 + phi_y));

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

    // Apply offset transformation if offsets are present
    // Apply offset transformation if offsets are present
    if (has_offsets()) {
        Eigen::Matrix<double, 12, 12> T_offset = offset_transformation_matrix();
        K = T_offset.transpose() * K * T_offset;
    }

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

Eigen::Matrix<double, 12, 12> BeamElement::local_mass_matrix(
    BeamFormulation formulation) const {
    Eigen::Matrix<double, 12, 12> M = Eigen::Matrix<double, 12, 12>::Zero();

    // Extract properties
    double rho = material->rho;  // Density [mT/m³]
    double E = material->E;
    double G = material->G;
    double A = section->A;        // Area [m²]
    double Iy = section->Iy;      // Second moment about y [m⁴]
    double Iz = section->Iz;      // Second moment about z [m⁴]
    double J = section->J;        // Torsional constant [m⁴]
    double L = length;            // Length [m]
    double L2 = L * L;

    // Total mass
    double mass = rho * A * L;

    // Compute shear deformation factors for Timoshenko beams
    double phi_y = 0.0;
    double phi_z = 0.0;

    if (formulation == BeamFormulation::Timoshenko) {
        double kappa = 5.0 / 6.0;
        double Asy = section->Asy > 0 ? section->Asy : kappa * A;
        double Asz = section->Asz > 0 ? section->Asz : kappa * A;

        phi_y = 12.0 * E * Iy / (Asy * G * L2);
        phi_z = 12.0 * E * Iz / (Asz * G * L2);
    }

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
    // Modified for Timoshenko beams with shear deformation
    double phi_z2 = phi_z * phi_z;
    double den_z = (1.0 + phi_z) * (1.0 + phi_z);

    M(1, 1) = m_trans * (156.0 + 294.0 * phi_z + 140.0 * phi_z2) / den_z;
    M(1, 5) = m_trans * L * (22.0 + 38.5 * phi_z + 17.5 * phi_z2) / den_z;
    M(1, 7) = m_trans * (54.0 + 126.0 * phi_z + 70.0 * phi_z2) / den_z;
    M(1, 11) = m_trans * L * (-13.0 - 31.5 * phi_z - 17.5 * phi_z2) / den_z;

    M(5, 1) = m_trans * L * (22.0 + 38.5 * phi_z + 17.5 * phi_z2) / den_z;
    M(5, 5) = m_trans * L2 * (4.0 + 7.0 * phi_z + 3.5 * phi_z2) / den_z + 140.0 * Iz * m_rot;
    M(5, 7) = m_trans * L * (13.0 + 31.5 * phi_z + 17.5 * phi_z2) / den_z;
    M(5, 11) = m_trans * L2 * (-3.0 - 7.0 * phi_z - 3.5 * phi_z2) / den_z + 70.0 * Iz * m_rot;

    M(7, 1) = m_trans * (54.0 + 126.0 * phi_z + 70.0 * phi_z2) / den_z;
    M(7, 5) = m_trans * L * (13.0 + 31.5 * phi_z + 17.5 * phi_z2) / den_z;
    M(7, 7) = m_trans * (156.0 + 294.0 * phi_z + 140.0 * phi_z2) / den_z;
    M(7, 11) = m_trans * L * (-22.0 - 38.5 * phi_z - 17.5 * phi_z2) / den_z;

    M(11, 1) = m_trans * L * (-13.0 - 31.5 * phi_z - 17.5 * phi_z2) / den_z;
    M(11, 5) = m_trans * L2 * (-3.0 - 7.0 * phi_z - 3.5 * phi_z2) / den_z + 70.0 * Iz * m_rot;
    M(11, 7) = m_trans * L * (-22.0 - 38.5 * phi_z - 17.5 * phi_z2) / den_z;
    M(11, 11) = m_trans * L2 * (4.0 + 7.0 * phi_z + 3.5 * phi_z2) / den_z + 140.0 * Iz * m_rot;

    // Bending in z-direction (DOFs 2, 4, 8, 10: w_i, θy_i, w_j, θy_j)
    // Includes translational and rotary inertia (Iy)
    // Modified for Timoshenko beams with shear deformation
    double phi_y2 = phi_y * phi_y;
    double den_y = (1.0 + phi_y) * (1.0 + phi_y);

    M(2, 2) = m_trans * (156.0 + 294.0 * phi_y + 140.0 * phi_y2) / den_y;
    M(2, 4) = -m_trans * L * (22.0 + 38.5 * phi_y + 17.5 * phi_y2) / den_y;
    M(2, 8) = m_trans * (54.0 + 126.0 * phi_y + 70.0 * phi_y2) / den_y;
    M(2, 10) = m_trans * L * (13.0 + 31.5 * phi_y + 17.5 * phi_y2) / den_y;

    M(4, 2) = -m_trans * L * (22.0 + 38.5 * phi_y + 17.5 * phi_y2) / den_y;
    M(4, 4) = m_trans * L2 * (4.0 + 7.0 * phi_y + 3.5 * phi_y2) / den_y + 140.0 * Iy * m_rot;
    M(4, 8) = -m_trans * L * (13.0 + 31.5 * phi_y + 17.5 * phi_y2) / den_y;
    M(4, 10) = -m_trans * L2 * (3.0 + 7.0 * phi_y + 3.5 * phi_y2) / den_y + 70.0 * Iy * m_rot;

    M(8, 2) = m_trans * (54.0 + 126.0 * phi_y + 70.0 * phi_y2) / den_y;
    M(8, 4) = -m_trans * L * (13.0 + 31.5 * phi_y + 17.5 * phi_y2) / den_y;
    M(8, 8) = m_trans * (156.0 + 294.0 * phi_y + 140.0 * phi_y2) / den_y;
    M(8, 10) = m_trans * L * (22.0 + 38.5 * phi_y + 17.5 * phi_y2) / den_y;

    M(10, 2) = m_trans * L * (13.0 + 31.5 * phi_y + 17.5 * phi_y2) / den_y;
    M(10, 4) = -m_trans * L2 * (3.0 + 7.0 * phi_y + 3.5 * phi_y2) / den_y + 70.0 * Iy * m_rot;
    M(10, 8) = m_trans * L * (22.0 + 38.5 * phi_y + 17.5 * phi_y2) / den_y;
    M(10, 10) = m_trans * L2 * (4.0 + 7.0 * phi_y + 3.5 * phi_y2) / den_y + 140.0 * Iy * m_rot;

    // Torsional inertia (DOFs 3, 9: θx_i, θx_j)
    double m_torsion = rho * J * L / 6.0;
    M(3, 3) = 2.0 * m_torsion;
    M(3, 9) = m_torsion;
    M(9, 3) = m_torsion;
    M(9, 9) = 2.0 * m_torsion;

    // Apply offset transformation if offsets are present
    if (has_offsets()) {
        Eigen::Matrix<double, 12, 12> T_offset = offset_transformation_matrix();
        M = T_offset.transpose() * M * T_offset;
    }

    return M;
}

Eigen::Matrix<double, 12, 12> BeamElement::global_mass_matrix() const {
    Eigen::Matrix<double, 12, 12> M_local = local_mass_matrix();
    Eigen::Matrix<double, 12, 12> T = transformation_matrix();

    // M_global = T^T * M_local * T
    return T.transpose() * M_local * T;
}

void BeamElement::set_offsets(const Eigen::Vector3d& offset_i, const Eigen::Vector3d& offset_j) {
    this->offset_i = offset_i;
    this->offset_j = offset_j;
}

bool BeamElement::has_offsets() const {
    return offset_i.norm() > 1e-12 || offset_j.norm() > 1e-12;
}

double BeamElement::effective_length() const {
    if (!has_offsets()) {
        return length;
    }

    // Compute beam end positions in global coordinates
    // Node positions
    Eigen::Vector3d pos_i = node_i->position();
    Eigen::Vector3d pos_j = node_j->position();

    // Transform offsets from local to global coordinates
    Eigen::Vector3d offset_i_global = local_axes.to_global(offset_i);
    Eigen::Vector3d offset_j_global = local_axes.to_global(offset_j);

    // Beam end positions
    Eigen::Vector3d beam_end_i = pos_i + offset_i_global;
    Eigen::Vector3d beam_end_j = pos_j + offset_j_global;

    // Effective length is distance between beam ends
    return (beam_end_j - beam_end_i).norm();
}

Eigen::Matrix<double, 12, 12> BeamElement::offset_transformation_matrix() const {
    Eigen::Matrix<double, 12, 12> T = Eigen::Matrix<double, 12, 12>::Identity();

    if (!has_offsets()) {
        return T;  // No transformation needed
    }

    // Build skew-symmetric matrices for cross products
    // For offset r, the skew-symmetric matrix [r×] is:
    //   [ 0   -rz   ry ]
    //   [ rz   0   -rx ]
    //   [-ry   rx   0  ]

    // Transformation for end i
    if (offset_i.norm() > 1e-12) {
        Eigen::Matrix3d r_i_skew;
        r_i_skew <<       0.0,     -offset_i(2),  offset_i(1),
                     offset_i(2),       0.0,     -offset_i(0),
                    -offset_i(1),  offset_i(0),      0.0;

        // Upper-left 6×6 block for node i
        // [I  -[r×]]
        // [0   I  ]
        T.block<3, 3>(0, 3) = -r_i_skew;  // Coupling between translations and rotations
    }

    // Transformation for end j
    if (offset_j.norm() > 1e-12) {
        Eigen::Matrix3d r_j_skew;
        r_j_skew <<       0.0,     -offset_j(2),  offset_j(1),
                     offset_j(2),       0.0,     -offset_j(0),
                    -offset_j(1),  offset_j(0),      0.0;

        // Lower-right 6×6 block for node j
        // [I  -[r×]]
        // [0   I  ]
        T.block<3, 3>(6, 9) = -r_j_skew;  // Coupling between translations and rotations
    }

    return T;
}

} // namespace grillex
