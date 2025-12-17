#include "grillex/beam_element.hpp"
#include "grillex/dof_handler.hpp"
#include <cmath>
#include <algorithm>

namespace grillex {

// ============================================================================
// EndRelease Implementation
// ============================================================================

bool EndRelease::has_any_release() const {
    return release_ux_i || release_uy_i || release_uz_i ||
           release_rx_i || release_ry_i || release_rz_i || release_warp_i ||
           release_ux_j || release_uy_j || release_uz_j ||
           release_rx_j || release_ry_j || release_rz_j || release_warp_j;
}

std::vector<int> EndRelease::get_released_indices(bool has_warping) const {
    std::vector<int> indices;

    // Node i DOFs (indices 0-5 or 0-6 with warping)
    if (release_ux_i) indices.push_back(0);
    if (release_uy_i) indices.push_back(1);
    if (release_uz_i) indices.push_back(2);
    if (release_rx_i) indices.push_back(3);
    if (release_ry_i) indices.push_back(4);
    if (release_rz_i) indices.push_back(5);
    if (has_warping && release_warp_i) indices.push_back(6);

    // Node j DOFs (indices 6-11 for 12-DOF or 7-13 for 14-DOF)
    int offset = has_warping ? 7 : 6;
    if (release_ux_j) indices.push_back(offset + 0);
    if (release_uy_j) indices.push_back(offset + 1);
    if (release_uz_j) indices.push_back(offset + 2);
    if (release_rx_j) indices.push_back(offset + 3);
    if (release_ry_j) indices.push_back(offset + 4);
    if (release_rz_j) indices.push_back(offset + 5);
    if (has_warping && release_warp_j) indices.push_back(offset + 6);

    return indices;
}

// ============================================================================
// BeamElement Implementation
// ============================================================================

BeamElement::BeamElement(int id, Node* node_i, Node* node_j,
                         Material* mat, Section* sec, double roll)
    : id(id), node_i(node_i), node_j(node_j), material(mat), section(sec),
      local_axes(node_i->position(), node_j->position(), roll) {
    length = compute_length();
}

BeamElement::BeamElement(int id, Node* node_i, Node* node_j,
                         Material* mat, Section* sec,
                         const BeamConfig& config, double roll)
    : id(id), node_i(node_i), node_j(node_j), material(mat), section(sec),
      config(config), local_axes(node_i->position(), node_j->position(), roll) {
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

    // Apply end releases via static condensation
    if (releases.has_any_release()) {
        K = apply_static_condensation(K, releases.get_released_indices(false));
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

    // Apply end releases via static condensation
    if (releases.has_any_release()) {
        M = apply_static_condensation(M, releases.get_released_indices(false));
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

Eigen::Matrix<double, 14, 14> BeamElement::offset_transformation_matrix_warping() const {
    Eigen::Matrix<double, 14, 14> T = Eigen::Matrix<double, 14, 14>::Identity();

    if (!has_offsets()) {
        return T;  // No transformation needed
    }

    // Build skew-symmetric matrices for cross products
    // For offset r, the skew-symmetric matrix [r×] is:
    //   [ 0   -rz   ry ]
    //   [ rz   0   -rx ]
    //   [-ry   rx   0  ]

    // DOF ordering for 14×14:
    // Node i: [0-2: UX UY UZ], [3-5: RX RY RZ], [6: WARP]
    // Node j: [7-9: UX UY UZ], [10-12: RX RY RZ], [13: WARP]

    // Transformation for end i
    if (offset_i.norm() > 1e-12) {
        Eigen::Matrix3d r_i_skew;
        r_i_skew <<       0.0,     -offset_i(2),  offset_i(1),
                     offset_i(2),       0.0,     -offset_i(0),
                    -offset_i(1),  offset_i(0),      0.0;

        // Translations couple with rotations via offset
        // [I  -[r×]  0]  // 3x7 block for node i (trans coupled with rot, warp identity)
        // [0   I     0]  // 3x7 block for rotations (identity)
        // [0   0     1]  // 1x7 block for warping (identity, no coupling)
        T.block<3, 3>(0, 3) = -r_i_skew;  // Coupling between translations and rotations
        // Warping DOF (index 6) remains identity - no coupling with offsets
    }

    // Transformation for end j
    if (offset_j.norm() > 1e-12) {
        Eigen::Matrix3d r_j_skew;
        r_j_skew <<       0.0,     -offset_j(2),  offset_j(1),
                     offset_j(2),       0.0,     -offset_j(0),
                    -offset_j(1),  offset_j(0),      0.0;

        // Translations couple with rotations via offset
        // [I  -[r×]  0]  // 3x7 block for node j
        // [0   I     0]  // 3x7 block for rotations
        // [0   0     1]  // 1x7 block for warping (identity)
        T.block<3, 3>(7, 10) = -r_j_skew;  // Coupling between translations and rotations
        // Warping DOF (index 13) remains identity - no coupling with offsets
    }

    return T;
}

Eigen::Matrix<double, 14, 14> BeamElement::local_stiffness_matrix_warping(
    BeamFormulation formulation) const {
    // Start with 14×14 zero matrix
    Eigen::Matrix<double, 14, 14> K = Eigen::Matrix<double, 14, 14>::Zero();

    // Get the 12×12 matrix without warping
    Eigen::Matrix<double, 12, 12> K12 = local_stiffness_matrix(formulation);

    // Extract properties
    double E = material->E;
    double G = material->G;
    double Iw = section->Iw;
    double L = length;

    // Common terms for warping stiffness
    double L2 = L * L;
    double L3 = L2 * L;
    double EIw = E * Iw;

    // DOF mapping for 14×14 matrix:
    // Node i: [0-2: UX UY UZ], [3-5: RX RY RZ], [6: WARP]
    // Node j: [7-9: UX UY UZ], [10-12: RX RY RZ], [13: WARP]

    // Copy translational DOFs (UX, UY, UZ) for both nodes
    K.block<3, 3>(0, 0) = K12.block<3, 3>(0, 0);      // Node i trans-trans
    K.block<3, 3>(0, 7) = K12.block<3, 3>(0, 6);      // Node i trans - node j trans
    K.block<3, 3>(7, 0) = K12.block<3, 3>(6, 0);      // Node j trans - node i trans
    K.block<3, 3>(7, 7) = K12.block<3, 3>(6, 6);      // Node j trans-trans

    // Copy rotational DOFs (RY, RZ) - skip RX (torsion) for now
    // RY: index 4 in 12×12 → index 4 in 14×14
    // RZ: index 5 in 12×12 → index 5 in 14×14
    for (int i : {1, 2, 4, 5}) {  // v, w, θy, θz at node i
        for (int j : {1, 2, 4, 5}) {
            K(i, j) = K12(i, j);
        }
    }
    // Node i bending - Node j trans/rot
    for (int i : {1, 2, 4, 5}) {
        K(i, 7) = K12(i, 6);      // v,w,θy,θz @ i - u @ j
        K(i, 8) = K12(i, 7);      // v,w,θy,θz @ i - v @ j
        K(i, 9) = K12(i, 8);      // v,w,θy,θz @ i - w @ j
        K(i, 11) = K12(i, 10);    // v,w,θy,θz @ i - θy @ j
        K(i, 12) = K12(i, 11);    // v,w,θy,θz @ i - θz @ j
    }
    // Node j trans/rot - Node i bending
    for (int j : {1, 2, 4, 5}) {
        K(7, j) = K12(6, j);      // u @ j - v,w,θy,θz @ i
        K(8, j) = K12(7, j);      // v @ j - v,w,θy,θz @ i
        K(9, j) = K12(8, j);      // w @ j - v,w,θy,θz @ i
        K(11, j) = K12(10, j);    // θy @ j - v,w,θy,θz @ i
        K(12, j) = K12(11, j);    // θz @ j - v,w,θy,θz @ i
    }
    // Node j bending - Node j bending
    for (int i : {8, 9, 11, 12}) {  // v, w, θy, θz at node j (in 14×14)
        for (int j : {8, 9, 11, 12}) {
            int i12 = (i == 8) ? 7 : (i == 9) ? 8 : (i == 11) ? 10 : 11;
            int j12 = (j == 8) ? 7 : (j == 9) ? 8 : (j == 11) ? 10 : 11;
            K(i, j) = K12(i12, j12);
        }
    }

    // Torsion-warping coupling: 4×4 block for [θx_i, φ'_i, θx_j, φ'_j]
    // Indices in 14×14: [3, 6, 10, 13]

    // According to Vlasov theory:
    // K_tw = [GJ/L + 12EIw/L³    6EIw/L²      -GJ/L - 12EIw/L³   6EIw/L²   ]
    //        [6EIw/L²            4EIw/L       -6EIw/L²           2EIw/L    ]
    //        [-GJ/L - 12EIw/L³   -6EIw/L²     GJ/L + 12EIw/L³    -6EIw/L²  ]
    //        [6EIw/L²            2EIw/L       -6EIw/L²           4EIw/L    ]

    double k_tw_11 = G * section->J / L + 12.0 * EIw / L3;
    double k_tw_12 = 6.0 * EIw / L2;
    double k_tw_13 = -G * section->J / L - 12.0 * EIw / L3;
    double k_tw_14 = 6.0 * EIw / L2;
    double k_tw_22 = 4.0 * EIw / L;
    double k_tw_23 = -6.0 * EIw / L2;
    double k_tw_24 = 2.0 * EIw / L;
    double k_tw_33 = G * section->J / L + 12.0 * EIw / L3;
    double k_tw_34 = -6.0 * EIw / L2;
    double k_tw_44 = 4.0 * EIw / L;

    // Fill the 4×4 torsion-warping block
    K(3, 3) = k_tw_11;    // θx_i, θx_i
    K(3, 6) = k_tw_12;    // θx_i, φ'_i
    K(3, 10) = k_tw_13;   // θx_i, θx_j
    K(3, 13) = k_tw_14;   // θx_i, φ'_j

    K(6, 3) = k_tw_12;    // φ'_i, θx_i (symmetric)
    K(6, 6) = k_tw_22;    // φ'_i, φ'_i
    K(6, 10) = k_tw_23;   // φ'_i, θx_j
    K(6, 13) = k_tw_24;   // φ'_i, φ'_j

    K(10, 3) = k_tw_13;   // θx_j, θx_i (symmetric)
    K(10, 6) = k_tw_23;   // θx_j, φ'_i (symmetric)
    K(10, 10) = k_tw_33;  // θx_j, θx_j
    K(10, 13) = k_tw_34;  // θx_j, φ'_j

    K(13, 3) = k_tw_14;   // φ'_j, θx_i (symmetric)
    K(13, 6) = k_tw_24;   // φ'_j, φ'_i (symmetric)
    K(13, 10) = k_tw_34;  // φ'_j, θx_j (symmetric)
    K(13, 13) = k_tw_44;  // φ'_j, φ'_j

    // Apply offset transformation if offsets are present
    if (has_offsets()) {
        Eigen::Matrix<double, 14, 14> T_offset = offset_transformation_matrix_warping();
        K = T_offset.transpose() * K * T_offset;
    }

    // Apply end releases via static condensation
    if (releases.has_any_release()) {
        K = apply_static_condensation(K, releases.get_released_indices(true));
    }

    return K;
}

Eigen::Matrix<double, 14, 14> BeamElement::local_mass_matrix_warping(
    BeamFormulation formulation) const {
    // Start with 14×14 zero matrix
    Eigen::Matrix<double, 14, 14> M = Eigen::Matrix<double, 14, 14>::Zero();

    // Get the 12×12 mass matrix without warping
    Eigen::Matrix<double, 12, 12> M12 = local_mass_matrix(formulation);

    // DOF mapping for 14×14 matrix:
    // Node i: [0-2: UX UY UZ], [3-5: RX RY RZ], [6: WARP]
    // Node j: [7-9: UX UY UZ], [10-12: RX RY RZ], [13: WARP]

    // Copy translational and rotational masses from 12×12 matrix
    // Node i: UX, UY, UZ (0-2 in both matrices)
    M.block<3, 3>(0, 0) = M12.block<3, 3>(0, 0);
    M.block<3, 3>(0, 7) = M12.block<3, 3>(0, 6);
    M.block<3, 3>(7, 0) = M12.block<3, 3>(6, 0);
    M.block<3, 3>(7, 7) = M12.block<3, 3>(6, 6);

    // Copy bending masses (v, w, θy, θz and couplings)
    // Similar pattern as stiffness matrix
    for (int i : {1, 2, 4, 5}) {
        for (int j : {1, 2, 4, 5}) {
            M(i, j) = M12(i, j);
        }
    }
    // Node i bending - Node j trans/rot
    for (int i : {1, 2, 4, 5}) {
        M(i, 7) = M12(i, 6);
        M(i, 8) = M12(i, 7);
        M(i, 9) = M12(i, 8);
        M(i, 11) = M12(i, 10);
        M(i, 12) = M12(i, 11);
    }
    // Node j trans/rot - Node i bending
    for (int j : {1, 2, 4, 5}) {
        M(7, j) = M12(6, j);
        M(8, j) = M12(7, j);
        M(9, j) = M12(8, j);
        M(11, j) = M12(10, j);
        M(12, j) = M12(11, j);
    }
    // Node j bending - Node j bending
    for (int i : {8, 9, 11, 12}) {
        for (int j : {8, 9, 11, 12}) {
            int i12 = (i == 8) ? 7 : (i == 9) ? 8 : (i == 11) ? 10 : 11;
            int j12 = (j == 8) ? 7 : (j == 9) ? 8 : (j == 11) ? 10 : 11;
            M(i, j) = M12(i12, j12);
        }
    }

    // Copy torsional inertia (RX DOF)
    // 12×12: indices 3 and 9
    // 14×14: indices 3 and 10
    M(3, 3) = M12(3, 3);
    M(3, 10) = M12(3, 9);
    M(10, 3) = M12(9, 3);
    M(10, 10) = M12(9, 9);

    // Warping inertia terms (indices 6 and 13)
    // Typically negligible for static analysis, left as zero
    // For dynamic analysis, could add: M(6,6) = M(13,13) = rho * Iw * L / 3.0;
    // but this is usually omitted in practice

    // Apply offset transformation if offsets are present
    if (has_offsets()) {
        Eigen::Matrix<double, 14, 14> T_offset = offset_transformation_matrix_warping();
        M = T_offset.transpose() * M * T_offset;
    }

    // Apply end releases via static condensation
    if (releases.has_any_release()) {
        M = apply_static_condensation(M, releases.get_released_indices(true));
    }

    return M;
}

Eigen::Matrix<double, 14, 14> BeamElement::transformation_matrix_warping() const {
    Eigen::Matrix<double, 14, 14> T = Eigen::Matrix<double, 14, 14>::Zero();

    // Get the 3x3 rotation matrix from local axes
    Eigen::Matrix3d R = local_axes.rotation_matrix;

    // Block diagonal structure:
    // - 3x3 for node i translations (UX, UY, UZ)
    // - 3x3 for node i rotations (RX, RY, RZ)
    // - 1x1 identity for node i warping (WARP) - transforms as scalar
    // - 3x3 for node j translations (UX, UY, UZ)
    // - 3x3 for node j rotations (RX, RY, RZ)
    // - 1x1 identity for node j warping (WARP) - transforms as scalar

    T.block<3, 3>(0, 0) = R;      // Node i translations
    T.block<3, 3>(3, 3) = R;      // Node i rotations
    T(6, 6) = 1.0;                // Node i warping (scalar, no transformation)
    T.block<3, 3>(7, 7) = R;      // Node j translations
    T.block<3, 3>(10, 10) = R;    // Node j rotations
    T(13, 13) = 1.0;              // Node j warping (scalar, no transformation)

    return T;
}

Eigen::Matrix<double, 14, 14> BeamElement::global_stiffness_matrix_warping(
    BeamFormulation formulation) const {
    Eigen::Matrix<double, 14, 14> K_local = local_stiffness_matrix_warping(formulation);
    Eigen::Matrix<double, 14, 14> T = transformation_matrix_warping();

    // K_global = T^T * K_local * T
    return T.transpose() * K_local * T;
}

Eigen::Matrix<double, 14, 14> BeamElement::global_mass_matrix_warping(
    BeamFormulation formulation) const {
    Eigen::Matrix<double, 14, 14> M_local = local_mass_matrix_warping(formulation);
    Eigen::Matrix<double, 14, 14> T = transformation_matrix_warping();

    // M_global = T^T * M_local * T
    return T.transpose() * M_local * T;
}

Eigen::MatrixXd BeamElement::apply_static_condensation(
    const Eigen::MatrixXd& K,
    const std::vector<int>& released_indices) const {

    if (released_indices.empty()) {
        return K;  // No releases, return original matrix
    }

    int n = K.rows();
    int n_released = released_indices.size();
    int n_retained = n - n_released;

    // Create lists of retained and released indices
    std::vector<int> retained_indices;
    retained_indices.reserve(n_retained);
    for (int i = 0; i < n; ++i) {
        if (std::find(released_indices.begin(), released_indices.end(), i) == released_indices.end()) {
            retained_indices.push_back(i);
        }
    }

    // Extract submatrices
    // K_rr: retained-retained
    // K_rc: retained-released
    // K_cr: released-retained
    // K_cc: released-released
    Eigen::MatrixXd K_rr(n_retained, n_retained);
    Eigen::MatrixXd K_rc(n_retained, n_released);
    Eigen::MatrixXd K_cr(n_released, n_retained);
    Eigen::MatrixXd K_cc(n_released, n_released);

    for (int i = 0; i < n_retained; ++i) {
        for (int j = 0; j < n_retained; ++j) {
            K_rr(i, j) = K(retained_indices[i], retained_indices[j]);
        }
        for (int j = 0; j < n_released; ++j) {
            K_rc(i, j) = K(retained_indices[i], released_indices[j]);
        }
    }

    for (int i = 0; i < n_released; ++i) {
        for (int j = 0; j < n_retained; ++j) {
            K_cr(i, j) = K(released_indices[i], retained_indices[j]);
        }
        for (int j = 0; j < n_released; ++j) {
            K_cc(i, j) = K(released_indices[i], released_indices[j]);
        }
    }

    // Perform static condensation: K_condensed = K_rr - K_rc * K_cc^(-1) * K_cr
    Eigen::MatrixXd K_cc_inv = K_cc.inverse();
    Eigen::MatrixXd K_condensed_retained = K_rr - K_rc * K_cc_inv * K_cr;

    // Build the full condensed matrix (same size as input)
    // Retained DOFs get the condensed stiffness
    // Released DOFs get zeros (they don't contribute to global stiffness)
    Eigen::MatrixXd K_result = Eigen::MatrixXd::Zero(n, n);

    for (int i = 0; i < n_retained; ++i) {
        for (int j = 0; j < n_retained; ++j) {
            K_result(retained_indices[i], retained_indices[j]) = K_condensed_retained(i, j);
        }
    }

    return K_result;
}

// ============================================================================
// Virtual Method Implementations (BeamElementBase interface)
// ============================================================================

Eigen::MatrixXd BeamElement::compute_local_stiffness() const {
    BeamFormulation form = config.get_formulation();
    if (config.include_warping) {
        return local_stiffness_matrix_warping(form);
    } else {
        return local_stiffness_matrix(form);
    }
}

Eigen::MatrixXd BeamElement::compute_local_mass() const {
    BeamFormulation form = config.get_formulation();
    if (config.include_warping) {
        return local_mass_matrix_warping(form);
    } else {
        return local_mass_matrix(form);
    }
}

Eigen::MatrixXd BeamElement::compute_transformation() const {
    if (config.include_warping) {
        return transformation_matrix_warping();
    } else {
        return transformation_matrix();
    }
}

int BeamElement::num_dofs() const {
    return config.include_warping ? 14 : 12;
}

BeamFormulation BeamElement::get_formulation() const {
    return config.get_formulation();
}

bool BeamElement::has_warping() const {
    return config.include_warping;
}

Eigen::Vector3d BeamElement::direction_vector() const {
    Eigen::Vector3d pos_i = node_i->position();
    Eigen::Vector3d pos_j = node_j->position();
    Eigen::Vector3d dir = pos_j - pos_i;
    return dir.normalized();
}

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<BeamElementBase> create_beam_element(
    int id, Node* node_i, Node* node_j,
    Material* mat, Section* sec,
    const BeamConfig& config,
    double roll) {
    return std::make_unique<BeamElement>(id, node_i, node_j, mat, sec, config, roll);
}

// ============================================================================
// Collinearity Detection
// ============================================================================

bool are_elements_collinear(
    const BeamElement& elem1,
    const BeamElement& elem2,
    int shared_node_id,
    double angle_tolerance_deg) {

    // Get direction vectors
    Eigen::Vector3d dir1 = elem1.direction_vector();
    Eigen::Vector3d dir2 = elem2.direction_vector();

    // If the shared node is at different ends of the elements, we need to
    // flip one direction to compare them properly.
    // When elements are collinear and continuous (like a spliced beam),
    // one element's direction points into the node while the other points out.

    // For elem1: if shared_node is node_j, direction points INTO the node
    // For elem1: if shared_node is node_i, direction points OUT OF the node
    // To be collinear and continuous, the directions should be opposite at the shared node

    bool elem1_ends_at_shared = (elem1.node_j->id == shared_node_id);
    bool elem2_starts_at_shared = (elem2.node_i->id == shared_node_id);

    // If elem1 ends at shared and elem2 starts at shared, they're continuous
    // if dir1 ≈ dir2 (both pointing in the same global direction)
    // If elem1 starts at shared and elem2 ends at shared, same logic
    // The key is: continuous beams have SAME direction at the shared node

    // Flip directions to compare them relative to the shared node
    // After flipping, collinear continuous elements should have dirs pointing the same way
    if (!elem1_ends_at_shared) {
        dir1 = -dir1;  // Now dir1 points toward the shared node
    }
    if (elem2_starts_at_shared) {
        dir2 = -dir2;  // Now dir2 points toward the shared node
    }

    // Collinear if directions are parallel (either same or opposite direction)
    // We take absolute value of dot product to handle both cases
    double dot = std::abs(dir1.dot(dir2));

    // Convert angle tolerance to cosine threshold
    // cos(5°) ≈ 0.9962
    double cos_threshold = std::cos(angle_tolerance_deg * M_PI / 180.0);

    return dot >= cos_threshold;
}

// ============================================================================
// Equivalent Nodal Forces for Distributed Loads
// ============================================================================

Eigen::Matrix<double, 12, 1> BeamElement::equivalent_nodal_forces(
    const Eigen::Vector3d& w_start,
    const Eigen::Vector3d& w_end) const
{
    // Initialize result vector
    Eigen::Matrix<double, 12, 1> f_global = Eigen::Matrix<double, 12, 1>::Zero();

    // Get rotation matrix from transformation (extract 3x3 block)
    Eigen::Matrix<double, 12, 12> T = transformation_matrix();
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);  // Local-to-global rotation

    // Transform global loads to local coordinates
    // R transforms local -> global, so R^T transforms global -> local
    Eigen::Vector3d w_start_local = R.transpose() * w_start;
    Eigen::Vector3d w_end_local = R.transpose() * w_end;

    // Element length
    double L = length;
    double L2 = L * L;

    // Compute equivalent nodal forces in local coordinates
    // Local DOF ordering: [u_i, v_i, w_i, θx_i, θy_i, θz_i, u_j, v_j, w_j, θx_j, θy_j, θz_j]
    // Local axes: x = axial, y = transverse (bending about z), z = transverse (bending about y)

    Eigen::Matrix<double, 12, 1> f_local = Eigen::Matrix<double, 12, 1>::Zero();

    // Extract load components in local coordinates
    double wx1 = w_start_local(0);  // Axial load at start
    double wy1 = w_start_local(1);  // Transverse y load at start
    double wz1 = w_start_local(2);  // Transverse z load at start
    double wx2 = w_end_local(0);    // Axial load at end
    double wy2 = w_end_local(1);    // Transverse y load at end
    double wz2 = w_end_local(2);    // Transverse z load at end

    // ---------------------------
    // Axial loads (local x direction)
    // ---------------------------
    // For trapezoidal axial load:
    // f_x_i = L(2w1 + w2)/6
    // f_x_j = L(w1 + 2w2)/6
    f_local(0) = L * (2.0 * wx1 + wx2) / 6.0;     // f_x at node i
    f_local(6) = L * (wx1 + 2.0 * wx2) / 6.0;     // f_x at node j

    // ---------------------------
    // Transverse loads in local y (bending about z-axis)
    // ---------------------------
    // For trapezoidal load (w1 at start, w2 at end):
    // Fy_i = L(7w1 + 3w2)/20
    // Mz_i = L²(3w1 + 2w2)/60
    // Fy_j = L(3w1 + 7w2)/20
    // Mz_j = -L²(2w1 + 3w2)/60
    f_local(1) = L * (7.0 * wy1 + 3.0 * wy2) / 20.0;       // f_y at node i
    f_local(5) = L2 * (3.0 * wy1 + 2.0 * wy2) / 60.0;      // m_z at node i (positive for CCW)
    f_local(7) = L * (3.0 * wy1 + 7.0 * wy2) / 20.0;       // f_y at node j
    f_local(11) = -L2 * (2.0 * wy1 + 3.0 * wy2) / 60.0;    // m_z at node j

    // ---------------------------
    // Transverse loads in local z (bending about y-axis)
    // ---------------------------
    // For trapezoidal load (w1 at start, w2 at end):
    // Fz_i = L(7w1 + 3w2)/20
    // My_i = -L²(3w1 + 2w2)/60   (sign convention: positive My causes tension at +z face)
    // Fz_j = L(3w1 + 7w2)/20
    // My_j = L²(2w1 + 3w2)/60
    f_local(2) = L * (7.0 * wz1 + 3.0 * wz2) / 20.0;       // f_z at node i
    f_local(4) = -L2 * (3.0 * wz1 + 2.0 * wz2) / 60.0;     // m_y at node i
    f_local(8) = L * (3.0 * wz1 + 7.0 * wz2) / 20.0;       // f_z at node j
    f_local(10) = L2 * (2.0 * wz1 + 3.0 * wz2) / 60.0;     // m_y at node j

    // Transform back to global coordinates
    // f_global = T^T * f_local
    f_global = T.transpose() * f_local;

    return f_global;
}

// ============================================================================
// Phase 7: Internal Actions (Element End Forces)
// ============================================================================

Eigen::VectorXd BeamElement::get_element_displacements_local(
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler) const
{
    // Get the location array for this element
    std::vector<int> loc_array = dof_handler.get_location_array(*this);
    int n_dofs = num_dofs();  // 12 or 14 depending on warping

    // Extract global displacements for this element
    Eigen::VectorXd u_global_elem(n_dofs);
    for (int i = 0; i < n_dofs; ++i) {
        int global_dof = loc_array[i];
        if (global_dof >= 0 && global_dof < global_displacements.size()) {
            u_global_elem(i) = global_displacements(global_dof);
        } else {
            u_global_elem(i) = 0.0;  // Fixed or invalid DOF
        }
    }

    // Get transformation matrix
    Eigen::MatrixXd T = compute_transformation();

    // Transform to local coordinates: u_local = T * u_global
    // Note: T transforms local to global, so T^T (= T^(-1) for orthogonal T) transforms global to local
    Eigen::VectorXd u_local = T * u_global_elem;

    return u_local;
}

std::pair<EndForces, EndForces> BeamElement::compute_end_forces(
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler) const
{
    // Get local displacements
    Eigen::VectorXd u_local = get_element_displacements_local(global_displacements, dof_handler);

    // Get local stiffness matrix
    Eigen::MatrixXd K_local = compute_local_stiffness();

    // Compute local forces: f_local = K_local * u_local
    Eigen::VectorXd f_local = K_local * u_local;

    // Extract forces at each end
    EndForces forces_i, forces_j;

    if (config.include_warping) {
        // 14-DOF element
        // DOF ordering: [u_i, v_i, w_i, θx_i, θy_i, θz_i, φ'_i, u_j, v_j, w_j, θx_j, θy_j, θz_j, φ'_j]

        // End i forces (indices 0-6)
        forces_i.N = f_local(0);    // Axial force
        forces_i.Vy = f_local(1);   // Shear y
        forces_i.Vz = f_local(2);   // Shear z
        forces_i.Mx = f_local(3);   // Torsion
        forces_i.My = f_local(4);   // Moment about y
        forces_i.Mz = f_local(5);   // Moment about z
        forces_i.B = f_local(6);    // Bimoment

        // End j forces (indices 7-13)
        // Negate for action on connected node (internal forces are in equilibrium)
        forces_j.N = -f_local(7);
        forces_j.Vy = -f_local(8);
        forces_j.Vz = -f_local(9);
        forces_j.Mx = -f_local(10);
        forces_j.My = -f_local(11);
        forces_j.Mz = -f_local(12);
        forces_j.B = -f_local(13);
    } else {
        // 12-DOF element
        // DOF ordering: [u_i, v_i, w_i, θx_i, θy_i, θz_i, u_j, v_j, w_j, θx_j, θy_j, θz_j]

        // End i forces (indices 0-5)
        forces_i.N = f_local(0);    // Axial force
        forces_i.Vy = f_local(1);   // Shear y
        forces_i.Vz = f_local(2);   // Shear z
        forces_i.Mx = f_local(3);   // Torsion
        forces_i.My = f_local(4);   // Moment about y
        forces_i.Mz = f_local(5);   // Moment about z

        // End j forces (indices 6-11)
        // Negate for action on connected node (internal forces are in equilibrium)
        forces_j.N = -f_local(6);
        forces_j.Vy = -f_local(7);
        forces_j.Vz = -f_local(8);
        forces_j.Mx = -f_local(9);
        forces_j.My = -f_local(10);
        forces_j.Mz = -f_local(11);
    }

    // Check for end releases and zero out released forces
    if (releases.has_any_release()) {
        // End i releases
        if (releases.release_ux_i) forces_i.N = 0.0;
        if (releases.release_uy_i) forces_i.Vy = 0.0;
        if (releases.release_uz_i) forces_i.Vz = 0.0;
        if (releases.release_rx_i) forces_i.Mx = 0.0;
        if (releases.release_ry_i) forces_i.My = 0.0;
        if (releases.release_rz_i) forces_i.Mz = 0.0;
        if (config.include_warping && releases.release_warp_i) forces_i.B = 0.0;

        // End j releases
        if (releases.release_ux_j) forces_j.N = 0.0;
        if (releases.release_uy_j) forces_j.Vy = 0.0;
        if (releases.release_uz_j) forces_j.Vz = 0.0;
        if (releases.release_rx_j) forces_j.Mx = 0.0;
        if (releases.release_ry_j) forces_j.My = 0.0;
        if (releases.release_rz_j) forces_j.Mz = 0.0;
        if (config.include_warping && releases.release_warp_j) forces_j.B = 0.0;
    }

    return {forces_i, forces_j};
}

} // namespace grillex
