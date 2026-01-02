#include "grillex/beam_element.hpp"
#include "grillex/dof_handler.hpp"
#include "grillex/load_case.hpp"
#include "grillex/internal_actions_computer.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

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
    // NOTE: Warping releases are NOT included here for static condensation.
    // Warping releases represent bimoment = 0 (force-free condition), not
    // structural elimination of the DOF. The warping DOF remains in the system
    // with its full stiffness coupling to torsion. The bimoment = 0 condition
    // is naturally satisfied when no warping loads are applied.
    // See: Vlasov beam theory for thin-walled sections.

    // Node j DOFs (indices 6-11 for 12-DOF or 7-13 for 14-DOF)
    int offset = has_warping ? 7 : 6;
    if (release_ux_j) indices.push_back(offset + 0);
    if (release_uy_j) indices.push_back(offset + 1);
    if (release_uz_j) indices.push_back(offset + 2);
    if (release_rx_j) indices.push_back(offset + 3);
    if (release_ry_j) indices.push_back(offset + 4);
    if (release_rz_j) indices.push_back(offset + 5);
    // NOTE: Warping releases at node j also NOT included (same reason as above)

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
    // For eigenvalue analysis, warping mass is needed for positive definite mass matrix.
    // Warping inertia: Iw_mass = rho * Iw * L / 3 (consistent mass formulation)
    // where Iw is the warping constant [m^6]
    double rho = material->rho;
    double Iw = section->Iw;
    double L = length;
    if (Iw > 0.0) {
        double warp_mass = rho * Iw * L / 3.0;
        M(6, 6) = warp_mass;
        M(13, 13) = warp_mass;
        // Coupling term for consistent mass (similar to axial mass)
        M(6, 13) = rho * Iw * L / 6.0;
        M(13, 6) = rho * Iw * L / 6.0;
    }

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
// Concentrated Load Equivalent Nodal Forces
// ============================================================================

std::pair<bool, double> BeamElement::point_on_element(
    const Eigen::Vector3d& point,
    double tolerance) const
{
    // Get node positions
    Eigen::Vector3d p1 = node_i->position();
    Eigen::Vector3d p2 = node_j->position();

    // Direction vector of beam
    Eigen::Vector3d d = p2 - p1;
    double L = d.norm();

    if (L < 1e-10) {
        // Degenerate element
        return {false, 0.0};
    }

    // Normalized direction
    Eigen::Vector3d d_norm = d / L;

    // Vector from p1 to point
    Eigen::Vector3d v = point - p1;

    // Project onto beam axis
    double t = v.dot(d_norm);

    // Parametric position (0 to 1)
    double xi = t / L;

    // Check if within element bounds (with small tolerance for endpoints)
    if (xi < -tolerance/L || xi > 1.0 + tolerance/L) {
        return {false, xi};
    }

    // Check perpendicular distance from beam axis
    Eigen::Vector3d closest_point = p1 + t * d_norm;
    double perp_distance = (point - closest_point).norm();

    if (perp_distance > tolerance) {
        return {false, xi};
    }

    // Clamp to [0, 1]
    xi = std::max(0.0, std::min(1.0, xi));

    return {true, xi};
}

Eigen::Matrix<double, 12, 1> BeamElement::equivalent_nodal_forces_concentrated(
    const Eigen::Vector3d& force,
    const Eigen::Vector3d& moment,
    double xi) const
{
    // Clamp xi to valid range
    xi = std::max(0.0, std::min(1.0, xi));

    // Get transformation matrix
    Eigen::Matrix<double, 12, 12> T = transformation_matrix();
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);  // Local-to-global rotation

    // Transform global force/moment to local coordinates
    Eigen::Vector3d f_local = R.transpose() * force;
    Eigen::Vector3d m_local = R.transpose() * moment;

    // Element length
    double L = length;

    // Shape function values at xi
    // N1 = (1 - xi)^2 * (1 + 2*xi)    - deflection at node i
    // N2 = xi * (1 - xi)^2 * L        - rotation at node i (scaled by L)
    // N3 = xi^2 * (3 - 2*xi)          - deflection at node j
    // N4 = -xi^2 * (1 - xi) * L       - rotation at node j (scaled by L)

    double one_minus_xi = 1.0 - xi;
    double xi2 = xi * xi;
    double one_minus_xi2 = one_minus_xi * one_minus_xi;

    double N1 = one_minus_xi2 * (1.0 + 2.0 * xi);
    double N2 = L * xi * one_minus_xi2;
    double N3 = xi2 * (3.0 - 2.0 * xi);
    double N4 = -L * xi2 * one_minus_xi;

    // Initialize local force vector
    Eigen::Matrix<double, 12, 1> f_equiv_local = Eigen::Matrix<double, 12, 1>::Zero();

    // ---------------------------
    // Axial force (local x)
    // ---------------------------
    // Linear distribution: F_i = P * (1 - xi), F_j = P * xi
    f_equiv_local(0) = f_local(0) * one_minus_xi;   // f_x at node i
    f_equiv_local(6) = f_local(0) * xi;             // f_x at node j

    // ---------------------------
    // Transverse force in local y (bending about z-axis)
    // ---------------------------
    f_equiv_local(1) = f_local(1) * N1;             // f_y at node i
    f_equiv_local(5) = f_local(1) * N2 / L;         // m_z at node i (N2 includes L, divide out for moment)
    f_equiv_local(7) = f_local(1) * N3;             // f_y at node j
    f_equiv_local(11) = f_local(1) * N4 / L;        // m_z at node j

    // Actually, the shape functions for equivalent nodal forces are:
    // For force P at position a from node i:
    // F_i = P * (1 - xi)^2 * (1 + 2*xi)
    // M_i = P * L * xi * (1 - xi)^2
    // F_j = P * xi^2 * (3 - 2*xi)
    // M_j = -P * L * xi^2 * (1 - xi)
    f_equiv_local(5) = f_local(1) * xi * one_minus_xi2 * L;     // m_z at node i
    f_equiv_local(11) = -f_local(1) * xi2 * one_minus_xi * L;   // m_z at node j

    // ---------------------------
    // Transverse force in local z (bending about y-axis)
    // ---------------------------
    f_equiv_local(2) = f_local(2) * N1;             // f_z at node i
    f_equiv_local(4) = -f_local(2) * xi * one_minus_xi2 * L;    // m_y at node i (sign convention)
    f_equiv_local(8) = f_local(2) * N3;             // f_z at node j
    f_equiv_local(10) = f_local(2) * xi2 * one_minus_xi * L;    // m_y at node j

    // ---------------------------
    // Torsion (moment about local x)
    // ---------------------------
    // Linear distribution: M_i = Mx * (1 - xi), M_j = Mx * xi
    f_equiv_local(3) = m_local(0) * one_minus_xi;   // m_x at node i
    f_equiv_local(9) = m_local(0) * xi;             // m_x at node j

    // ---------------------------
    // Applied moments about local y and z
    // ---------------------------
    // For applied moment at position xi, use shape function derivatives
    // Simplified: distribute linearly
    f_equiv_local(4) += m_local(1) * one_minus_xi;  // m_y at node i
    f_equiv_local(10) += m_local(1) * xi;           // m_y at node j
    f_equiv_local(5) += m_local(2) * one_minus_xi;  // m_z at node i
    f_equiv_local(11) += m_local(2) * xi;           // m_z at node j

    // Transform to global coordinates
    Eigen::Matrix<double, 12, 1> f_equiv_global = T.transpose() * f_equiv_local;

    return f_equiv_global;
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

DisplacementLine BeamElement::get_displacements_at(
    double x,
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler,
    const LoadCase* load_case) const
{
    // Clamp x to valid range [0, L]
    x = std::max(0.0, std::min(x, length));

    // Get local displacements at nodes
    Eigen::VectorXd u_local = get_element_displacements_local(global_displacements, dof_handler);

    // Normalized coordinate
    double L = length;
    double xi = x / L;
    double xi2 = xi * xi;
    double xi3 = xi2 * xi;

    // Extract local displacements at nodes
    // DOF ordering (12-DOF): [u_i, v_i, w_i, θx_i, θy_i, θz_i, u_j, v_j, w_j, θx_j, θy_j, θz_j]
    double u1 = u_local(0);        // Axial at i
    double v1 = u_local(1);        // y-displacement at i
    double w1 = u_local(2);        // z-displacement at i
    double theta_x1 = u_local(3);  // Torsion at i
    double theta_y1 = u_local(4);  // Rotation about y at i
    double theta_z1 = u_local(5);  // Rotation about z at i

    int offset = config.include_warping ? 7 : 6;
    double u2 = u_local(offset);        // Axial at j
    double v2 = u_local(offset + 1);    // y-displacement at j
    double w2 = u_local(offset + 2);    // z-displacement at j
    double theta_x2 = u_local(offset + 3);  // Torsion at j
    double theta_y2 = u_local(offset + 4);  // Rotation about y at j
    double theta_z2 = u_local(offset + 5);  // Rotation about z at j

    // Initialize result
    DisplacementLine result(x);

    // Get distributed loads if load case provided
    DistributedLoad q_y, q_z;
    if (load_case) {
        q_y = get_distributed_load_y(*load_case);
        q_z = get_distributed_load_z(*load_case);
    }

    // Axial displacement - linear interpolation (no distributed axial deflection effect)
    // u(x) = (1-xi)*u1 + xi*u2
    result.u = (1.0 - xi) * u1 + xi * u2;

    // Torsion - linear interpolation
    // θx(x) = (1-xi)*θx1 + xi*θx2
    result.theta_x = (1.0 - xi) * theta_x1 + xi * theta_x2;

    // Extract material and section properties for deflection computers
    double E = material->E;
    double EIz = E * section->Iz;
    double EIy = E * section->Iy;

    // Detect release combinations
    ReleaseCombo4DOF bending_y_release = detect_release_combination_bending_y();
    ReleaseCombo4DOF bending_z_release = detect_release_combination_bending_z();

    // Compute v (y-deflection) and θz (rotation about z)
    // For bending in x-y plane (about z-axis)
    if (config.get_formulation() == BeamFormulation::Timoshenko) {
        // For Timoshenko beams, use Hermite interpolation for displacement
        // and linear interpolation for rotation (θ ≠ dv/dx)
        double N1 = 1.0 - 3.0 * xi2 + 2.0 * xi3;
        double N2 = L * (xi - 2.0 * xi2 + xi3);
        double N3 = 3.0 * xi2 - 2.0 * xi3;
        double N4 = L * (-xi2 + xi3);

        result.v = N1 * v1 + N2 * theta_z1 + N3 * v2 + N4 * theta_z2;
        result.theta_z = (1.0 - xi) * theta_z1 + xi * theta_z2;
    } else {
        // For Euler-Bernoulli, use analytical deflection formulas
        DeflectionYEulerComputer v_computer(L, EIz, v1, theta_z1, v2, theta_z2,
                                             q_y.q_start, q_y.q_end);
        result.v = v_computer.compute(x, bending_z_release);

        // Use linear interpolation for rotation (same as Timoshenko)
        // The analytical rotation formula had issues with release detection
        // for single-element cantilevers, and rotation output is rarely used.
        result.theta_z = (1.0 - xi) * theta_z1 + xi * theta_z2;
    }

    // Compute w (z-deflection) and θy (rotation about y)
    // For bending in x-z plane (about y-axis)
    // Note: Sign convention - θy is related to -dw/dx
    if (config.get_formulation() == BeamFormulation::Timoshenko) {
        double N1 = 1.0 - 3.0 * xi2 + 2.0 * xi3;
        double N2 = L * (xi - 2.0 * xi2 + xi3);
        double N3 = 3.0 * xi2 - 2.0 * xi3;
        double N4 = L * (-xi2 + xi3);

        result.w = N1 * w1 - N2 * theta_y1 + N3 * w2 - N4 * theta_y2;
        result.theta_y = (1.0 - xi) * theta_y1 + xi * theta_y2;
    } else {
        // Use -θy as the slope input (sign convention)
        DeflectionZEulerComputer w_computer(L, EIy, w1, -theta_y1, w2, -theta_y2,
                                             q_z.q_start, q_z.q_end);
        result.w = w_computer.compute(x, bending_y_release);

        // Use linear interpolation for rotation (same as Timoshenko)
        // The analytical rotation formula had issues with release detection
        // for single-element cantilevers, and rotation output is rarely used.
        result.theta_y = (1.0 - xi) * theta_y1 + xi * theta_y2;
    }

    // Warping parameter (for 14-DOF elements)
    if (config.include_warping) {
        double phi1 = u_local(6);   // Rate of twist at i
        double phi2 = u_local(13);  // Rate of twist at j
        // Linear interpolation for warping parameter
        result.phi_prime = (1.0 - xi) * phi1 + xi * phi2;
    } else {
        result.phi_prime = 0.0;
    }

    return result;
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

// ============================================================================
// Task 7.0: Distributed Load Query Methods
// ============================================================================

DistributedLoad BeamElement::get_distributed_load_y(const LoadCase& load_case) const {
    DistributedLoad result;
    result.q_start = 0.0;
    result.q_end = 0.0;

    // Iterate through all line loads in the load case
    for (const auto& line_load : load_case.get_line_loads()) {
        // Check if this line load applies to this element
        if (line_load.element_id != id) {
            continue;
        }

        // Transform load from global to local coordinates
        // line_load.w_start and w_end are in global coordinates [kN/m]
        Eigen::Vector3d w_start_local = local_axes.to_local(line_load.w_start);
        Eigen::Vector3d w_end_local = local_axes.to_local(line_load.w_end);

        // Extract the y-component (index 1)
        // Accumulate in case there are multiple line loads on this element
        result.q_start += w_start_local(1);
        result.q_end += w_end_local(1);
    }

    // Add gravity contribution from acceleration field
    // q_gravity = rho * A * a (mass per unit length * acceleration)
    const Eigen::Vector<double, 6>& accel = load_case.get_acceleration();
    if (accel.head<3>().norm() > 1e-10) {
        double rho = material->rho;  // mT/m³
        double A = section->A;       // m²
        // Distributed load in global coordinates: q = rho * A * a [kN/m]
        Eigen::Vector3d q_gravity_global = rho * A * accel.head<3>();
        // Transform to local coordinates
        Eigen::Vector3d q_gravity_local = local_axes.to_local(q_gravity_global);
        // Add y-component (uniform along the element)
        result.q_start += q_gravity_local(1);
        result.q_end += q_gravity_local(1);
    }

    return result;
}

DistributedLoad BeamElement::get_distributed_load_z(const LoadCase& load_case) const {
    DistributedLoad result;
    result.q_start = 0.0;
    result.q_end = 0.0;

    // Iterate through all line loads in the load case
    for (const auto& line_load : load_case.get_line_loads()) {
        // Check if this line load applies to this element
        if (line_load.element_id != id) {
            continue;
        }

        // Transform load from global to local coordinates
        // line_load.w_start and w_end are in global coordinates [kN/m]
        Eigen::Vector3d w_start_local = local_axes.to_local(line_load.w_start);
        Eigen::Vector3d w_end_local = local_axes.to_local(line_load.w_end);

        // Extract the z-component (index 2)
        // Accumulate in case there are multiple line loads on this element
        result.q_start += w_start_local(2);
        result.q_end += w_end_local(2);
    }

    // Add gravity contribution from acceleration field
    // q_gravity = rho * A * a (mass per unit length * acceleration)
    const Eigen::Vector<double, 6>& accel = load_case.get_acceleration();
    if (accel.head<3>().norm() > 1e-10) {
        double rho = material->rho;  // mT/m³
        double A = section->A;       // m²
        // Distributed load in global coordinates: q = rho * A * a [kN/m]
        // (rho in mT/m³, A in m², a in m/s² → mT*m/s²/m = kN/m)
        Eigen::Vector3d q_gravity_global = rho * A * accel.head<3>();
        // Transform to local coordinates
        Eigen::Vector3d q_gravity_local = local_axes.to_local(q_gravity_global);
        // Add z-component (uniform along the element)
        result.q_start += q_gravity_local(2);
        result.q_end += q_gravity_local(2);
    }

    return result;
}

DistributedLoad BeamElement::get_distributed_load_axial(const LoadCase& load_case) const {
    DistributedLoad result;
    result.q_start = 0.0;
    result.q_end = 0.0;

    // Iterate through all line loads in the load case
    for (const auto& line_load : load_case.get_line_loads()) {
        // Check if this line load applies to this element
        if (line_load.element_id != id) {
            continue;
        }

        // Transform load from global to local coordinates
        // line_load.w_start and w_end are in global coordinates [kN/m]
        Eigen::Vector3d w_start_local = local_axes.to_local(line_load.w_start);
        Eigen::Vector3d w_end_local = local_axes.to_local(line_load.w_end);

        // Extract the x-component (index 0) - axial direction
        // Positive x is towards node_j
        // Accumulate in case there are multiple line loads on this element
        result.q_start += w_start_local(0);
        result.q_end += w_end_local(0);
    }

    // Add gravity contribution from acceleration field
    // q_gravity = rho * A * a (mass per unit length * acceleration)
    const Eigen::Vector<double, 6>& accel = load_case.get_acceleration();
    if (accel.head<3>().norm() > 1e-10) {
        double rho = material->rho;  // mT/m³
        double A = section->A;       // m²
        // Distributed load in global coordinates: q = rho * A * a [kN/m]
        Eigen::Vector3d q_gravity_global = rho * A * accel.head<3>();
        // Transform to local coordinates
        Eigen::Vector3d q_gravity_local = local_axes.to_local(q_gravity_global);
        // Add x-component (axial, uniform along the element)
        result.q_start += q_gravity_local(0);
        result.q_end += q_gravity_local(0);
    }

    return result;
}

// ============================================================================
// Task 7.2: Internal Action Functions Along Beam
// ============================================================================

ReleaseCombo4DOF BeamElement::detect_release_combination_bending_y() const {
    // Bending about y-axis: DOFs are w (z-displacement) and θy (rotation about y)
    // For each end: check release_uz (displacement) and release_ry (rotation)

    bool w1_free = releases.release_uz_i;
    bool phi1_free = releases.release_ry_i;
    bool w2_free = releases.release_uz_j;
    bool phi2_free = releases.release_ry_j;

    // Compute enum value: 4-bit number where each bit represents a free DOF
    int code = (w1_free ? 8 : 0) | (phi1_free ? 4 : 0) | (w2_free ? 2 : 0) | (phi2_free ? 1 : 0);

    return static_cast<ReleaseCombo4DOF>(code);
}

ReleaseCombo4DOF BeamElement::detect_release_combination_bending_z() const {
    // Bending about z-axis: DOFs are v (y-displacement) and θz (rotation about z)
    // For each end: check release_uy (displacement) and release_rz (rotation)

    bool w1_free = releases.release_uy_i;
    bool phi1_free = releases.release_rz_i;
    bool w2_free = releases.release_uy_j;
    bool phi2_free = releases.release_rz_j;

    // Compute enum value: 4-bit number where each bit represents a free DOF
    int code = (w1_free ? 8 : 0) | (phi1_free ? 4 : 0) | (w2_free ? 2 : 0) | (phi2_free ? 1 : 0);

    return static_cast<ReleaseCombo4DOF>(code);
}

ReleaseCombo2DOF BeamElement::detect_release_combination_axial() const {
    // Axial: DOFs are u1 (axial displacement at i) and u2 (at j)
    bool u1_free = releases.release_ux_i;
    bool u2_free = releases.release_ux_j;

    int code = (u1_free ? 2 : 0) | (u2_free ? 1 : 0);

    return static_cast<ReleaseCombo2DOF>(code);
}

ReleaseCombo2DOF BeamElement::detect_release_combination_torsion() const {
    // Torsion: DOFs are θx1 (torsion at i) and θx2 (at j)
    bool theta1_free = releases.release_rx_i;
    bool theta2_free = releases.release_rx_j;

    int code = (theta1_free ? 2 : 0) | (theta2_free ? 1 : 0);

    return static_cast<ReleaseCombo2DOF>(code);
}

InternalActions BeamElement::get_internal_actions(
    double x,
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler,
    const LoadCase* load_case) const
{
    // Clamp x to valid range [0, L]
    x = std::max(0.0, std::min(x, length));

    // Get local displacements
    Eigen::VectorXd u_local = get_element_displacements_local(global_displacements, dof_handler);

    // Extract material and section properties
    double E = material->E;
    double G = material->G;
    double A = section->A;
    double Iy = section->Iy;
    double Iz = section->Iz;
    double J = section->J;
    double L = length;

    // Extract local displacements
    // DOF ordering (12-DOF): [u_i, v_i, w_i, θx_i, θy_i, θz_i, u_j, v_j, w_j, θx_j, θy_j, θz_j]
    double u1 = u_local(0);   // Axial displacement at node i
    double v1 = u_local(1);   // y-displacement at node i
    double w1 = u_local(2);   // z-displacement at node i
    double theta_x1 = u_local(3);  // Torsion rotation at node i
    double theta_y1 = u_local(4);  // Rotation about y at node i
    double theta_z1 = u_local(5);  // Rotation about z at node i

    int offset = config.include_warping ? 7 : 6;
    double u2 = u_local(offset);      // Axial displacement at node j
    double v2 = u_local(offset + 1);  // y-displacement at node j
    double w2 = u_local(offset + 2);  // z-displacement at node j
    double theta_x2 = u_local(offset + 3);  // Torsion rotation at node j
    double theta_y2 = u_local(offset + 4);  // Rotation about y at node j
    double theta_z2 = u_local(offset + 5);  // Rotation about z at node j

    // Get distributed loads if load case provided
    DistributedLoad q_y, q_z, q_x;
    if (load_case) {
        q_y = get_distributed_load_y(*load_case);
        q_z = get_distributed_load_z(*load_case);
        q_x = get_distributed_load_axial(*load_case);
    }

    // Detect release combinations
    ReleaseCombo2DOF axial_release = detect_release_combination_axial();
    ReleaseCombo2DOF torsion_release = detect_release_combination_torsion();
    ReleaseCombo4DOF bending_y_release = detect_release_combination_bending_y();
    ReleaseCombo4DOF bending_z_release = detect_release_combination_bending_z();

    // Initialize result
    InternalActions result(x);

    // Compute axial force N
    AxialForceComputer axial_computer(L, E * A, u1, u2, q_x.q_start, q_x.q_end);
    result.N = axial_computer.compute(x, axial_release);

    // Compute torsion moment Mx
    TorsionComputer torsion_computer(L, G * J, theta_x1, theta_x2, 0.0, 0.0);
    result.Mx = torsion_computer.compute(x, torsion_release);

    // Compute bending about z-axis (Mz) and shear in y (Vy)
    // For bending in x-y plane: v is the displacement, θz is the rotation
    if (config.get_formulation() == BeamFormulation::Timoshenko) {
        // Timoshenko formulation
        double kappa = 5.0 / 6.0;
        double Asz = section->Asz > 0 ? section->Asz : kappa * A;
        double kAG_z = Asz * G;

        MomentZTimoshenkoComputer mz_computer(L, E * Iz, kAG_z,
                                               v1, theta_z1, v2, theta_z2,
                                               q_y.q_start, q_y.q_end);
        result.Mz = mz_computer.compute(x, bending_z_release);

        ShearYTimoshenkoComputer vy_computer(L, E * Iz, kAG_z,
                                              v1, theta_z1, v2, theta_z2,
                                              q_y.q_start, q_y.q_end);
        result.Vy = vy_computer.compute(x, bending_z_release);
    } else {
        // Euler-Bernoulli formulation
        MomentZEulerComputer mz_computer(L, E * Iz, v1, theta_z1, v2, theta_z2,
                                          q_y.q_start, q_y.q_end);
        result.Mz = mz_computer.compute(x, bending_z_release);

        ShearYEulerComputer vy_computer(L, E * Iz, v1, theta_z1, v2, theta_z2,
                                         q_y.q_start, q_y.q_end);
        result.Vy = vy_computer.compute(x, bending_z_release);
    }

    // Compute bending about y-axis (My) and shear in z (Vz)
    // For bending in x-z plane: w is the displacement, θy is the rotation
    // Note: Sign convention requires care for My (θy is related to -dw/dx for positive moment)
    if (config.get_formulation() == BeamFormulation::Timoshenko) {
        double kappa = 5.0 / 6.0;
        double Asy = section->Asy > 0 ? section->Asy : kappa * A;
        double kAG_y = Asy * G;

        MomentYTimoshenkoComputer my_computer(L, E * Iy, kAG_y,
                                               w1, -theta_y1, w2, -theta_y2,
                                               q_z.q_start, q_z.q_end);
        result.My = -my_computer.compute(x, bending_y_release);

        ShearZTimoshenkoComputer vz_computer(L, E * Iy, kAG_y,
                                              w1, -theta_y1, w2, -theta_y2,
                                              q_z.q_start, q_z.q_end);
        result.Vz = vz_computer.compute(x, bending_y_release);
    } else {
        MomentYEulerComputer my_computer(L, E * Iy, w1, -theta_y1, w2, -theta_y2,
                                          q_z.q_start, q_z.q_end);
        result.My = -my_computer.compute(x, bending_y_release);

        ShearZEulerComputer vz_computer(L, E * Iy, w1, -theta_y1, w2, -theta_y2,
                                         q_z.q_start, q_z.q_end);
        result.Vz = vz_computer.compute(x, bending_y_release);
    }

    return result;
}

std::pair<ActionExtreme, ActionExtreme> BeamElement::find_moment_extremes(
    char axis,
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler,
    const LoadCase* load_case) const
{
    // Algorithm:
    // 1. Compute moment and shear at endpoints
    // 2. Find interior critical points where V = 0 (shear = dM/dx = 0)
    // 3. Evaluate moment at all critical points
    // 4. Return min and max

    double L = length;
    const int num_sample_points = 21;  // For fallback sampling

    std::vector<double> critical_points;
    critical_points.push_back(0.0);  // Start
    critical_points.push_back(L);    // End

    // For distributed loads, try to find analytical zero-crossing of shear
    // Shear is typically linear or parabolic, so solve V(x) = 0 analytically
    // For now, use sampling approach for robustness

    // Sample along the beam to find shear zero crossings
    double prev_shear = 0.0;
    for (int i = 0; i <= num_sample_points; ++i) {
        double x_sample = L * i / num_sample_points;
        InternalActions actions = get_internal_actions(x_sample, global_displacements,
                                                       dof_handler, load_case);

        double shear = (axis == 'y') ? actions.Vy : actions.Vz;

        if (i > 0 && prev_shear * shear < 0.0) {
            // Sign change detected - find zero crossing using bisection
            double x_left = L * (i - 1) / num_sample_points;
            double x_right = x_sample;

            for (int iter = 0; iter < 10; ++iter) {
                double x_mid = (x_left + x_right) / 2.0;
                InternalActions actions_mid = get_internal_actions(x_mid, global_displacements,
                                                                    dof_handler, load_case);
                double shear_mid = (axis == 'y') ? actions_mid.Vy : actions_mid.Vz;

                InternalActions actions_left = get_internal_actions(x_left, global_displacements,
                                                                     dof_handler, load_case);
                double shear_left = (axis == 'y') ? actions_left.Vy : actions_left.Vz;

                if (shear_left * shear_mid < 0.0) {
                    x_right = x_mid;
                } else {
                    x_left = x_mid;
                }
            }

            critical_points.push_back((x_left + x_right) / 2.0);
        }

        prev_shear = shear;
    }

    // Evaluate moment at all critical points and find min/max
    ActionExtreme min_extremum(0.0, std::numeric_limits<double>::infinity());
    ActionExtreme max_extremum(0.0, -std::numeric_limits<double>::infinity());

    for (double x_crit : critical_points) {
        InternalActions actions = get_internal_actions(x_crit, global_displacements,
                                                       dof_handler, load_case);
        double M = (axis == 'y') ? actions.My : actions.Mz;

        if (M < min_extremum.value) {
            min_extremum.x = x_crit;
            min_extremum.value = M;
        }
        if (M > max_extremum.value) {
            max_extremum.x = x_crit;
            max_extremum.value = M;
        }
    }

    return {min_extremum, max_extremum};
}

// ============================================================================
// Task 7.2b: Warping Internal Actions
// ============================================================================

ReleaseComboWarping BeamElement::detect_release_combination_warping() const {
    // Map the end releases to the 16 warping combinations
    // Order: θ₁, φ₁, θ₂, φ₂ (torsion_i, warp_i, torsion_j, warp_j)
    //
    // In our EndRelease struct:
    // - release_rx_i/j = torsion release (θ)
    // - release_warp_i/j = warping release (φ)

    bool theta1_free = releases.release_rx_i;
    bool phi1_free = releases.release_warp_i;
    bool theta2_free = releases.release_rx_j;
    bool phi2_free = releases.release_warp_j;

    // Encode as 4-bit integer: (θ₁_free << 3) | (φ₁_free << 2) | (θ₂_free << 1) | φ₂_free
    int code = (theta1_free ? 8 : 0) | (phi1_free ? 4 : 0) |
               (theta2_free ? 2 : 0) | (phi2_free ? 1 : 0);

    // The enum values are ordered: FIXED_FIXED_FIXED_FIXED = 0, etc.
    // The bit pattern matches: 0000 = all fixed, 1111 = all free
    // But our enum uses FIXED=0 for each DOF, so we need to invert

    // code = 0 means all fixed (all bits 0)
    // code = 15 means all free (all bits 1)
    return static_cast<ReleaseComboWarping>(code);
}

WarpingInternalActions BeamElement::get_warping_internal_actions(
    double x,
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler) const
{
    // Get standard internal actions first
    InternalActions base = get_internal_actions(x, global_displacements, dof_handler);

    WarpingInternalActions result;
    result.x = base.x;
    result.N = base.N;
    result.Vy = base.Vy;
    result.Vz = base.Vz;
    result.Mx = base.Mx;
    result.My = base.My;
    result.Mz = base.Mz;

    // For 12-DOF elements or sections without warping, return with zero warping values
    if (!config.include_warping || section->Iw < 1e-20) {
        result.B = 0.0;
        result.Mx_sv = base.Mx;  // All torsion is St. Venant
        result.Mx_w = 0.0;
        result.sigma_w_max = 0.0;
        return result;
    }

    // Get local displacements for warping analysis
    Eigen::VectorXd u_local = get_element_displacements_local(global_displacements, dof_handler);

    // For 14-DOF element, extract warping DOFs
    // DOF ordering: [u_i, v_i, w_i, θx_i, θy_i, θz_i, φ'_i, u_j, v_j, w_j, θx_j, θy_j, θz_j, φ'_j]
    // Index 3 = θx_i (twist at i), Index 6 = φ'_i (rate of twist at i)
    // Index 10 = θx_j (twist at j), Index 13 = φ'_j (rate of twist at j)

    double theta1 = u_local(3);  // Twist angle at node i
    double theta2 = u_local(10); // Twist angle at node j
    double phi1 = u_local(6);    // Rate of twist at node i (warping DOF)
    double phi2 = u_local(13);   // Rate of twist at node j (warping DOF)

    // Material and section properties
    double E = material->E;
    double G = material->G;
    double J = section->J;
    double Iw = section->Iw;

    double GJ = G * J;
    double EIw = E * Iw;

    // Create warping torsion computer
    WarpingTorsionComputer computer(length, GJ, EIw, theta1, phi1, theta2, phi2);

    // Detect release combination
    ReleaseComboWarping release = detect_release_combination_warping();

    // Compute warping results
    WarpingTorsionResults warping = computer.compute(x, release);

    // Populate result
    result.B = warping.B;
    result.Mx_sv = warping.Mx_sv;
    result.Mx_w = warping.Mx_w;
    result.Mx = warping.Mx_total;  // Override Mx with total torsion

    // Compute warping stress
    result.sigma_w_max = compute_warping_stress(warping.B);

    return result;
}

double BeamElement::compute_warping_stress(double bimoment) const {
    // σ_w = -B × ω_max / Iw
    //
    // where:
    // - B = bimoment [kN·m²]
    // - ω_max = maximum sectorial coordinate [m²]
    // - Iw = warping constant [m⁶]
    //
    // Units: [kN·m²] × [m²] / [m⁶] = [kN/m²]

    if (section->Iw < 1e-20) {
        return 0.0;  // No warping possible
    }

    double omega_max = section->omega_max;
    if (omega_max < 1e-20) {
        // If omega_max is not set, estimate from section geometry
        // For I-sections: ω_max ≈ h × b / 4 where h=height, b=flange width
        // This is a rough approximation; accurate ω_max should be provided
        return 0.0;
    }

    return -bimoment * omega_max / section->Iw;
}

// ============================================================================
// Task 7.2c: Displacement/Rotation Lines
// ============================================================================

// DEPRECATED: This overload uses Hermite interpolation only and does not include
// distributed load effects. Use get_displacements_at(x, displacements, dof_handler, load_case)
// with load_case=nullptr for the same behavior, or provide a load_case for accurate results
// under distributed loads.
DisplacementLine BeamElement::get_displacements_at(
    double x,
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler) const
{
    // Clamp x to valid range [0, L]
    x = std::max(0.0, std::min(x, length));

    // Get local displacements
    Eigen::VectorXd u_local = get_element_displacements_local(global_displacements, dof_handler);

    // Normalized position
    double L = length;
    double xi = x / L;  // ξ ∈ [0, 1]

    // Powers of xi for shape functions
    double xi2 = xi * xi;
    double xi3 = xi2 * xi;

    // Initialize result
    DisplacementLine result(x);

    // Extract local displacements at element ends
    // DOF ordering (12-DOF): [u_i, v_i, w_i, θx_i, θy_i, θz_i, u_j, v_j, w_j, θx_j, θy_j, θz_j]
    double u1 = u_local(0);      // Axial displacement at node i
    double v1 = u_local(1);      // y-displacement at node i
    double w1 = u_local(2);      // z-displacement at node i
    double theta_x1 = u_local(3);  // Torsion rotation at node i
    double theta_y1 = u_local(4);  // Rotation about y at node i
    double theta_z1 = u_local(5);  // Rotation about z at node i

    int offset = config.include_warping ? 7 : 6;
    double u2 = u_local(offset);      // Axial displacement at node j
    double v2 = u_local(offset + 1);  // y-displacement at node j
    double w2 = u_local(offset + 2);  // z-displacement at node j
    double theta_x2 = u_local(offset + 3);  // Torsion rotation at node j
    double theta_y2 = u_local(offset + 4);  // Rotation about y at node j
    double theta_z2 = u_local(offset + 5);  // Rotation about z at node j

    // Warping DOFs for 14-DOF elements
    double phi_prime_1 = 0.0;
    double phi_prime_2 = 0.0;
    if (config.include_warping) {
        phi_prime_1 = u_local(6);   // Rate of twist at node i
        phi_prime_2 = u_local(13);  // Rate of twist at node j
    }

    // ===========================================
    // Axial displacement: Linear interpolation
    // ===========================================
    // u(x) = N1_lin * u1 + N2_lin * u2
    // N1_lin = (1 - ξ), N2_lin = ξ
    result.u = (1.0 - xi) * u1 + xi * u2;

    // ===========================================
    // Torsion: Linear interpolation
    // ===========================================
    // θx(x) = N1_lin * θx1 + N2_lin * θx2
    result.theta_x = (1.0 - xi) * theta_x1 + xi * theta_x2;

    // ===========================================
    // Bending in x-y plane (lateral displacement v)
    // ===========================================
    // Hermite shape functions for displacement:
    //   N1(ξ) = 1 - 3ξ² + 2ξ³       (displacement at i)
    //   N2(ξ) = L(ξ - 2ξ² + ξ³)     (rotation at i)
    //   N3(ξ) = 3ξ² - 2ξ³           (displacement at j)
    //   N4(ξ) = L(-ξ² + ξ³)         (rotation at j)
    //
    // v(x) = N1*v1 + N2*θz1 + N3*v2 + N4*θz2

    double N1 = 1.0 - 3.0*xi2 + 2.0*xi3;
    double N2 = L * (xi - 2.0*xi2 + xi3);
    double N3 = 3.0*xi2 - 2.0*xi3;
    double N4 = L * (-xi2 + xi3);

    result.v = N1 * v1 + N2 * theta_z1 + N3 * v2 + N4 * theta_z2;

    // Rotation about z-axis (θz = dv/dx for Euler-Bernoulli)
    // Shape function derivatives:
    //   N1' = (-6ξ + 6ξ²) / L
    //   N2' = (1 - 4ξ + 3ξ²)
    //   N3' = (6ξ - 6ξ²) / L
    //   N4' = (-2ξ + 3ξ²)
    //
    // θz(x) = dv/dx = N1'*v1 + N2'*θz1 + N3'*v2 + N4'*θz2

    double dN1 = (-6.0*xi + 6.0*xi2) / L;
    double dN2 = (1.0 - 4.0*xi + 3.0*xi2);
    double dN3 = (6.0*xi - 6.0*xi2) / L;
    double dN4 = (-2.0*xi + 3.0*xi2);

    if (config.get_formulation() == BeamFormulation::Timoshenko) {
        // For Timoshenko beams, rotation is an independent DOF
        // Use linear interpolation for the rotation itself
        result.theta_z = (1.0 - xi) * theta_z1 + xi * theta_z2;
    } else {
        // For Euler-Bernoulli, rotation equals slope: θz = dv/dx
        result.theta_z = dN1 * v1 + dN2 * theta_z1 + dN3 * v2 + dN4 * theta_z2;
    }

    // ===========================================
    // Bending in x-z plane (lateral displacement w)
    // ===========================================
    // w(x) = N1*w1 + N2*(-θy1) + N3*w2 + N4*(-θy2)
    // Note: The sign convention for θy is such that dw/dx = -θy for positive bending

    result.w = N1 * w1 - N2 * theta_y1 + N3 * w2 - N4 * theta_y2;

    // Rotation about y-axis (θy = -dw/dx for Euler-Bernoulli, consistent with sign convention)
    if (config.get_formulation() == BeamFormulation::Timoshenko) {
        // For Timoshenko beams, rotation is an independent DOF
        result.theta_y = (1.0 - xi) * theta_y1 + xi * theta_y2;
    } else {
        // For Euler-Bernoulli, rotation is related to slope: θy = -dw/dx
        result.theta_y = -(dN1 * w1 - dN2 * theta_y1 + dN3 * w2 - dN4 * theta_y2);
    }

    // ===========================================
    // Warping parameter (for 14-DOF elements)
    // ===========================================
    if (config.include_warping) {
        // Linear interpolation for rate of twist
        result.phi_prime = (1.0 - xi) * phi_prime_1 + xi * phi_prime_2;
    }

    return result;
}

} // namespace grillex
