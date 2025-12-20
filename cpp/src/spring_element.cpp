#include "grillex/spring_element.hpp"
#include <cmath>

namespace grillex {

SpringElement::SpringElement(int id, Node* node_i, Node* node_j)
    : id(id), node_i(node_i), node_j(node_j) {}

Eigen::Matrix<double, 12, 12> SpringElement::global_stiffness_matrix() const {
    Eigen::Matrix<double, 12, 12> K = Eigen::Matrix<double, 12, 12>::Zero();

    // Stiffness values for each DOF
    std::array<double, 6> k_values = {kx, ky, kz, krx, kry, krz};

    // Build stiffness matrix
    // For each DOF pair (i at node_i, i+6 at node_j):
    //   K[i,i] = +k
    //   K[i,i+6] = -k
    //   K[i+6,i] = -k
    //   K[i+6,i+6] = +k
    for (int i = 0; i < 6; ++i) {
        double k = k_values[i];
        if (std::abs(k) > 1e-20) {
            K(i, i) = k;
            K(i, i + 6) = -k;
            K(i + 6, i) = -k;
            K(i + 6, i + 6) = k;
        }
    }

    return K;
}

Eigen::Matrix<double, 12, 12> SpringElement::global_mass_matrix() const {
    // Springs are massless - return zero matrix
    return Eigen::Matrix<double, 12, 12>::Zero();
}

void SpringElement::set_translational_stiffness(double kx_val, double ky_val, double kz_val) {
    kx = kx_val;
    ky = ky_val;
    kz = kz_val;
}

void SpringElement::set_rotational_stiffness(double krx_val, double kry_val, double krz_val) {
    krx = krx_val;
    kry = kry_val;
    krz = krz_val;
}

bool SpringElement::has_stiffness() const {
    return std::abs(kx) > 1e-20 || std::abs(ky) > 1e-20 || std::abs(kz) > 1e-20 ||
           std::abs(krx) > 1e-20 || std::abs(kry) > 1e-20 || std::abs(krz) > 1e-20;
}

bool SpringElement::is_active_for_load_case(LoadCaseType type) const {
    switch (loading_condition) {
        case LoadingCondition::All:
            return true;
        case LoadingCondition::Static:
            // Static connections only active for Permanent (gravity, dead load)
            return (type == LoadCaseType::Permanent);
        case LoadingCondition::Dynamic:
            // Dynamic connections active for all non-permanent load cases
            return (type == LoadCaseType::Variable ||
                    type == LoadCaseType::Environmental ||
                    type == LoadCaseType::Accidental);
    }
    return true;  // Default: always active
}

} // namespace grillex
