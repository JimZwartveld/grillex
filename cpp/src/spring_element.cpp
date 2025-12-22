#include "grillex/spring_element.hpp"
#include "grillex/dof_handler.hpp"
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

// === Nonlinear spring methods (Phase 15) ===

void SpringElement::update_state(const Eigen::VectorXd& u,
                                  const DOFHandler& dof_handler) {
    // Store previous state to detect changes
    std::array<bool, 6> previous_state = is_active;

    // Get DOF indices for both nodes
    for (int i = 0; i < 6; ++i) {
        int dof_i = dof_handler.get_global_dof(node_i->id, i);
        int dof_j = dof_handler.get_global_dof(node_j->id, i);

        // Compute deformation: δ = u_j - u_i
        double u_i_val = (dof_i >= 0 && dof_i < u.size()) ? u(dof_i) : 0.0;
        double u_j_val = (dof_j >= 0 && dof_j < u.size()) ? u(dof_j) : 0.0;
        deformation[i] = u_j_val - u_i_val;

        double g = gap[i];
        double delta = deformation[i];

        // Update active state based on behavior and gap
        switch (behavior[i]) {
            case SpringBehavior::Linear:
                // Linear spring: always active (gap handling for linear is rare but supported)
                if (g <= gap_tolerance_) {
                    // No gap: always active
                    is_active[i] = true;
                } else {
                    // Linear with gap: active when |δ| > gap
                    is_active[i] = (std::abs(delta) > g - gap_tolerance_);
                }
                break;

            case SpringBehavior::TensionOnly:
                // Active when elongation exceeds gap: δ > gap
                is_active[i] = (delta > g - gap_tolerance_);
                break;

            case SpringBehavior::CompressionOnly:
                // Active when compression exceeds gap: δ < -gap
                is_active[i] = (delta < -g + gap_tolerance_);
                break;
        }
    }

    // Check if any state changed
    state_changed_ = (previous_state != is_active);
}

void SpringElement::update_state_with_hysteresis(const Eigen::VectorXd& u,
                                                   const DOFHandler& dof_handler,
                                                   double hysteresis_band) {
    // Store previous state to detect changes
    std::array<bool, 6> previous_state = is_active;

    // Get DOF indices for both nodes
    for (int i = 0; i < 6; ++i) {
        int dof_i = dof_handler.get_global_dof(node_i->id, i);
        int dof_j = dof_handler.get_global_dof(node_j->id, i);

        // Compute deformation: δ = u_j - u_i
        double u_i_val = (dof_i >= 0 && dof_i < u.size()) ? u(dof_i) : 0.0;
        double u_j_val = (dof_j >= 0 && dof_j < u.size()) ? u(dof_j) : 0.0;
        deformation[i] = u_j_val - u_i_val;

        double g = gap[i];
        double delta = deformation[i];
        double hyst = hysteresis_band;

        // Update active state with hysteresis:
        // - Different thresholds for activation (gap + hyst) vs deactivation (gap - hyst)
        // - This prevents chattering when deformation oscillates around threshold
        switch (behavior[i]) {
            case SpringBehavior::Linear:
                // Linear spring: always active (gap handling for linear is rare but supported)
                if (g <= gap_tolerance_) {
                    is_active[i] = true;
                } else {
                    // Linear with gap: use hysteresis for bidirectional gap
                    if (is_active[i]) {
                        // Currently active: deactivate when within gap - hysteresis
                        is_active[i] = (std::abs(delta) > g - hyst - gap_tolerance_);
                    } else {
                        // Currently inactive: activate when outside gap + hysteresis
                        is_active[i] = (std::abs(delta) > g + hyst - gap_tolerance_);
                    }
                }
                break;

            case SpringBehavior::TensionOnly:
                // Active when elongation exceeds gap
                if (is_active[i]) {
                    // Currently active: deactivate when δ < gap - hysteresis
                    is_active[i] = (delta > g - hyst - gap_tolerance_);
                } else {
                    // Currently inactive: activate when δ > gap + hysteresis
                    is_active[i] = (delta > g + hyst - gap_tolerance_);
                }
                break;

            case SpringBehavior::CompressionOnly:
                // Active when compression exceeds gap: δ < -gap
                if (is_active[i]) {
                    // Currently active: deactivate when δ > -gap + hysteresis
                    is_active[i] = (delta < -g + hyst + gap_tolerance_);
                } else {
                    // Currently inactive: activate when δ < -gap - hysteresis
                    is_active[i] = (delta < -g - hyst + gap_tolerance_);
                }
                break;
        }
    }

    // Check if any state changed
    state_changed_ = (previous_state != is_active);
}

bool SpringElement::has_gap() const {
    for (int i = 0; i < 6; ++i) {
        if (std::abs(gap[i]) > gap_tolerance_) {
            return true;
        }
    }
    return false;
}

bool SpringElement::is_nonlinear() const {
    for (int i = 0; i < 6; ++i) {
        if (behavior[i] != SpringBehavior::Linear) {
            return true;
        }
    }
    return false;
}

std::array<double, 6> SpringElement::compute_forces() const {
    std::array<double, 6> forces = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> k_values = {{kx, ky, kz, krx, kry, krz}};

    for (int i = 0; i < 6; ++i) {
        if (!is_active[i]) {
            forces[i] = 0.0;
            continue;
        }

        double k = k_values[i];
        double delta = deformation[i];
        double g = gap[i];

        switch (behavior[i]) {
            case SpringBehavior::Linear:
                if (g <= gap_tolerance_) {
                    // No gap: F = k × δ
                    forces[i] = k * delta;
                } else if (delta > g) {
                    // Tension beyond gap: F = k × (δ - gap)
                    forces[i] = k * (delta - g);
                } else if (delta < -g) {
                    // Compression beyond gap: F = k × (δ + gap)
                    forces[i] = k * (delta + g);
                } else {
                    // Within gap: no force
                    forces[i] = 0.0;
                }
                break;

            case SpringBehavior::TensionOnly:
                // F = k × (δ - gap)
                forces[i] = k * (delta - g);
                break;

            case SpringBehavior::CompressionOnly:
                // F = k × (δ + gap), where δ is negative
                forces[i] = k * (delta + g);
                break;
        }
    }
    return forces;
}

Eigen::Matrix<double, 12, 1> SpringElement::compute_gap_forces() const {
    // Returns the gap closure forces to add to F vector
    // F_gap = ±k×gap for active gap springs

    Eigen::Matrix<double, 12, 1> F_gap = Eigen::Matrix<double, 12, 1>::Zero();
    std::array<double, 6> k_values = {{kx, ky, kz, krx, kry, krz}};

    for (int i = 0; i < 6; ++i) {
        if (!is_active[i] || gap[i] <= gap_tolerance_) continue;

        double k = k_values[i];
        double g = gap[i];
        double delta = deformation[i];

        double f_gap = 0.0;
        switch (behavior[i]) {
            case SpringBehavior::TensionOnly:
                // Gap force pulls nodes together: -k×gap
                // F = k × (δ - gap) = k×δ - k×gap
                // The k×δ is in the stiffness contribution
                // The -k×gap is what we add to RHS
                f_gap = -k * g;
                break;

            case SpringBehavior::CompressionOnly:
                // Gap force pushes nodes apart: +k×gap
                // F = k × (δ + gap) = k×δ + k×gap
                // The k×δ is in the stiffness contribution
                // The +k×gap is what we add to RHS
                f_gap = k * g;
                break;

            case SpringBehavior::Linear:
                // Bidirectional gap - determine based on current state
                if (delta > g) {
                    f_gap = -k * g;  // Tension
                } else if (delta < -g) {
                    f_gap = k * g;   // Compression
                }
                break;
        }

        // Apply to DOF indices
        // Force on node_i: opposite sign
        // Force on node_j: same sign as f_gap
        F_gap(i) = -f_gap;      // Force on node_i
        F_gap(i + 6) = f_gap;   // Force on node_j (opposite)
    }

    return F_gap;
}

Eigen::Matrix<double, 12, 12> SpringElement::current_stiffness_matrix() const {
    Eigen::Matrix<double, 12, 12> K = Eigen::Matrix<double, 12, 12>::Zero();
    std::array<double, 6> k_values = {{kx, ky, kz, krx, kry, krz}};

    for (int i = 0; i < 6; ++i) {
        if (!is_active[i]) continue;  // Skip inactive DOFs

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

void SpringElement::set_behavior(int dof, SpringBehavior b) {
    if (dof >= 0 && dof < 6) {
        behavior[dof] = b;
    }
}

void SpringElement::set_all_behavior(SpringBehavior b) {
    behavior.fill(b);
}

void SpringElement::set_gap(int dof, double g) {
    if (dof >= 0 && dof < 6) {
        gap[dof] = g;
    }
}

void SpringElement::set_all_gaps(double g) {
    gap.fill(g);
}

} // namespace grillex
