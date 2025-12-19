#include "grillex/internal_actions_computer.hpp"

namespace grillex {

// ============================================================================
// AxialForceComputer Implementation
// ============================================================================

AxialForceComputer::AxialForceComputer(double L, double EA, double u1, double u2,
                                       double q1, double q2)
    : L_(L), EA_(EA), u1_(u1), u2_(u2), q1_(q1), q2_(q2) {}

double AxialForceComputer::compute(double x, ReleaseCombo2DOF release) const {
    switch (release) {
        case ReleaseCombo2DOF::FIXED_FIXED:
            return fixed_fixed(x);
        case ReleaseCombo2DOF::FIXED_FREE:
            return fixed_free(x);
        case ReleaseCombo2DOF::FREE_FIXED:
            return free_fixed(x);
        case ReleaseCombo2DOF::FREE_FREE:
            return 0.0;  // Rigid body motion, no internal force
        default:
            throw std::runtime_error("Unknown axial release combination");
    }
}

double AxialForceComputer::fixed_fixed(double x) const {
    // Both ends restrained axially
    // From pystructeng: N(x) = (6*EA*(-u1 + u2) + L*(2*L*q1 + L*q2 - 6*q1*x)
    //                           + 3*x^2*(q1 - q2)) / (6*L)
    //
    // Note: This formula includes both the displacement-induced force
    // and the distributed load contribution.
    //
    // For pure displacement (no distributed load, q1=q2=0):
    //   N = EA * (u2 - u1) / L  (constant along beam)
    //
    // For distributed load on statically indeterminate bar:
    //   The formula integrates the load effect accounting for both end restraints.

    double L = L_;
    double EA = EA_;
    double u1 = u1_;
    double u2 = u2_;
    double q1 = q1_;
    double q2 = q2_;

    return (6.0 * EA * (-u1 + u2)
            + L * (2.0 * L * q1 + L * q2 - 6.0 * q1 * x)
            + 3.0 * x * x * (q1 - q2)) / (6.0 * L);
}

double AxialForceComputer::fixed_free(double x) const {
    // Start fixed, end free (cantilever-like for axial)
    // From pystructeng: N(x) = (L*(L*(q1 + q2) - 2*q1*x) + x^2*(q1 - q2)) / (2*L)
    //
    // For statically determinate case:
    //   End displacement u2 has no influence (free end)
    //   Axial force = integral of distributed load from x to L

    double L = L_;
    double q1 = q1_;
    double q2 = q2_;

    // The formula represents the axial force due to distributed load only
    // (displacements don't generate force when end is free)
    return (L * (L * (q1 + q2) - 2.0 * q1 * x) + x * x * (q1 - q2)) / (2.0 * L);
}

double AxialForceComputer::free_fixed(double x) const {
    // Start free, end fixed
    // From pystructeng: N(x) = x*(-2*L*q1 + x*(q1 - q2)) / (2*L)
    //
    // For statically determinate case:
    //   Start displacement u1 has no influence (free end)
    //   Axial force = integral of distributed load from 0 to x

    double L = L_;
    double q1 = q1_;
    double q2 = q2_;

    return x * (-2.0 * L * q1 + x * (q1 - q2)) / (2.0 * L);
}

// ============================================================================
// TorsionComputer Implementation
// ============================================================================

TorsionComputer::TorsionComputer(double L, double GJ, double theta1, double theta2,
                                 double m1, double m2)
    : L_(L), GJ_(GJ), theta1_(theta1), theta2_(theta2), m1_(m1), m2_(m2) {}

double TorsionComputer::compute(double x, ReleaseCombo2DOF release) const {
    switch (release) {
        case ReleaseCombo2DOF::FIXED_FIXED:
            return fixed_fixed(x);
        case ReleaseCombo2DOF::FIXED_FREE:
            return fixed_free(x);
        case ReleaseCombo2DOF::FREE_FIXED:
            return free_fixed(x);
        case ReleaseCombo2DOF::FREE_FREE:
            return 0.0;  // Rigid body rotation, no internal moment
        default:
            throw std::runtime_error("Unknown torsion release combination");
    }
}

double TorsionComputer::fixed_fixed(double x) const {
    // Both ends restrained in torsion (standard St. Venant)
    // Mx = GJ * dθ/dx
    // For linear θ(x): θ(x) = θ1 + (θ2 - θ1) * x / L
    // dθ/dx = (θ2 - θ1) / L
    // Mx = GJ * (θ2 - θ1) / L
    //
    // With distributed torsion m(x):
    // Analogous to axial force formulas

    double L = L_;
    double GJ = GJ_;
    double theta1 = theta1_;
    double theta2 = theta2_;
    double m1 = m1_;
    double m2 = m2_;

    // Base torsion from twist angle difference (uniform along length)
    double Mx_disp = GJ * (theta2 - theta1) / L;

    // Contribution from distributed torsion (if any)
    // Using similar formula structure as axial
    double Mx_load = (L * (2.0 * L * m1 + L * m2 - 6.0 * m1 * x)
                      + 3.0 * x * x * (m1 - m2)) / (6.0 * L);

    return Mx_disp + Mx_load;
}

double TorsionComputer::fixed_free(double x) const {
    // Start fixed, end free
    // End rotation θ2 has no influence (free end)

    double L = L_;
    double m1 = m1_;
    double m2 = m2_;

    // Torsion from distributed load only
    return (L * (L * (m1 + m2) - 2.0 * m1 * x) + x * x * (m1 - m2)) / (2.0 * L);
}

double TorsionComputer::free_fixed(double x) const {
    // Start free, end fixed

    double L = L_;
    double m1 = m1_;
    double m2 = m2_;

    return x * (-2.0 * L * m1 + x * (m1 - m2)) / (2.0 * L);
}

} // namespace grillex
