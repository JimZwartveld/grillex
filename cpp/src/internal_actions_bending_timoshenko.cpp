#include "grillex/internal_actions_computer.hpp"

namespace grillex {

// ============================================================================
// MomentZTimoshenkoComputer Implementation
// ============================================================================

MomentZTimoshenkoComputer::MomentZTimoshenkoComputer(double L, double EI, double kAG,
                                                      double w1, double phi1,
                                                      double w2, double phi2,
                                                      double q1, double q2)
    : L_(L), EI_(EI), kAG_(kAG), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2),
      q1_(q1), q2_(q2)
{
    // Shear deformation parameter Φ = 12EI / (L² * kAG)
    // When Φ → 0, Timoshenko reduces to Euler-Bernoulli
    Phi_ = (kAG > 0.0) ? (12.0 * EI / (L * L * kAG)) : 0.0;
}

double MomentZTimoshenkoComputer::compute(double x, ReleaseCombo4DOF release) const {
    // For Timoshenko beams, we implement the most common cases
    // Other cases fall back to Euler-Bernoulli approximation
    switch (release) {
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
            return fixed_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE:
            return fixed_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED:
            return fixed_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE:
            return fixed_free_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE:
            return fixed_fixed_free_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED:
            return free_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FREE:
            return 0.0;
        default:
            // For other release combinations, use Euler-Bernoulli as approximation
            // (The shear deformation effect is typically small for these cases)
            MomentZEulerComputer euler(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
            return euler.compute(x, release);
    }
}

double MomentZTimoshenkoComputer::fixed_fixed_fixed_fixed(double x) const {
    // Timoshenko fixed-fixed beam moment
    // The denominator includes shear deformation effect: (1 + Φ)
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;
    double Phi = Phi_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;

    // Modified formula accounting for shear deformation
    // As Φ → 0, this reduces to Euler-Bernoulli formula
    double denom = 1.0 + Phi;

    // Displacement contribution (modified for Timoshenko)
    double M_disp = (-120.0 * EI * L2 * (2.0 * phi1 + phi2) / denom
                     + 360.0 * EI * L * (-w1 + w2) / denom
                     + 3.0 * x * (120.0 * EI * L * (phi1 + phi2) / denom
                                 + 240.0 * EI * (w1 - w2) / denom))
                    / (60.0 * L3);

    // Load contribution (same as Euler for distributed load)
    double M_load = (L3 * (3.0 * L2 * q1 + 2.0 * L2 * q2 + 30.0 * q1 * x * x)
                     + 10.0 * L2 * x * x * x * (-q1 + q2)
                     - 3.0 * x * L4 * (7.0 * q1 + 3.0 * q2))
                    / (60.0 * L3);

    return M_disp + M_load;
}

double MomentZTimoshenkoComputer::fixed_fixed_fixed_free(double x) const {
    // Propped cantilever with Timoshenko effect
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_;
    double q1 = q1_, q2 = q2_;
    double Phi = Phi_;

    double L2 = L * L;
    double L3 = L2 * L;
    double denom = 3.0 + Phi;  // Modified stiffness distribution

    // Displacement contribution
    double M_disp = (48.0 * EI * L * (w2 - w1) / denom
                     - 24.0 * EI * L2 * phi1 * (3.0 / denom)
                     + x * (48.0 * EI * (w1 - w2) / denom
                           + 24.0 * EI * L * phi1 * (3.0 / denom)))
                    / (24.0 * L3);

    // Load contribution
    double M_load = (L3 * (L * (9.0 * q1 + 11.0 * q2) - 20.0 * x * (q1 + q2))
                     + 10.0 * L * x * x * (q1 + q2)
                     - x * L3 * (9.0 * q1 + 11.0 * q2))
                    / (24.0 * L3);

    return M_disp + M_load;
}

double MomentZTimoshenkoComputer::fixed_free_fixed_fixed(double x) const {
    // Hinge at i, fully fixed at j with Timoshenko effect
    double L = L_, EI = EI_;
    double w1 = w1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;
    double Phi = Phi_;

    double L2 = L * L;
    double L3 = L2 * L;
    double denom = 3.0 + Phi;

    double M_disp = (48.0 * EI * L * (w2 - w1) / denom
                     + 24.0 * EI * L2 * phi2 * (3.0 / denom)
                     + x * (48.0 * EI * (w1 - w2) / denom
                           - 24.0 * EI * L * phi2 * (3.0 / denom)))
                    / (24.0 * L3);

    double M_load = (-L3 * (L * (11.0 * q1 + 9.0 * q2) - 20.0 * x * (q1 + q2))
                     - 10.0 * L * x * x * (q1 + q2)
                     + x * L3 * (11.0 * q1 + 9.0 * q2))
                    / (24.0 * L3);

    return M_disp + M_load;
}

double MomentZTimoshenkoComputer::fixed_free_fixed_free(double x) const {
    // Simply supported beam (double hinge)
    // Timoshenko effect doesn't change moment diagram for SS beam
    // (only affects deflection)
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Simply supported beam moment
    double R1 = (L * (7.0 * q1 + 3.0 * q2)) / 20.0;
    return R1 * x - (q1 * x * x / 2.0) - (q2 - q1) * x * x * x / (6.0 * L);
}

double MomentZTimoshenkoComputer::fixed_fixed_free_free(double x) const {
    // Cantilever - same as Euler (no change in statics)
    double L = L_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;

    return (L2 * (q1 + q2)
            - 2.0 * L * x * (q1 + q2)
            + x * x * (q1 + q2)
            + (q2 - q1) * (L2 * L - 3.0 * L * x * x + 2.0 * x * x * x) / (6.0 * L))
           / 2.0;
}

double MomentZTimoshenkoComputer::free_free_fixed_fixed(double x) const {
    // Cantilever from j end - same as Euler
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return -(q1 * x * x / 2.0 + (q2 - q1) * x * x * x / (6.0 * L));
}

// ============================================================================
// ShearYTimoshenkoComputer Implementation
// ============================================================================

ShearYTimoshenkoComputer::ShearYTimoshenkoComputer(double L, double EI, double kAG,
                                                    double w1, double phi1,
                                                    double w2, double phi2,
                                                    double q1, double q2)
    : L_(L), EI_(EI), kAG_(kAG), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2),
      q1_(q1), q2_(q2)
{
    Phi_ = (kAG > 0.0) ? (12.0 * EI / (L * L * kAG)) : 0.0;
}

double ShearYTimoshenkoComputer::compute(double x, ReleaseCombo4DOF release) const {
    switch (release) {
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
            return fixed_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE:
            return fixed_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED:
            return fixed_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE:
            return fixed_free_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE:
            return fixed_fixed_free_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED:
            return free_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FREE:
            return 0.0;
        default:
            ShearYEulerComputer euler(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
            return euler.compute(x, release);
    }
}

double ShearYTimoshenkoComputer::fixed_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;
    double Phi = Phi_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;
    double denom = 1.0 + Phi;

    // Displacement contribution (shear from derivative of moment)
    double V_disp = 3.0 * (120.0 * EI * L * (phi1 + phi2) / denom
                          + 240.0 * EI * (w1 - w2) / denom)
                    / (60.0 * L3);

    // Load contribution (same as Euler for distributed load on shear)
    double V_load = (60.0 * L2 * q1 * x + 30.0 * L2 * x * x * (-q1 + q2)
                     - 3.0 * L4 * (7.0 * q1 + 3.0 * q2))
                    / (60.0 * L3);

    return V_disp + V_load;
}

double ShearYTimoshenkoComputer::fixed_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_;
    double q1 = q1_, q2 = q2_;
    double Phi = Phi_;

    double L2 = L * L;
    double L3 = L2 * L;
    double denom = 3.0 + Phi;

    double V_disp = (48.0 * EI * (w1 - w2) / denom + 24.0 * EI * L * phi1 * (3.0 / denom))
                    / (24.0 * L3);

    double V_load = (-20.0 * L2 * (q1 + q2) + 20.0 * L * x * (q1 + q2)
                     - L3 * (9.0 * q1 + 11.0 * q2))
                    / (24.0 * L3);

    return V_disp + V_load;
}

double ShearYTimoshenkoComputer::fixed_free_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;
    double Phi = Phi_;

    double L2 = L * L;
    double L3 = L2 * L;
    double denom = 3.0 + Phi;

    double V_disp = (48.0 * EI * (w1 - w2) / denom - 24.0 * EI * L * phi2 * (3.0 / denom))
                    / (24.0 * L3);

    double V_load = (20.0 * L2 * (q1 + q2) - 20.0 * L * x * (q1 + q2)
                     + L3 * (11.0 * q1 + 9.0 * q2))
                    / (24.0 * L3);

    return V_disp + V_load;
}

double ShearYTimoshenkoComputer::fixed_free_fixed_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Simply supported - shear is from reactions and load
    double R1 = (L * (7.0 * q1 + 3.0 * q2)) / 20.0;
    return R1 - q1 * x - (q2 - q1) * x * x / (2.0 * L);
}

double ShearYTimoshenkoComputer::fixed_fixed_free_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Cantilever shear
    return -((L - x) * (q1 + (q1 + (q2 - q1) * x / L + q2) / 2.0));
}

double ShearYTimoshenkoComputer::free_free_fixed_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return -(q1 * x + (q2 - q1) * x * x / (2.0 * L));
}

// ============================================================================
// MomentYTimoshenkoComputer Implementation (x-z plane)
// ============================================================================

MomentYTimoshenkoComputer::MomentYTimoshenkoComputer(double L, double EI, double kAG,
                                                      double w1, double phi1,
                                                      double w2, double phi2,
                                                      double q1, double q2)
    : L_(L), EI_(EI), kAG_(kAG), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2),
      q1_(q1), q2_(q2)
{
    Phi_ = (kAG > 0.0) ? (12.0 * EI / (L * L * kAG)) : 0.0;
}

double MomentYTimoshenkoComputer::compute(double x, ReleaseCombo4DOF release) const {
    // Delegate to MomentZTimoshenkoComputer with same parameters
    // (same beam theory, different bending plane)
    MomentZTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, release);
}

double MomentYTimoshenkoComputer::fixed_fixed_fixed_fixed(double x) const {
    MomentZTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED);
}

double MomentYTimoshenkoComputer::fixed_fixed_fixed_free(double x) const {
    MomentZTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE);
}

double MomentYTimoshenkoComputer::fixed_free_fixed_fixed(double x) const {
    MomentZTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED);
}

double MomentYTimoshenkoComputer::fixed_free_fixed_free(double x) const {
    MomentZTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE);
}

double MomentYTimoshenkoComputer::fixed_fixed_free_free(double x) const {
    MomentZTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE);
}

double MomentYTimoshenkoComputer::free_free_fixed_fixed(double x) const {
    MomentZTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED);
}

// ============================================================================
// ShearZTimoshenkoComputer Implementation
// ============================================================================

ShearZTimoshenkoComputer::ShearZTimoshenkoComputer(double L, double EI, double kAG,
                                                    double w1, double phi1,
                                                    double w2, double phi2,
                                                    double q1, double q2)
    : L_(L), EI_(EI), kAG_(kAG), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2),
      q1_(q1), q2_(q2)
{
    Phi_ = (kAG > 0.0) ? (12.0 * EI / (L * L * kAG)) : 0.0;
}

double ShearZTimoshenkoComputer::compute(double x, ReleaseCombo4DOF release) const {
    ShearYTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, release);
}

double ShearZTimoshenkoComputer::fixed_fixed_fixed_fixed(double x) const {
    ShearYTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED);
}

double ShearZTimoshenkoComputer::fixed_fixed_fixed_free(double x) const {
    ShearYTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE);
}

double ShearZTimoshenkoComputer::fixed_free_fixed_fixed(double x) const {
    ShearYTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED);
}

double ShearZTimoshenkoComputer::fixed_free_fixed_free(double x) const {
    ShearYTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE);
}

double ShearZTimoshenkoComputer::fixed_fixed_free_free(double x) const {
    ShearYTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE);
}

double ShearZTimoshenkoComputer::free_free_fixed_fixed(double x) const {
    ShearYTimoshenkoComputer comp(L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return comp.compute(x, ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED);
}

} // namespace grillex
