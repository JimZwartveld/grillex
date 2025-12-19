#include "grillex/internal_actions_computer.hpp"

namespace grillex {

// ============================================================================
// MomentZEulerComputer Implementation
// ============================================================================

MomentZEulerComputer::MomentZEulerComputer(double L, double EI, double w1, double phi1,
                                           double w2, double phi2, double q1, double q2)
    : L_(L), EI_(EI), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2), q1_(q1), q2_(q2) {}

double MomentZEulerComputer::compute(double x, ReleaseCombo4DOF release) const {
    switch (release) {
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
            return fixed_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FIXED:
            return fixed_fixed_free_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE:
            return fixed_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE:
            return fixed_fixed_free_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED:
            return fixed_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FREE_FIXED:
            return fixed_free_free_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE:
            return fixed_free_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FREE_FREE:
            return fixed_free_free_free(x);
        case ReleaseCombo4DOF::FREE_FIXED_FIXED_FIXED:
            return free_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FIXED_FREE_FIXED:
            return free_fixed_free_fixed(x);
        case ReleaseCombo4DOF::FREE_FIXED_FIXED_FREE:
            return free_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FREE_FIXED_FREE_FREE:
            return free_fixed_free_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED:
            return free_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FIXED:
            return free_free_free_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FREE:
            return free_free_fixed_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FREE:
            return 0.0;  // Rigid body motion, no internal moment
        default:
            throw std::runtime_error("Unknown bending release combination");
    }
}

// All formulas below are derived from beam differential equations and adapted
// from pystructeng lines.py reference implementation.

double MomentZEulerComputer::fixed_fixed_fixed_fixed(double x) const {
    // Both ends fully fixed (w and φ fixed at both nodes)
    // Standard fixed-fixed beam
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;

    // Formula from pystructeng lines.py
    return (-120.0 * EI * L2 * (2.0 * phi1 + phi2)
            + 360.0 * EI * L * (-w1 + w2)
            + L3 * (3.0 * L2 * q1 + 2.0 * L2 * q2 + 30.0 * q1 * x * x)
            + 10.0 * L2 * x * x * x * (-q1 + q2)
            + 3.0 * x * (120.0 * EI * L * (phi1 + phi2) + 240.0 * EI * (w1 - w2)
                        - L4 * (7.0 * q1 + 3.0 * q2)))
           / (60.0 * L3);
}

double MomentZEulerComputer::fixed_fixed_free_fixed(double x) const {
    // w1, φ1, φ2 fixed; w2 free (roller at j for displacement)
    double L = L_, EI = EI_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    return (-24.0 * EI * phi1 + 24.0 * EI * phi2
            + 3.0 * L * L * L * q1 + 5.0 * L * L * L * q2
            - 12.0 * L * x * (L * (q1 + q2) - q1 * x)
            - 4.0 * x * x * x * (q1 - q2))
           / (24.0 * L);
}

double MomentZEulerComputer::fixed_fixed_fixed_free(double x) const {
    // w1, φ1, w2 fixed; φ2 free (hinge at j)
    // Propped cantilever-like condition
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    return (48.0 * EI * L * (w2 - w1) - 24.0 * EI * L2 * phi1
            + L3 * (L * (9.0 * q1 + 11.0 * q2) - 20.0 * x * (q1 + q2))
            + 10.0 * L * x * x * (q1 + q2)
            + x * (48.0 * EI * (w1 - w2) + 24.0 * EI * L * phi1
                   - L3 * (9.0 * q1 + 11.0 * q2)))
           / (24.0 * L3);
}

double MomentZEulerComputer::fixed_fixed_free_free(double x) const {
    // w1, φ1 fixed; w2, φ2 free (cantilever)
    double L = L_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;

    // Cantilever moment: M(x) = integral of load from x to L
    // For trapezoidal load: q(ξ) = q1 + (q2-q1)*ξ/L
    return (L2 * (q1 + q2)
            - 2.0 * L * x * (q1 + q2)
            + x * x * (q1 + q2)
            + (q2 - q1) * (L2 * L - 3.0 * L * x * x + 2.0 * x * x * x) / (6.0 * L))
           / 2.0;
}

double MomentZEulerComputer::fixed_free_fixed_fixed(double x) const {
    // w1, w2, φ2 fixed; φ1 free (hinge at i)
    double L = L_, EI = EI_;
    double w1 = w1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    return (48.0 * EI * L * (w2 - w1) + 24.0 * EI * L2 * phi2
            - L3 * (L * (11.0 * q1 + 9.0 * q2) - 20.0 * x * (q1 + q2))
            - 10.0 * L * x * x * (q1 + q2)
            + x * (48.0 * EI * (w1 - w2) - 24.0 * EI * L * phi2
                   + L3 * (11.0 * q1 + 9.0 * q2)))
           / (24.0 * L3);
}

double MomentZEulerComputer::fixed_free_free_fixed(double x) const {
    // w1, φ2 fixed; φ1, w2 free
    double L = L_;
    double phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;

    // Simply supported-like with rotation φ2 prescribed
    return phi2 * (L - x) +
           (L2 * (q1 + q2) * (L - x)
            - L * (q1 + q2) * (L - x) * (L - x) / 2.0
            + (q2 - q1) * (L * L2 - L2 * x - L * x * (L - x) / 2.0
                          + (L - x) * (L - x) * (2.0 * L + x) / 6.0)) / (L * 2.0);
}

double MomentZEulerComputer::fixed_free_fixed_free(double x) const {
    // w1, w2 fixed; φ1, φ2 free (simply supported / double hinge)
    double L = L_, EI = EI_;
    double w1 = w1_, w2 = w2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    // Simply supported beam moment
    // M = EI * κ + load effect
    // For simply supported with UDL: M = q*L*x/2 - q*x²/2
    double M_disp = 6.0 * EI * (w2 - w1) * x / L3 - 6.0 * EI * (w2 - w1) * x * x / (L3 * L);

    // Load contribution for simply supported
    double M_load = (L2 * q1 + L2 * q2) * x / (2.0 * L)
                    - (q1 + q2) * x * x / 2.0
                    + (q2 - q1) * x * x * x / (6.0 * L);

    // Correction for reaction
    double R1 = (L * (7.0 * q1 + 3.0 * q2)) / 20.0;  // Reaction at node 1 for trapezoidal
    double R2 = (L * (3.0 * q1 + 7.0 * q2)) / 20.0;  // Reaction at node 2

    return R1 * x - (q1 * x * x / 2.0) - (q2 - q1) * x * x * x / (6.0 * L);
}

double MomentZEulerComputer::fixed_free_free_free(double x) const {
    // w1 fixed; φ1, w2, φ2 free
    // Only displacement w1 fixed - unstable configuration
    // This would typically result in zero moment (rigid body mode)
    return 0.0;
}

double MomentZEulerComputer::free_fixed_fixed_fixed(double x) const {
    // φ1, w2, φ2 fixed; w1 free (roller at i for displacement)
    double L = L_, EI = EI_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    return (24.0 * EI * phi1 - 24.0 * EI * phi2
            - 5.0 * L * L * L * q1 - 3.0 * L * L * L * q2
            + 12.0 * L * x * (L * (q1 + q2) - q1 * x)
            + 4.0 * x * x * x * (q1 - q2))
           / (24.0 * L);
}

double MomentZEulerComputer::free_fixed_free_fixed(double x) const {
    // φ1, φ2 fixed; w1, w2 free
    double L = L_, EI = EI_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    // Beam with fixed rotations at both ends, free to translate
    double L2 = L * L;

    return EI * (phi2 - phi1) / L
           + (L2 * q1 + L2 * q2) / 2.0
           - (q1 + q2) * x
           + (q2 - q1) * x * x / (2.0 * L);
}

double MomentZEulerComputer::free_fixed_fixed_free(double x) const {
    // φ1, w2 fixed; w1, φ2 free
    double L = L_, EI = EI_;
    double phi1 = phi1_, w2 = w2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;

    // Beam with rotation fixed at i, displacement fixed at j
    return EI * phi1 +
           (L2 * (q1 + q2) - 2.0 * L * x * (q1 + q2)
            + x * x * (q1 + q2)
            + (q2 - q1) * (L2 * L - 3.0 * L * x * x + 2.0 * x * x * x) / (6.0 * L)) / 2.0;
}

double MomentZEulerComputer::free_fixed_free_free(double x) const {
    // φ1 fixed; w1, w2, φ2 free
    // Only rotation φ1 fixed - partially unstable
    return 0.0;
}

double MomentZEulerComputer::free_free_fixed_fixed(double x) const {
    // w2, φ2 fixed; w1, φ1 free (cantilever from j end)
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Cantilever from the other end - moment at x for load from 0 to x
    return -(q1 * x * x / 2.0 + (q2 - q1) * x * x * x / (6.0 * L));
}

double MomentZEulerComputer::free_free_free_fixed(double x) const {
    // φ2 fixed; w1, φ1, w2 free
    // Only rotation φ2 fixed - partially unstable
    return 0.0;
}

double MomentZEulerComputer::free_free_fixed_free(double x) const {
    // w2 fixed; w1, φ1, φ2 free
    // Only displacement w2 fixed - unstable
    return 0.0;
}

// ============================================================================
// ShearYEulerComputer Implementation
// ============================================================================

ShearYEulerComputer::ShearYEulerComputer(double L, double EI, double w1, double phi1,
                                         double w2, double phi2, double q1, double q2)
    : L_(L), EI_(EI), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2), q1_(q1), q2_(q2) {}

double ShearYEulerComputer::compute(double x, ReleaseCombo4DOF release) const {
    switch (release) {
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
            return fixed_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FIXED:
            return fixed_fixed_free_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE:
            return fixed_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE:
            return fixed_fixed_free_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED:
            return fixed_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FREE_FIXED:
            return fixed_free_free_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE:
            return fixed_free_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FREE_FREE:
            return fixed_free_free_free(x);
        case ReleaseCombo4DOF::FREE_FIXED_FIXED_FIXED:
            return free_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FIXED_FREE_FIXED:
            return free_fixed_free_fixed(x);
        case ReleaseCombo4DOF::FREE_FIXED_FIXED_FREE:
            return free_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FREE_FIXED_FREE_FREE:
            return free_fixed_free_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED:
            return free_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FIXED:
            return free_free_free_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FREE:
            return free_free_fixed_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FREE:
            return 0.0;  // Rigid body motion
        default:
            throw std::runtime_error("Unknown bending release combination");
    }
}

// Shear is the derivative of moment: V = dM/dx

double ShearYEulerComputer::fixed_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;

    // Derivative of moment formula
    return (60.0 * L2 * q1 * x + 30.0 * L2 * x * x * (-q1 + q2)
            + 3.0 * (120.0 * EI * L * (phi1 + phi2) + 240.0 * EI * (w1 - w2)
                    - L4 * (7.0 * q1 + 3.0 * q2)))
           / (60.0 * L3);
}

double ShearYEulerComputer::fixed_fixed_free_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return (-12.0 * L * (L * (q1 + q2) - 2.0 * q1 * x) - 12.0 * x * x * (q1 - q2))
           / (24.0 * L);
}

double ShearYEulerComputer::fixed_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    return (-20.0 * L2 * (q1 + q2) + 20.0 * L * x * (q1 + q2)
            + (48.0 * EI * (w1 - w2) + 24.0 * EI * L * phi1 - L3 * (9.0 * q1 + 11.0 * q2)))
           / (24.0 * L3);
}

double ShearYEulerComputer::fixed_fixed_free_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Cantilever shear: V(x) = integral of load from x to L
    return -((L - x) * (q1 + (q1 + (q2 - q1) * x / L + q2) / 2.0));
}

double ShearYEulerComputer::fixed_free_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    return (20.0 * L2 * (q1 + q2) - 20.0 * L * x * (q1 + q2)
            + (48.0 * EI * (w1 - w2) - 24.0 * EI * L * phi2 + L3 * (11.0 * q1 + 9.0 * q2)))
           / (24.0 * L3);
}

double ShearYEulerComputer::fixed_free_free_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Shear for simply supported-like condition
    return (L * (q1 + q2) - (q1 + q2) * x + (q2 - q1) * x * x / (2.0 * L)) / 2.0;
}

double ShearYEulerComputer::fixed_free_fixed_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Simply supported shear
    double R1 = (L * (7.0 * q1 + 3.0 * q2)) / 20.0;
    return R1 - q1 * x - (q2 - q1) * x * x / (2.0 * L);
}

double ShearYEulerComputer::fixed_free_free_free(double x) const {
    return 0.0;  // Unstable
}

double ShearYEulerComputer::free_fixed_fixed_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return (12.0 * L * (L * (q1 + q2) - 2.0 * q1 * x) + 12.0 * x * x * (q1 - q2))
           / (24.0 * L);
}

double ShearYEulerComputer::free_fixed_free_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return -(q1 + q2) + (q2 - q1) * x / L;
}

double ShearYEulerComputer::free_fixed_fixed_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Similar to cantilever shear
    return -((L - x) * (q1 + q2) / 2.0 - (q2 - q1) * (L - x) * (L - x) / (4.0 * L));
}

double ShearYEulerComputer::free_fixed_free_free(double x) const {
    return 0.0;  // Unstable
}

double ShearYEulerComputer::free_free_fixed_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    // Cantilever from j end - shear from load up to x
    return -(q1 * x + (q2 - q1) * x * x / (2.0 * L));
}

double ShearYEulerComputer::free_free_free_fixed(double x) const {
    return 0.0;  // Unstable
}

double ShearYEulerComputer::free_free_fixed_free(double x) const {
    return 0.0;  // Unstable
}

// ============================================================================
// MomentYEulerComputer Implementation (x-z bending plane)
// ============================================================================

MomentYEulerComputer::MomentYEulerComputer(double L, double EI, double w1, double phi1,
                                           double w2, double phi2, double q1, double q2)
    : L_(L), EI_(EI), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2), q1_(q1), q2_(q2) {}

double MomentYEulerComputer::compute(double x, ReleaseCombo4DOF release) const {
    // Same formulas as MomentZ but for x-z plane
    // Note: Sign convention may differ based on local axis orientation
    switch (release) {
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
            return fixed_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FIXED:
            return fixed_fixed_free_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE:
            return fixed_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE:
            return fixed_fixed_free_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED:
            return fixed_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FREE_FIXED:
            return fixed_free_free_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE:
            return fixed_free_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FREE_FREE:
            return fixed_free_free_free(x);
        case ReleaseCombo4DOF::FREE_FIXED_FIXED_FIXED:
            return free_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FIXED_FREE_FIXED:
            return free_fixed_free_fixed(x);
        case ReleaseCombo4DOF::FREE_FIXED_FIXED_FREE:
            return free_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FREE_FIXED_FREE_FREE:
            return free_fixed_free_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED:
            return free_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FIXED:
            return free_free_free_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FREE:
            return free_free_fixed_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FREE:
            return 0.0;
        default:
            throw std::runtime_error("Unknown bending release combination");
    }
}

// Reuse same formulas (bending in different plane uses same beam theory)
double MomentYEulerComputer::fixed_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;

    // Note: sign convention for My may need adjustment depending on axis orientation
    // For consistency with beam element, My uses same formula structure as Mz
    return (-120.0 * EI * L2 * (2.0 * phi1 + phi2)
            + 360.0 * EI * L * (-w1 + w2)
            + L3 * (3.0 * L2 * q1 + 2.0 * L2 * q2 + 30.0 * q1 * x * x)
            + 10.0 * L2 * x * x * x * (-q1 + q2)
            + 3.0 * x * (120.0 * EI * L * (phi1 + phi2) + 240.0 * EI * (w1 - w2)
                        - L4 * (7.0 * q1 + 3.0 * q2)))
           / (60.0 * L3);
}

double MomentYEulerComputer::fixed_fixed_free_fixed(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    return (-24.0 * EI * phi1 + 24.0 * EI * phi2
            + 3.0 * L * L * L * q1 + 5.0 * L * L * L * q2
            - 12.0 * L * x * (L * (q1 + q2) - q1 * x)
            - 4.0 * x * x * x * (q1 - q2))
           / (24.0 * L);
}

double MomentYEulerComputer::fixed_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    return (48.0 * EI * L * (w2 - w1) - 24.0 * EI * L2 * phi1
            + L3 * (L * (9.0 * q1 + 11.0 * q2) - 20.0 * x * (q1 + q2))
            + 10.0 * L * x * x * (q1 + q2)
            + x * (48.0 * EI * (w1 - w2) + 24.0 * EI * L * phi1
                   - L3 * (9.0 * q1 + 11.0 * q2)))
           / (24.0 * L3);
}

double MomentYEulerComputer::fixed_fixed_free_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;

    return (L2 * (q1 + q2)
            - 2.0 * L * x * (q1 + q2)
            + x * x * (q1 + q2)
            + (q2 - q1) * (L2 * L - 3.0 * L * x * x + 2.0 * x * x * x) / (6.0 * L))
           / 2.0;
}

double MomentYEulerComputer::fixed_free_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    return (48.0 * EI * L * (w2 - w1) + 24.0 * EI * L2 * phi2
            - L3 * (L * (11.0 * q1 + 9.0 * q2) - 20.0 * x * (q1 + q2))
            - 10.0 * L * x * x * (q1 + q2)
            + x * (48.0 * EI * (w1 - w2) - 24.0 * EI * L * phi2
                   + L3 * (11.0 * q1 + 9.0 * q2)))
           / (24.0 * L3);
}

double MomentYEulerComputer::fixed_free_free_fixed(double x) const {
    double L = L_;
    double phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;

    return phi2 * (L - x) +
           (L2 * (q1 + q2) * (L - x)
            - L * (q1 + q2) * (L - x) * (L - x) / 2.0
            + (q2 - q1) * (L * L2 - L2 * x - L * x * (L - x) / 2.0
                          + (L - x) * (L - x) * (2.0 * L + x) / 6.0)) / (L * 2.0);
}

double MomentYEulerComputer::fixed_free_fixed_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    double R1 = (L * (7.0 * q1 + 3.0 * q2)) / 20.0;
    return R1 * x - (q1 * x * x / 2.0) - (q2 - q1) * x * x * x / (6.0 * L);
}

double MomentYEulerComputer::fixed_free_free_free(double x) const {
    return 0.0;
}

double MomentYEulerComputer::free_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    return (24.0 * EI * phi1 - 24.0 * EI * phi2
            - 5.0 * L * L * L * q1 - 3.0 * L * L * L * q2
            + 12.0 * L * x * (L * (q1 + q2) - q1 * x)
            + 4.0 * x * x * x * (q1 - q2))
           / (24.0 * L);
}

double MomentYEulerComputer::free_fixed_free_fixed(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;

    return EI * (phi2 - phi1) / L
           + (L2 * q1 + L2 * q2) / 2.0
           - (q1 + q2) * x
           + (q2 - q1) * x * x / (2.0 * L);
}

double MomentYEulerComputer::free_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;

    return EI * phi1 +
           (L2 * (q1 + q2) - 2.0 * L * x * (q1 + q2)
            + x * x * (q1 + q2)
            + (q2 - q1) * (L2 * L - 3.0 * L * x * x + 2.0 * x * x * x) / (6.0 * L)) / 2.0;
}

double MomentYEulerComputer::free_fixed_free_free(double x) const {
    return 0.0;
}

double MomentYEulerComputer::free_free_fixed_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return -(q1 * x * x / 2.0 + (q2 - q1) * x * x * x / (6.0 * L));
}

double MomentYEulerComputer::free_free_free_fixed(double x) const {
    return 0.0;
}

double MomentYEulerComputer::free_free_fixed_free(double x) const {
    return 0.0;
}

// ============================================================================
// ShearZEulerComputer Implementation
// ============================================================================

ShearZEulerComputer::ShearZEulerComputer(double L, double EI, double w1, double phi1,
                                         double w2, double phi2, double q1, double q2)
    : L_(L), EI_(EI), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2), q1_(q1), q2_(q2) {}

double ShearZEulerComputer::compute(double x, ReleaseCombo4DOF release) const {
    switch (release) {
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
            return fixed_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FIXED:
            return fixed_fixed_free_fixed(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE:
            return fixed_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE:
            return fixed_fixed_free_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED:
            return fixed_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FREE_FIXED:
            return fixed_free_free_fixed(x);
        case ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE:
            return fixed_free_fixed_free(x);
        case ReleaseCombo4DOF::FIXED_FREE_FREE_FREE:
            return fixed_free_free_free(x);
        case ReleaseCombo4DOF::FREE_FIXED_FIXED_FIXED:
            return free_fixed_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FIXED_FREE_FIXED:
            return free_fixed_free_fixed(x);
        case ReleaseCombo4DOF::FREE_FIXED_FIXED_FREE:
            return free_fixed_fixed_free(x);
        case ReleaseCombo4DOF::FREE_FIXED_FREE_FREE:
            return free_fixed_free_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED:
            return free_free_fixed_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FIXED:
            return free_free_free_fixed(x);
        case ReleaseCombo4DOF::FREE_FREE_FIXED_FREE:
            return free_free_fixed_free(x);
        case ReleaseCombo4DOF::FREE_FREE_FREE_FREE:
            return 0.0;
        default:
            throw std::runtime_error("Unknown bending release combination");
    }
}

// Reuse ShearY formulas (same beam theory)
double ShearZEulerComputer::fixed_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;

    return (60.0 * L2 * q1 * x + 30.0 * L2 * x * x * (-q1 + q2)
            + 3.0 * (120.0 * EI * L * (phi1 + phi2) + 240.0 * EI * (w1 - w2)
                    - L4 * (7.0 * q1 + 3.0 * q2)))
           / (60.0 * L3);
}

double ShearZEulerComputer::fixed_fixed_free_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return (-12.0 * L * (L * (q1 + q2) - 2.0 * q1 * x) - 12.0 * x * x * (q1 - q2))
           / (24.0 * L);
}

double ShearZEulerComputer::fixed_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, phi1 = phi1_, w2 = w2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    return (-20.0 * L2 * (q1 + q2) + 20.0 * L * x * (q1 + q2)
            + (48.0 * EI * (w1 - w2) + 24.0 * EI * L * phi1 - L3 * (9.0 * q1 + 11.0 * q2)))
           / (24.0 * L3);
}

double ShearZEulerComputer::fixed_fixed_free_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return -((L - x) * (q1 + (q1 + (q2 - q1) * x / L + q2) / 2.0));
}

double ShearZEulerComputer::fixed_free_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double w1 = w1_, w2 = w2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;

    return (20.0 * L2 * (q1 + q2) - 20.0 * L * x * (q1 + q2)
            + (48.0 * EI * (w1 - w2) - 24.0 * EI * L * phi2 + L3 * (11.0 * q1 + 9.0 * q2)))
           / (24.0 * L3);
}

double ShearZEulerComputer::fixed_free_free_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return (L * (q1 + q2) - (q1 + q2) * x + (q2 - q1) * x * x / (2.0 * L)) / 2.0;
}

double ShearZEulerComputer::fixed_free_fixed_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    double R1 = (L * (7.0 * q1 + 3.0 * q2)) / 20.0;
    return R1 - q1 * x - (q2 - q1) * x * x / (2.0 * L);
}

double ShearZEulerComputer::fixed_free_free_free(double x) const {
    return 0.0;
}

double ShearZEulerComputer::free_fixed_fixed_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return (12.0 * L * (L * (q1 + q2) - 2.0 * q1 * x) + 12.0 * x * x * (q1 - q2))
           / (24.0 * L);
}

double ShearZEulerComputer::free_fixed_free_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return -(q1 + q2) + (q2 - q1) * x / L;
}

double ShearZEulerComputer::free_fixed_fixed_free(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return -((L - x) * (q1 + q2) / 2.0 - (q2 - q1) * (L - x) * (L - x) / (4.0 * L));
}

double ShearZEulerComputer::free_fixed_free_free(double x) const {
    return 0.0;
}

double ShearZEulerComputer::free_free_fixed_fixed(double x) const {
    double L = L_;
    double q1 = q1_, q2 = q2_;

    return -(q1 * x + (q2 - q1) * x * x / (2.0 * L));
}

double ShearZEulerComputer::free_free_free_fixed(double x) const {
    return 0.0;
}

double ShearZEulerComputer::free_free_fixed_free(double x) const {
    return 0.0;
}

} // namespace grillex
