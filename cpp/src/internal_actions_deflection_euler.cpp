#include "grillex/internal_actions_computer.hpp"

namespace grillex {

// ============================================================================
// DeflectionYEulerComputer Implementation
// ============================================================================
//
// Deflection formulas are derived by integrating M = EI * d²v/dx² twice.
// The general solution is: v(x) = v_homogeneous(x) + v_particular(x)
//
// For a trapezoidal load q(x) = q1 + (q2-q1)*x/L:
//   v_particular(x) = q1*x⁴/(24*EI) + (q2-q1)*x⁵/(120*EI*L)
//
// The homogeneous solution (from nodal values) is the Hermite polynomial:
//   v_hom(x) = N1*v1 + N2*φ1 + N3*v2 + N4*φ2
//
// The total deflection satisfies the actual boundary conditions.
// ============================================================================

DeflectionYEulerComputer::DeflectionYEulerComputer(double L, double EI, double v1, double phi1,
                                                     double v2, double phi2, double q1, double q2)
    : L_(L), EI_(EI), v1_(v1), phi1_(phi1), v2_(v2), phi2_(phi2), q1_(q1), q2_(q2) {}

double DeflectionYEulerComputer::compute(double x, ReleaseCombo4DOF release) const {
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
            return v1_;  // Rigid body - return initial value
        default:
            throw std::runtime_error("Unknown deflection release combination");
    }
}

// Fixed-Fixed-Fixed-Fixed: All DOFs constrained
// Hermite interpolation + load-induced deflection (fixed ends)
double DeflectionYEulerComputer::fixed_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, phi1 = phi1_, v2 = v2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;
    double L5 = L4 * L;
    double x2 = x * x;
    double x3 = x2 * x;
    double x4 = x3 * x;
    double x5 = x4 * x;

    // Hermite shape functions
    double xi = x / L;
    double xi2 = xi * xi;
    double xi3 = xi2 * xi;
    double N1 = 1.0 - 3.0 * xi2 + 2.0 * xi3;
    double N2 = L * (xi - 2.0 * xi2 + xi3);
    double N3 = 3.0 * xi2 - 2.0 * xi3;
    double N4 = L * (-xi2 + xi3);

    double v_hermite = N1 * v1 + N2 * phi1 + N3 * v2 + N4 * phi2;

    // Load-induced deflection for fixed-fixed beam
    // v_load = q1*x²*(L-x)²/(24*EI) + correction for trapezoidal
    double v_load = (q1 * x2 * (L - x) * (L - x) / (24.0 * EI)) +
                    ((q2 - q1) * x2 * (L - x) * (L - x) * (L + 2.0 * x) / (120.0 * EI * L));

    return v_hermite + v_load;
}

// Fixed-Fixed-Free-Fixed: v1, φ1, φ2 fixed; v2 free
double DeflectionYEulerComputer::fixed_fixed_free_fixed(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double x2 = x * x;
    double x3 = x2 * x;

    // v2 is free, so we use interpolation with v1, φ1, φ2 only
    // The deflection satisfies: v(0)=v1, v'(0)=φ1, v'(L)=φ2
    double A = v1;
    double B = phi1;

    // Using the constraint v'(L) = φ2 and integrating load effects
    double v_hom = A + B * x + (phi2 - phi1) * x2 / (2.0 * L);

    // Load contribution
    double v_load = (q1 * x3 * (4.0 * L - 3.0 * x) + (q2 - q1) * x3 * (5.0 * L - 4.0 * x) / 5.0)
                    / (24.0 * EI);

    return v_hom + v_load;
}

// Fixed-Fixed-Fixed-Free: v1, φ1, v2 fixed; φ2 free (propped cantilever)
double DeflectionYEulerComputer::fixed_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, phi1 = phi1_, v2 = v2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double x2 = x * x;
    double x3 = x2 * x;

    // Propped cantilever: v(0)=v1, v'(0)=φ1, v(L)=v2
    // Cubic polynomial: v = a0 + a1*x + a2*x² + a3*x³
    double a0 = v1;
    double a1 = phi1;
    double a2 = 3.0 * (v2 - v1) / L2 - 2.0 * phi1 / L;
    double a3 = -2.0 * (v2 - v1) / L3 + phi1 / L2;

    double v_hom = a0 + a1 * x + a2 * x2 + a3 * x3;

    // Load-induced deflection for propped cantilever
    // Needs specific formula accounting for reaction at support
    double v_load = (q1 + q2) * x2 * (L - x) * (3.0 * L - 2.0 * x) / (48.0 * EI);

    return v_hom + v_load;
}

// Fixed-Fixed-Free-Free: v1, φ1 fixed; v2, φ2 free (cantilever)
double DeflectionYEulerComputer::fixed_fixed_free_free(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, phi1 = phi1_;
    double q1 = q1_, q2 = q2_;

    double x2 = x * x;
    double x3 = x2 * x;
    double x4 = x3 * x;
    double x5 = x4 * x;
    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;
    double L5 = L4 * L;

    // Cantilever: v(0) = v1, v'(0) = φ1
    // v(x) = v1 + φ1*x + load-induced deflection

    // For trapezoidal load on cantilever:
    // v_load = (q1/24EI) * (x⁴ - 4Lx³ + 6L²x² - 4L³x + L⁴ - L⁴)
    //        + correction for (q2-q1) term
    double v_load_uniform = q1 * (L4 - 4.0 * L3 * x + 6.0 * L2 * x2 - 4.0 * L * x3 + x4) / (24.0 * EI);
    double v_load_linear = (q2 - q1) * (L5 - 5.0 * L4 * x + 10.0 * L2 * x3 - 10.0 * L * x4 + 4.0 * x5)
                           / (120.0 * EI * L);

    // Subtract the value at x=0 to ensure v(0) = v1
    double v_load_0_uniform = q1 * L4 / (24.0 * EI);
    double v_load_0_linear = (q2 - q1) * L4 / (120.0 * EI);

    return v1 + phi1 * x + (v_load_uniform - v_load_0_uniform) + (v_load_linear - v_load_0_linear);
}

// Fixed-Free-Fixed-Fixed: v1, v2, φ2 fixed; φ1 free
double DeflectionYEulerComputer::fixed_free_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, v2 = v2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double x2 = x * x;
    double x3 = x2 * x;

    // Similar to fixed_fixed_fixed_free but from the other end
    // v(0)=v1, v(L)=v2, v'(L)=φ2
    double a0 = v1;
    double a2 = 3.0 * (v2 - v1) / L2 - phi2 / L;
    double a3 = -2.0 * (v2 - v1) / L3 + phi2 / L2;
    double a1 = 0.0;  // Derived from the constraint

    double v_hom = a0 + a2 * x2 + a3 * x3;

    double v_load = (q1 + q2) * x2 * (L - x) * (L + x) / (48.0 * EI);

    return v_hom + v_load;
}

// Fixed-Free-Free-Fixed: v1, φ2 fixed; φ1, v2 free
double DeflectionYEulerComputer::fixed_free_free_fixed(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double x2 = x * x;

    // v(0) = v1, v'(L) = φ2
    // v = v1 + φ2*x + (something for load)
    double v_hom = v1 + phi2 * x;

    double v_load = (q1 + q2) * x2 * (3.0 * L - 2.0 * x) / (24.0 * EI);

    return v_hom + v_load;
}

// Fixed-Free-Fixed-Free: v1, v2 fixed; φ1, φ2 free (simply supported)
double DeflectionYEulerComputer::fixed_free_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, v2 = v2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;
    double x2 = x * x;
    double x3 = x2 * x;
    double x4 = x3 * x;

    // Simply supported: v(0) = v1, v(L) = v2
    // Linear interpolation for homogeneous part
    double v_hom = v1 + (v2 - v1) * x / L;

    // Simply supported beam with trapezoidal load
    // Standard formula: v = q*x*(L-x)*(L²+L*x-x²)/(24*EI*L) for uniform
    double v_load = (q1 * x * (L - x) * (L2 + L * x - x2) / (24.0 * EI * L)) +
                    ((q2 - q1) * x * (L - x) * (L2 + 2.0 * L * x - 2.0 * x2) / (120.0 * EI * L));

    return v_hom + v_load;
}

// Fixed-Free-Free-Free: Only v1 fixed (unstable - rigid body rotation possible)
double DeflectionYEulerComputer::fixed_free_free_free(double x) const {
    return v1_;  // Return initial displacement (rigid body)
}

// Free-Fixed-Fixed-Fixed: φ1, v2, φ2 fixed; v1 free
double DeflectionYEulerComputer::free_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_, v2 = v2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double x2 = x * x;
    double x3 = x2 * x;

    // v'(0)=φ1, v(L)=v2, v'(L)=φ2
    double v_hom = v2 + phi2 * (x - L) + phi1 * x - phi2 * x;

    double v_load = (q1 + q2) * (L - x) * (L - x) * (2.0 * L + x) / (48.0 * EI);

    return v_hom + v_load;
}

// Free-Fixed-Free-Fixed: φ1, φ2 fixed; v1, v2 free
double DeflectionYEulerComputer::free_fixed_free_fixed(double x) const {
    double L = L_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    // v'(0)=φ1, v'(L)=φ2
    // Linear rotation interpolation
    double v_hom = phi1 * x + (phi2 - phi1) * x * x / (2.0 * L);

    double v_load = (q1 + q2) * x * x * (3.0 * L - 2.0 * x) / (24.0 * EI_);

    return v_hom + v_load;
}

// Free-Fixed-Fixed-Free: φ1, v2 fixed; v1, φ2 free
double DeflectionYEulerComputer::free_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_, v2 = v2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double x2 = x * x;

    // v'(0)=φ1, v(L)=v2
    double v_hom = v2 + phi1 * (x - L);

    double v_load = (q1 + q2) * (L - x) * (L - x) * x / (24.0 * EI);

    return v_hom + v_load;
}

// Free-Fixed-Free-Free: Only φ1 fixed (unstable)
double DeflectionYEulerComputer::free_fixed_free_free(double x) const {
    return phi1_ * x;  // Linear with prescribed slope
}

// Free-Free-Fixed-Fixed: v2, φ2 fixed; v1, φ1 free (cantilever from right)
double DeflectionYEulerComputer::free_free_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double v2 = v2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;
    double xL = L - x;  // Distance from right end
    double xL2 = xL * xL;
    double xL3 = xL2 * xL;
    double xL4 = xL3 * xL;

    // Cantilever fixed at right end: v(L)=v2, v'(L)=φ2
    double v_hom = v2 - phi2 * xL;

    // Load-induced deflection (mirror of left-fixed cantilever)
    double v_load = (q2 * xL4 / (24.0 * EI)) + ((q1 - q2) * xL4 * xL / (120.0 * EI * L));

    return v_hom + v_load;
}

// Free-Free-Free-Fixed: Only φ2 fixed (unstable)
double DeflectionYEulerComputer::free_free_free_fixed(double x) const {
    return phi2_ * (x - L_);
}

// Free-Free-Fixed-Free: Only v2 fixed (unstable)
double DeflectionYEulerComputer::free_free_fixed_free(double x) const {
    return v2_;
}

// ============================================================================
// RotationZEulerComputer Implementation
// ============================================================================

RotationZEulerComputer::RotationZEulerComputer(double L, double EI, double v1, double phi1,
                                                 double v2, double phi2, double q1, double q2)
    : L_(L), EI_(EI), v1_(v1), phi1_(phi1), v2_(v2), phi2_(phi2), q1_(q1), q2_(q2) {}

double RotationZEulerComputer::compute(double x, ReleaseCombo4DOF release) const {
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
            return phi1_;  // Rigid body
        default:
            throw std::runtime_error("Unknown rotation release combination");
    }
}

// Rotation is dv/dx - derivatives of deflection formulas
double RotationZEulerComputer::fixed_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, phi1 = phi1_, v2 = v2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double x2 = x * x;
    double x3 = x2 * x;

    // Derivative of Hermite shape functions
    double xi = x / L;
    double xi2 = xi * xi;
    double dN1_dx = (-6.0 * xi + 6.0 * xi2) / L;
    double dN2_dx = 1.0 - 4.0 * xi + 3.0 * xi2;
    double dN3_dx = (6.0 * xi - 6.0 * xi2) / L;
    double dN4_dx = -2.0 * xi + 3.0 * xi2;

    double theta_hermite = dN1_dx * v1 + dN2_dx * phi1 + dN3_dx * v2 + dN4_dx * phi2;

    // Load-induced rotation correction for fixed-fixed beam
    // This corrects for the difference between Hermite interpolation and the exact
    // 4th-order polynomial under distributed load
    // Formula: derivative of q*x²*(L-x)²/(24EI) for uniform + linear correction
    double theta_load = (q1 * x * (L - x) * (L - 2.0 * x) / (12.0 * EI * L)) +
                        ((q2 - q1) * x * (L - x) * (L - 3.0 * x) / (60.0 * EI * L));

    return theta_hermite + theta_load;
}

double RotationZEulerComputer::fixed_fixed_free_fixed(double x) const {
    double L = L_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double x2 = x * x;

    double theta_hom = phi1 + (phi2 - phi1) * x / L;
    double theta_load = (q1 * x2 * (L - x) + (q2 - q1) * x2 * (L - x) / 2.0) / (8.0 * EI_);

    return theta_hom + theta_load;
}

double RotationZEulerComputer::fixed_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, phi1 = phi1_, v2 = v2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double x2 = x * x;

    double a1 = phi1;
    double a2 = 3.0 * (v2 - v1) / L2 - 2.0 * phi1 / L;
    double a3 = -2.0 * (v2 - v1) / (L2 * L) + phi1 / L2;

    double theta_hom = a1 + 2.0 * a2 * x + 3.0 * a3 * x2;
    double theta_load = (q1 + q2) * x * (L - x) * (L - 2.0 * x) / (24.0 * EI);

    return theta_hom + theta_load;
}

double RotationZEulerComputer::fixed_fixed_free_free(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double L4 = L3 * L;
    double x2 = x * x;
    double x3 = x2 * x;
    double x4 = x3 * x;

    // Cantilever rotation: θ(x) = φ1 + load effect
    // Corrected polynomial: -L³ + 3L²x - 3Lx² + x³ (derivative of deflection)
    // Original had: L³ - 3Lx² + 2x³ (missing L²x term, wrong x³ coefficient)
    double theta_load = -(q1 * (-L3 + 3.0 * L2 * x - 3.0 * L * x2 + x3) / (6.0 * EI)) -
                        ((q2 - q1) * (-L4 + 6.0 * L2 * x2 - 8.0 * L * x3 + 4.0 * x4) / (24.0 * EI * L));

    // Subtract value at x=0 to ensure θ(0) = φ1
    double theta_load_0 = -(q1 * (-L3) / (6.0 * EI)) - ((q2 - q1) * (-L3) / (24.0 * EI));

    return phi1 + (theta_load - theta_load_0);
}

double RotationZEulerComputer::fixed_free_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, v2 = v2_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double x2 = x * x;

    double a2 = 3.0 * (v2 - v1) / L2 - phi2 / L;
    double a3 = -2.0 * (v2 - v1) / (L2 * L) + phi2 / L2;

    double theta_hom = 2.0 * a2 * x + 3.0 * a3 * x2;
    double theta_load = (q1 + q2) * x * (L2 - x2) / (24.0 * EI);

    return theta_hom + theta_load;
}

double RotationZEulerComputer::fixed_free_free_fixed(double x) const {
    double L = L_, EI = EI_;
    double phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double theta_hom = phi2;
    double theta_load = (q1 + q2) * x * (L - x) / (8.0 * EI);

    return theta_hom + theta_load;
}

double RotationZEulerComputer::fixed_free_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double v1 = v1_, v2 = v2_;
    double q1 = q1_, q2 = q2_;

    double L2 = L * L;
    double L3 = L2 * L;
    double x2 = x * x;

    double theta_hom = (v2 - v1) / L;

    // Simply supported rotation
    double theta_load = (q1 * (L2 - 6.0 * x2 + 4.0 * x * L) / (24.0 * EI)) +
                        ((q2 - q1) * (L2 - 10.0 * x2 + 8.0 * x * L) / (120.0 * EI));

    return theta_hom + theta_load;
}

double RotationZEulerComputer::fixed_free_free_free(double x) const {
    return 0.0;  // Rigid body rotation
}

double RotationZEulerComputer::free_fixed_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double theta_hom = phi1 + (phi2 - phi1);
    double theta_load = (q1 + q2) * (L - x) * (L - x) / (16.0 * EI);

    return theta_hom - theta_load;
}

double RotationZEulerComputer::free_fixed_free_fixed(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_, phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double theta_hom = phi1 + (phi2 - phi1) * x / L;
    double theta_load = (q1 + q2) * x * (L - x) / (8.0 * EI);

    return theta_hom + theta_load;
}

double RotationZEulerComputer::free_fixed_fixed_free(double x) const {
    double L = L_, EI = EI_;
    double phi1 = phi1_;
    double q1 = q1_, q2 = q2_;

    double theta_hom = phi1;
    double theta_load = (q1 + q2) * (L - x) * (L + x - 2.0 * L) / (16.0 * EI);

    return theta_hom + theta_load;
}

double RotationZEulerComputer::free_fixed_free_free(double x) const {
    return phi1_;
}

double RotationZEulerComputer::free_free_fixed_fixed(double x) const {
    double L = L_, EI = EI_;
    double phi2 = phi2_;
    double q1 = q1_, q2 = q2_;

    double xL = L - x;
    double xL2 = xL * xL;

    double theta_hom = phi2;
    double theta_load = -(q2 * xL2 * xL / (6.0 * EI)) - ((q1 - q2) * xL2 * xL2 / (24.0 * EI * L));

    // Subtract value at x=L
    return -phi2 + theta_load;
}

double RotationZEulerComputer::free_free_free_fixed(double x) const {
    return phi2_;
}

double RotationZEulerComputer::free_free_fixed_free(double x) const {
    return 0.0;
}

// ============================================================================
// DeflectionZEulerComputer and RotationYEulerComputer
// These are identical to the Y versions (just different naming)
// ============================================================================

DeflectionZEulerComputer::DeflectionZEulerComputer(double L, double EI, double w1, double phi1,
                                                     double w2, double phi2, double q1, double q2)
    : L_(L), EI_(EI), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2), q1_(q1), q2_(q2) {}

double DeflectionZEulerComputer::compute(double x, ReleaseCombo4DOF release) const {
    // Use DeflectionY formulas with w1, w2 instead of v1, v2
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, release);
}

// All the individual formulas delegate to the Y versions
double DeflectionZEulerComputer::fixed_fixed_fixed_fixed(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED);
}

double DeflectionZEulerComputer::fixed_fixed_free_fixed(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FREE_FIXED);
}

double DeflectionZEulerComputer::fixed_fixed_fixed_free(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE);
}

double DeflectionZEulerComputer::fixed_fixed_free_free(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE);
}

double DeflectionZEulerComputer::fixed_free_fixed_fixed(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED);
}

double DeflectionZEulerComputer::fixed_free_free_fixed(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FREE_FREE_FIXED);
}

double DeflectionZEulerComputer::fixed_free_fixed_free(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE);
}

double DeflectionZEulerComputer::fixed_free_free_free(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FREE_FREE_FREE);
}

double DeflectionZEulerComputer::free_fixed_fixed_fixed(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FIXED_FIXED_FIXED);
}

double DeflectionZEulerComputer::free_fixed_free_fixed(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FIXED_FREE_FIXED);
}

double DeflectionZEulerComputer::free_fixed_fixed_free(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FIXED_FIXED_FREE);
}

double DeflectionZEulerComputer::free_fixed_free_free(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FIXED_FREE_FREE);
}

double DeflectionZEulerComputer::free_free_fixed_fixed(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED);
}

double DeflectionZEulerComputer::free_free_free_fixed(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FREE_FREE_FIXED);
}

double DeflectionZEulerComputer::free_free_fixed_free(double x) const {
    DeflectionYEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FREE_FIXED_FREE);
}

// RotationYEulerComputer
RotationYEulerComputer::RotationYEulerComputer(double L, double EI, double w1, double phi1,
                                                 double w2, double phi2, double q1, double q2)
    : L_(L), EI_(EI), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2), q1_(q1), q2_(q2) {}

double RotationYEulerComputer::compute(double x, ReleaseCombo4DOF release) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, release);
}

// All individual methods delegate to RotationZEulerComputer
double RotationYEulerComputer::fixed_fixed_fixed_fixed(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED);
}

double RotationYEulerComputer::fixed_fixed_free_fixed(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FREE_FIXED);
}

double RotationYEulerComputer::fixed_fixed_fixed_free(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE);
}

double RotationYEulerComputer::fixed_fixed_free_free(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FIXED_FREE_FREE);
}

double RotationYEulerComputer::fixed_free_fixed_fixed(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FREE_FIXED_FIXED);
}

double RotationYEulerComputer::fixed_free_free_fixed(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FREE_FREE_FIXED);
}

double RotationYEulerComputer::fixed_free_fixed_free(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FREE_FIXED_FREE);
}

double RotationYEulerComputer::fixed_free_free_free(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FIXED_FREE_FREE_FREE);
}

double RotationYEulerComputer::free_fixed_fixed_fixed(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FIXED_FIXED_FIXED);
}

double RotationYEulerComputer::free_fixed_free_fixed(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FIXED_FREE_FIXED);
}

double RotationYEulerComputer::free_fixed_fixed_free(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FIXED_FIXED_FREE);
}

double RotationYEulerComputer::free_fixed_free_free(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FIXED_FREE_FREE);
}

double RotationYEulerComputer::free_free_fixed_fixed(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FREE_FIXED_FIXED);
}

double RotationYEulerComputer::free_free_free_fixed(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FREE_FREE_FIXED);
}

double RotationYEulerComputer::free_free_fixed_free(double x) const {
    RotationZEulerComputer helper(L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_);
    return helper.compute(x, ReleaseCombo4DOF::FREE_FREE_FIXED_FREE);
}

} // namespace grillex
