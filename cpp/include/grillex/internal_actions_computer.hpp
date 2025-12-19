#pragma once

#include "grillex/internal_actions.hpp"
#include "grillex/load_case.hpp"
#include <cmath>
#include <stdexcept>

namespace grillex {

/**
 * @brief Compute axial force at position x
 *
 * Differential equation: dN/dx + q_x = 0, N = EA * du/dx
 * Solution: N(x) depends on end displacements u1, u2 and distributed load q_x
 *
 * Formulas derived from pystructeng reference implementation.
 */
class AxialForceComputer {
public:
    /**
     * @brief Construct axial force computer
     * @param L Element length [m]
     * @param EA Axial stiffness E*A [kN]
     * @param u1 Axial displacement at node i [m]
     * @param u2 Axial displacement at node j [m]
     * @param q1 Distributed axial load at node i [kN/m]
     * @param q2 Distributed axial load at node j [kN/m]
     */
    AxialForceComputer(double L, double EA, double u1, double u2,
                       double q1, double q2);

    /**
     * @brief Compute axial force at position x
     * @param x Position along beam [0, L]
     * @param release Release combination
     * @return Axial force N [kN] (positive = tension)
     */
    double compute(double x, ReleaseCombo2DOF release) const;

private:
    double L_, EA_, u1_, u2_, q1_, q2_;

    // Analytical formulas for each release combination
    double fixed_fixed(double x) const;
    double fixed_free(double x) const;
    double free_fixed(double x) const;
};

/**
 * @brief Compute torsion moment at position x
 *
 * For uniform (St. Venant) torsion: Mx = GJ * dθ/dx
 *
 * Formulas derived for 2-DOF torsion (twist angles θ1, θ2).
 */
class TorsionComputer {
public:
    /**
     * @brief Construct torsion computer
     * @param L Element length [m]
     * @param GJ Torsional stiffness [kN·m²]
     * @param theta1 Twist angle at node i [rad]
     * @param theta2 Twist angle at node j [rad]
     * @param m1 Distributed torsion at node i [kN·m/m]
     * @param m2 Distributed torsion at node j [kN·m/m]
     */
    TorsionComputer(double L, double GJ, double theta1, double theta2,
                    double m1 = 0.0, double m2 = 0.0);

    /**
     * @brief Compute torsion moment at position x
     * @param x Position along beam [0, L]
     * @param release Release combination
     * @return Torsion moment Mx [kN·m]
     */
    double compute(double x, ReleaseCombo2DOF release) const;

private:
    double L_, GJ_, theta1_, theta2_, m1_, m2_;

    double fixed_fixed(double x) const;
    double fixed_free(double x) const;
    double free_fixed(double x) const;
};

/**
 * @brief Compute moment Mz (bending in x-y plane) - Euler-Bernoulli
 *
 * Differential equations:
 *   dV/dx + q = 0
 *   dM/dx - V = 0
 *   M = EI * d²w/dx²
 *
 * End conditions: w1, φ1 (slope), w2, φ2
 * Distributed load: q(x) = q1 + (q2 - q1) * x / L (trapezoidal)
 *
 * Formulas from pystructeng lines.py reference implementation.
 */
class MomentZEulerComputer {
public:
    /**
     * @brief Construct Euler-Bernoulli moment computer for z-axis bending
     * @param L Element length [m]
     * @param EI Bending stiffness E*Iz [kN·m²]
     * @param w1 Lateral displacement at node i [m]
     * @param phi1 Rotation (slope) at node i [rad]
     * @param w2 Lateral displacement at node j [m]
     * @param phi2 Rotation (slope) at node j [rad]
     * @param q1 Distributed lateral load at node i [kN/m]
     * @param q2 Distributed lateral load at node j [kN/m]
     */
    MomentZEulerComputer(double L, double EI, double w1, double phi1,
                         double w2, double phi2, double q1, double q2);

    /**
     * @brief Compute moment about z-axis at position x
     * @param x Position along beam [0, L]
     * @param release Release combination
     * @return Bending moment Mz [kN·m]
     */
    double compute(double x, ReleaseCombo4DOF release) const;

private:
    double L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_;

    // All 16 release combinations
    double fixed_fixed_fixed_fixed(double x) const;
    double fixed_fixed_free_fixed(double x) const;
    double fixed_fixed_fixed_free(double x) const;
    double fixed_fixed_free_free(double x) const;
    double fixed_free_fixed_fixed(double x) const;
    double fixed_free_free_fixed(double x) const;
    double fixed_free_fixed_free(double x) const;
    double fixed_free_free_free(double x) const;
    double free_fixed_fixed_fixed(double x) const;
    double free_fixed_free_fixed(double x) const;
    double free_fixed_fixed_free(double x) const;
    double free_fixed_free_free(double x) const;
    double free_free_fixed_fixed(double x) const;
    double free_free_free_fixed(double x) const;
    double free_free_fixed_free(double x) const;
};

/**
 * @brief Compute shear force Vy (bending in x-y plane) - Euler-Bernoulli
 *
 * V(x) = dM/dx (derivative of moment)
 *
 * Formulas from pystructeng lines.py reference implementation.
 */
class ShearYEulerComputer {
public:
    /**
     * @brief Construct Euler-Bernoulli shear computer for y-direction
     * @param L Element length [m]
     * @param EI Bending stiffness E*Iz [kN·m²]
     * @param w1 Lateral displacement at node i [m]
     * @param phi1 Rotation (slope) at node i [rad]
     * @param w2 Lateral displacement at node j [m]
     * @param phi2 Rotation (slope) at node j [rad]
     * @param q1 Distributed lateral load at node i [kN/m]
     * @param q2 Distributed lateral load at node j [kN/m]
     */
    ShearYEulerComputer(double L, double EI, double w1, double phi1,
                        double w2, double phi2, double q1, double q2);

    /**
     * @brief Compute shear force in y-direction at position x
     * @param x Position along beam [0, L]
     * @param release Release combination
     * @return Shear force Vy [kN]
     */
    double compute(double x, ReleaseCombo4DOF release) const;

private:
    double L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_;

    // All 16 release combinations
    double fixed_fixed_fixed_fixed(double x) const;
    double fixed_fixed_free_fixed(double x) const;
    double fixed_fixed_fixed_free(double x) const;
    double fixed_fixed_free_free(double x) const;
    double fixed_free_fixed_fixed(double x) const;
    double fixed_free_free_fixed(double x) const;
    double fixed_free_fixed_free(double x) const;
    double fixed_free_free_free(double x) const;
    double free_fixed_fixed_fixed(double x) const;
    double free_fixed_free_fixed(double x) const;
    double free_fixed_fixed_free(double x) const;
    double free_fixed_free_free(double x) const;
    double free_free_fixed_fixed(double x) const;
    double free_free_free_fixed(double x) const;
    double free_free_fixed_free(double x) const;
};

/**
 * @brief Compute moment My (bending in x-z plane) - Euler-Bernoulli
 *
 * Same formulas as MomentZEulerComputer but for x-z plane.
 * Uses w (z-displacement) and θy (rotation about y-axis).
 */
class MomentYEulerComputer {
public:
    MomentYEulerComputer(double L, double EI, double w1, double phi1,
                         double w2, double phi2, double q1, double q2);

    double compute(double x, ReleaseCombo4DOF release) const;

private:
    double L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_;

    double fixed_fixed_fixed_fixed(double x) const;
    double fixed_fixed_free_fixed(double x) const;
    double fixed_fixed_fixed_free(double x) const;
    double fixed_fixed_free_free(double x) const;
    double fixed_free_fixed_fixed(double x) const;
    double fixed_free_free_fixed(double x) const;
    double fixed_free_fixed_free(double x) const;
    double fixed_free_free_free(double x) const;
    double free_fixed_fixed_fixed(double x) const;
    double free_fixed_free_fixed(double x) const;
    double free_fixed_fixed_free(double x) const;
    double free_fixed_free_free(double x) const;
    double free_free_fixed_fixed(double x) const;
    double free_free_free_fixed(double x) const;
    double free_free_fixed_free(double x) const;
};

/**
 * @brief Compute shear force Vz (bending in x-z plane) - Euler-Bernoulli
 */
class ShearZEulerComputer {
public:
    ShearZEulerComputer(double L, double EI, double w1, double phi1,
                        double w2, double phi2, double q1, double q2);

    double compute(double x, ReleaseCombo4DOF release) const;

private:
    double L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_;

    double fixed_fixed_fixed_fixed(double x) const;
    double fixed_fixed_free_fixed(double x) const;
    double fixed_fixed_fixed_free(double x) const;
    double fixed_fixed_free_free(double x) const;
    double fixed_free_fixed_fixed(double x) const;
    double fixed_free_free_fixed(double x) const;
    double fixed_free_fixed_free(double x) const;
    double fixed_free_free_free(double x) const;
    double free_fixed_fixed_fixed(double x) const;
    double free_fixed_free_fixed(double x) const;
    double free_fixed_fixed_free(double x) const;
    double free_fixed_free_free(double x) const;
    double free_free_fixed_fixed(double x) const;
    double free_free_free_fixed(double x) const;
    double free_free_fixed_free(double x) const;
};

/**
 * @brief Compute moment Mz (bending in x-y plane) - Timoshenko
 *
 * Includes shear deformation effects via kAG (shear stiffness).
 * Formulas include (12*EI + L²*kAG) denominator terms.
 */
class MomentZTimoshenkoComputer {
public:
    /**
     * @brief Construct Timoshenko moment computer for z-axis bending
     * @param L Element length [m]
     * @param EI Bending stiffness E*Iz [kN·m²]
     * @param kAG Shear stiffness k*A*G [kN] where k is shear correction factor
     * @param w1 Lateral displacement at node i [m]
     * @param phi1 Rotation (slope) at node i [rad]
     * @param w2 Lateral displacement at node j [m]
     * @param phi2 Rotation (slope) at node j [rad]
     * @param q1 Distributed lateral load at node i [kN/m]
     * @param q2 Distributed lateral load at node j [kN/m]
     */
    MomentZTimoshenkoComputer(double L, double EI, double kAG,
                               double w1, double phi1,
                               double w2, double phi2,
                               double q1, double q2);

    double compute(double x, ReleaseCombo4DOF release) const;

private:
    double L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_;
    double Phi_;  // Shear parameter = 12*EI / (L²*kAG)

    double fixed_fixed_fixed_fixed(double x) const;
    double fixed_fixed_fixed_free(double x) const;
    double fixed_free_fixed_fixed(double x) const;
    double fixed_free_fixed_free(double x) const;
    double fixed_fixed_free_free(double x) const;
    double free_free_fixed_fixed(double x) const;
};

/**
 * @brief Compute shear force Vy (bending in x-y plane) - Timoshenko
 */
class ShearYTimoshenkoComputer {
public:
    ShearYTimoshenkoComputer(double L, double EI, double kAG,
                              double w1, double phi1,
                              double w2, double phi2,
                              double q1, double q2);

    double compute(double x, ReleaseCombo4DOF release) const;

private:
    double L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_;
    double Phi_;

    double fixed_fixed_fixed_fixed(double x) const;
    double fixed_fixed_fixed_free(double x) const;
    double fixed_free_fixed_fixed(double x) const;
    double fixed_free_fixed_free(double x) const;
    double fixed_fixed_free_free(double x) const;
    double free_free_fixed_fixed(double x) const;
};

/**
 * @brief Compute moment My and shear Vz for x-z plane - Timoshenko
 */
class MomentYTimoshenkoComputer {
public:
    MomentYTimoshenkoComputer(double L, double EI, double kAG,
                               double w1, double phi1,
                               double w2, double phi2,
                               double q1, double q2);

    double compute(double x, ReleaseCombo4DOF release) const;

private:
    double L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_;
    double Phi_;

    double fixed_fixed_fixed_fixed(double x) const;
    double fixed_fixed_fixed_free(double x) const;
    double fixed_free_fixed_fixed(double x) const;
    double fixed_free_fixed_free(double x) const;
    double fixed_fixed_free_free(double x) const;
    double free_free_fixed_fixed(double x) const;
};

class ShearZTimoshenkoComputer {
public:
    ShearZTimoshenkoComputer(double L, double EI, double kAG,
                              double w1, double phi1,
                              double w2, double phi2,
                              double q1, double q2);

    double compute(double x, ReleaseCombo4DOF release) const;

private:
    double L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_;
    double Phi_;

    double fixed_fixed_fixed_fixed(double x) const;
    double fixed_fixed_fixed_free(double x) const;
    double fixed_free_fixed_fixed(double x) const;
    double fixed_free_fixed_free(double x) const;
    double fixed_fixed_free_free(double x) const;
    double free_free_fixed_fixed(double x) const;
};

} // namespace grillex
