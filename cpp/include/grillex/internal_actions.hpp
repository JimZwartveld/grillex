#pragma once

#include <Eigen/Dense>

namespace grillex {

/**
 * @brief Element end forces in local coordinates
 *
 * Represents the internal forces and moments at an element end.
 * Sign conventions follow standard structural engineering practice:
 * - Axial: Tension positive, compression negative
 * - Shear: Positive per right-hand rule about local axes
 * - Moments: Positive per right-hand rule about local axes
 */
struct EndForces {
    double N = 0.0;   ///< Axial force [kN] (positive = tension)
    double Vy = 0.0;  ///< Shear force in local y [kN]
    double Vz = 0.0;  ///< Shear force in local z [kN]
    double Mx = 0.0;  ///< Torsion moment [kN·m]
    double My = 0.0;  ///< Bending moment about local y [kN·m]
    double Mz = 0.0;  ///< Bending moment about local z [kN·m]

    // For 14-DOF warping elements:
    double B = 0.0;   ///< Bimoment [kN·m²] (zero for 12-DOF elements)

    /**
     * @brief Default constructor (zero forces)
     */
    EndForces() = default;

    /**
     * @brief Construct from individual components
     */
    EndForces(double n, double vy, double vz, double mx, double my, double mz, double b = 0.0)
        : N(n), Vy(vy), Vz(vz), Mx(mx), My(my), Mz(mz), B(b) {}

    /**
     * @brief Construct from 6-component vector [N, Vy, Vz, Mx, My, Mz]
     */
    explicit EndForces(const Eigen::Vector<double, 6>& f)
        : N(f(0)), Vy(f(1)), Vz(f(2)), Mx(f(3)), My(f(4)), Mz(f(5)) {}

    /**
     * @brief Construct from 7-component vector [N, Vy, Vz, Mx, My, Mz, B]
     */
    explicit EndForces(const Eigen::Vector<double, 7>& f)
        : N(f(0)), Vy(f(1)), Vz(f(2)), Mx(f(3)), My(f(4)), Mz(f(5)), B(f(6)) {}

    /**
     * @brief Convert to 6-component vector [N, Vy, Vz, Mx, My, Mz]
     */
    Eigen::Vector<double, 6> to_vector6() const {
        return Eigen::Vector<double, 6>{N, Vy, Vz, Mx, My, Mz};
    }

    /**
     * @brief Convert to 7-component vector [N, Vy, Vz, Mx, My, Mz, B]
     */
    Eigen::Vector<double, 7> to_vector7() const {
        return Eigen::Vector<double, 7>{N, Vy, Vz, Mx, My, Mz, B};
    }
};

/**
 * @brief Internal actions at a position along the beam
 *
 * Represents internal forces and moments at any position x along the element.
 */
struct InternalActions {
    double x = 0.0;   ///< Position along beam [0, L] in meters
    double N = 0.0;   ///< Axial force [kN]
    double Vy = 0.0;  ///< Shear force in y [kN]
    double Vz = 0.0;  ///< Shear force in z [kN]
    double Mx = 0.0;  ///< Torsion moment [kN·m]
    double My = 0.0;  ///< Moment about y [kN·m]
    double Mz = 0.0;  ///< Moment about z [kN·m]

    /**
     * @brief Default constructor
     */
    InternalActions() = default;

    /**
     * @brief Construct at position x with zero actions
     */
    explicit InternalActions(double position) : x(position) {}

    /**
     * @brief Construct with all values
     */
    InternalActions(double position, double n, double vy, double vz,
                   double mx, double my, double mz)
        : x(position), N(n), Vy(vy), Vz(vz), Mx(mx), My(my), Mz(mz) {}
};

/**
 * @brief Extremum location and value
 *
 * Used to report moment/shear extrema along beam elements.
 */
struct ActionExtreme {
    double x = 0.0;      ///< Position along beam [m]
    double value = 0.0;  ///< Value at extremum

    ActionExtreme() = default;
    ActionExtreme(double pos, double val) : x(pos), value(val) {}
};

// Note: DistributedLoad is defined in load_case.hpp

/**
 * @brief Release combinations for bending (4-DOF)
 *
 * Each bending plane (x-y or x-z) has 4 DOFs at beam ends:
 * - w1: Transverse displacement at node i
 * - φ1: Rotation (slope) at node i
 * - w2: Transverse displacement at node j
 * - φ2: Rotation (slope) at node j
 *
 * FIXED = DOF is active (connected to global DOF)
 * FREE = DOF is released (internal hinge or roller)
 */
enum class ReleaseCombo4DOF {
    FIXED_FIXED_FIXED_FIXED = 0,   ///< w1, φ1, w2, φ2 all fixed (standard beam)
    FIXED_FIXED_FREE_FIXED = 1,    ///< w1, φ1, φ2 fixed; w2 free (roller at j)
    FIXED_FIXED_FIXED_FREE = 2,    ///< w1, φ1, w2 fixed; φ2 free (hinge at j)
    FIXED_FIXED_FREE_FREE = 3,     ///< w1, φ1 fixed; w2, φ2 free (cantilever-like at i)
    FIXED_FREE_FIXED_FIXED = 4,    ///< w1, w2, φ2 fixed; φ1 free (hinge at i)
    FIXED_FREE_FREE_FIXED = 5,     ///< w1, φ2 fixed; φ1, w2 free
    FIXED_FREE_FIXED_FREE = 6,     ///< w1, w2 fixed; φ1, φ2 free (double hinge)
    FIXED_FREE_FREE_FREE = 7,      ///< w1 fixed; φ1, w2, φ2 free
    FREE_FIXED_FIXED_FIXED = 8,    ///< φ1, w2, φ2 fixed; w1 free (roller at i)
    FREE_FIXED_FREE_FIXED = 9,     ///< φ1, φ2 fixed; w1, w2 free
    FREE_FIXED_FIXED_FREE = 10,    ///< φ1, w2 fixed; w1, φ2 free
    FREE_FIXED_FREE_FREE = 11,     ///< φ1 fixed; w1, w2, φ2 free
    FREE_FREE_FIXED_FIXED = 12,    ///< w2, φ2 fixed; w1, φ1 free (cantilever-like at j)
    FREE_FREE_FREE_FIXED = 13,     ///< φ2 fixed; w1, φ1, w2 free
    FREE_FREE_FIXED_FREE = 14,     ///< w2 fixed; w1, φ1, φ2 free
    FREE_FREE_FREE_FREE = 15       ///< All free (rigid body motion, unstable)
};

/**
 * @brief Release combinations for axial/torsion (2-DOF)
 *
 * Axial and torsion each have 2 DOFs:
 * - u1/θ1: Displacement/rotation at node i
 * - u2/θ2: Displacement/rotation at node j
 */
enum class ReleaseCombo2DOF {
    FIXED_FIXED = 0,  ///< Both ends fixed (standard)
    FIXED_FREE = 1,   ///< Start fixed, end free (cantilever-like)
    FREE_FIXED = 2,   ///< Start free, end fixed
    FREE_FREE = 3     ///< Both ends free (rigid body motion, unstable)
};

/**
 * @brief Displacements and rotations at a position along the beam
 *
 * Used for deflection diagrams and displacement queries.
 */
struct DisplacementLine {
    double x = 0.0;   ///< Position along beam [0, L] in meters
    double u = 0.0;   ///< Axial displacement [m]
    double v = 0.0;   ///< Lateral displacement in y [m]
    double w = 0.0;   ///< Lateral displacement in z [m]
    double theta_x = 0.0;  ///< Twist rotation (torsion) [rad]
    double theta_y = 0.0;  ///< Bending rotation about y [rad]
    double theta_z = 0.0;  ///< Bending rotation about z [rad]

    // For 14-DOF elements:
    double phi_prime = 0.0;  ///< Warping parameter (rate of twist) [rad]

    DisplacementLine() = default;
    DisplacementLine(double pos) : x(pos) {}
};

} // namespace grillex
