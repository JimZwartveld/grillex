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

} // namespace grillex
