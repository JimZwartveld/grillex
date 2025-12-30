#pragma once

#include <string>

namespace grillex {

/**
 * @brief Material properties for structural elements
 *
 * Stores material properties in consistent units:
 * - E: Young's modulus [kN/m²]
 * - G: Shear modulus [kN/m²]
 * - nu: Poisson's ratio (dimensionless)
 * - rho: Density [mT/m³] (metric tonnes per cubic meter)
 * - fy: Yield stress [kN/m²] (optional, for design checks)
 * - fu: Ultimate tensile strength [kN/m²] (optional, for design checks)
 *
 * Note: 1 mT = 1000 kg
 * Typical steel density: 7.85 mT/m³ (7850 kg/m³)
 */
class Material {
public:
    int id;             ///< Unique material identifier
    std::string name;   ///< Material name
    double E;           ///< Young's modulus [kN/m²]
    double G;           ///< Shear modulus [kN/m²]
    double nu;          ///< Poisson's ratio (dimensionless)
    double rho;         ///< Density [mT/m³]
    double fy;          ///< Yield stress [kN/m²] (0 if not specified)
    double fu;          ///< Ultimate tensile strength [kN/m²] (0 if not specified)

    /**
     * @brief Construct a new Material
     *
     * The shear modulus G is automatically computed from E and nu using:
     * G = E / (2 * (1 + nu))
     *
     * @param id Unique material identifier
     * @param name Material name
     * @param E Young's modulus [kN/m²]
     * @param nu Poisson's ratio (dimensionless)
     * @param rho Density [mT/m³] (typical steel: 7.85 mT/m³)
     * @param fy Yield stress [kN/m²] (default: 0, not specified)
     * @param fu Ultimate tensile strength [kN/m²] (default: 0, not specified)
     */
    Material(int id, std::string name, double E, double nu, double rho,
             double fy = 0.0, double fu = 0.0);

    /**
     * @brief Compute shear modulus from Young's modulus and Poisson's ratio
     *
     * Formula: G = E / (2 * (1 + nu))
     *
     * @param E Young's modulus [kN/m²]
     * @param nu Poisson's ratio (dimensionless)
     * @return double Shear modulus [kN/m²]
     */
    static double compute_G(double E, double nu);
};

} // namespace grillex
