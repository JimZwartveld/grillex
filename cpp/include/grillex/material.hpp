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
 *
 * Note: 1 mT = 1000 kg
 */
class Material {
public:
    int id;             ///< Unique material identifier
    std::string name;   ///< Material name
    double E;           ///< Young's modulus [kN/m²]
    double G;           ///< Shear modulus [kN/m²]
    double nu;          ///< Poisson's ratio (dimensionless)
    double rho;         ///< Density [mT/m³]

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
     * @param rho Density [mT/m³]
     */
    Material(int id, std::string name, double E, double nu, double rho);

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
