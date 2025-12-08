#pragma once

#include <string>

namespace grillex {

/**
 * @brief Cross-section properties for beam elements
 *
 * Stores geometric properties of beam cross-sections in consistent units:
 * - A: Cross-sectional area [m²]
 * - Iy, Iz: Second moments of area about local y and z axes [m⁴]
 * - J: Torsional constant [m⁴]
 * - Iw: Warping constant [m⁶] (optional, for thin-walled sections)
 * - Asy, Asz: Shear areas in y and z directions [m²]
 *
 * The local coordinate system is defined as:
 * - x: Along beam axis
 * - y, z: Principal axes of the cross-section
 */
class Section {
public:
    int id;             ///< Unique section identifier
    std::string name;   ///< Section name

    // Geometric properties
    double A;           ///< Cross-sectional area [m²]
    double Iy;          ///< Second moment of area about local y-axis [m⁴]
    double Iz;          ///< Second moment of area about local z-axis [m⁴]
    double J;           ///< Torsional constant [m⁴]
    double Iw;          ///< Warping constant [m⁶] (optional)
    double Asy;         ///< Shear area for forces in y-direction [m²]
    double Asz;         ///< Shear area for forces in z-direction [m²]

    // Distances to extreme fibres for stress calculation
    double zy_top;      ///< Distance to top extreme fibre for y-axis bending [m]
    double zy_bot;      ///< Distance to bottom extreme fibre for y-axis bending [m]
    double zz_top;      ///< Distance to top extreme fibre for z-axis bending [m]
    double zz_bot;      ///< Distance to bottom extreme fibre for z-axis bending [m]

    /**
     * @brief Construct a new Section
     *
     * Optional parameters (Iw, Asy, Asz, and fibre distances) default to 0.
     * For shear areas, common approximations are:
     * - Rectangular: As = (5/6) * A
     * - I-sections: As ≈ A_web
     *
     * @param id Unique section identifier
     * @param name Section name
     * @param A Cross-sectional area [m²]
     * @param Iy Second moment of area about y [m⁴]
     * @param Iz Second moment of area about z [m⁴]
     * @param J Torsional constant [m⁴]
     */
    Section(int id, std::string name, double A, double Iy, double Iz, double J);

    /**
     * @brief Set the warping constant
     *
     * @param Iw Warping constant [m⁶]
     */
    void set_warping_constant(double Iw);

    /**
     * @brief Set the shear areas
     *
     * @param Asy Shear area in y-direction [m²]
     * @param Asz Shear area in z-direction [m²]
     */
    void set_shear_areas(double Asy, double Asz);

    /**
     * @brief Set the distances to extreme fibres for stress calculations
     *
     * @param zy_top Distance to top fibre for y-bending [m]
     * @param zy_bot Distance to bottom fibre for y-bending [m]
     * @param zz_top Distance to top fibre for z-bending [m]
     * @param zz_bot Distance to bottom fibre for z-bending [m]
     */
    void set_fibre_distances(double zy_top, double zy_bot, double zz_top, double zz_bot);
};

} // namespace grillex
