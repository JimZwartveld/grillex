"""Unit and axis conversion utilities for section properties.

This module handles the conversion between sectionbuilder's conventions
and grillex's conventions, including:
- Unit scaling (mm -> m or m -> m)
- Axis mapping (Ixx -> Iz, Iyy -> Iy)
"""

from typing import Dict, Any, Optional
from dataclasses import dataclass


@dataclass
class ScaleFactors:
    """Scale factors for converting section properties.

    Attributes:
        length: Scale factor for length quantities (e.g., fibre distances)
        area: Scale factor for area quantities (e.g., A, Asy, Asz, omega_max)
        I: Scale factor for second moment of area (m^4)
        Iw: Scale factor for warping constant (m^6)
        S: Scale factor for section moduli (m^3)
    """
    length: float
    area: float
    I: float
    Iw: float
    S: float


def get_scale_factors(input_units: str) -> ScaleFactors:
    """Get scale factors based on input units.

    Args:
        input_units: "mm" for millimeter-based databases, "m" for meter-based

    Returns:
        ScaleFactors with appropriate conversion factors

    Raises:
        ValueError: If input_units is not "mm" or "m"

    Examples:
        >>> sf = get_scale_factors("mm")
        >>> sf.length
        0.001
        >>> sf.area
        1e-06
        >>> sf = get_scale_factors("m")
        >>> sf.length
        1.0
    """
    if input_units == "m":
        return ScaleFactors(
            length=1.0,
            area=1.0,
            I=1.0,
            Iw=1.0,
            S=1.0
        )
    elif input_units == "mm":
        L = 1e-3  # mm to m
        return ScaleFactors(
            length=L,
            area=L ** 2,
            I=L ** 4,
            Iw=L ** 6,
            S=L ** 3
        )
    else:
        raise ValueError(
            f"Invalid input_units '{input_units}'. Must be 'mm' or 'm'."
        )


def convert_section_properties(
    sb_props: Any,
    input_units: str = "mm"
) -> Dict[str, Any]:
    """Convert sectionbuilder properties to grillex units and conventions.

    Handles both unit scaling and axis convention mapping:
    - Ixx (sectionbuilder, horizontal/strong) -> Iz (grillex, vertical/strong)
    - Iyy (sectionbuilder, vertical/weak) -> Iy (grillex, horizontal/weak)
    - Avy (sectionbuilder) -> Asy (grillex)
    - Avx (sectionbuilder) -> Asz (grillex)

    Args:
        sb_props: SectionBuilder SectionProperties object with attributes like
            area, Ixx, Iyy, J, Cw, omega_max, Avx, Avy, y_top, y_bottom, etc.
        input_units: "mm" for standard databases, "m" for custom sections

    Returns:
        Dict with grillex section parameters:
            - A: Cross-sectional area [m²]
            - Iy: Second moment, weak axis [m⁴]
            - Iz: Second moment, strong axis [m⁴]
            - J: Torsional constant [m⁴]
            - Iw: Warping constant [m⁶]
            - omega_max: Maximum sectorial coordinate [m²]
            - Asy: Shear area in y-direction [m²]
            - Asz: Shear area in z-direction [m²]
            - zy_top, zy_bot: Fibre distances for y-axis bending [m]
            - zz_top, zz_bot: Fibre distances for z-axis bending [m]
            - requires_warping: Whether warping analysis is needed

    Raises:
        ValueError: If input_units is invalid
    """
    sf = get_scale_factors(input_units)

    # Helper function to safely get attribute with default
    def get_attr(obj: Any, name: str, default: float = 0.0) -> float:
        val = getattr(obj, name, default)
        return val if val is not None else default

    # Core properties with axis mapping
    result = {
        # Area
        "A": get_attr(sb_props, "area") * sf.area,

        # Second moments of area (with axis mapping)
        # sectionbuilder Ixx (horizontal/strong) -> grillex Iz (strong axis)
        # sectionbuilder Iyy (vertical/weak) -> grillex Iy (weak axis)
        "Iz": get_attr(sb_props, "Ixx") * sf.I,
        "Iy": get_attr(sb_props, "Iyy") * sf.I,

        # Torsional constant
        "J": get_attr(sb_props, "J") * sf.I,

        # Warping constant (Cw in sectionbuilder -> Iw in grillex)
        "Iw": get_attr(sb_props, "Cw") * sf.Iw,

        # Maximum sectorial coordinate
        "omega_max": get_attr(sb_props, "omega_max") * sf.area,

        # Shear areas (with axis mapping)
        # sectionbuilder Avy (vertical shear) -> grillex Asy
        # sectionbuilder Avx (horizontal shear) -> grillex Asz
        "Asy": get_attr(sb_props, "Avy") * sf.area,
        "Asz": get_attr(sb_props, "Avx") * sf.area,

        # Fibre distances (with axis mapping)
        # y_top/y_bottom -> zy_top/zy_bot (for strong axis bending)
        "zy_top": get_attr(sb_props, "y_top") * sf.length,
        "zy_bot": get_attr(sb_props, "y_bottom") * sf.length,
        # x_left/x_right -> zz_top/zz_bot (for weak axis bending)
        "zz_top": get_attr(sb_props, "x_right") * sf.length,
        "zz_bot": get_attr(sb_props, "x_left") * sf.length,
    }

    # Determine if warping analysis is required
    # Thin-walled open sections with Iw > 0 typically require warping
    result["requires_warping"] = result["Iw"] > 0

    return result


def convert_section_dict(
    props_dict: Dict[str, Any],
    input_units: str = "mm"
) -> Dict[str, Any]:
    """Convert a dictionary of section properties to grillex conventions.

    This is useful when working with JSON libraries or custom section data
    that's already in dictionary form rather than a SectionProperties object.

    Args:
        props_dict: Dictionary with keys matching sectionbuilder naming
            (area, Ixx, Iyy, J, Cw, etc.)
        input_units: "mm" for standard databases, "m" for custom sections

    Returns:
        Dict with grillex section parameters

    Examples:
        >>> props = {"area": 5380, "Ixx": 83600000, "Iyy": 6040000, "J": 201000}
        >>> result = convert_section_dict(props, input_units="mm")
        >>> abs(result["A"] - 0.00538) < 1e-6
        True
    """
    sf = get_scale_factors(input_units)

    def get_val(key: str, default: float = 0.0) -> float:
        val = props_dict.get(key, default)
        return val if val is not None else default

    result = {
        "A": get_val("area") * sf.area,
        "Iz": get_val("Ixx") * sf.I,
        "Iy": get_val("Iyy") * sf.I,
        "J": get_val("J") * sf.I,
        "Iw": get_val("Cw") * sf.Iw,
        "omega_max": get_val("omega_max") * sf.area,
        "Asy": get_val("Avy") * sf.area,
        "Asz": get_val("Avx") * sf.area,
        "zy_top": get_val("y_top") * sf.length,
        "zy_bot": get_val("y_bottom") * sf.length,
        "zz_top": get_val("x_right") * sf.length,
        "zz_bot": get_val("x_left") * sf.length,
    }

    result["requires_warping"] = result["Iw"] > 0

    return result
