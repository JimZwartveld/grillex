"""Extended section properties for design code checks.

This module provides additional section properties that are needed
for design code checks (like EC3) but not required for FE analysis.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import math


@dataclass
class ExtendedSectionProperties:
    """Extended section properties for design code checks.

    Stores additional properties computed by sectionbuilder that are
    needed for design code unity checks but not for FE analysis.

    All properties are in grillex units (m, m², m³, m⁴, m⁶).

    Attributes:
        # From grillex Section (for reference)
        A: Cross-sectional area [m²]
        Iy: Second moment, weak axis [m⁴]
        Iz: Second moment, strong axis [m⁴]
        J: Torsional constant [m⁴]
        Iw: Warping constant [m⁶]

        # Additional for design
        Zpy: Plastic modulus, weak axis [m³]
        Zpz: Plastic modulus, strong axis [m³]
        Wely: Elastic modulus, weak axis [m³]
        Welz: Elastic modulus, strong axis [m³]
        iy: Radius of gyration, weak axis [m]
        iz: Radius of gyration, strong axis [m]

        # Section dimensions (for classification)
        h: Total height [m]
        b: Flange width [m]
        tw: Web thickness [m]
        tf: Flange thickness [m]

        # Classification helpers
        hw: Clear web height [m]
        cf: Flange outstand [m]
    """
    # Core FEM properties
    A: float = 0.0
    Iy: float = 0.0
    Iz: float = 0.0
    J: float = 0.0
    Iw: float = 0.0

    # Design properties - section moduli
    Zpy: float = 0.0  # Plastic modulus, weak axis
    Zpz: float = 0.0  # Plastic modulus, strong axis
    Wely: float = 0.0  # Elastic modulus, weak axis
    Welz: float = 0.0  # Elastic modulus, strong axis

    # Radii of gyration
    iy: float = 0.0
    iz: float = 0.0

    # Section dimensions
    h: float = 0.0   # Total height
    b: float = 0.0   # Flange width
    tw: float = 0.0  # Web thickness
    tf: float = 0.0  # Flange thickness

    # Classification helpers
    hw: float = 0.0  # Clear web height
    cf: float = 0.0  # Flange outstand

    # Shear areas
    Asy: float = 0.0
    Asz: float = 0.0

    # Fibre distances
    zy_top: float = 0.0
    zy_bot: float = 0.0
    zz_top: float = 0.0
    zz_bot: float = 0.0

    # Warping properties
    omega_max: float = 0.0
    requires_warping: bool = False

    # Metadata
    designation: str = ""
    section_type: str = ""  # "I", "H", "channel", "angle", etc.

    @classmethod
    def from_grillex_params(
        cls,
        params: Dict[str, Any],
        designation: str = "",
        section_type: str = ""
    ) -> "ExtendedSectionProperties":
        """Create from grillex section parameters dict.

        Args:
            params: Dict from GrillexSectionAdapter.get_grillex_section_params()
            designation: Section designation (e.g., "HEB300")
            section_type: Section type (e.g., "I", "H")

        Returns:
            ExtendedSectionProperties instance
        """
        return cls(
            A=params.get("A", 0.0),
            Iy=params.get("Iy", 0.0),
            Iz=params.get("Iz", 0.0),
            J=params.get("J", 0.0),
            Iw=params.get("Iw", 0.0),
            Asy=params.get("Asy", 0.0),
            Asz=params.get("Asz", 0.0),
            zy_top=params.get("zy_top", 0.0),
            zy_bot=params.get("zy_bot", 0.0),
            zz_top=params.get("zz_top", 0.0),
            zz_bot=params.get("zz_bot", 0.0),
            omega_max=params.get("omega_max", 0.0),
            requires_warping=params.get("requires_warping", False),
            designation=designation,
            section_type=section_type,
        )

    @classmethod
    def from_sectionbuilder(
        cls,
        sb_props: Any,
        input_units: str = "mm",
        designation: str = "",
        section_type: str = ""
    ) -> "ExtendedSectionProperties":
        """Create from sectionbuilder SectionProperties object.

        Handles unit conversion and axis mapping.

        Args:
            sb_props: SectionBuilder SectionProperties object
            input_units: "mm" for standard databases, "m" for custom
            designation: Section designation
            section_type: Section type

        Returns:
            ExtendedSectionProperties instance
        """
        from .conversion import get_scale_factors

        sf = get_scale_factors(input_units)

        def get_attr(name: str, default: float = 0.0) -> float:
            val = getattr(sb_props, name, default)
            return val if val is not None else default

        props = cls(
            # Core properties with axis mapping
            A=get_attr("area") * sf.area,
            Iz=get_attr("Ixx") * sf.I,  # Strong axis
            Iy=get_attr("Iyy") * sf.I,  # Weak axis
            J=get_attr("J") * sf.I,
            Iw=get_attr("Cw") * sf.Iw,

            # Section moduli with axis mapping
            # Zx -> Zpz (strong axis plastic)
            # Zy -> Zpy (weak axis plastic)
            Zpz=get_attr("Zx") * sf.S,
            Zpy=get_attr("Zy") * sf.S,
            # Sx_pos -> Welz (strong axis elastic)
            # Sy_pos -> Wely (weak axis elastic)
            Welz=get_attr("Sx_pos") * sf.S,
            Wely=get_attr("Sy_pos") * sf.S,

            # Shear areas
            Asy=get_attr("Avy") * sf.area,
            Asz=get_attr("Avx") * sf.area,

            # Fibre distances
            zy_top=get_attr("y_top") * sf.length,
            zy_bot=get_attr("y_bottom") * sf.length,
            zz_top=get_attr("x_right") * sf.length,
            zz_bot=get_attr("x_left") * sf.length,

            # Warping
            omega_max=get_attr("omega_max") * sf.area,

            # Metadata
            designation=designation,
            section_type=section_type,
        )

        # Calculate derived properties
        props._calculate_derived()

        return props

    def _calculate_derived(self) -> None:
        """Calculate derived properties from core values."""
        # Radii of gyration
        if self.A > 0:
            self.iy = math.sqrt(self.Iy / self.A) if self.Iy > 0 else 0.0
            self.iz = math.sqrt(self.Iz / self.A) if self.Iz > 0 else 0.0

        # Determine if warping is required
        self.requires_warping = self.Iw > 0

        # Estimate elastic moduli from I and fibre distances if not provided
        if self.Welz == 0 and self.Iz > 0 and self.zy_top > 0:
            self.Welz = self.Iz / max(self.zy_top, self.zy_bot)
        if self.Wely == 0 and self.Iy > 0 and self.zz_top > 0:
            self.Wely = self.Iy / max(self.zz_top, self.zz_bot)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "A": self.A,
            "Iy": self.Iy,
            "Iz": self.Iz,
            "J": self.J,
            "Iw": self.Iw,
            "Zpy": self.Zpy,
            "Zpz": self.Zpz,
            "Wely": self.Wely,
            "Welz": self.Welz,
            "iy": self.iy,
            "iz": self.iz,
            "h": self.h,
            "b": self.b,
            "tw": self.tw,
            "tf": self.tf,
            "hw": self.hw,
            "cf": self.cf,
            "Asy": self.Asy,
            "Asz": self.Asz,
            "zy_top": self.zy_top,
            "zy_bot": self.zy_bot,
            "zz_top": self.zz_top,
            "zz_bot": self.zz_bot,
            "omega_max": self.omega_max,
            "requires_warping": self.requires_warping,
            "designation": self.designation,
            "section_type": self.section_type,
        }

    def to_grillex_params(self) -> Dict[str, Any]:
        """Get parameters for grillex add_section().

        Returns:
            Dict with A, Iy, Iz, J, Iw, Asy, Asz, etc.
        """
        return {
            "A": self.A,
            "Iy": self.Iy,
            "Iz": self.Iz,
            "J": self.J,
            "Iw": self.Iw,
            "Asy": self.Asy,
            "Asz": self.Asz,
            "zy_top": self.zy_top,
            "zy_bot": self.zy_bot,
            "zz_top": self.zz_top,
            "zz_bot": self.zz_bot,
            "omega_max": self.omega_max,
            "requires_warping": self.requires_warping,
        }
