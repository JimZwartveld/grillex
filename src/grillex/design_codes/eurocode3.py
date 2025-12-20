"""Eurocode 3 (EN 1993-1-1) design code implementation.

This module provides basic structural steel checks according to Eurocode 3.
It is an illustrative implementation covering the most common checks:
- Cross-section resistance (axial, bending, shear)
- Combined axial and bending interaction

Note: This is a simplified implementation. A full Eurocode 3 implementation
would include buckling checks, lateral-torsional buckling, connection design,
and many additional provisions.

Reference:
    EN 1993-1-1:2005 - Eurocode 3: Design of steel structures -
    Part 1-1: General rules and rules for buildings

Example usage:
    >>> from grillex.design_codes.eurocode3 import Eurocode3
    >>> code = Eurocode3()
    >>> results = code.check_beam(beam, result_case, combination)
"""

from dataclasses import dataclass
from typing import Any, List, Optional

from .base import CheckResult, DesignCheck, DesignCode


# =============================================================================
# Safety Factors (Table 2.1, Recommended values)
# =============================================================================

# Partial factors for resistance
GAMMA_M0 = 1.0  # Resistance of cross-sections
GAMMA_M1 = 1.0  # Resistance of members to instability
GAMMA_M2 = 1.25  # Resistance of cross-sections in tension to fracture


# =============================================================================
# Section Classification Helper
# =============================================================================


@dataclass
class SectionClass:
    """Cross-section classification result.

    Eurocode 3 classifies sections into 4 classes based on local buckling:
    - Class 1: Can form plastic hinge with rotation capacity
    - Class 2: Can develop plastic moment but limited rotation
    - Class 3: Stress in extreme fibre can reach yield (elastic)
    - Class 4: Local buckling prevents yield being reached

    For simplicity, this implementation assumes Class 1 or 2 sections
    where the full plastic capacity can be used.
    """

    classification: int = 1  # Default to Class 1 (most favorable)
    flange_class: int = 1
    web_class: int = 1


def classify_section(section: Any, material: Any, actions: Optional[dict] = None) -> SectionClass:
    """Classify a cross-section according to EC3 Table 5.2.

    This is a simplified implementation that returns Class 1 by default.
    A full implementation would check c/t ratios for flanges and web.

    Args:
        section: Section object with geometric properties.
        material: Material object with fy (yield strength).
        actions: Optional internal actions for stress state classification.

    Returns:
        SectionClass with classification result.
    """
    # Simplified: assume all sections are Class 1 or 2
    # A real implementation would compute c/t ratios and compare to Table 5.2
    return SectionClass(classification=1, flange_class=1, web_class=1)


# =============================================================================
# Individual Checks
# =============================================================================


class EC3AxialCheck(DesignCheck):
    """EC3 cross-section resistance to axial force (Clause 6.2.4).

    Checks: N_Ed / N_c,Rd <= 1.0

    where:
        N_Ed = design axial force
        N_c,Rd = A * fy / gamma_M0 (for Class 1, 2, 3 sections)

    For tension, the net section resistance (with bolt holes) may govern,
    but this simplified implementation uses gross section only.
    """

    @property
    def name(self) -> str:
        return "EC3 Axial (6.2.4)"

    def compute_utilization(
        self,
        actions: dict,
        section: Any,
        material: Any,
        **kwargs: Any,
    ) -> float:
        """Compute axial utilization ratio.

        Args:
            actions: Must contain 'N' (axial force in kN).
            section: Must have 'A' (area in m²).
            material: Must have 'fy' (yield strength in kN/m²).
            **kwargs: Optional 'gamma_M0' (default 1.0).

        Returns:
            Utilization ratio N_Ed / N_c,Rd.
        """
        N_Ed = abs(actions.get("N", 0.0))
        A = getattr(section, "A", 0.0)
        fy = getattr(material, "fy", 0.0)
        gamma_M0 = kwargs.get("gamma_M0", GAMMA_M0)

        if A <= 0 or fy <= 0:
            return 0.0

        N_c_Rd = A * fy / gamma_M0
        return N_Ed / N_c_Rd


class EC3BendingYCheck(DesignCheck):
    """EC3 cross-section resistance to bending about y-axis (Clause 6.2.5).

    Checks: M_y,Ed / M_y,c,Rd <= 1.0

    where:
        M_y,Ed = design bending moment about y-axis
        M_y,c,Rd = W_pl,y * fy / gamma_M0 (for Class 1, 2 sections)
                 = W_el,y * fy / gamma_M0 (for Class 3 sections)

    This implementation uses elastic section modulus (W_el = I/z).
    """

    @property
    def name(self) -> str:
        return "EC3 Bending My (6.2.5)"

    def compute_utilization(
        self,
        actions: dict,
        section: Any,
        material: Any,
        **kwargs: Any,
    ) -> float:
        """Compute bending utilization ratio for My.

        Args:
            actions: Must contain 'My' (bending moment in kNm).
            section: Must have 'Iy' (moment of inertia in m⁴) and
                    optionally 'zy_max' (distance to extreme fiber in m).
            material: Must have 'fy' (yield strength in kN/m²).
            **kwargs: Optional 'gamma_M0', 'W_el_y' (section modulus).

        Returns:
            Utilization ratio M_y,Ed / M_y,c,Rd.
        """
        M_y_Ed = abs(actions.get("My", 0.0))
        fy = getattr(material, "fy", 0.0)
        gamma_M0 = kwargs.get("gamma_M0", GAMMA_M0)

        # Get or compute section modulus
        W_el_y = kwargs.get("W_el_y", None)
        if W_el_y is None:
            Iy = getattr(section, "Iy", 0.0)
            zy_max = getattr(section, "zy_max", None)
            if zy_max is None:
                # Estimate for symmetric section (e.g., I-beam)
                # Assume typical depth/Iy relationship
                zy_max = kwargs.get("zy_max", 0.15)  # Default guess
            W_el_y = Iy / zy_max if zy_max > 0 else 0.0

        if W_el_y <= 0 or fy <= 0:
            return 0.0

        M_y_c_Rd = W_el_y * fy / gamma_M0
        return M_y_Ed / M_y_c_Rd


class EC3BendingZCheck(DesignCheck):
    """EC3 cross-section resistance to bending about z-axis (Clause 6.2.5).

    Similar to EC3BendingYCheck but for the minor axis.
    """

    @property
    def name(self) -> str:
        return "EC3 Bending Mz (6.2.5)"

    def compute_utilization(
        self,
        actions: dict,
        section: Any,
        material: Any,
        **kwargs: Any,
    ) -> float:
        """Compute bending utilization ratio for Mz.

        Args:
            actions: Must contain 'Mz' (bending moment in kNm).
            section: Must have 'Iz' (moment of inertia in m⁴).
            material: Must have 'fy' (yield strength in kN/m²).
            **kwargs: Optional 'gamma_M0', 'W_el_z' (section modulus).

        Returns:
            Utilization ratio M_z,Ed / M_z,c,Rd.
        """
        M_z_Ed = abs(actions.get("Mz", 0.0))
        fy = getattr(material, "fy", 0.0)
        gamma_M0 = kwargs.get("gamma_M0", GAMMA_M0)

        # Get or compute section modulus
        W_el_z = kwargs.get("W_el_z", None)
        if W_el_z is None:
            Iz = getattr(section, "Iz", 0.0)
            zz_max = getattr(section, "zz_max", None)
            if zz_max is None:
                zz_max = kwargs.get("zz_max", 0.075)  # Default guess
            W_el_z = Iz / zz_max if zz_max > 0 else 0.0

        if W_el_z <= 0 or fy <= 0:
            return 0.0

        M_z_c_Rd = W_el_z * fy / gamma_M0
        return M_z_Ed / M_z_c_Rd


class EC3ShearYCheck(DesignCheck):
    """EC3 cross-section resistance to shear in y-direction (Clause 6.2.6).

    Checks: V_y,Ed / V_y,c,Rd <= 1.0

    where:
        V_y,Ed = design shear force in y-direction
        V_y,c,Rd = A_v,y * (fy / sqrt(3)) / gamma_M0

    This is a simplified plastic shear resistance check.
    """

    @property
    def name(self) -> str:
        return "EC3 Shear Vy (6.2.6)"

    def compute_utilization(
        self,
        actions: dict,
        section: Any,
        material: Any,
        **kwargs: Any,
    ) -> float:
        """Compute shear utilization ratio for Vy.

        Args:
            actions: Must contain 'Vy' (shear force in kN).
            section: Must have 'A' (area in m²) or 'Av_y' (shear area).
            material: Must have 'fy' (yield strength in kN/m²).
            **kwargs: Optional 'gamma_M0', 'Av_y' (shear area).

        Returns:
            Utilization ratio V_y,Ed / V_y,c,Rd.
        """
        V_y_Ed = abs(actions.get("Vy", 0.0))
        fy = getattr(material, "fy", 0.0)
        gamma_M0 = kwargs.get("gamma_M0", GAMMA_M0)

        # Shear area (simplified: use 60% of gross area if not specified)
        Av_y = kwargs.get("Av_y", None)
        if Av_y is None:
            Av_y = getattr(section, "Av_y", None)
        if Av_y is None:
            A = getattr(section, "A", 0.0)
            Av_y = 0.6 * A  # Approximate for I-sections

        if Av_y <= 0 or fy <= 0:
            return 0.0

        # Plastic shear resistance: tau_y = fy / sqrt(3)
        import math
        V_y_c_Rd = Av_y * (fy / math.sqrt(3)) / gamma_M0
        return V_y_Ed / V_y_c_Rd


class EC3ShearZCheck(DesignCheck):
    """EC3 cross-section resistance to shear in z-direction (Clause 6.2.6).

    Similar to EC3ShearYCheck but for shear in z-direction.
    """

    @property
    def name(self) -> str:
        return "EC3 Shear Vz (6.2.6)"

    def compute_utilization(
        self,
        actions: dict,
        section: Any,
        material: Any,
        **kwargs: Any,
    ) -> float:
        """Compute shear utilization ratio for Vz."""
        V_z_Ed = abs(actions.get("Vz", 0.0))
        fy = getattr(material, "fy", 0.0)
        gamma_M0 = kwargs.get("gamma_M0", GAMMA_M0)

        # Shear area
        Av_z = kwargs.get("Av_z", None)
        if Av_z is None:
            Av_z = getattr(section, "Av_z", None)
        if Av_z is None:
            A = getattr(section, "A", 0.0)
            Av_z = 0.6 * A

        if Av_z <= 0 or fy <= 0:
            return 0.0

        import math
        V_z_c_Rd = Av_z * (fy / math.sqrt(3)) / gamma_M0
        return V_z_Ed / V_z_c_Rd


class EC3CombinedCheck(DesignCheck):
    """EC3 combined axial and bending interaction (Clause 6.2.1, 6.2.9).

    For Class 1 and 2 sections, the simplified linear interaction is:
        N_Ed/N_Rd + M_y,Ed/M_y,Rd + M_z,Ed/M_z,Rd <= 1.0

    For more accurate plastic interaction (6.2.9.1), the reduced moment
    capacities considering axial force should be used, but this simplified
    implementation uses the linear formula.

    Note: A full implementation would include:
    - Reduced plastic moment capacity MN,Rd
    - Different interaction for Class 3/4 sections
    - Member buckling interaction (6.3.3)
    """

    @property
    def name(self) -> str:
        return "EC3 Combined N+M (6.2.1)"

    def compute_utilization(
        self,
        actions: dict,
        section: Any,
        material: Any,
        **kwargs: Any,
    ) -> float:
        """Compute combined axial + bending utilization.

        Uses simplified linear interaction:
        util = N_Ed/N_Rd + M_y,Ed/M_y,Rd + M_z,Ed/M_z,Rd

        Args:
            actions: Must contain 'N', 'My', 'Mz'.
            section: Must have 'A', 'Iy', 'Iz'.
            material: Must have 'fy'.
            **kwargs: Optional section moduli 'W_el_y', 'W_el_z'.

        Returns:
            Combined utilization ratio.
        """
        # Compute individual utilizations
        axial_check = EC3AxialCheck()
        bending_y_check = EC3BendingYCheck()
        bending_z_check = EC3BendingZCheck()

        n_util = axial_check.compute_utilization(actions, section, material, **kwargs)
        my_util = bending_y_check.compute_utilization(actions, section, material, **kwargs)
        mz_util = bending_z_check.compute_utilization(actions, section, material, **kwargs)

        # Linear interaction
        return n_util + my_util + mz_util


# =============================================================================
# Eurocode 3 Design Code
# =============================================================================


class Eurocode3(DesignCode):
    """Eurocode 3 (EN 1993-1-1) design code implementation.

    This is a simplified implementation covering basic cross-section
    resistance checks. A full implementation would include:
    - Section classification (Class 1-4)
    - Member buckling (flexural, torsional, lateral-torsional)
    - Connection design
    - Fatigue
    - Fire design
    - And many additional provisions

    Attributes:
        gamma_M0: Partial factor for cross-section resistance (default 1.0).
        check_locations: Normalized positions along beam to check (0.0 to 1.0).

    Example:
        >>> code = Eurocode3(gamma_M0=1.0)
        >>> code.check_locations = [0.0, 0.25, 0.5, 0.75, 1.0]
        >>> results = code.check_beam(beam, result_case, combination)
    """

    def __init__(
        self,
        gamma_M0: float = GAMMA_M0,
        gamma_M1: float = GAMMA_M1,
        gamma_M2: float = GAMMA_M2,
        check_locations: Optional[List[float]] = None,
    ):
        """Initialize Eurocode 3 design code.

        Args:
            gamma_M0: Partial factor for cross-section resistance.
            gamma_M1: Partial factor for member instability.
            gamma_M2: Partial factor for tension fracture.
            check_locations: Positions along beam to check (default: 5 locations).
        """
        self.gamma_M0 = gamma_M0
        self.gamma_M1 = gamma_M1
        self.gamma_M2 = gamma_M2
        self.check_locations = check_locations or [0.0, 0.25, 0.5, 0.75, 1.0]

    @property
    def name(self) -> str:
        return "Eurocode 3 (EN 1993-1-1)"

    @property
    def version(self) -> str:
        return "2005"

    def get_checks(self) -> List[DesignCheck]:
        """Return list of all implemented checks.

        Returns:
            List containing axial, bending, shear, and combined checks.
        """
        return [
            EC3AxialCheck(),
            EC3BendingYCheck(),
            EC3BendingZCheck(),
            EC3ShearYCheck(),
            EC3ShearZCheck(),
            EC3CombinedCheck(),
        ]

    def check_beam(
        self,
        beam: Any,
        result_case: Any,
        combination: Any,
    ) -> List[CheckResult]:
        """Perform all design checks on a beam member.

        Args:
            beam: Beam element with 'id', 'section', 'material' attributes.
                  Also needs method to get internal actions at locations.
            result_case: Analysis results (provides get_actions_at method).
            combination: Load combination with 'name' attribute.

        Returns:
            List of CheckResult objects for all checks at all locations.
        """
        results = []
        section = getattr(beam, "section", None)
        material = getattr(beam, "material", None)

        if section is None or material is None:
            return results

        element_id = getattr(beam, "id", 0)
        combination_name = getattr(combination, "name", "Unknown")

        # Prepare kwargs with safety factors and section properties
        kwargs = {
            "gamma_M0": self.gamma_M0,
            "gamma_M1": self.gamma_M1,
            "gamma_M2": self.gamma_M2,
        }

        # Add section moduli if available
        if hasattr(section, "Wy"):
            kwargs["W_el_y"] = section.Wy
        if hasattr(section, "Wz"):
            kwargs["W_el_z"] = section.Wz

        # Check at each location
        for loc in self.check_locations:
            # Get internal actions at this location
            if hasattr(result_case, "get_actions_at"):
                actions = result_case.get_actions_at(element_id, loc)
            elif hasattr(result_case, "get_internal_actions"):
                # Alternative interface
                ia = result_case.get_internal_actions(element_id, loc)
                actions = {
                    "N": getattr(ia, "N", 0.0),
                    "Vy": getattr(ia, "Vy", 0.0),
                    "Vz": getattr(ia, "Vz", 0.0),
                    "Mx": getattr(ia, "Mx", 0.0),
                    "My": getattr(ia, "My", 0.0),
                    "Mz": getattr(ia, "Mz", 0.0),
                }
            else:
                # Fallback: no actions available
                actions = {"N": 0.0, "Vy": 0.0, "Vz": 0.0, "Mx": 0.0, "My": 0.0, "Mz": 0.0}

            # Perform each check
            for check in self.get_checks():
                result = check.check(
                    actions=actions,
                    section=section,
                    material=material,
                    element_id=element_id,
                    location=loc,
                    load_combination=combination_name,
                    **kwargs,
                )
                results.append(result)

        # Mark governing result
        self.get_governing_results(results)

        return results
