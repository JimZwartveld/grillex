"""Adapter for using SectionBuilder sections in Grillex.

This module provides the GrillexSectionAdapter class that wraps
SectionBuilder library adapters and handles:
- Unit conversion (mm -> m or passthrough)
- Axis convention mapping
- Property caching
- Error handling for missing properties
"""

from typing import Dict, Any, List, Optional, Union, TYPE_CHECKING
from dataclasses import dataclass

from .conversion import convert_section_properties, get_scale_factors

# Type checking imports - sectionbuilder may not be installed
if TYPE_CHECKING:
    from sectionbuilder.adapters import SectionLibraryAdapter
    from sectionbuilder.core.properties import SectionProperties


@dataclass
class SectionInfo:
    """Information about a section from a library.

    Attributes:
        designation: Section designation (e.g., "HEB300", "W14X22")
        description: Optional description or type information
        library_name: Name of the source library
    """
    designation: str
    description: str = ""
    library_name: str = ""


class GrillexSectionAdapter:
    """Adapter to use SectionBuilder sections in Grillex.

    Handles unit conversion and axis mapping automatically based on
    the configured input_units parameter.

    Attributes:
        input_units: The unit system of the underlying adapter ("mm" or "m")

    Examples:
        >>> # With a sectionbuilder Eurocode adapter
        >>> from sectionbuilder.adapters import EurocodeAdapter  # doctest: +SKIP
        >>> eurocode = EurocodeAdapter()  # doctest: +SKIP
        >>> adapter = GrillexSectionAdapter(eurocode, input_units="mm")  # doctest: +SKIP
        >>> params = adapter.get_grillex_section_params("HEB300")  # doctest: +SKIP
        >>> # params contains A, Iy, Iz, J in meters
    """

    def __init__(
        self,
        sb_adapter: "SectionLibraryAdapter",
        input_units: str = "mm"
    ):
        """Initialize the adapter.

        Args:
            sb_adapter: SectionBuilder library adapter (e.g., EurocodeAdapter)
            input_units: "mm" for standard databases (default), "m" for custom sections

        Raises:
            ValueError: If input_units is not "mm" or "m"
        """
        if input_units not in ("mm", "m"):
            raise ValueError(
                f"Invalid input_units '{input_units}'. Must be 'mm' or 'm'."
            )

        self._sb_adapter = sb_adapter
        self._input_units = input_units
        self._cache: Dict[str, Dict[str, Any]] = {}

    @property
    def input_units(self) -> str:
        """The input unit system."""
        return self._input_units

    @property
    def adapter_name(self) -> str:
        """Get the name of the underlying adapter."""
        if hasattr(self._sb_adapter, "name"):
            return self._sb_adapter.name
        return type(self._sb_adapter).__name__

    def get_section_properties(
        self,
        designation: str
    ) -> "SectionProperties":
        """Get raw section properties from underlying adapter.

        Args:
            designation: Section designation (e.g., "HEB300")

        Returns:
            SectionBuilder SectionProperties object

        Raises:
            KeyError: If section not found in library
        """
        return self._sb_adapter.get_section(designation)

    def get_grillex_section_params(
        self,
        designation: str,
        use_cache: bool = True
    ) -> Dict[str, Any]:
        """Get section parameters ready for grillex add_section().

        Returns a dictionary with all parameters needed to create a grillex
        Section, with proper unit conversion and axis mapping applied.

        Args:
            designation: Section designation (e.g., "HEB300", "W14X22")
            use_cache: Whether to use cached results (default True)

        Returns:
            Dict with keys:
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
            KeyError: If section not found in library
            ValueError: If required properties are missing
        """
        # Check cache first
        if use_cache and designation in self._cache:
            return self._cache[designation].copy()

        # Get raw properties from sectionbuilder
        sb_props = self.get_section_properties(designation)

        # Convert to grillex units and conventions
        grillex_params = convert_section_properties(
            sb_props,
            input_units=self._input_units
        )

        # Validate required properties
        if grillex_params["A"] <= 0:
            raise ValueError(
                f"Section '{designation}' has invalid area: {grillex_params['A']}"
            )

        # Cache the result
        if use_cache:
            self._cache[designation] = grillex_params.copy()

        return grillex_params

    def search(self, query: str) -> List[SectionInfo]:
        """Search underlying adapter for sections matching query.

        Args:
            query: Search string (e.g., "HEB", "W14")

        Returns:
            List of SectionInfo objects for matching sections
        """
        if hasattr(self._sb_adapter, "search"):
            results = self._sb_adapter.search(query)
            return [
                SectionInfo(
                    designation=r.designation if hasattr(r, "designation") else str(r),
                    description=getattr(r, "description", ""),
                    library_name=self.adapter_name
                )
                for r in results
            ]
        elif hasattr(self._sb_adapter, "list_sections"):
            # Fallback: filter section list by query
            all_sections = self._sb_adapter.list_sections()
            query_lower = query.lower()
            return [
                SectionInfo(designation=s, library_name=self.adapter_name)
                for s in all_sections
                if query_lower in s.lower()
            ]
        else:
            return []

    def list_sections(self) -> List[str]:
        """List all available sections in the library.

        Returns:
            List of section designations
        """
        if hasattr(self._sb_adapter, "list_sections"):
            return self._sb_adapter.list_sections()
        elif hasattr(self._sb_adapter, "sections"):
            return list(self._sb_adapter.sections.keys())
        else:
            return []

    def clear_cache(self) -> None:
        """Clear the property cache."""
        self._cache.clear()

    def __contains__(self, designation: str) -> bool:
        """Check if a section exists in the library."""
        try:
            self.get_section_properties(designation)
            return True
        except (KeyError, ValueError):
            return False


def create_adapter_from_json(
    json_path: str,
    input_units: str = "mm"
) -> GrillexSectionAdapter:
    """Create a GrillexSectionAdapter from a JSON section library file.

    Args:
        json_path: Path to JSON library file
        input_units: Unit system of the JSON data ("mm" or "m")

    Returns:
        GrillexSectionAdapter wrapping the JSON library

    Raises:
        ImportError: If sectionbuilder is not installed
        FileNotFoundError: If JSON file doesn't exist
    """
    try:
        from sectionbuilder.adapters import JsonLibraryAdapter
    except ImportError as e:
        raise ImportError(
            "sectionbuilder package is required. "
            "Install it with: pip install sectionbuilder"
        ) from e

    json_adapter = JsonLibraryAdapter(json_path)
    return GrillexSectionAdapter(json_adapter, input_units=input_units)
