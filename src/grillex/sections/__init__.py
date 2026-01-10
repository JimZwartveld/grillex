"""SectionBuilder integration module for Grillex.

This module provides adapters and utilities for using section libraries
with Grillex, including:
- Unit conversion between sectionbuilder (mm) and grillex (m)
- Axis convention mapping
- Section library registry
- Extended section properties for design code checks

Example Usage:
    >>> from grillex.sections import GrillexSectionAdapter, SectionLibraryRegistry
    >>> # Create registry
    >>> registry = SectionLibraryRegistry()  # doctest: +SKIP
    >>> # Register a JSON library
    >>> registry.register_json("eurocode", "eurocode_sections.json")  # doctest: +SKIP
    >>> # Get section params for grillex
    >>> params = registry.get_section_params("HEB300")  # doctest: +SKIP
    >>> # Use in StructuralModel
    >>> model.add_section("HEB300", **params)  # doctest: +SKIP

Note:
    The sectionbuilder package is an optional dependency. If not installed,
    only dict-based conversion utilities will be available.
"""

from .conversion import (
    ScaleFactors,
    get_scale_factors,
    convert_section_properties,
    convert_section_dict,
)

from .adapter import (
    SectionInfo,
    GrillexSectionAdapter,
    create_adapter_from_json,
)

from .library_registry import (
    SectionLibraryRegistry,
)

from .properties import (
    ExtendedSectionProperties,
)

__all__ = [
    # Conversion utilities
    "ScaleFactors",
    "get_scale_factors",
    "convert_section_properties",
    "convert_section_dict",
    # Adapter
    "SectionInfo",
    "GrillexSectionAdapter",
    "create_adapter_from_json",
    # Registry
    "SectionLibraryRegistry",
    # Properties
    "ExtendedSectionProperties",
]


# Check for sectionbuilder availability
def _check_sectionbuilder_available() -> bool:
    """Check if sectionbuilder package is installed."""
    try:
        import sectionbuilder
        return True
    except ImportError:
        return False


SECTIONBUILDER_AVAILABLE = _check_sectionbuilder_available()
