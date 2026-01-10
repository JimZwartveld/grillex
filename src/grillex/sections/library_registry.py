"""Registry for managing multiple section libraries.

This module provides a registry pattern for managing multiple section
library adapters, allowing users to register libraries by name and
search across all registered libraries.
"""

from typing import Dict, List, Optional, Union
from pathlib import Path

from .adapter import GrillexSectionAdapter, SectionInfo, create_adapter_from_json


class SectionLibraryRegistry:
    """Registry for managing multiple section libraries.

    Allows registering multiple section library adapters and provides
    unified search and access across all registered libraries.

    Attributes:
        libraries: Dictionary of registered adapters by name

    Examples:
        >>> registry = SectionLibraryRegistry()
        >>> # Register libraries  # doctest: +SKIP
        >>> registry.register("eurocode", eurocode_adapter)  # doctest: +SKIP
        >>> registry.register("aisc", aisc_adapter)  # doctest: +SKIP
        >>> # Search across all libraries  # doctest: +SKIP
        >>> results = registry.search("W14")  # doctest: +SKIP
    """

    def __init__(self):
        """Initialize an empty registry."""
        self._adapters: Dict[str, GrillexSectionAdapter] = {}

    @property
    def libraries(self) -> Dict[str, GrillexSectionAdapter]:
        """Get all registered adapters."""
        return self._adapters.copy()

    def register(
        self,
        name: str,
        adapter: GrillexSectionAdapter,
        replace: bool = False
    ) -> None:
        """Register a section library adapter.

        Args:
            name: Name to register the library under
            adapter: GrillexSectionAdapter to register
            replace: If True, replace existing library with same name

        Raises:
            ValueError: If name already registered and replace=False
            TypeError: If adapter is not a GrillexSectionAdapter
        """
        if not isinstance(adapter, GrillexSectionAdapter):
            raise TypeError(
                f"Expected GrillexSectionAdapter, got {type(adapter).__name__}"
            )

        if name in self._adapters and not replace:
            raise ValueError(
                f"Library '{name}' already registered. "
                "Use replace=True to overwrite."
            )

        self._adapters[name] = adapter

    def register_json(
        self,
        name: str,
        json_path: Union[str, Path],
        input_units: str = "mm",
        replace: bool = False
    ) -> GrillexSectionAdapter:
        """Register a section library from a JSON file.

        Args:
            name: Name to register the library under
            json_path: Path to JSON library file
            input_units: Unit system of the JSON data ("mm" or "m")
            replace: If True, replace existing library with same name

        Returns:
            The created GrillexSectionAdapter

        Raises:
            ImportError: If sectionbuilder is not installed
            FileNotFoundError: If JSON file doesn't exist
        """
        adapter = create_adapter_from_json(str(json_path), input_units)
        self.register(name, adapter, replace=replace)
        return adapter

    def unregister(self, name: str) -> Optional[GrillexSectionAdapter]:
        """Remove a library from the registry.

        Args:
            name: Name of library to remove

        Returns:
            The removed adapter, or None if not found
        """
        return self._adapters.pop(name, None)

    def get(self, name: str) -> Optional[GrillexSectionAdapter]:
        """Get a registered adapter by name.

        Args:
            name: Library name

        Returns:
            The adapter, or None if not found
        """
        return self._adapters.get(name)

    def __getitem__(self, name: str) -> GrillexSectionAdapter:
        """Get a registered adapter by name (dict-like access).

        Args:
            name: Library name

        Returns:
            The adapter

        Raises:
            KeyError: If library not found
        """
        if name not in self._adapters:
            raise KeyError(f"Library '{name}' not registered")
        return self._adapters[name]

    def __contains__(self, name: str) -> bool:
        """Check if a library is registered."""
        return name in self._adapters

    def __len__(self) -> int:
        """Return number of registered libraries."""
        return len(self._adapters)

    def list_libraries(self) -> List[str]:
        """List all registered library names."""
        return list(self._adapters.keys())

    def search(
        self,
        query: str,
        library: Optional[str] = None
    ) -> List[SectionInfo]:
        """Search for sections across registered libraries.

        Args:
            query: Search string (e.g., "HEB", "W14")
            library: Optional specific library to search (searches all if None)

        Returns:
            List of SectionInfo objects for matching sections

        Raises:
            KeyError: If specified library not found
        """
        results: List[SectionInfo] = []

        if library is not None:
            if library not in self._adapters:
                raise KeyError(f"Library '{library}' not registered")
            adapters = {library: self._adapters[library]}
        else:
            adapters = self._adapters

        for name, adapter in adapters.items():
            for info in adapter.search(query):
                info.library_name = name
                results.append(info)

        return results

    def find_section(
        self,
        designation: str,
        library: Optional[str] = None
    ) -> Optional[tuple]:
        """Find a section by designation.

        Args:
            designation: Section designation (e.g., "HEB300")
            library: Optional specific library to search

        Returns:
            Tuple of (library_name, adapter) if found, None otherwise
        """
        if library is not None:
            if library in self._adapters:
                adapter = self._adapters[library]
                if designation in adapter:
                    return (library, adapter)
            return None

        # Search all libraries
        for name, adapter in self._adapters.items():
            if designation in adapter:
                return (name, adapter)

        return None

    def get_section_params(
        self,
        designation: str,
        library: Optional[str] = None
    ) -> Dict:
        """Get section parameters for grillex.

        Args:
            designation: Section designation (e.g., "HEB300")
            library: Optional specific library to search

        Returns:
            Dict with grillex section parameters

        Raises:
            KeyError: If section not found in any library
        """
        result = self.find_section(designation, library)
        if result is None:
            if library:
                raise KeyError(
                    f"Section '{designation}' not found in library '{library}'"
                )
            else:
                raise KeyError(
                    f"Section '{designation}' not found in any registered library. "
                    f"Registered libraries: {self.list_libraries()}"
                )

        _, adapter = result
        return adapter.get_grillex_section_params(designation)

    def clear(self) -> None:
        """Remove all registered libraries."""
        self._adapters.clear()
