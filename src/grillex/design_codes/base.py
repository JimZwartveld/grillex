"""Base classes for design code plugin architecture.

This module provides the abstract base classes for implementing design code
checks in a pluggable manner. Design codes like Eurocode, DNV, AISC, etc.
can be implemented as separate modules that inherit from these base classes.

Example usage:
    >>> from grillex.design_codes.base import DesignCode, DesignCheck
    >>> class MyAxialCheck(DesignCheck):
    ...     @property
    ...     def name(self) -> str:
    ...         return "Axial Capacity"
    ...     def compute_utilization(self, actions, section, material, **kwargs):
    ...         N = abs(actions.get('N', 0.0))
    ...         return N / (section.A * material.fy)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, List, Optional


@dataclass
class CheckResult:
    """Result of a single design check on an element.

    Attributes:
        element_id: ID of the element that was checked.
        location: Position along element (0.0 = start, 1.0 = end).
        check_name: Name of the design check performed.
        utilization: Utilization ratio (demand/capacity). Values > 1.0 indicate failure.
        load_combination: Name of the load combination used.
        governing: Whether this is the governing (maximum) utilization for the element.
        status: "PASS" if utilization <= 1.0, "FAIL" otherwise.
        details: Optional dictionary with additional check-specific information.

    Example:
        >>> result = CheckResult(
        ...     element_id=1,
        ...     location=0.5,
        ...     check_name="Axial Buckling",
        ...     utilization=0.85,
        ...     load_combination="ULS1"
        ... )
        >>> result.status
        'PASS'
    """

    element_id: int
    location: float
    check_name: str
    utilization: float
    load_combination: str
    governing: bool = False
    details: Optional[dict] = field(default_factory=dict)

    @property
    def status(self) -> str:
        """Return 'PASS' if utilization <= 1.0, 'FAIL' otherwise."""
        return "PASS" if self.utilization <= 1.0 else "FAIL"

    def __repr__(self) -> str:
        gov_str = " [GOVERNING]" if self.governing else ""
        return (
            f"CheckResult({self.check_name}: {self.utilization:.3f} "
            f"[{self.status}]{gov_str} @ elem {self.element_id}, loc {self.location:.2f})"
        )


class DesignCheck(ABC):
    """Abstract base class for a single design check.

    A DesignCheck represents one specific check within a design code,
    such as axial capacity, bending capacity, buckling, etc.

    Subclasses must implement:
        - name: Property returning the check name
        - compute_utilization: Method computing demand/capacity ratio

    Example:
        >>> class AxialCheck(DesignCheck):
        ...     @property
        ...     def name(self) -> str:
        ...         return "Axial Capacity"
        ...
        ...     def compute_utilization(self, actions, section, material, **kwargs):
        ...         N = abs(actions.get('N', 0.0))
        ...         N_Rd = section.A * material.fy / kwargs.get('gamma_M0', 1.0)
        ...         return N / N_Rd if N_Rd > 0 else 0.0
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Name of the check (e.g., 'Axial Buckling', 'Bending Capacity').

        Returns:
            Human-readable name identifying this check.
        """
        pass

    @abstractmethod
    def compute_utilization(
        self,
        actions: dict,
        section: Any,
        material: Any,
        **kwargs: Any,
    ) -> float:
        """Compute utilization ratio (demand/capacity).

        Args:
            actions: Dictionary of internal actions at the check location.
                Expected keys: 'N' (axial), 'Vy', 'Vz' (shear),
                'Mx' (torsion), 'My', 'Mz' (bending).
            section: Section object with geometric properties (A, Iy, Iz, J, etc.).
            material: Material object with strength properties (fy, fu, E, etc.).
            **kwargs: Additional parameters (safety factors, buckling lengths, etc.).

        Returns:
            Utilization ratio. Values > 1.0 indicate the check has failed.
        """
        pass

    def check(
        self,
        actions: dict,
        section: Any,
        material: Any,
        element_id: int,
        location: float,
        load_combination: str,
        **kwargs: Any,
    ) -> CheckResult:
        """Perform the check and return a CheckResult.

        This is a convenience method that wraps compute_utilization.

        Args:
            actions: Dictionary of internal actions.
            section: Section object.
            material: Material object.
            element_id: ID of the element being checked.
            location: Position along element (0.0-1.0).
            load_combination: Name of the load combination.
            **kwargs: Additional parameters passed to compute_utilization.

        Returns:
            CheckResult with the computed utilization.
        """
        utilization = self.compute_utilization(actions, section, material, **kwargs)
        return CheckResult(
            element_id=element_id,
            location=location,
            check_name=self.name,
            utilization=utilization,
            load_combination=load_combination,
        )


class DesignCode(ABC):
    """Abstract base class for a design code module.

    A DesignCode represents a complete design standard (e.g., Eurocode 3,
    DNV-RP-C201, AISC 360) and contains multiple DesignChecks.

    Subclasses must implement:
        - name: Property returning the code name
        - get_checks: Method returning list of applicable checks
        - check_beam: Method performing all checks on a beam

    Example:
        >>> class MyDesignCode(DesignCode):
        ...     @property
        ...     def name(self) -> str:
        ...         return "My Standard v1.0"
        ...
        ...     def get_checks(self) -> List[DesignCheck]:
        ...         return [AxialCheck(), BendingCheck()]
        ...
        ...     def check_beam(self, beam, result_case, combination) -> List[CheckResult]:
        ...         results = []
        ...         for check in self.get_checks():
        ...             # Perform check at multiple locations
        ...             ...
        ...         return results
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Name of the design code (e.g., 'DNV-RP-C201', 'EN 1993-1-1').

        Returns:
            Official name or reference for this design code.
        """
        pass

    @property
    def version(self) -> str:
        """Version or edition of the design code.

        Override this property to specify a particular edition.

        Returns:
            Version string (default: "1.0").
        """
        return "1.0"

    @abstractmethod
    def get_checks(self) -> List[DesignCheck]:
        """Return list of design checks applicable under this code.

        Returns:
            List of DesignCheck instances that this code will apply.
        """
        pass

    @abstractmethod
    def check_beam(
        self,
        beam: Any,
        result_case: Any,
        combination: Any,
    ) -> List[CheckResult]:
        """Perform all design checks on a beam member.

        This method should:
        1. Get internal actions at critical locations along the beam
        2. Apply all relevant checks from get_checks()
        3. Mark the governing result

        Args:
            beam: Beam element to check (provides section, material, geometry).
            result_case: Analysis results for the beam (displacements, actions).
            combination: Load combination being checked (provides factors, name).

        Returns:
            List of CheckResult objects, one per check per location.
        """
        pass

    def check_all_beams(
        self,
        beams: List[Any],
        result_case: Any,
        combination: Any,
    ) -> List[CheckResult]:
        """Perform all checks on multiple beams.

        This is a convenience method that calls check_beam for each beam
        and aggregates the results.

        Args:
            beams: List of beam elements to check.
            result_case: Analysis results.
            combination: Load combination being checked.

        Returns:
            List of all CheckResult objects from all beams.
        """
        all_results = []
        for beam in beams:
            results = self.check_beam(beam, result_case, combination)
            all_results.extend(results)
        return all_results

    def get_governing_results(
        self,
        results: List[CheckResult],
    ) -> List[CheckResult]:
        """Identify and mark governing results per element.

        Finds the maximum utilization for each element and marks it as governing.

        Args:
            results: List of CheckResult objects.

        Returns:
            The same list with governing flags updated.
        """
        # Group by element_id
        by_element: dict[int, List[CheckResult]] = {}
        for result in results:
            if result.element_id not in by_element:
                by_element[result.element_id] = []
            by_element[result.element_id].append(result)

        # Find max utilization per element
        for element_results in by_element.values():
            if not element_results:
                continue
            max_result = max(element_results, key=lambda r: r.utilization)
            max_result.governing = True

        return results

    def get_summary(self, results: List[CheckResult]) -> dict:
        """Generate a summary of check results.

        Args:
            results: List of CheckResult objects.

        Returns:
            Dictionary with summary statistics.
        """
        if not results:
            return {
                "total_checks": 0,
                "passed": 0,
                "failed": 0,
                "max_utilization": 0.0,
                "governing_element": None,
                "governing_check": None,
            }

        passed = sum(1 for r in results if r.status == "PASS")
        failed = len(results) - passed
        max_result = max(results, key=lambda r: r.utilization)

        return {
            "total_checks": len(results),
            "passed": passed,
            "failed": failed,
            "max_utilization": max_result.utilization,
            "governing_element": max_result.element_id,
            "governing_check": max_result.check_name,
        }
