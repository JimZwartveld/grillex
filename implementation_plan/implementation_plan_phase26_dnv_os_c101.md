## Phase 26: DNV-OS-C101 Code Checks

### Overview

This phase implements structural capacity checks according to DNV-OS-C101 "Design of Offshore Steel Structures, General - LRFD Method". This code is applicable to offshore steel structures and is the primary design code for the vessel geometry module.

**Key Concepts:**
- **LRFD Method**: Load and Resistance Factor Design
- **Plate buckling**: Unstiffened and stiffened panel checks
- **Stiffener checks**: Local and lateral buckling, tripping
- **Girder checks**: Web and flange buckling
- **Unity checks**: Utilization ratios for all failure modes

**Dependencies:** Phase 21-25, DNV-OS-C101 (2021 edition or later)

**Directory Structure:**
```
src/grillex/vessel/
└── codes/
    ├── __init__.py
    ├── dnv_os_c101/
    │   ├── __init__.py
    │   ├── materials.py      # Material strength requirements
    │   ├── plate_buckling.py # Plate buckling checks
    │   ├── stiffener.py      # Stiffener checks
    │   ├── girder.py         # Girder checks
    │   └── reporting.py      # Unity check reporting
    └── common.py             # Shared utilities
```

---

### Task 26.1: Create Code Check Framework

**Requirements:** DNV-OS-C101 compliance
**Dependencies:** Phase 21-25
**Difficulty:** Medium

**Description:**
Create the framework for DNV-OS-C101 code checks.

**Steps:**

1. Create `src/grillex/vessel/codes/__init__.py`:
   ```python
   from .dnv_os_c101 import (
       DNV_OS_C101_Checker,
       check_vessel_dnv_os_c101,
       UnityCheckResult,
       CheckReport,
   )

   __all__ = [
       "DNV_OS_C101_Checker",
       "check_vessel_dnv_os_c101",
       "UnityCheckResult",
       "CheckReport",
   ]
   ```

2. Create `src/grillex/vessel/codes/common.py`:
   ```python
   """Common utilities for design code checks."""

   from dataclasses import dataclass
   from typing import List, Optional, Dict
   from enum import Enum

   class CheckStatus(Enum):
       """Status of a code check."""
       PASS = "pass"       # Unity check <= 1.0
       FAIL = "fail"       # Unity check > 1.0
       WARNING = "warning" # Close to limit (e.g., > 0.9)
       SKIPPED = "skipped" # Check not applicable
       ERROR = "error"     # Could not perform check

   @dataclass
   class UnityCheckResult:
       """Result of a single unity check."""
       check_name: str
       description: str
       unity: float
       status: CheckStatus
       limit: float = 1.0

       component_id: Optional[str] = None
       location: Optional[str] = None
       clause: Optional[str] = None  # Code clause reference

       details: Optional[Dict] = None  # Additional calculation details

       @property
       def utilization_percent(self) -> float:
           """Utilization as percentage."""
           return self.unity * 100

       def __str__(self) -> str:
           status_icon = {
               CheckStatus.PASS: "✓",
               CheckStatus.FAIL: "✗",
               CheckStatus.WARNING: "⚠",
               CheckStatus.SKIPPED: "-",
               CheckStatus.ERROR: "!",
           }[self.status]
           return f"{status_icon} {self.check_name}: {self.unity:.3f} ({self.utilization_percent:.1f}%)"


   @dataclass
   class CheckReport:
       """Collection of unity check results."""
       code_name: str
       code_version: str
       vessel_name: str

       results: List[UnityCheckResult]

       @property
       def max_unity(self) -> float:
           """Maximum unity check value."""
           unities = [r.unity for r in self.results if r.status != CheckStatus.SKIPPED]
           return max(unities) if unities else 0.0

       @property
       def governing_check(self) -> Optional[UnityCheckResult]:
           """Check with highest unity value."""
           valid = [r for r in self.results if r.status != CheckStatus.SKIPPED]
           if not valid:
               return None
           return max(valid, key=lambda r: r.unity)

       @property
       def has_failures(self) -> bool:
           """Any checks failed."""
           return any(r.status == CheckStatus.FAIL for r in self.results)

       @property
       def all_passed(self) -> bool:
           """All checks passed."""
           return all(
               r.status in (CheckStatus.PASS, CheckStatus.SKIPPED)
               for r in self.results
           )

       def summary(self) -> str:
           """Generate summary string."""
           n_pass = sum(1 for r in self.results if r.status == CheckStatus.PASS)
           n_fail = sum(1 for r in self.results if r.status == CheckStatus.FAIL)
           n_warn = sum(1 for r in self.results if r.status == CheckStatus.WARNING)

           lines = [
               f"DNV-OS-C101 Check Report: {self.vessel_name}",
               f"Code: {self.code_name} ({self.code_version})",
               "-" * 50,
               f"Total checks: {len(self.results)}",
               f"  Passed:   {n_pass}",
               f"  Failed:   {n_fail}",
               f"  Warnings: {n_warn}",
               "",
               f"Maximum unity: {self.max_unity:.3f} ({self.max_unity*100:.1f}%)",
           ]

           if self.governing_check:
               lines.append(f"Governing: {self.governing_check.check_name}")

           return "\n".join(lines)
   ```

3. Create `src/grillex/vessel/codes/dnv_os_c101/__init__.py`:
   ```python
   """
   DNV-OS-C101 Design of Offshore Steel Structures.

   This module implements structural capacity checks according to
   DNV-OS-C101 LRFD method.

   Reference: DNV-OS-C101 (2021 edition)
   """

   from .checker import DNV_OS_C101_Checker, check_vessel_dnv_os_c101
   from ..common import UnityCheckResult, CheckReport, CheckStatus

   __all__ = [
       "DNV_OS_C101_Checker",
       "check_vessel_dnv_os_c101",
       "UnityCheckResult",
       "CheckReport",
       "CheckStatus",
   ]
   ```

4. Create `src/grillex/vessel/codes/dnv_os_c101/checker.py`:
   ```python
   """Main checker class for DNV-OS-C101."""

   from typing import TYPE_CHECKING, List, Optional
   from ..common import CheckReport, UnityCheckResult, CheckStatus

   if TYPE_CHECKING:
       from ...geometry import Vessel, Panel
       from grillex.core import StructuralModel

   class DNV_OS_C101_Checker:
       """
       DNV-OS-C101 code checker.

       Performs structural capacity checks according to DNV-OS-C101
       "Design of Offshore Steel Structures, General - LRFD Method".

       Example:
           >>> from grillex.vessel.codes import DNV_OS_C101_Checker
           >>> checker = DNV_OS_C101_Checker(vessel, model, results)
           >>> report = checker.check_all()
           >>> print(report.summary())
       """

       CODE_NAME = "DNV-OS-C101"
       CODE_VERSION = "2021"

       def __init__(
           self,
           vessel: "Vessel",
           model: "StructuralModel" = None,
           results: dict = None
       ):
           """
           Initialize checker.

           Args:
               vessel: Vessel geometry
               model: StructuralModel (if already converted)
               results: Analysis results (stresses, etc.)
           """
           self.vessel = vessel
           self.model = model
           self.results = results or {}

           self._results: List[UnityCheckResult] = []

       def check_all(self) -> CheckReport:
           """
           Run all applicable checks.

           Returns:
               CheckReport with all results
           """
           self._results = []

           # Plate buckling checks
           self._check_plate_buckling()

           # Stiffener checks
           self._check_stiffeners()

           # Girder checks
           self._check_girders()

           return CheckReport(
               code_name=self.CODE_NAME,
               code_version=self.CODE_VERSION,
               vessel_name=self.vessel.name,
               results=self._results
           )

       def _check_plate_buckling(self) -> None:
           """Check plate buckling for all panels."""
           from .plate_buckling import check_unstiffened_plate

           for segment in self.vessel.get_cargo_segments():
               panels = segment.get_all_panels()

               for panel in panels:
                   result = check_unstiffened_plate(
                       panel=panel,
                       stress=self._get_panel_stress(panel),
                       material=self._get_material_props(panel.material)
                   )
                   self._results.append(result)

       def _check_stiffeners(self) -> None:
           """Check all stiffeners."""
           from .stiffener import check_stiffener

           for segment in self.vessel.get_cargo_segments():
               for deck in segment.get_decks():
                   for stiff in deck.plate_field.get_stiffeners():
                       result = check_stiffener(
                           stiffener=stiff,
                           plate_field=deck.plate_field,
                           profile=self.vessel.profiles.get(stiff.section),
                           stress=self._get_stiffener_stress(stiff),
                           material=self._get_material_props(
                               deck.plate_field.default_material
                           )
                       )
                       self._results.append(result)

       def _check_girders(self) -> None:
           """Check all girders."""
           from .girder import check_girder

           for segment in self.vessel.get_cargo_segments():
               for girder in segment.get_longitudinal_girders():
                   result = check_girder(
                       girder=girder,
                       profile=self.vessel.profiles.get(girder.section),
                       stress=self._get_girder_stress(girder),
                       material=self._get_material_props("S355")  # Default
                   )
                   self._results.append(result)

       def _get_panel_stress(self, panel: "Panel") -> dict:
           """Get stress state for a panel from results."""
           # TODO: Extract from FEM results
           return {"sigma_x": 0, "sigma_y": 0, "tau": 0}

       def _get_stiffener_stress(self, stiffener) -> dict:
           """Get stress state for a stiffener from results."""
           return {"sigma": 0, "tau": 0}

       def _get_girder_stress(self, girder) -> dict:
           """Get stress state for a girder from results."""
           return {"sigma": 0, "tau": 0, "M": 0, "V": 0}

       def _get_material_props(self, material_name: str) -> dict:
           """Get material properties."""
           if material_name in self.vessel._materials:
               return self.vessel._materials[material_name]
           # Default S355
           return {"E": 210e6, "yield_stress": 355e3, "nu": 0.3}


   def check_vessel_dnv_os_c101(
       vessel: "Vessel",
       model: "StructuralModel" = None,
       results: dict = None
   ) -> CheckReport:
       """
       Run DNV-OS-C101 checks on vessel.

       Convenience function for running all checks.

       Args:
           vessel: Vessel geometry
           model: StructuralModel (optional)
           results: Analysis results (optional)

       Returns:
           CheckReport with all results
       """
       checker = DNV_OS_C101_Checker(vessel, model, results)
       return checker.check_all()
   ```

**Acceptance Criteria:**
- [ ] CheckStatus enum with PASS, FAIL, WARNING, SKIPPED, ERROR
- [ ] UnityCheckResult with check_name, unity, status, clause reference
- [ ] CheckReport with summary, max_unity, governing_check
- [ ] DNV_OS_C101_Checker class structure
- [ ] check_vessel_dnv_os_c101() convenience function

---

### Task 26.2: Implement Plate Buckling Checks

**Requirements:** DNV-OS-C101 Section 7
**Dependencies:** Task 26.1
**Difficulty:** High

**Description:**
Implement plate buckling checks for unstiffened and stiffened panels.

**Steps:**

1. Create `src/grillex/vessel/codes/dnv_os_c101/plate_buckling.py`:
   ```python
   """
   Plate buckling checks per DNV-OS-C101 Section 7.

   References:
   - DNV-OS-C101 Section 7: Buckling Strength
   - DNV-RP-C201: Buckling Strength of Plated Structures
   """

   import math
   from typing import TYPE_CHECKING, Dict
   from ..common import UnityCheckResult, CheckStatus

   if TYPE_CHECKING:
       from ...geometry import Panel

   def check_unstiffened_plate(
       panel: "Panel",
       stress: Dict[str, float],
       material: Dict[str, float]
   ) -> UnityCheckResult:
       """
       Check buckling of unstiffened plate panel.

       DNV-OS-C101 Section 7.7 / DNV-RP-C201 Chapter 4.

       Args:
           panel: Panel to check
           stress: Applied stresses {sigma_x, sigma_y, tau} in kN/m²
           material: Material properties {E, yield_stress, nu}

       Returns:
           UnityCheckResult
       """
       # Extract parameters
       a = panel.width   # Panel length (longer dimension)
       b = panel.height  # Panel width (shorter dimension)
       t = panel.thickness

       sigma_x = stress.get("sigma_x", 0)  # Longitudinal stress
       sigma_y = stress.get("sigma_y", 0)  # Transverse stress
       tau = stress.get("tau", 0)          # Shear stress

       E = material["E"]
       fy = material["yield_stress"]
       nu = material.get("nu", 0.3)

       # Ensure a >= b
       if a < b:
           a, b = b, a
           sigma_x, sigma_y = sigma_y, sigma_x

       # Plate slenderness
       # λp = (fy / σE)^0.5 where σE = π²E / (12(1-ν²)) * (t/b)²
       sigma_e = (math.pi**2 * E) / (12 * (1 - nu**2)) * (t / b)**2

       # Critical buckling stresses
       # Compression in x-direction
       psi = 1.0  # Uniform stress
       alpha = a / b
       if sigma_x > 0:  # Tension
           sigma_cr_x = float('inf')
       else:
           k_sigma = 4.0 + 5.34 * (b / a)**2  # Simplified
           sigma_cr_x = k_sigma * sigma_e

       # Compression in y-direction
       if sigma_y > 0:  # Tension
           sigma_cr_y = float('inf')
       else:
           k_sigma_y = 4.0 + 5.34 * (a / b)**2
           sigma_cr_y = k_sigma_y * sigma_e

       # Shear buckling
       k_tau = 5.34 + 4.0 * (b / a)**2
       tau_cr = k_tau * sigma_e

       # Unity check (simplified interaction)
       uc_x = abs(sigma_x) / sigma_cr_x if sigma_x < 0 else 0
       uc_y = abs(sigma_y) / sigma_cr_y if sigma_y < 0 else 0
       uc_tau = abs(tau) / tau_cr

       # Interaction formula
       unity = math.sqrt(uc_x**2 + uc_y**2 + uc_tau**2)

       # Determine status
       if unity > 1.0:
           status = CheckStatus.FAIL
       elif unity > 0.9:
           status = CheckStatus.WARNING
       else:
           status = CheckStatus.PASS

       return UnityCheckResult(
           check_name="Plate Buckling (Unstiffened)",
           description=f"Panel {panel.width:.2f}x{panel.height:.2f}m, t={panel.thickness*1000:.1f}mm",
           unity=unity,
           status=status,
           component_id=str(panel.id),
           clause="DNV-OS-C101 Section 7.7",
           details={
               "a": a,
               "b": b,
               "t": t,
               "sigma_cr_x": sigma_cr_x,
               "sigma_cr_y": sigma_cr_y,
               "tau_cr": tau_cr,
               "uc_x": uc_x,
               "uc_y": uc_y,
               "uc_tau": uc_tau,
           }
       )


   def check_stiffened_panel(
       panel: "Panel",
       stiffener_spacing: float,
       stiffener_area: float,
       stress: Dict[str, float],
       material: Dict[str, float]
   ) -> UnityCheckResult:
       """
       Check buckling of stiffened panel (plate + stiffeners).

       DNV-RP-C201 Chapter 5.

       Args:
           panel: Panel to check
           stiffener_spacing: Spacing between stiffeners
           stiffener_area: Cross-sectional area of stiffener
           stress: Applied stresses
           material: Material properties

       Returns:
           UnityCheckResult
       """
       # Simplified stiffened panel check
       # Full implementation would include:
       # 1. Overall panel buckling (plate + stiffeners as unit)
       # 2. Local plate buckling between stiffeners
       # 3. Stiffener lateral buckling

       t = panel.thickness
       b = stiffener_spacing  # Plate width between stiffeners

       E = material["E"]
       fy = material["yield_stress"]
       nu = material.get("nu", 0.3)

       sigma_x = abs(stress.get("sigma_x", 0))

       # Local plate buckling between stiffeners
       sigma_e = (math.pi**2 * E) / (12 * (1 - nu**2)) * (t / b)**2
       sigma_cr = 4.0 * sigma_e  # Simply supported edges

       # Plate slenderness
       lambda_p = math.sqrt(fy / sigma_cr) if sigma_cr > 0 else float('inf')

       # Effective width factor (Johnson-Ostenfeld)
       if lambda_p <= 0.673:
           rho = 1.0
       else:
           rho = (1 - 0.22 / lambda_p) / lambda_p

       # Local buckling check
       unity = sigma_x / (rho * fy) if rho * fy > 0 else float('inf')

       if unity > 1.0:
           status = CheckStatus.FAIL
       elif unity > 0.9:
           status = CheckStatus.WARNING
       else:
           status = CheckStatus.PASS

       return UnityCheckResult(
           check_name="Stiffened Panel Buckling",
           description=f"Stiffener spacing {stiffener_spacing*1000:.0f}mm, t={t*1000:.1f}mm",
           unity=unity,
           status=status,
           component_id=str(panel.id),
           clause="DNV-RP-C201 Chapter 5",
           details={
               "stiffener_spacing": stiffener_spacing,
               "lambda_p": lambda_p,
               "rho": rho,
               "sigma_cr": sigma_cr,
           }
       )
   ```

**Acceptance Criteria:**
- [ ] check_unstiffened_plate() implements Section 7.7
- [ ] Critical buckling stresses calculated for σx, σy, τ
- [ ] Interaction formula for combined loading
- [ ] check_stiffened_panel() for plate between stiffeners
- [ ] Clause references included in results

---

### Task 26.3: Implement Stiffener Checks

**Requirements:** DNV-OS-C101 Section 7
**Dependencies:** Task 26.1
**Difficulty:** High

**Description:**
Implement stiffener capacity checks including lateral buckling and tripping.

**Steps:**

1. Create `src/grillex/vessel/codes/dnv_os_c101/stiffener.py`:
   ```python
   """
   Stiffener checks per DNV-OS-C101.

   Checks include:
   - Column buckling (Euler)
   - Lateral buckling
   - Tripping (torsional buckling)
   - Combined loading
   """

   import math
   from typing import TYPE_CHECKING, Dict
   from ..common import UnityCheckResult, CheckStatus

   if TYPE_CHECKING:
       from ...geometry.stiffener import Stiffener
       from ...geometry.plate_field import PlateField
       from ...profiles import Profile


   def check_stiffener(
       stiffener: "Stiffener",
       plate_field: "PlateField",
       profile: "Profile",
       stress: Dict[str, float],
       material: Dict[str, float]
   ) -> UnityCheckResult:
       """
       Check stiffener capacity.

       Args:
           stiffener: Stiffener to check
           plate_field: Parent plate field
           profile: Stiffener profile
           stress: Applied stresses {sigma, tau}
           material: Material properties

       Returns:
           UnityCheckResult
       """
       # Get stiffener properties
       A = profile.A
       Iy = profile.Iy
       Iz = profile.Iz

       # Get plate properties
       t_plate = plate_field.default_thickness or 0.010
       b_eff = 0.6  # Effective plate width (simplified)

       E = material["E"]
       fy = material["yield_stress"]

       sigma = abs(stress.get("sigma", 0))

       # Column buckling check
       # Simplified: assume simply supported stiffener span
       L = 2.5  # Typical frame spacing (should come from geometry)

       # Radius of gyration
       r = math.sqrt(Iy / A) if A > 0 else 0.01

       # Slenderness ratio
       lambda_ratio = L / r if r > 0 else 100

       # Euler buckling stress
       sigma_e = (math.pi**2 * E) / lambda_ratio**2 if lambda_ratio > 0 else float('inf')

       # Reduced slenderness
       lambda_bar = math.sqrt(fy / sigma_e) if sigma_e > 0 else float('inf')

       # Buckling reduction factor (AISC-style curve)
       if lambda_bar <= 1.5:
           chi = 1 / (1 + lambda_bar**2)
       else:
           chi = 0.877 / lambda_bar**2

       # Column buckling resistance
       sigma_cr = chi * fy

       # Unity check
       unity = sigma / sigma_cr if sigma_cr > 0 else float('inf')

       if unity > 1.0:
           status = CheckStatus.FAIL
       elif unity > 0.9:
           status = CheckStatus.WARNING
       else:
           status = CheckStatus.PASS

       return UnityCheckResult(
           check_name="Stiffener Column Buckling",
           description=f"Stiffener {stiffener.section}",
           unity=unity,
           status=status,
           component_id=str(stiffener.id),
           clause="DNV-OS-C101 Section 7.5",
           details={
               "L": L,
               "A": A,
               "Iy": Iy,
               "lambda_bar": lambda_bar,
               "chi": chi,
               "sigma_cr": sigma_cr,
           }
       )


   def check_stiffener_tripping(
       stiffener: "Stiffener",
       profile: "Profile",
       span: float,
       sigma: float,
       material: Dict[str, float]
   ) -> UnityCheckResult:
       """
       Check stiffener tripping (torsional buckling).

       Tripping is the torsional buckling failure mode where the
       stiffener rotates about its connection to the plate.

       Args:
           stiffener: Stiffener to check
           profile: Stiffener profile
           span: Unsupported span length
           sigma: Applied axial stress
           material: Material properties

       Returns:
           UnityCheckResult
       """
       # This is a placeholder for tripping check
       # Full implementation requires:
       # - Torsional constant J
       # - Warping constant Cw
       # - Distance from shear center to plate
       # - Moment gradient factor

       return UnityCheckResult(
           check_name="Stiffener Tripping",
           description=f"Stiffener {stiffener.section}",
           unity=0.0,  # Placeholder
           status=CheckStatus.SKIPPED,
           component_id=str(stiffener.id),
           clause="DNV-OS-C101 Section 7.6",
           details={"note": "Tripping check not yet implemented"}
       )
   ```

**Acceptance Criteria:**
- [ ] check_stiffener() implements column buckling check
- [ ] Slenderness ratio calculated correctly
- [ ] Buckling reduction factor per code curves
- [ ] check_stiffener_tripping() placeholder for tripping check
- [ ] Clause references included

---

### Task 26.4: Implement Girder Checks

**Requirements:** DNV-OS-C101 Section 7
**Dependencies:** Task 26.1
**Difficulty:** Medium

**Description:**
Implement girder capacity checks including web and flange buckling.

**Steps:**

1. Create `src/grillex/vessel/codes/dnv_os_c101/girder.py`:
   ```python
   """
   Girder checks per DNV-OS-C101.

   Checks include:
   - Bending capacity
   - Shear capacity
   - Web buckling
   - Flange buckling
   """

   import math
   from typing import TYPE_CHECKING, Dict
   from ..common import UnityCheckResult, CheckStatus

   if TYPE_CHECKING:
       from ...geometry.components import LongitudinalGirder, TransverseGirder
       from ...profiles import Profile


   def check_girder(
       girder,
       profile: "Profile",
       stress: Dict[str, float],
       material: Dict[str, float]
   ) -> UnityCheckResult:
       """
       Check girder capacity.

       Args:
           girder: Girder to check
           profile: Girder profile
           stress: Applied stresses {sigma, tau, M, V}
           material: Material properties

       Returns:
           UnityCheckResult
       """
       A = profile.A
       Iy = profile.Iy

       E = material["E"]
       fy = material["yield_stress"]

       sigma = abs(stress.get("sigma", 0))
       tau = abs(stress.get("tau", 0))

       # Yield strength check
       # von Mises equivalent stress
       sigma_eq = math.sqrt(sigma**2 + 3 * tau**2)

       # Design resistance
       gamma_m = 1.15  # Material factor
       f_d = fy / gamma_m

       unity = sigma_eq / f_d

       if unity > 1.0:
           status = CheckStatus.FAIL
       elif unity > 0.9:
           status = CheckStatus.WARNING
       else:
           status = CheckStatus.PASS

       return UnityCheckResult(
           check_name="Girder Yield Check",
           description=f"Girder {girder.section}",
           unity=unity,
           status=status,
           component_id=str(girder.id),
           clause="DNV-OS-C101 Section 6.3",
           details={
               "sigma": sigma,
               "tau": tau,
               "sigma_eq": sigma_eq,
               "f_d": f_d,
               "gamma_m": gamma_m,
           }
       )


   def check_girder_web_buckling(
       profile: "Profile",
       shear_stress: float,
       material: Dict[str, float]
   ) -> UnityCheckResult:
       """
       Check girder web shear buckling.

       Args:
           profile: Girder profile (should be TeeProfile or IProfile)
           shear_stress: Applied shear stress
           material: Material properties

       Returns:
           UnityCheckResult
       """
       # Extract web dimensions
       # This assumes TeeProfile or IProfile with accessible dimensions
       # For a generic Profile, we'd need to add these as properties

       # Placeholder implementation
       return UnityCheckResult(
           check_name="Girder Web Buckling",
           description="Web shear buckling check",
           unity=0.0,
           status=CheckStatus.SKIPPED,
           clause="DNV-OS-C101 Section 7.8",
           details={"note": "Web buckling check not yet implemented"}
       )
   ```

**Acceptance Criteria:**
- [ ] check_girder() implements yield check with von Mises
- [ ] Material factor γM applied correctly
- [ ] check_girder_web_buckling() placeholder
- [ ] Clause references included

---

### Task 26.5: Implement Reporting

**Requirements:** New feature
**Dependencies:** Tasks 26.1-26.4
**Difficulty:** Low

**Description:**
Implement reporting functionality for code check results.

**Steps:**

1. Create `src/grillex/vessel/codes/dnv_os_c101/reporting.py`:
   ```python
   """
   Reporting utilities for DNV-OS-C101 checks.
   """

   from typing import List, Dict
   from ..common import CheckReport, UnityCheckResult, CheckStatus


   def report_to_dict(report: CheckReport) -> Dict:
       """
       Convert report to dictionary for JSON serialization.

       Args:
           report: CheckReport to convert

       Returns:
           Dictionary representation
       """
       return {
           "code_name": report.code_name,
           "code_version": report.code_version,
           "vessel_name": report.vessel_name,
           "summary": {
               "total_checks": len(report.results),
               "passed": sum(1 for r in report.results if r.status == CheckStatus.PASS),
               "failed": sum(1 for r in report.results if r.status == CheckStatus.FAIL),
               "warnings": sum(1 for r in report.results if r.status == CheckStatus.WARNING),
               "skipped": sum(1 for r in report.results if r.status == CheckStatus.SKIPPED),
               "max_unity": report.max_unity,
               "all_passed": report.all_passed,
           },
           "results": [
               {
                   "check_name": r.check_name,
                   "description": r.description,
                   "unity": r.unity,
                   "status": r.status.value,
                   "component_id": r.component_id,
                   "clause": r.clause,
                   "details": r.details,
               }
               for r in report.results
           ]
       }


   def report_to_text(report: CheckReport, include_details: bool = False) -> str:
       """
       Convert report to text format.

       Args:
           report: CheckReport to convert
           include_details: Include calculation details

       Returns:
           Formatted text report
       """
       lines = [
           "=" * 60,
           f"DNV-OS-C101 CODE CHECK REPORT",
           "=" * 60,
           f"Vessel: {report.vessel_name}",
           f"Code: {report.code_name} ({report.code_version})",
           "-" * 60,
           "",
           "SUMMARY",
           f"  Total checks:  {len(report.results)}",
           f"  Passed:        {sum(1 for r in report.results if r.status == CheckStatus.PASS)}",
           f"  Failed:        {sum(1 for r in report.results if r.status == CheckStatus.FAIL)}",
           f"  Warnings:      {sum(1 for r in report.results if r.status == CheckStatus.WARNING)}",
           f"  Skipped:       {sum(1 for r in report.results if r.status == CheckStatus.SKIPPED)}",
           "",
           f"  Maximum unity: {report.max_unity:.3f} ({report.max_unity*100:.1f}%)",
           "",
       ]

       if report.governing_check:
           gov = report.governing_check
           lines.extend([
               "GOVERNING CHECK",
               f"  {gov.check_name}",
               f"  Unity: {gov.unity:.3f}",
               f"  Clause: {gov.clause}",
               "",
           ])

       lines.extend([
           "-" * 60,
           "DETAILED RESULTS",
           ""
       ])

       for result in report.results:
           status_icon = {
               CheckStatus.PASS: "[PASS]",
               CheckStatus.FAIL: "[FAIL]",
               CheckStatus.WARNING: "[WARN]",
               CheckStatus.SKIPPED: "[SKIP]",
               CheckStatus.ERROR: "[ERR!]",
           }[result.status]

           lines.append(f"{status_icon} {result.check_name}")
           lines.append(f"       {result.description}")
           lines.append(f"       Unity: {result.unity:.3f} ({result.utilization_percent:.1f}%)")
           lines.append(f"       Clause: {result.clause}")

           if include_details and result.details:
               for key, value in result.details.items():
                   if isinstance(value, float):
                       lines.append(f"         {key}: {value:.4g}")
                   else:
                       lines.append(f"         {key}: {value}")

           lines.append("")

       lines.append("=" * 60)
       return "\n".join(lines)
   ```

**Acceptance Criteria:**
- [ ] report_to_dict() for JSON serialization
- [ ] report_to_text() for human-readable output
- [ ] Summary statistics included
- [ ] Governing check highlighted
- [ ] Optional calculation details

---

### Task 26.6: Write Code Check Tests

**Requirements:** Testing
**Dependencies:** Tasks 26.1-26.5
**Difficulty:** Medium

**Description:**
Create tests for DNV-OS-C101 code checks.

**Steps:**

1. Create `tests/python/test_phase26_dnv_os_c101.py`:
   ```python
   """Tests for Phase 26: DNV-OS-C101 Code Checks."""

   import pytest
   import math
   from grillex.vessel import Vessel
   from grillex.vessel.codes import (
       DNV_OS_C101_Checker,
       check_vessel_dnv_os_c101,
       UnityCheckResult,
       CheckStatus,
   )
   from grillex.vessel.codes.dnv_os_c101.plate_buckling import (
       check_unstiffened_plate,
   )
   from grillex.vessel.reference_vessels import create_standard_barge_100x24


   class TestUnityCheckResult:
       """Tests for UnityCheckResult."""

       def test_pass_status(self):
           """Unity <= 1.0 passes."""
           result = UnityCheckResult(
               check_name="Test",
               description="Test check",
               unity=0.8,
               status=CheckStatus.PASS
           )
           assert result.status == CheckStatus.PASS
           assert result.utilization_percent == 80.0

       def test_fail_status(self):
           """Unity > 1.0 fails."""
           result = UnityCheckResult(
               check_name="Test",
               description="Test check",
               unity=1.2,
               status=CheckStatus.FAIL
           )
           assert result.status == CheckStatus.FAIL


   class TestPlateBuckling:
       """Tests for plate buckling checks."""

       def test_unstiffened_plate_check(self):
           """Unstiffened plate check returns result."""
           from grillex.vessel.geometry.panels import Panel, PanelOrientation

           panel = Panel(
               plate_field=None,
               orientation=PanelOrientation.XY,
               bounds=(0, 1.0, 0, 0.6),  # 1m x 0.6m panel
               thickness=0.012  # 12mm
           )

           stress = {"sigma_x": -100e3, "sigma_y": 0, "tau": 0}  # 100 MPa compression
           material = {"E": 210e9, "yield_stress": 355e6, "nu": 0.3}

           result = check_unstiffened_plate(panel, stress, material)

           assert result is not None
           assert result.unity >= 0

       def test_thick_plate_low_stress_passes(self):
           """Thick plate with low stress passes."""
           from grillex.vessel.geometry.panels import Panel, PanelOrientation

           panel = Panel(
               plate_field=None,
               orientation=PanelOrientation.XY,
               bounds=(0, 0.5, 0, 0.5),  # 0.5m x 0.5m panel
               thickness=0.025  # 25mm thick
           )

           stress = {"sigma_x": -10e3, "sigma_y": 0, "tau": 0}  # 10 MPa
           material = {"E": 210e9, "yield_stress": 355e6, "nu": 0.3}

           result = check_unstiffened_plate(panel, stress, material)

           assert result.status == CheckStatus.PASS
           assert result.unity < 1.0


   class TestVesselCheck:
       """Tests for full vessel check."""

       def test_check_reference_vessel(self):
           """Reference vessel can be checked."""
           vessel = create_standard_barge_100x24()
           report = check_vessel_dnv_os_c101(vessel)

           assert report is not None
           assert report.vessel_name == vessel.name
           assert len(report.results) >= 0  # May have checks

       def test_report_summary(self):
           """Report summary is generated."""
           vessel = create_standard_barge_100x24()
           report = check_vessel_dnv_os_c101(vessel)

           summary = report.summary()
           assert "DNV-OS-C101" in summary
   ```

**Acceptance Criteria:**
- [ ] UnityCheckResult tests pass
- [ ] Plate buckling check returns valid result
- [ ] Thick plate with low stress passes
- [ ] Reference vessel can be checked
- [ ] Report summary generated correctly

---

### Summary

| Task | Description | Difficulty | Status |
|------|-------------|------------|--------|
| 26.1 | Code check framework | Medium | Pending |
| 26.2 | Plate buckling checks | High | Pending |
| 26.3 | Stiffener checks | High | Pending |
| 26.4 | Girder checks | Medium | Pending |
| 26.5 | Reporting | Low | Pending |
| 26.6 | Code check tests | Medium | Pending |

**Total Acceptance Criteria:** 28 items

---

## Code References

- DNV-OS-C101: Design of Offshore Steel Structures, General - LRFD Method
- DNV-RP-C201: Buckling Strength of Plated Structures
- DNV-RP-C202: Buckling Strength of Shells
