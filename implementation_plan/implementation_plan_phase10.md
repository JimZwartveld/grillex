## Phase 10: Design Codes (Plugin Structure)

### Task 10.1: Create Design Code Plugin Architecture
**Requirements:** R-CODE-001, R-CODE-002
**Dependencies:** Task 7.2
**Difficulty:** Medium

**Description:**
Create the pluggable design code module structure.

**Steps:**
1. Create `grillex/design_codes/base.py`:
   ```python
   from abc import ABC, abstractmethod
   from typing import Any

   class DesignCheck(ABC):
       """Base class for a single design check."""

       @property
       @abstractmethod
       def name(self) -> str:
           """Name of the check (e.g., 'Axial Buckling')."""
           pass

       @abstractmethod
       def compute_utilization(self, actions: dict, section: Any,
                               material: Any, **kwargs) -> float:
           """Compute utilization ratio (demand/capacity)."""
           pass

   class DesignCode(ABC):
       """Base class for a design code module."""

       @property
       @abstractmethod
       def name(self) -> str:
           """Name of the design code (e.g., 'DNV-RP-C201')."""
           pass

       @abstractmethod
       def get_checks(self) -> list[DesignCheck]:
           """Return list of applicable design checks."""
           pass

       @abstractmethod
       def check_beam(self, beam: Any, result_case: Any,
                      combination: Any) -> "CheckResult":
           """Perform all checks on a beam member."""
           pass
   ```

2. Create result structure:
   ```python
   @dataclass
   class CheckResult:
       element_id: int
       location: float  # Position along element
       check_name: str
       utilization: float
       load_combination: str
       governing: bool = False
   ```

**Acceptance Criteria:**
- [x] Base classes are defined
- [x] Plugin structure allows multiple codes
- [x] Check results include all required info

### Execution Notes (Completed 2025-12-20)

**Steps Taken:**
1. Created `src/grillex/design_codes/base.py` with:
   - `CheckResult` dataclass for design check results
   - `DesignCheck` abstract base class for individual checks
   - `DesignCode` abstract base class for complete design standards
2. Updated `src/grillex/design_codes/__init__.py` with exports
3. Created comprehensive test suite in `tests/python/test_phase10_design_codes.py`

**Implementation Details:**
- `CheckResult`: Contains element_id, location, check_name, utilization, load_combination, governing flag, and optional details dict. Has `status` property that returns "PASS"/"FAIL" based on utilization <= 1.0
- `DesignCheck`: Abstract base with `name` property and `compute_utilization()` method. Includes convenience `check()` method that returns a CheckResult
- `DesignCode`: Abstract base with `name`, `version` properties and `get_checks()`, `check_beam()` methods. Includes helper methods `check_all_beams()`, `get_governing_results()`, `get_summary()`

**Problems Encountered:**
- None

**Verification:**
- 24 unit tests passing ✓
- Full test suite: 667 tests passing ✓
- Abstract classes correctly prevent direct instantiation
- Concrete implementations work as expected
- Multiple design codes can coexist independently

---

### Task 10.2: Implement Example Design Code (Eurocode)
**Requirements:** R-CODE-002, R-CODE-003, R-CODE-004
**Dependencies:** Task 10.1
**Difficulty:** High

**Description:**
Implement a sample design code module.

**Steps:**
1. Create `grillex/design_codes/eurocode3.py`:
   ```python
   class EC3AxialCheck(DesignCheck):
       name = "EC3 Axial"

       def compute_utilization(self, actions, section, material, **kwargs):
           N_Ed = abs(actions['N'])
           A = section.A
           fy = material.fy
           N_Rd = A * fy / 1.0  # gamma_M0 = 1.0
           return N_Ed / N_Rd

   class EC3BendingCheck(DesignCheck):
       # Similar for bending
       pass

   class Eurocode3(DesignCode):
       name = "Eurocode 3 (EN 1993-1-1)"

       def get_checks(self):
           return [EC3AxialCheck(), EC3BendingCheck(), ...]
   ```

2. This is an illustrative example; full Eurocode implementation is extensive

**Acceptance Criteria:**
- [x] At least basic checks are implemented
- [x] Utilization is computed correctly
- [x] Governing check is identified

### Execution Notes (Completed 2025-12-20)

**Steps Taken:**
1. Created `src/grillex/design_codes/eurocode3.py` with:
   - `EC3AxialCheck`: Cross-section axial resistance (Clause 6.2.4)
   - `EC3BendingYCheck`: Bending about y-axis (Clause 6.2.5)
   - `EC3BendingZCheck`: Bending about z-axis (Clause 6.2.5)
   - `EC3ShearYCheck`: Shear in y-direction (Clause 6.2.6)
   - `EC3ShearZCheck`: Shear in z-direction (Clause 6.2.6)
   - `EC3CombinedCheck`: Combined axial + bending interaction (Clause 6.2.1)
   - `Eurocode3`: Main design code class with configurable safety factors
2. Updated `__init__.py` to export all Eurocode 3 classes
3. Added 24 new tests for Eurocode 3 implementation

**Implementation Details:**
- Uses simplified linear interaction for combined checks: N/N_Rd + My/M_y,Rd + Mz/M_z,Rd
- Supports configurable safety factors (gamma_M0, gamma_M1, gamma_M2)
- Supports custom check locations along beam
- Plastic shear resistance using tau_y = fy/sqrt(3)
- Default check locations: [0.0, 0.25, 0.5, 0.75, 1.0]

**Problems Encountered:**
- None

**Verification:**
- 48 unit tests passing (24 new for Eurocode 3) ✓
- Full test suite: 691 tests passing ✓
- Utilization computed correctly (verified against hand calculations)
- Governing check identified per element
- Both passing and failing checks detected correctly

**Limitations (addressed in Tasks 10.3-10.11):**
- Section classification not implemented (assumes Class 1/2) → Task 10.5
- No buckling checks (flexural, lateral-torsional) → Tasks 10.6, 10.7, 10.8
- No connection design
- Simplified shear area calculation (60% of gross area)

---

### Task 10.3: BeamDesignSettings Class
**Requirements:** R-CODE-005
**Dependencies:** Task 10.2
**Difficulty:** Medium

**Description:**
Create a general-purpose settings class for beam design parameters that are not specific to any design code (buckling lengths, effective length factors, lateral restraints).

**Steps:**
1. Create `src/grillex/design_codes/beam_settings.py`:
   - `MomentDiagramType` enum for standard moment shapes
   - `BeamDesignSettings` dataclass with:
     - `L_cr_y`, `L_cr_z`: Absolute buckling lengths
     - `k_y`, `k_z`: Effective length factors
     - `L_LT`: Unbraced length for LTB
     - `lateral_restraints`: List of restraint positions
     - `C1`, `C2`, `C3`: Moment gradient factors
     - `moment_diagram`: Type for automatic C factor calculation
   - Helper methods: `get_buckling_length_y()`, `get_buckling_length_z()`, `get_LT_length()`

2. Export from `__init__.py`
3. Add tests

**Acceptance Criteria:**
- [ ] BeamDesignSettings stores all buckling parameters
- [ ] Effective length computation works with factors and absolute lengths
- [ ] Moment gradient factors can be specified or computed
- [ ] Settings can be attached to Beam elements

---

### Task 10.4: EC3BeamSettings Class
**Requirements:** R-CODE-005
**Dependencies:** Task 10.3
**Difficulty:** Medium

**Description:**
Create EC3-specific beam settings that extend the general settings.

**Steps:**
1. Add to `src/grillex/design_codes/eurocode3.py`:
   - `BucklingCurve` enum (a0, a, b, c, d)
   - `SectionType` enum for automatic curve selection
   - `EC3BeamSettings` dataclass with:
     - `section_class`: Override (1-4)
     - `section_type`: For automatic buckling curve selection
     - `buckling_curve_y`, `buckling_curve_z`, `buckling_curve_LT`
     - `use_general_LTB_method`: bool
     - `ignore_LTB`: bool
   - `get_imperfection_factor()` method

2. Update exports and tests

**Acceptance Criteria:**
- [ ] EC3BeamSettings stores all EC3-specific parameters
- [ ] Buckling curves can be specified or auto-selected
- [ ] Section class can override automatic classification
- [ ] Imperfection factors computed correctly

---

### Task 10.5: Section Classification (EC3 Table 5.2)
**Requirements:** R-CODE-003
**Dependencies:** Task 10.4
**Difficulty:** High

**Description:**
Implement full section classification for I-sections according to EC3 Table 5.2.

**Key Equations:**
- ε = √(235/fy) where fy in N/mm²
- Flange limits: Class 1: c/t ≤ 9ε, Class 2: ≤ 10ε, Class 3: ≤ 14ε
- Web limits depend on stress distribution (pure bending vs compression)

**Acceptance Criteria:**
- [ ] Flange classification correct for I-sections
- [ ] Web classification correct for pure bending
- [ ] Web classification correct for combined N+M
- [ ] Section class = max(flange_class, web_class)
- [ ] Returns Class 4 when limits exceeded
- [ ] Epsilon factor computed correctly from fy

---

### Task 10.6: Flexural Buckling Check (EC3 6.3.1)
**Requirements:** R-CODE-003
**Dependencies:** Task 10.3, 10.4
**Difficulty:** High

**Description:**
Implement column buckling check for compression members.

**Key Equations:**
- λ̄ = √(A·fy/Ncr) = Lcr/(i·π) · √(fy/E)
- Φ = 0.5·[1 + α·(λ̄ - 0.2) + λ̄²]
- χ = 1 / (Φ + √(Φ² - λ̄²))
- Nb,Rd = χ·A·fy/γM1

**Acceptance Criteria:**
- [ ] Slenderness computed correctly for y and z axes
- [ ] Chi reduction factor matches tabulated values (within 1%)
- [ ] Buckling curves a0, a, b, c, d all work correctly
- [ ] Check performs for both axes, reports governing
- [ ] Class 4 effective area supported

---

### Task 10.7: Lateral-Torsional Buckling Check (EC3 6.3.2)
**Requirements:** R-CODE-003
**Dependencies:** Task 10.6
**Difficulty:** High

**Description:**
Implement LTB check for beams in bending.

**Key Equations:**
- Mcr = C1 · (π²·E·Iz/L²) · √[(Iw/Iz) + (L²·G·It)/(π²·E·Iz)]
- λ̄LT = √(Wy·fy/Mcr)
- χLT from curve with αLT and λ̄LT,0 = 0.4, β = 0.75

**Acceptance Criteria:**
- [ ] Mcr computed correctly for I-sections
- [ ] C1 factors work for standard moment diagrams
- [ ] χLT matches EC3 tables (within 1%)
- [ ] Both general and rolled section methods available
- [ ] Short beams (λ̄LT ≤ 0.4) skip LTB check (χLT = 1.0)

---

### Task 10.8: Member Buckling Interaction (EC3 6.3.3)
**Requirements:** R-CODE-003
**Dependencies:** Task 10.6, 10.7
**Difficulty:** Very High

**Description:**
Implement interaction checks for members subject to combined compression and bending.

**Key Equations (Method 2 - Annex B):**
- NEd/(χy·NRk/γM1) + kyy·My,Ed/(χLT·My,Rk/γM1) + kyz·Mz,Ed/(Mz,Rk/γM1) ≤ 1
- NEd/(χz·NRk/γM1) + kzy·My,Ed/(χLT·My,Rk/γM1) + kzz·Mz,Ed/(Mz,Rk/γM1) ≤ 1
- Interaction factors kyy, kyz, kzy, kzz from Annex B

**Acceptance Criteria:**
- [ ] Both interaction equations (y-y and z-z) checked
- [ ] Interaction factors kyy, kyz, kzy, kzz computed correctly
- [ ] Cm factors for different moment diagrams
- [ ] ΔM shift moments for Class 4 sections
- [ ] Reports governing interaction equation

---

### Task 10.9: Additional Cross-Section Checks
**Requirements:** R-CODE-003
**Dependencies:** Task 10.2
**Difficulty:** Medium

**Description:**
Complete the cross-section level checks: Torsion (6.2.7), Bending-Shear interaction (6.2.8), N+V+M interaction (6.2.10).

**Acceptance Criteria:**
- [ ] Torsion stress computed correctly
- [ ] Shear-moment interaction reduces capacity when V > 0.5Vpl
- [ ] Combined N+V+M interaction implemented
- [ ] Warping stresses included for open sections

---

### Task 10.10: Integrate Settings with Beam and Model
**Requirements:** R-CODE-005
**Dependencies:** Task 10.3, 10.4
**Difficulty:** Medium

**Description:**
Integrate the settings classes with the existing Beam and StructuralModel classes.

**Acceptance Criteria:**
- [ ] Settings can be attached to individual beams
- [ ] Default settings can be set at model level
- [ ] EC3 checks use beam settings when available
- [ ] Falls back to defaults when no settings specified

---

### Task 10.11: Comprehensive Tests for Extended EC3
**Requirements:** R-CODE-004
**Dependencies:** All above
**Difficulty:** Medium

**Description:**
Create comprehensive test suite covering all new functionality.

**Acceptance Criteria:**
- [ ] At least 50 new tests for extended EC3
- [ ] Buckling reduction factors within 1% of tabulated values
- [ ] All edge cases handled (λ̄ < 0.2, λ̄ > 3.0, etc.)
- [ ] Tests cover all buckling curves

---

