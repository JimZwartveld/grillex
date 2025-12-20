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
- [ ] At least basic checks are implemented
- [ ] Utilization is computed correctly
- [ ] Governing check is identified

---

