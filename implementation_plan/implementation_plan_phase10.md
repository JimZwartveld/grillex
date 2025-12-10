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
- [ ] Base classes are defined
- [ ] Plugin structure allows multiple codes
- [ ] Check results include all required info

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

