# Phase 10 Extension: Complete Eurocode 3 Implementation

## Overview

This plan extends Phase 10 to include a complete Eurocode 3 (EN 1993-1-1) implementation with proper separation of concerns between general beam settings and code-specific parameters.

## Architecture Design

### Separation of Concerns

The design separates settings into three levels:

```
┌─────────────────────────────────────────────────────────────────┐
│                     BeamDesignSettings                          │
│  (General, code-agnostic settings attached to beam elements)    │
│  - Buckling lengths (L_cr_y, L_cr_z)                           │
│  - Effective length factors (k_y, k_z)                          │
│  - Lateral restraint positions                                  │
│  - Unbraced length for LTB (L_LT)                              │
│  - Moment gradient factors (C1, C2, C3)                         │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     EC3BeamSettings                             │
│  (EC3-specific settings, extends or wraps BeamDesignSettings)   │
│  - Section class override (1-4)                                 │
│  - Buckling curve selection (a0, a, b, c, d)                    │
│  - LTB curve selection                                          │
│  - National annex selection                                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Eurocode3 (DesignCode)                      │
│  (Code-level parameters)                                        │
│  - Partial safety factors (γ_M0, γ_M1, γ_M2)                   │
│  - Default buckling curves per section type                     │
│  - Check locations                                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## Task 10.3: BeamDesignSettings Class

**Dependencies:** Task 10.2
**Difficulty:** Medium

**Description:**
Create a general-purpose settings class for beam design parameters that are not specific to any design code.

**File:** `src/grillex/design_codes/beam_settings.py`

```python
from dataclasses import dataclass, field
from typing import Optional, List
from enum import Enum

class MomentDiagramType(Enum):
    """Standard moment diagram shapes for LTB calculation."""
    UNIFORM = "uniform"           # Constant moment
    LINEAR = "linear"             # Linear variation
    PARABOLIC = "parabolic"       # UDL-type
    CONCENTRATED = "concentrated"  # Point load at midspan
    CUSTOM = "custom"             # User-defined C factors

@dataclass
class BeamDesignSettings:
    """General beam design settings (code-agnostic).

    These settings define geometric and restraint parameters needed
    for stability checks. They are independent of any specific design code.

    Attributes:
        L_cr_y: Buckling length about y-axis (strong axis) [m].
                If None, uses element length.
        L_cr_z: Buckling length about z-axis (weak axis) [m].
                If None, uses element length.
        k_y: Effective length factor for y-axis buckling.
             Typical values: 0.5 (fixed-fixed), 0.7 (fixed-pinned),
             1.0 (pinned-pinned), 2.0 (cantilever).
        k_z: Effective length factor for z-axis buckling.
        L_LT: Unbraced length for lateral-torsional buckling [m].
              If None, uses element length.
        lateral_restraints: Positions of lateral restraints along beam
                           (normalized 0.0 to 1.0).
        moment_diagram: Type of moment diagram for C factor calculation.
        C1, C2, C3: Moment gradient factors for LTB.
                    If None, computed from moment_diagram type.
        end_moment_ratio: Ratio of end moments ψ = M_min/M_max
                         for linear moment diagrams.

    Example:
        >>> settings = BeamDesignSettings(
        ...     k_y=1.0,  # Pinned about strong axis
        ...     k_z=0.7,  # Fixed-pinned about weak axis
        ...     L_LT=3.0,  # 3m unbraced length
        ... )
    """
    # Buckling lengths (absolute)
    L_cr_y: Optional[float] = None
    L_cr_z: Optional[float] = None

    # Effective length factors (relative)
    k_y: float = 1.0
    k_z: float = 1.0

    # Lateral-torsional buckling
    L_LT: Optional[float] = None
    lateral_restraints: List[float] = field(default_factory=list)
    moment_diagram: MomentDiagramType = MomentDiagramType.UNIFORM

    # Moment gradient factors (for LTB)
    C1: Optional[float] = None
    C2: Optional[float] = None
    C3: Optional[float] = None
    end_moment_ratio: float = 1.0  # ψ = M_min/M_max

    def get_buckling_length_y(self, element_length: float) -> float:
        """Get effective buckling length about y-axis."""
        if self.L_cr_y is not None:
            return self.L_cr_y
        return self.k_y * element_length

    def get_buckling_length_z(self, element_length: float) -> float:
        """Get effective buckling length about z-axis."""
        if self.L_cr_z is not None:
            return self.L_cr_z
        return self.k_z * element_length

    def get_LT_length(self, element_length: float) -> float:
        """Get unbraced length for lateral-torsional buckling."""
        if self.L_LT is not None:
            return self.L_LT
        return element_length
```

**Acceptance Criteria:**
- [ ] BeamDesignSettings stores all buckling parameters
- [ ] Effective length computation works with factors and absolute lengths
- [ ] Moment gradient factors can be specified or computed
- [ ] Settings can be attached to Beam elements

---

## Task 10.4: EC3BeamSettings Class

**Dependencies:** Task 10.3
**Difficulty:** Medium

**Description:**
Create EC3-specific beam settings that extend or wrap the general settings.

**File:** `src/grillex/design_codes/eurocode3.py` (add to existing)

```python
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional

class BucklingCurve(Enum):
    """EC3 buckling curves (Table 6.1, 6.2)."""
    a0 = "a0"  # α = 0.13
    a = "a"    # α = 0.21
    b = "b"    # α = 0.34
    c = "c"    # α = 0.49
    d = "d"    # α = 0.76

class SectionType(Enum):
    """Section types for automatic buckling curve selection."""
    ROLLED_I = "rolled_I"
    WELDED_I = "welded_I"
    HOLLOW_HOT = "hollow_hot"
    HOLLOW_COLD = "hollow_cold"
    WELDED_BOX = "welded_box"
    CHANNEL = "channel"
    ANGLE = "angle"
    T_SECTION = "T"

@dataclass
class EC3BeamSettings:
    """Eurocode 3 specific settings for a beam.

    These settings control EC3-specific check parameters.
    Use in conjunction with BeamDesignSettings for complete configuration.

    Attributes:
        section_class: Override automatic section classification (1-4).
        section_type: Section type for automatic buckling curve selection.
        buckling_curve_y: Buckling curve for y-axis (strong axis).
        buckling_curve_z: Buckling curve for z-axis (weak axis).
        buckling_curve_LT: Buckling curve for lateral-torsional buckling.
        use_general_LTB_method: Use general method (6.3.2.2) vs rolled sections method.
        ignore_LTB: Skip lateral-torsional buckling check.

    Example:
        >>> settings = EC3BeamSettings(
        ...     section_type=SectionType.ROLLED_I,
        ...     buckling_curve_y=BucklingCurve.a,
        ...     buckling_curve_z=BucklingCurve.b,
        ... )
    """
    # Section classification
    section_class: Optional[int] = None  # 1, 2, 3, or 4 (None = auto)
    section_type: SectionType = SectionType.ROLLED_I

    # Buckling curves
    buckling_curve_y: Optional[BucklingCurve] = None  # None = auto from section_type
    buckling_curve_z: Optional[BucklingCurve] = None
    buckling_curve_LT: Optional[BucklingCurve] = None

    # LTB options
    use_general_LTB_method: bool = True
    ignore_LTB: bool = False

    def get_imperfection_factor(self, curve: BucklingCurve) -> float:
        """Get imperfection factor α for buckling curve (Table 6.1)."""
        factors = {
            BucklingCurve.a0: 0.13,
            BucklingCurve.a: 0.21,
            BucklingCurve.b: 0.34,
            BucklingCurve.c: 0.49,
            BucklingCurve.d: 0.76,
        }
        return factors[curve]
```

**Acceptance Criteria:**
- [ ] EC3BeamSettings stores all EC3-specific parameters
- [ ] Buckling curves can be specified or auto-selected
- [ ] Section class can override automatic classification
- [ ] Imperfection factors computed correctly

---

## Task 10.5: Section Classification (EC3 Table 5.2)

**Dependencies:** Task 10.4
**Difficulty:** High

**Description:**
Implement full section classification for I-sections, channels, and hollow sections.

**Key Equations:**
```
ε = √(235/fy)  where fy in N/mm²

For I-section flanges (outstand):
  Class 1: c/t ≤ 9ε
  Class 2: c/t ≤ 10ε
  Class 3: c/t ≤ 14ε

For I-section webs in compression:
  Class 1: c/t ≤ 33ε
  Class 2: c/t ≤ 38ε
  Class 3: c/t ≤ 42ε

For I-section webs in bending:
  Class 1: c/t ≤ 72ε
  Class 2: c/t ≤ 83ε
  Class 3: c/t ≤ 124ε

For combined axial + bending, use α parameter:
  α = (c + Ned/(2*tw*fy)) / (2c)  when α > 0.5
```

**Acceptance Criteria:**
- [ ] Flange classification correct for I-sections
- [ ] Web classification correct for pure bending
- [ ] Web classification correct for combined N+M
- [ ] Section class = max(flange_class, web_class)
- [ ] Returns Class 4 when limits exceeded
- [ ] Epsilon factor computed correctly from fy

---

## Task 10.6: Flexural Buckling Check (EC3 6.3.1)

**Dependencies:** Task 10.3, 10.4
**Difficulty:** High

**Description:**
Implement column buckling check for compression members.

**Key Equations:**
```
Non-dimensional slenderness:
  λ̄ = √(A·fy/Ncr) = Lcr/(i·π) · √(fy/E)  for Class 1, 2, 3

Euler critical load:
  Ncr = π²·E·I / Lcr²

Reduction factor χ (Clause 6.3.1.2):
  Φ = 0.5·[1 + α·(λ̄ - 0.2) + λ̄²]
  χ = 1 / (Φ + √(Φ² - λ̄²))  but χ ≤ 1.0

Design buckling resistance:
  Nb,Rd = χ·A·fy/γM1  for Class 1, 2, 3
  Nb,Rd = χ·Aeff·fy/γM1  for Class 4

Utilization:
  NEd/Nb,Rd ≤ 1.0
```

**Acceptance Criteria:**
- [ ] Slenderness computed correctly for y and z axes
- [ ] Chi reduction factor matches tabulated values
- [ ] Buckling curves a0, a, b, c, d all work correctly
- [ ] Check performs for both axes, reports governing
- [ ] Class 4 effective area supported

---

## Task 10.7: Lateral-Torsional Buckling Check (EC3 6.3.2)

**Dependencies:** Task 10.6
**Difficulty:** High

**Description:**
Implement LTB check for beams in bending.

**Key Equations:**

**Elastic critical moment (simplified for doubly-symmetric I-sections):**
```
Mcr = C1 · (π²·E·Iz/L²) · √[(Iw/Iz) + (L²·G·It)/(π²·E·Iz)]

where:
  C1 = moment gradient factor (depends on moment diagram shape)
  Iw = warping constant
  It = St. Venant torsional constant
```

**C1 factors for standard cases:**
```
Uniform moment:        C1 = 1.0
Linear (ψ = M2/M1):   C1 = 1.88 - 1.40ψ + 0.52ψ²  (but C1 ≤ 2.70)
Concentrated at mid:   C1 = 1.35
UDL:                   C1 = 1.13
```

**Reduction factor χLT:**
```
λ̄LT = √(Wy·fy/Mcr)

For rolled sections (6.3.2.3):
  ΦLT = 0.5·[1 + αLT·(λ̄LT - λ̄LT,0) + β·λ̄LT²]
  χLT = 1 / (ΦLT + √(ΦLT² - β·λ̄LT²))

  where λ̄LT,0 = 0.4, β = 0.75 (recommended)

Design buckling resistance:
  Mb,Rd = χLT · Wy · fy / γM1

Utilization:
  MEd / Mb,Rd ≤ 1.0
```

**Acceptance Criteria:**
- [ ] Mcr computed correctly for I-sections
- [ ] C1 factors work for standard moment diagrams
- [ ] χLT matches EC3 tables
- [ ] Both general and rolled section methods available
- [ ] Short beams (λ̄LT ≤ 0.4) skip LTB check (χLT = 1.0)

---

## Task 10.8: Member Buckling Interaction (EC3 6.3.3)

**Dependencies:** Task 10.6, 10.7
**Difficulty:** Very High

**Description:**
Implement interaction checks for members subject to combined compression and bending.

**Key Equations (Method 2 - Annex B):**
```
NEd/(χy·NRk/γM1) + kyy·(My,Ed + ΔMy,Ed)/(χLT·My,Rk/γM1) + kyz·(Mz,Ed + ΔMz,Ed)/(Mz,Rk/γM1) ≤ 1

NEd/(χz·NRk/γM1) + kzy·(My,Ed + ΔMy,Ed)/(χLT·My,Rk/γM1) + kzz·(Mz,Ed + ΔMz,Ed)/(Mz,Rk/γM1) ≤ 1

Interaction factors (simplified for Class 1 and 2):
  kyy = Cmy · (1 + (λ̄y - 0.2)·NEd/(χy·NRk/γM1))
  kzz = Cmz · (1 + (λ̄z - 0.2)·NEd/(χz·NRk/γM1))
  kyz = 0.6 · kzz
  kzy = 0.6 · kyy

Moment factors Cmy, Cmz depend on moment diagram shape.
```

**Acceptance Criteria:**
- [ ] Both interaction equations (y-y and z-z) checked
- [ ] Interaction factors kyy, kyz, kzy, kzz computed correctly
- [ ] Cm factors for different moment diagrams
- [ ] ΔM shift moments for Class 4 sections
- [ ] Reports governing interaction equation

---

## Task 10.9: Additional Cross-Section Checks

**Dependencies:** Task 10.2
**Difficulty:** Medium

**Description:**
Complete the cross-section level checks.

### 10.9a: Torsion Check (EC3 6.2.7)
```
τEd ≤ τRd = fy/(√3·γM0)

For open sections (I, channel):
  τEd = Mt,Ed·t / It  (St. Venant)
  σw = Bω·ω / Iw     (warping, if applicable)

For closed sections:
  τEd = Mt,Ed / (2·Am·t)
```

### 10.9b: Bending and Shear Interaction (EC3 6.2.8)
```
When VEd > 0.5·Vpl,Rd:
  Reduce fy for web: fy,red = (1 - ρ)·fy
  where ρ = (2·VEd/Vpl,Rd - 1)²
```

### 10.9c: Bending, Shear and Axial Interaction (EC3 6.2.10)
```
For Class 1 and 2 sections:
  If n = NEd/Npl,Rd ≤ 0.25 and NEd ≤ 0.5·hw·tw·fy/γM0:
    No reduction needed
  Otherwise reduce MRd according to interaction formulae
```

**Acceptance Criteria:**
- [ ] Torsion stress computed correctly
- [ ] Shear-moment interaction reduces capacity when V > 0.5Vpl
- [ ] Combined N+V+M interaction implemented
- [ ] Warping stresses included for open sections

---

## Task 10.10: Integrate Settings with Beam and Model

**Dependencies:** Task 10.3, 10.4
**Difficulty:** Medium

**Description:**
Integrate the settings classes with the existing Beam and StructuralModel classes.

**Changes to model_wrapper.py:**
```python
class Beam:
    def __init__(self, ...):
        # Existing attributes
        self.design_settings: Optional[BeamDesignSettings] = None
        self.ec3_settings: Optional[EC3BeamSettings] = None

    def set_design_settings(self, settings: BeamDesignSettings) -> None:
        """Attach general design settings to this beam."""
        self.design_settings = settings

    def set_ec3_settings(self, settings: EC3BeamSettings) -> None:
        """Attach EC3-specific settings to this beam."""
        self.ec3_settings = settings

class StructuralModel:
    def set_default_design_settings(self, settings: BeamDesignSettings) -> None:
        """Set default design settings for all beams."""
        for beam in self.beams:
            if beam.design_settings is None:
                beam.design_settings = settings
```

**Acceptance Criteria:**
- [ ] Settings can be attached to individual beams
- [ ] Default settings can be set at model level
- [ ] EC3 checks use beam settings when available
- [ ] Falls back to defaults when no settings specified

---

## Task 10.11: Comprehensive Tests

**Dependencies:** All above
**Difficulty:** Medium

**Description:**
Create test suite covering all new functionality.

**Test Categories:**
1. Section classification tests (I-sections, various fy values)
2. Flexural buckling tests (compare to tabulated χ values)
3. LTB tests (compare to hand calculations)
4. Interaction tests (benchmark cases from EC3 guides)
5. Settings integration tests

**Acceptance Criteria:**
- [ ] At least 50 new tests for extended EC3
- [ ] Buckling reduction factors within 1% of tabulated values
- [ ] All edge cases handled (λ̄ < 0.2, λ̄ > 3.0, etc.)
- [ ] Tests cover all buckling curves

---

## Summary: New Phase 10 Tasks

| Task | Description | Difficulty | Est. Criteria |
|------|-------------|------------|---------------|
| 10.3 | BeamDesignSettings class | Medium | 4 |
| 10.4 | EC3BeamSettings class | Medium | 4 |
| 10.5 | Section Classification | High | 6 |
| 10.6 | Flexural Buckling | High | 5 |
| 10.7 | Lateral-Torsional Buckling | High | 5 |
| 10.8 | Member Buckling Interaction | Very High | 5 |
| 10.9 | Additional Cross-Section Checks | Medium | 4 |
| 10.10 | Integration with Beam/Model | Medium | 4 |
| 10.11 | Comprehensive Tests | Medium | 4 |
| **Total** | | | **41** |

---

## Dependencies Graph

```
10.1 (Base Classes) ─────────────────────────────────────────┐
    │                                                         │
    ▼                                                         ▼
10.2 (Basic EC3) ──────────────────────────┐            10.3 (BeamDesignSettings)
    │                                       │                 │
    │                                       │                 ▼
    │                                       │            10.4 (EC3BeamSettings)
    │                                       │                 │
    ▼                                       ▼                 ▼
10.5 (Classification) ◄──────────────── 10.6 (Flexural) ◄────┘
    │                                       │
    │                                       ▼
    │                                  10.7 (LTB)
    │                                       │
    │                                       ▼
    │                                  10.8 (Interaction)
    │                                       │
    ▼                                       │
10.9 (Cross-Section) ◄──────────────────────┘
    │
    ▼
10.10 (Integration)
    │
    ▼
10.11 (Tests)
```

---

## Implementation Priority

**Phase 1 (Foundation):**
1. Task 10.3: BeamDesignSettings
2. Task 10.4: EC3BeamSettings

**Phase 2 (Core Buckling):**
3. Task 10.5: Section Classification
4. Task 10.6: Flexural Buckling

**Phase 3 (LTB):**
5. Task 10.7: Lateral-Torsional Buckling

**Phase 4 (Interaction):**
6. Task 10.8: Member Buckling Interaction
7. Task 10.9: Additional Cross-Section Checks

**Phase 5 (Polish):**
8. Task 10.10: Integration
9. Task 10.11: Tests
