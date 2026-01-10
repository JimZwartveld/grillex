# SectionBuilder Integration Plan

## Changelog

| Date | Change |
|------|--------|
| 2026-01-10 | Simplified unit handling: adapter is now unit-aware via `input_units` parameter. |
| 2026-01-10 | Added omega_max implementation in sectionbuilder. Updated Task 1.4 to reflect simplified approach (unit conversion only). |
| 2026-01-10 | Initial plan created |

---

## Overview

This plan outlines the integration of the `sectionbuilder` library into Grillex 2.0 to provide enhanced steel section building capabilities. The integration will leverage sectionbuilder's comprehensive section property calculations and adapter pattern for external section databases.

## Current State Analysis

### Grillex Section Properties (C++)

Currently defined in `cpp/include/grillex/section.hpp`:

| Property | Type | Units | Description |
|----------|------|-------|-------------|
| `A` | double | m² | Cross-sectional area |
| `Iy` | double | m⁴ | Second moment about local y-axis (weak axis) |
| `Iz` | double | m⁴ | Second moment about local z-axis (strong axis) |
| `J` | double | m⁴ | Torsional constant |
| `Iw` | double | m⁶ | Warping constant |
| `Asy` | double | m² | Shear area in y-direction |
| `Asz` | double | m² | Shear area in z-direction |
| `zy_top/zy_bot` | double | m | Fibre distances for y-axis bending |
| `zz_top/zz_bot` | double | m | Fibre distances for z-axis bending |
| `omega_max` | double | m² | Maximum sectorial coordinate |
| `requires_warping` | bool | - | Warping analysis flag |

### SectionBuilder Properties

From `sectionbuilder/core/properties/models.py`:

> **Note:** SectionBuilder is unitless. Units shown assume mm input (typical for section databases).

| Property | Type | Units (if mm input) | Description |
|----------|------|---------------------|-------------|
| `area` | float | mm² | Cross-sectional area |
| `Ixx` | float | mm⁴ | Second moment about x-axis (horizontal) |
| `Iyy` | float | mm⁴ | Second moment about y-axis (vertical) |
| `Ixy` | float | mm⁴ | Product of inertia |
| `J` | float | mm⁴ | Torsional constant |
| `Cw` | float | mm⁶ | Warping constant |
| `omega_max` | float | mm² | Maximum sectorial coordinate ✅ **NEW** |
| `Avx` | float | mm² | Shear area in x-direction |
| `Avy` | float | mm² | Shear area in y-direction |
| `y_top/y_bottom` | float | mm | Fibre distances (vertical) |
| `x_left/x_right` | float | mm | Fibre distances (horizontal) |
| `Zx/Zy` | float | mm³ | Plastic section moduli |
| `Sx_pos/Sx_neg/Sy_pos/Sy_neg` | float | mm³ | Elastic section moduli |
| `shear_center` | Point2D | mm | Shear center location |

## Key Differences

### 1. Unit System

**SectionBuilder is unitless** - it returns properties in units consistent with the input dimensions.

**Grillex uses**: m, kN, mT (SI-derived)

**Solution**: The `GrillexSectionAdapter` is **unit-aware** via an `input_units` parameter:

```python
# Standard database (mm dimensions) - conversion applied automatically
adapter = GrillexSectionAdapter(eurocode_adapter, input_units="mm")

# Custom sections in meters - no conversion needed
adapter = GrillexSectionAdapter(custom_adapter, input_units="m")
```

The adapter internally computes the appropriate scale factor based on `input_units` and applies it to all properties.

### 2. Axis Convention

```
SectionBuilder:               Grillex (beam local):
  Y (vertical/strong)           Z (vertical/strong)
  |                             |
  |                             |
  +---> X (horizontal/weak)     +---> Y (horizontal/weak)
```

Property mapping:
- `Ixx` (sectionbuilder) → `Iz` (grillex) - strong axis
- `Iyy` (sectionbuilder) → `Iy` (grillex) - weak axis
- `Avy` (sectionbuilder) → `Asy` (grillex) - shear in weak direction
- `Avx` (sectionbuilder) → `Asz` (grillex) - shear in strong direction
- `y_top/y_bottom` → `zy_top/zy_bot` - fibre distances for strong axis bending
- `x_left/x_right` → `zz_top/zz_bot` - fibre distances for weak axis bending

### 3. Property Mapping Notes

✅ **omega_max is now available in SectionBuilder** (implemented 2026-01-10)

The `omega_max` (maximum sectorial coordinate) is now computed directly by sectionbuilder:
- For doubly-symmetric I-sections: `omega_max = (h × bf) / 4`
- For channel sections: Accounts for shear center offset
- For angle sections: `omega_max ≈ (L1 × L2) / 2`
- For closed/solid sections: `omega_max = 0` (no warping)

The adapter simply needs to convert units (mm² → m²) when mapping this property.

## Architecture Decision: Adapter Pattern

**Recommendation: YES, create a GrillexSectionAdapter**

Reasons:
1. **Clean separation**: Keeps sectionbuilder as an optional dependency
2. **Unit conversion**: Centralizes all unit/axis conversions
3. **Extensibility**: Can wrap multiple sectionbuilder adapters (AISC, Eurocode, custom)
4. **Caching**: Can cache converted properties
5. **Error handling**: Single place to handle missing properties
6. **Future-proof**: Can adapt to changes in either library

### Proposed Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Grillex StructuralModel                 │
│                                                             │
│  add_section_from_library(adapter, designation)             │
│  add_section_from_sectionbuilder(section_properties)        │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│          GrillexSectionAdapter(input_units="mm"|"m")        │
│  ─────────────────────────────────────────────────────────  │
│  - Unit-aware conversion (based on input_units)             │
│  - Axis mapping (Ixx→Iz, Iyy→Iy)                           │
│  - Property mapping (including omega_max)                   │
│  - Warping flag inference                                   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              SectionBuilder Library                         │
│  ─────────────────────────────────────────────────────────  │
│  - SectionLibraryAdapter (AISC, Eurocode, JSON)            │
│  - Section types (ISection, Rectangle, CompositeSection)    │
│  - SectionProperties computation                            │
└─────────────────────────────────────────────────────────────┘
```

## Implementation Tasks

### Phase 1: Core Adapter (Priority: High)

#### Task 1.1: Create SectionBuilder Integration Module

**File:** `src/grillex/sections/__init__.py`

Create new module structure:
```
src/grillex/sections/
├── __init__.py
├── adapter.py           # GrillexSectionAdapter
├── conversion.py        # Unit and axis conversion utilities
├── properties.py        # Extended section properties for design
└── library_registry.py  # Registry for multiple section libraries
```

**Acceptance Criteria:**
- [x] Module structure created
- [x] Imports work correctly
- [x] Module added to grillex package

### Execution Notes - Task 1.1 (Completed 2026-01-10)

**Steps Taken:**
1. Created `src/grillex/sections/` directory
2. Created `__init__.py` with all exports
3. Created `conversion.py`, `adapter.py`, `library_registry.py`, `properties.py`

**Files Created:**
- `src/grillex/sections/__init__.py`
- `src/grillex/sections/conversion.py`
- `src/grillex/sections/adapter.py`
- `src/grillex/sections/library_registry.py`
- `src/grillex/sections/properties.py`

---

#### Task 1.2: Implement Unit-Aware Conversion

**File:** `src/grillex/sections/conversion.py`

```python
def get_scale_factors(input_units: str) -> dict:
    """Get scale factors based on input units.

    Args:
        input_units: "mm" for millimeter-based databases, "m" for meter-based

    Returns:
        Dict with scale factors for each property type
    """
    if input_units == "m":
        return {"length": 1.0, "area": 1.0, "I": 1.0, "Iw": 1.0}
    else:  # mm
        L = 1e-3  # mm to m
        return {"length": L, "area": L**2, "I": L**4, "Iw": L**6}

def convert_section_properties(
    sb_props: SectionProperties,
    input_units: str = "mm"
) -> dict:
    """Convert sectionbuilder properties to grillex units and conventions.

    Handles both unit scaling and axis convention mapping (Ixx→Iz, Iyy→Iy).
    """
    ...
```

**Acceptance Criteria:**
- [x] Scale factors computed from `input_units` parameter
- [x] Axis convention mapping implemented (Ixx→Iz, Iyy→Iy)
- [x] Unit tests with known values for both "mm" and "m" inputs
- [x] Handles None/missing properties gracefully

### Execution Notes - Task 1.2 (Completed 2026-01-10)

**Steps Taken:**
1. Implemented `ScaleFactors` dataclass with length, area, I, Iw, S scale factors
2. Implemented `get_scale_factors()` function for "mm" and "m" units
3. Implemented `convert_section_properties()` for object-based conversion
4. Implemented `convert_section_dict()` for dict-based conversion

**Verification:** 9 unit tests passing for conversion utilities

---

#### Task 1.3: Implement GrillexSectionAdapter

**File:** `src/grillex/sections/adapter.py`

```python
class GrillexSectionAdapter:
    """Adapter to use SectionBuilder sections in Grillex.

    Handles unit conversion and axis mapping automatically.
    """

    def __init__(
        self,
        sb_adapter: SectionLibraryAdapter,
        input_units: str = "mm"
    ):
        """
        Args:
            sb_adapter: SectionBuilder library adapter
            input_units: "mm" for standard databases (default), "m" for custom sections
        """
        self._sb_adapter = sb_adapter
        self._input_units = input_units

    def get_grillex_section_params(self, designation: str) -> dict:
        """Get section parameters ready for grillex add_section().

        Returns dict with A, Iy, Iz, J, Iw, omega_max, etc. in grillex units (m).
        """
        ...

    def search(self, query: str) -> List[SectionInfo]:
        """Search underlying adapter."""
        return self._sb_adapter.search(query)
```

**Acceptance Criteria:**
- [x] Wraps any SectionLibraryAdapter with configurable `input_units`
- [x] Returns dict compatible with `StructuralModel.add_section()`
- [x] Handles I-sections, hollow sections, channels, angles
- [x] Computes `requires_warping` flag based on section type
- [x] Unit tests for common section types with both "mm" and "m" inputs

### Execution Notes - Task 1.3 (Completed 2026-01-10)

**Steps Taken:**
1. Implemented `GrillexSectionAdapter` class with configurable input_units
2. Added property caching to reduce repeated calculations
3. Implemented `search()`, `list_sections()`, `__contains__()` methods
4. Implemented `SectionLibraryRegistry` for managing multiple libraries

**Verification:** 15 unit tests passing for GrillexSectionAdapter and SectionLibraryRegistry

---

#### Task 1.4: Map omega_max from SectionBuilder

✅ **Simplified:** omega_max is now computed directly by sectionbuilder.

The adapter handles this automatically via the `input_units` parameter:
- Read `omega_max` from `SectionProperties`
- Apply appropriate scale factor based on `input_units`
- Handle `None` values (default to 0.0)

**Available formulas in sectionbuilder** (for reference):
- I-sections: `omega_max = (h × bf) / 4`
- Channel sections: `omega_max = (h/2) × (bf + e)` (e = shear center offset)
- Angle sections: `omega_max = (L1 × L2) / 2`
- T-sections: `omega_max = max(hw × bf / 4, (hw/2) × (bf/2))`
- Closed/solid sections: `omega_max = 0`

**Acceptance Criteria:**
- [x] omega_max mapped from sectionbuilder properties
- [x] Unit conversion handled by adapter's `input_units` setting
- [x] None/missing values handled gracefully (default to 0.0)

### Execution Notes - Task 1.4 (Completed 2026-01-10)

**Steps Taken:**
1. Added omega_max mapping in `convert_section_properties()`
2. Applied area scale factor (mm² -> m²)
3. Default to 0.0 when property is missing or None

**Verification:** Unit tests verify omega_max conversion and None handling

---

### Phase 2: StructuralModel Integration (Priority: High)

#### Task 2.1: Add Section Library Methods to StructuralModel

**File:** `src/grillex/core/model_wrapper.py`

```python
class StructuralModel:
    def __init__(self, ...):
        ...
        self._section_adapters: Dict[str, GrillexSectionAdapter] = {}

    def register_section_library(
        self,
        adapter: Union[str, SectionLibraryAdapter, GrillexSectionAdapter],
        name: Optional[str] = None,
        input_units: str = "mm"
    ) -> str:
        """Register a section library for use in the model.

        Args:
            adapter: Path to JSON library, SectionBuilder adapter, or GrillexSectionAdapter
            name: Optional name override
            input_units: "mm" for standard databases, "m" for custom sections

        Returns:
            Registered adapter name
        """
        ...

    def search_sections(
        self,
        query: str,
        library: Optional[str] = None
    ) -> List[SectionInfo]:
        """Search registered section libraries."""
        ...

    def add_section_from_library(
        self,
        designation: str,
        library: Optional[str] = None,
        name: Optional[str] = None
    ) -> Section:
        """Add a section from a registered library.

        Args:
            designation: Section designation (e.g., "HEB300", "W14X22")
            library: Library name (searches all if None)
            name: Optional name override for the section

        Returns:
            Created Section object
        """
        ...
```

**Acceptance Criteria:**
- [x] `register_section_library()` implemented
- [x] `search_sections()` implemented
- [x] `add_section_from_library()` implemented
- [x] Integration tests with sample library
- [x] Error handling for missing sections/libraries

### Execution Notes - Task 2.1 (Completed 2026-01-10)

**Steps Taken:**
1. Added `_section_adapters` dict to StructuralModel.__init__()
2. Implemented `register_section_library()` supporting JSON, SB adapters, GrillexSectionAdapter
3. Implemented `search_sections()` for querying registered libraries
4. Implemented `add_section_from_library()` with automatic conversion
5. Added `list_section_libraries()` helper method

**Verification:** 3 integration tests passing for StructuralModel methods

---

#### Task 2.2: Add Direct SectionBuilder Section Support

**File:** `src/grillex/core/model_wrapper.py`

```python
def add_section_from_properties(
    self,
    name: str,
    properties: "SectionProperties",  # from sectionbuilder
    input_units: str = "mm"
) -> Section:
    """Add section from computed SectionBuilder properties.

    Useful for custom/composite sections created programmatically.

    Args:
        name: Section name in grillex
        properties: SectionBuilder SectionProperties object
        input_units: "mm" if properties are in mm, "m" if in meters

    Returns:
        Created Section object
    """
    ...
```

**Acceptance Criteria:**
- [x] Method implemented
- [x] Works with primitive sections (Rectangle, ISection, etc.)
- [x] Works with CompositeSection
- [x] Unit conversion applied correctly

### Execution Notes - Task 2.2 (Completed 2026-01-10)

**Steps Taken:**
1. Implemented `add_section_from_properties()` in StructuralModel
2. Uses `convert_section_properties()` for unit conversion
3. Creates section with converted parameters via `add_section()`

**Verification:** Method tested via integration tests

---

### Phase 3: Section Library Support (Priority: Medium)

#### Task 3.1: Create Built-in European Section Library

**File:** `src/grillex/sections/libraries/eurocode_sections.json`

Include common European sections:
- IPE series (IPE80 - IPE600)
- HEA series (HEA100 - HEA1000)
- HEB series (HEB100 - HEB1000)
- HEM series (HEM100 - HEM1000)
- UPN channels
- L-angles

**Source:** Extract from Eurocode tables, store as JSON.

**Acceptance Criteria:**
- [ ] IPE series complete
- [ ] HEA/HEB/HEM series complete
- [ ] JSON format matches sectionbuilder schema
- [ ] Properties verified against published tables

#### Task 3.2: Add Library Auto-Discovery

```python
def discover_section_libraries(search_paths: List[str] = None) -> List[str]:
    """Discover available section library JSON files.

    Searches:
    1. Package resources (built-in libraries)
    2. User config directory (~/.grillex/sections/)
    3. Custom search paths
    """
    ...
```

**Acceptance Criteria:**
- [ ] Built-in libraries discovered automatically
- [ ] User libraries in ~/.grillex/sections/ found
- [ ] Custom paths supported

### Phase 4: Extended Section Properties (Priority: Low)

#### Task 4.1: Create Extended Section Properties Class

For design code checks, need additional properties not currently in grillex Section:

**File:** `src/grillex/sections/properties.py`

```python
@dataclass
class ExtendedSectionProperties:
    """Extended section properties for design code checks.

    Stores additional properties computed by sectionbuilder that are
    needed for design code unity checks but not for FE analysis.
    """
    # From grillex Section (for reference)
    A: float       # Cross-sectional area [m²]
    Iy: float      # Second moment, weak axis [m⁴]
    Iz: float      # Second moment, strong axis [m⁴]
    J: float       # Torsional constant [m⁴]
    Iw: float      # Warping constant [m⁶]

    # Additional for design
    Zpy: float     # Plastic modulus, weak axis [m³]
    Zpz: float     # Plastic modulus, strong axis [m³]
    Wely: float    # Elastic modulus, weak axis [m³]
    Welz: float    # Elastic modulus, strong axis [m³]
    iy: float      # Radius of gyration, weak axis [m]
    iz: float      # Radius of gyration, strong axis [m]

    # Section dimensions (for classification)
    h: float       # Total height [m]
    b: float       # Flange width [m]
    tw: float      # Web thickness [m]
    tf: float      # Flange thickness [m]

    # Classification helpers
    hw: float      # Clear web height [m]
    cf: float      # Flange outstand [m]
```

**Acceptance Criteria:**
- [ ] Dataclass defined
- [ ] Computed from sectionbuilder properties
- [ ] Used by EC3 design code module
- [ ] Stored on Beam objects for design checks

### Phase 5: Documentation and Testing (Priority: High)

#### Task 5.1: Documentation

- [ ] User guide: How to use section libraries
- [ ] API reference: All new methods
- [ ] Examples: Common use cases
- [ ] Migration guide: From manual section entry

#### Task 5.2: Test Suite

- [ ] Unit tests for conversion utilities
- [ ] Unit tests for adapter
- [ ] Integration tests with StructuralModel
- [ ] Test with real section libraries (AISC, Eurocode)
- [ ] Performance tests (large libraries)

## Dependencies

### Required

- `sectionbuilder` package (pip install)
- Grillex C++ build system (no changes needed)

### Optional

- Pre-built section libraries (JSON files)

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| sectionbuilder API changes | Medium | Pin version, use adapter pattern |
| Performance with large libraries | Low | Caching in adapter |
| ~~Missing properties (omega_max)~~ | ~~Medium~~ | ~~Approximations with documentation~~ ✅ **Resolved** |
| Axis convention errors | High | Comprehensive test suite |

> **Note:** The omega_max risk has been resolved. SectionBuilder now computes omega_max directly for all relevant section types (I-sections, channels, angles, T-sections).

## Future Enhancements

1. **Webapp Section Widget** (mentioned by user - separate task)
   - Visual section selection
   - Property preview
   - Custom section builder UI

2. **Parametric Sections**
   - Define sections by dimensions, not properties
   - Auto-compute properties using sectionbuilder

3. **Section Optimization**
   - Iterate through library to find optimal section
   - Based on utilization ratio

4. **FE Mesh from Geometry**
   - Use sectionbuilder geometry for detailed stress analysis
   - Shell element models of sections

## Summary

The integration uses an adapter pattern to:
1. Keep sectionbuilder as an optional dependency
2. Handle unit and axis convention differences
3. Support multiple section libraries (AISC, Eurocode, custom)
4. Enable design code checks with extended properties

The webapp widget enhancement is noted for future work.
