# SectionBuilder Integration Plan

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

| Property | Type | Units | Description |
|----------|------|-------|-------------|
| `area` | float | mm² | Cross-sectional area |
| `Ixx` | float | mm⁴ | Second moment about x-axis (horizontal) |
| `Iyy` | float | mm⁴ | Second moment about y-axis (vertical) |
| `Ixy` | float | mm⁴ | Product of inertia |
| `J` | float | mm⁴ | Torsional constant |
| `Cw` | float | mm⁶ | Warping constant |
| `Avx` | float | mm² | Shear area in x-direction |
| `Avy` | float | mm² | Shear area in y-direction |
| `y_top/y_bottom` | float | mm | Fibre distances (vertical) |
| `x_left/x_right` | float | mm | Fibre distances (horizontal) |
| `Zx/Zy` | float | mm³ | Plastic section moduli |
| `Sx_pos/Sx_neg/Sy_pos/Sy_neg` | float | mm³ | Elastic section moduli |
| `shear_center` | Point2D | mm | Shear center location |

## Key Differences

### 1. Unit System

| Quantity | Grillex | SectionBuilder | Conversion Factor |
|----------|---------|----------------|-------------------|
| Length | m | mm | 1e-3 |
| Area | m² | mm² | 1e-6 |
| I (2nd moment) | m⁴ | mm⁴ | 1e-12 |
| Iw (warping) | m⁶ | mm⁶ | 1e-18 |

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

### 3. Properties Not in SectionBuilder

- `omega_max` (maximum sectorial coordinate) - needed for warping stress calculations
  - Could be computed from shear center and section geometry
  - May need to add to sectionbuilder or compute in adapter

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
│                  GrillexSectionAdapter                      │
│  ─────────────────────────────────────────────────────────  │
│  - Unit conversion (mm → m)                                 │
│  - Axis mapping (Ixx→Iz, Iyy→Iy)                           │
│  - Property mapping                                         │
│  - omega_max calculation                                    │
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
- [ ] Module structure created
- [ ] Imports work correctly
- [ ] Module added to grillex package

#### Task 1.2: Implement Unit Conversion Utilities

**File:** `src/grillex/sections/conversion.py`

```python
# Conversion factors
MM_TO_M = 1e-3
MM2_TO_M2 = 1e-6
MM4_TO_M4 = 1e-12
MM6_TO_M6 = 1e-18

def convert_section_properties(sb_props: SectionProperties) -> dict:
    """Convert sectionbuilder properties to grillex units and conventions."""
    ...
```

**Acceptance Criteria:**
- [ ] All unit conversions implemented
- [ ] Axis convention mapping implemented
- [ ] Unit tests with known values
- [ ] Handles None/missing properties gracefully

#### Task 1.3: Implement GrillexSectionAdapter

**File:** `src/grillex/sections/adapter.py`

```python
class GrillexSectionAdapter:
    """Adapter to use SectionBuilder sections in Grillex."""

    def __init__(self, sb_adapter: SectionLibraryAdapter):
        self._sb_adapter = sb_adapter

    def get_grillex_section_params(self, designation: str) -> dict:
        """Get section parameters ready for grillex add_section()."""
        ...

    def search(self, query: str) -> List[SectionInfo]:
        """Search underlying adapter."""
        return self._sb_adapter.search(query)
```

**Acceptance Criteria:**
- [ ] Wraps any SectionLibraryAdapter
- [ ] Returns dict compatible with `StructuralModel.add_section()`
- [ ] Handles I-sections, hollow sections, channels, angles
- [ ] Computes `requires_warping` flag based on section type
- [ ] Unit tests for common section types

#### Task 1.4: Add omega_max Calculation

For warping stress calculations, need maximum sectorial coordinate.

**Approach Options:**

1. **Approximate for I-sections:**
   ```python
   # For doubly-symmetric I-sections
   omega_max = (h - tf) * bf / 4  # where h=height, bf=flange width, tf=flange thickness
   ```

2. **Compute from geometry:**
   - Use sectionbuilder's geometry to compute sectorial coordinates
   - May need enhancement in sectionbuilder

3. **Skip if not available:**
   - Set `omega_max = 0.0` with warning
   - User can set manually if needed

**Recommendation:** Start with option 1 (approximate), document limitation.

**Acceptance Criteria:**
- [ ] omega_max computed for I-sections
- [ ] Warning logged for unsupported section types
- [ ] Documentation of approximation formula

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
        name: Optional[str] = None
    ) -> str:
        """Register a section library for use in the model.

        Args:
            adapter: Path to JSON library, SectionBuilder adapter, or GrillexSectionAdapter
            name: Optional name override

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
- [ ] `register_section_library()` implemented
- [ ] `search_sections()` implemented
- [ ] `add_section_from_library()` implemented
- [ ] Integration tests with sample library
- [ ] Error handling for missing sections/libraries

#### Task 2.2: Add Direct SectionBuilder Section Support

**File:** `src/grillex/core/model_wrapper.py`

```python
def add_section_from_properties(
    self,
    name: str,
    properties: "SectionProperties",  # from sectionbuilder
) -> Section:
    """Add section from computed SectionBuilder properties.

    Useful for custom/composite sections created programmatically.

    Args:
        name: Section name in grillex
        properties: SectionBuilder SectionProperties object

    Returns:
        Created Section object
    """
    ...
```

**Acceptance Criteria:**
- [ ] Method implemented
- [ ] Works with primitive sections (Rectangle, ISection, etc.)
- [ ] Works with CompositeSection
- [ ] Unit conversion applied correctly

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
| Missing properties (omega_max) | Medium | Approximations with documentation |
| Axis convention errors | High | Comprehensive test suite |

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
