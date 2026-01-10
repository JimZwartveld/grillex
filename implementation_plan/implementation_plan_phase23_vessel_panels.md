## Phase 23: Vessel Geometry - Panel Derivation & Validation

### Overview

This phase implements automatic panel identification from the stiffener/girder layout and geometry validation.

**Key Concepts:**
- **Panel**: Rectangular plate region bounded by stiffeners, girders, or edges
- **Panel derivation**: Automatic identification of panels from component layout
- **Thickness boundaries**: Panels split at thickness change regions
- **Geometry validation**: Check for gaps, overlaps, and consistency

**Dependencies:** Phase 21 (Core), Phase 22 (Profiles)

**Directory Structure:**
```
src/grillex/vessel/
└── geometry/
    ├── panels.py           # Panel derivation logic
    └── validation.py       # Geometry validation
```

---

### Task 23.1: Create Panel Class

**Requirements:** New feature
**Dependencies:** Phase 21, 22
**Difficulty:** Medium

**Description:**
Create the Panel class representing a derived plate region.

**Steps:**

1. Create `src/grillex/vessel/geometry/panels.py`:
   ```python
   from dataclasses import dataclass, field
   from typing import List, Optional, TYPE_CHECKING, Tuple
   from uuid import uuid4, UUID
   from enum import Enum

   if TYPE_CHECKING:
       from .plate_field import PlateField
       from .stiffener import Stiffener

   class PanelOrientation(Enum):
       """Panel orientation in 3D space."""
       XY = "xy"  # Horizontal (deck, bottom)
       YZ = "yz"  # Transverse vertical (trans. bulkhead)
       XZ = "xz"  # Longitudinal vertical (long. bulkhead, side shell)

   @dataclass
   class Panel:
       """
       A derived plate panel bounded by stiffeners, girders, or edges.

       Panels are automatically derived from the stiffener layout on
       plate fields. Each panel represents a region that can be
       modeled as a single plate element or further meshed.

       Attributes:
           plate_field: Parent plate field
           orientation: Panel plane (XY, YZ, XZ)
           bounds: (coord1_min, coord1_max, coord2_min, coord2_max)
           thickness: Plate thickness at this panel
           material: Material name
           boundary_stiffeners: Stiffeners forming the boundary
       """
       plate_field: "PlateField"
       orientation: PanelOrientation
       bounds: Tuple[float, float, float, float]  # (min1, max1, min2, max2)
       thickness: float
       material: Optional[str] = None

       id: UUID = field(default_factory=uuid4)
       _boundary_stiffeners: List["Stiffener"] = field(default_factory=list)

       @property
       def coord1_min(self) -> float:
           """Minimum of first coordinate."""
           return self.bounds[0]

       @property
       def coord1_max(self) -> float:
           """Maximum of first coordinate."""
           return self.bounds[1]

       @property
       def coord2_min(self) -> float:
           """Minimum of second coordinate."""
           return self.bounds[2]

       @property
       def coord2_max(self) -> float:
           """Maximum of second coordinate."""
           return self.bounds[3]

       @property
       def width(self) -> float:
           """Panel width (first coordinate direction)."""
           return self.coord1_max - self.coord1_min

       @property
       def height(self) -> float:
           """Panel height (second coordinate direction)."""
           return self.coord2_max - self.coord2_min

       @property
       def area(self) -> float:
           """Panel area in m²."""
           return self.width * self.height

       @property
       def aspect_ratio(self) -> float:
           """Aspect ratio (larger/smaller)."""
           if self.width == 0 or self.height == 0:
               return float('inf')
           return max(self.width, self.height) / min(self.width, self.height)

       def get_center(self) -> Tuple[float, float]:
           """Get panel center in local coordinates."""
           return (
               (self.coord1_min + self.coord1_max) / 2,
               (self.coord2_min + self.coord2_max) / 2
           )

       def get_corners_3d(self, fixed_coord: float) -> List[Tuple[float, float, float]]:
           """
           Get 3D corner coordinates.

           Args:
               fixed_coord: The fixed coordinate value (x, y, or z depending on orientation)

           Returns:
               List of 4 corner points as (x, y, z) tuples
           """
           c1_min, c1_max = self.coord1_min, self.coord1_max
           c2_min, c2_max = self.coord2_min, self.coord2_max

           if self.orientation == PanelOrientation.XY:
               # Fixed z, coords are (x, y)
               return [
                   (c1_min, c2_min, fixed_coord),
                   (c1_max, c2_min, fixed_coord),
                   (c1_max, c2_max, fixed_coord),
                   (c1_min, c2_max, fixed_coord),
               ]
           elif self.orientation == PanelOrientation.YZ:
               # Fixed x, coords are (y, z)
               return [
                   (fixed_coord, c1_min, c2_min),
                   (fixed_coord, c1_max, c2_min),
                   (fixed_coord, c1_max, c2_max),
                   (fixed_coord, c1_min, c2_max),
               ]
           else:  # XZ
               # Fixed y, coords are (x, z)
               return [
                   (c1_min, fixed_coord, c2_min),
                   (c1_max, fixed_coord, c2_min),
                   (c1_max, fixed_coord, c2_max),
                   (c1_min, fixed_coord, c2_max),
               ]


   def derive_panels_from_plate_field(
       plate_field: "PlateField",
       orientation: PanelOrientation,
       bounds: Tuple[float, float, float, float],
       include_thickness_boundaries: bool = True
   ) -> List[Panel]:
       """
       Derive panels from a plate field by identifying regions bounded
       by stiffeners and optionally thickness changes.

       Args:
           plate_field: The plate field to derive panels from
           orientation: Panel orientation
           bounds: Overall bounds (min1, max1, min2, max2)
           include_thickness_boundaries: Split at thickness changes

       Returns:
           List of derived Panel objects
       """
       panels = []

       # Get stiffener positions
       stiffeners = plate_field.get_stiffeners()

       # Collect division lines in each direction
       # Direction 1 divisions (e.g., Y for deck, Y for trans bhd)
       div1 = {bounds[0], bounds[1]}  # Start with edges
       # Direction 2 divisions (e.g., X for deck, Z for trans bhd)
       div2 = {bounds[2], bounds[3]}

       for stiff in stiffeners:
           pos = stiff.position
           # Determine which direction this stiffener divides
           # Based on stiffener direction vs panel orientation
           from .stiffener import StiffenerDirection
           if stiff.direction == StiffenerDirection.LONGITUDINAL:
               # Runs in X, so divides in Y
               if orientation == PanelOrientation.XY:
                   div1.add(pos)
           elif stiff.direction == StiffenerDirection.TRANSVERSE:
               # Runs in Y, so divides in X
               if orientation == PanelOrientation.XY:
                   div2.add(pos)
           elif stiff.direction == StiffenerDirection.VERTICAL:
               # Runs in Z
               if orientation == PanelOrientation.YZ:
                   div1.add(pos)  # Divides in Y
               elif orientation == PanelOrientation.XZ:
                   div2.add(pos)  # Divides in X

       # Add thickness region boundaries if requested
       if include_thickness_boundaries:
           for region in plate_field.thickness_regions:
               div1.add(region.y_start)
               div1.add(region.y_end)
               div2.add(region.z_start)
               div2.add(region.z_end)

       # Sort divisions
       div1_sorted = sorted(div1)
       div2_sorted = sorted(div2)

       # Create panels for each cell
       for i in range(len(div1_sorted) - 1):
           for j in range(len(div2_sorted) - 1):
               c1_min, c1_max = div1_sorted[i], div1_sorted[i + 1]
               c2_min, c2_max = div2_sorted[j], div2_sorted[j + 1]

               # Get thickness at panel center
               center1 = (c1_min + c1_max) / 2
               center2 = (c2_min + c2_max) / 2
               thickness = plate_field.get_thickness_at(center1, center2)
               material = plate_field.get_material_at(center1, center2)

               if thickness is None or thickness <= 0:
                   continue  # Skip if no valid thickness

               panel = Panel(
                   plate_field=plate_field,
                   orientation=orientation,
                   bounds=(c1_min, c1_max, c2_min, c2_max),
                   thickness=thickness,
                   material=material
               )
               panels.append(panel)

       return panels
   ```

**Acceptance Criteria:**
- [ ] Panel class with bounds, thickness, material, orientation
- [ ] PanelOrientation enum (XY, YZ, XZ)
- [ ] width, height, area, aspect_ratio properties
- [ ] get_center() returns panel center
- [ ] get_corners_3d() returns 3D corner coordinates
- [ ] derive_panels_from_plate_field() identifies panels from stiffeners
- [ ] Thickness boundaries create panel divisions

---

### Task 23.2: Add Panel Derivation to Components

**Requirements:** New feature
**Dependencies:** Task 23.1
**Difficulty:** Medium

**Description:**
Add panel derivation methods to structural components.

**Steps:**

1. Update component classes to add `get_panels()` methods:
   ```python
   # In components.py, add to each relevant class:

   class Deck:
       # ... existing code ...

       def get_panels(self) -> List[Panel]:
           """Derive panels from deck plating and stiffeners."""
           from .panels import derive_panels_from_plate_field, PanelOrientation
           return derive_panels_from_plate_field(
               plate_field=self._plate_field,
               orientation=PanelOrientation.XY,
               bounds=(
                   self.segment.x_start,
                   self.segment.x_end,
                   self.segment.y_start,
                   self.segment.y_end
               )
           )

   class TransverseBulkhead:
       def get_panels(self) -> List[Panel]:
           """Derive panels from bulkhead plating and stiffeners."""
           from .panels import derive_panels_from_plate_field, PanelOrientation
           return derive_panels_from_plate_field(
               plate_field=self._plate_field,
               orientation=PanelOrientation.YZ,
               bounds=(
                   self.segment.y_start,
                   self.segment.y_end,
                   0.0,  # Bottom
                   self.segment.vessel.depth
               )
           )

   # Similar for LongitudinalBulkhead, SideShell, WebFrame (plate type)
   ```

2. Add method to CargoSegment to get all panels:
   ```python
   class CargoSegment:
       def get_all_panels(self) -> List[Panel]:
           """Get all derived panels from all components."""
           panels = []

           # Deck panels
           for deck in self._decks.values():
               panels.extend(deck.get_panels())

           # Bulkhead panels
           if self._transverse_bulkhead_aft:
               panels.extend(self._transverse_bulkhead_aft.get_panels())
           if self._transverse_bulkhead_fwd:
               panels.extend(self._transverse_bulkhead_fwd.get_panels())

           for bhd in self._longitudinal_bulkheads:
               panels.extend(bhd.get_panels())

           # Side shell panels
           for shell in self._side_shells.values():
               panels.extend(shell.get_panels())

           # Webframe panels (plate type only)
           for wf in self._webframes:
               if wf.type == "plate":
                   panels.extend(wf.get_panels())

           return panels
   ```

**Acceptance Criteria:**
- [ ] Deck.get_panels() returns deck panels
- [ ] TransverseBulkhead.get_panels() returns bulkhead panels
- [ ] LongitudinalBulkhead.get_panels() returns panels
- [ ] SideShell.get_panels() returns side shell panels
- [ ] WebFrame.get_panels() works for plate-type webframes
- [ ] CargoSegment.get_all_panels() aggregates all panels

---

### Task 23.3: Create Geometry Validation

**Requirements:** New feature
**Dependencies:** Tasks 23.1, 23.2
**Difficulty:** Medium

**Description:**
Create geometry validation to check for common issues.

**Steps:**

1. Create `src/grillex/vessel/geometry/validation.py`:
   ```python
   from dataclasses import dataclass
   from typing import List, Optional, TYPE_CHECKING
   from enum import Enum

   if TYPE_CHECKING:
       from .vessel import Vessel
       from .cargo_segment import CargoSegment

   class ValidationSeverity(Enum):
       ERROR = "error"      # Must be fixed
       WARNING = "warning"  # Should be reviewed
       INFO = "info"        # Informational

   @dataclass
   class ValidationIssue:
       """A validation issue found in vessel geometry."""
       severity: ValidationSeverity
       code: str
       message: str
       component: Optional[str] = None
       location: Optional[str] = None

   class GeometryValidator:
       """Validates vessel geometry for common issues."""

       def __init__(self, vessel: "Vessel"):
           self.vessel = vessel
           self.issues: List[ValidationIssue] = []

       def validate(self) -> List[ValidationIssue]:
           """Run all validation checks."""
           self.issues = []

           self._check_vessel_dimensions()
           self._check_cargo_segments()
           self._check_components()
           self._check_profiles()

           return self.issues

       def _add_issue(
           self,
           severity: ValidationSeverity,
           code: str,
           message: str,
           component: str = None,
           location: str = None
       ):
           self.issues.append(ValidationIssue(
               severity=severity,
               code=code,
               message=message,
               component=component,
               location=location
           ))

       def _check_vessel_dimensions(self):
           """Check vessel dimension validity."""
           v = self.vessel

           if v.length <= 0:
               self._add_issue(
                   ValidationSeverity.ERROR,
                   "INVALID_LENGTH",
                   f"Vessel length must be positive, got {v.length}"
               )

           if v.beam <= 0:
               self._add_issue(
                   ValidationSeverity.ERROR,
                   "INVALID_BEAM",
                   f"Vessel beam must be positive, got {v.beam}"
               )

           if v.depth <= 0:
               self._add_issue(
                   ValidationSeverity.ERROR,
                   "INVALID_DEPTH",
                   f"Vessel depth must be positive, got {v.depth}"
               )

           if v.frame_spacing <= 0:
               self._add_issue(
                   ValidationSeverity.ERROR,
                   "INVALID_FRAME_SPACING",
                   f"Frame spacing must be positive, got {v.frame_spacing}"
               )

           # Warnings for unusual values
           if v.length / v.beam > 10:
               self._add_issue(
                   ValidationSeverity.WARNING,
                   "UNUSUAL_ASPECT_RATIO",
                   f"Length/beam ratio {v.length/v.beam:.1f} is unusually high"
               )

       def _check_cargo_segments(self):
           """Check cargo segment validity."""
           for seg in self.vessel.get_cargo_segments():
               # Check bounds
               if seg.x_start >= seg.x_end:
                   self._add_issue(
                       ValidationSeverity.ERROR,
                       "INVALID_SEGMENT_BOUNDS",
                       f"Segment x_start ({seg.x_start}) >= x_end ({seg.x_end})",
                       component=seg.name
                   )

               if seg.y_start >= seg.y_end:
                   self._add_issue(
                       ValidationSeverity.ERROR,
                       "INVALID_SEGMENT_BOUNDS",
                       f"Segment y_start ({seg.y_start}) >= y_end ({seg.y_end})",
                       component=seg.name
                   )

               # Check segment within vessel
               if seg.x_start < 0 or seg.x_end > self.vessel.length:
                   self._add_issue(
                       ValidationSeverity.WARNING,
                       "SEGMENT_OUTSIDE_VESSEL",
                       f"Segment extends outside vessel length",
                       component=seg.name
                   )

       def _check_components(self):
           """Check component validity."""
           for seg in self.vessel.get_cargo_segments():
               # Check decks have plating
               for deck in seg.get_decks():
                   if deck.plate_field.default_thickness is None:
                       self._add_issue(
                           ValidationSeverity.WARNING,
                           "MISSING_PLATING",
                           f"Deck '{deck.name}' has no plating thickness defined",
                           component=f"{seg.name}/{deck.name}"
                       )

               # Check webframes
               for wf in seg.get_webframes():
                   if wf.type == "beam" and wf.section is None:
                       self._add_issue(
                           ValidationSeverity.ERROR,
                           "MISSING_SECTION",
                           f"Beam-type webframe at x={wf.x} has no section",
                           component=seg.name,
                           location=f"x={wf.x}"
                       )

               # Check girders
               for girder in seg.get_longitudinal_girders():
                   if girder.section is None:
                       self._add_issue(
                           ValidationSeverity.ERROR,
                           "MISSING_SECTION",
                           f"Longitudinal girder at y={girder.y} has no section",
                           component=seg.name
                       )

       def _check_profiles(self):
           """Check that all referenced profiles exist."""
           lib = self.vessel.profiles
           referenced = set()

           for seg in self.vessel.get_cargo_segments():
               # Collect all section references
               for wf in seg.get_webframes():
                   if wf.section:
                       referenced.add(wf.section)

               for girder in seg.get_longitudinal_girders():
                   if girder.section:
                       referenced.add(girder.section)

               for deck in seg.get_decks():
                   for stiff in deck.plate_field.get_stiffeners():
                       referenced.add(stiff.section)

           # Check each referenced profile exists
           for section in referenced:
               if section not in lib:
                   # Try auto-resolve
                   try:
                       lib.get(section)
                   except KeyError:
                       self._add_issue(
                           ValidationSeverity.ERROR,
                           "UNKNOWN_PROFILE",
                           f"Profile '{section}' not found in library"
                       )

       def has_errors(self) -> bool:
           """Check if any errors were found."""
           return any(i.severity == ValidationSeverity.ERROR for i in self.issues)

       def has_warnings(self) -> bool:
           """Check if any warnings were found."""
           return any(i.severity == ValidationSeverity.WARNING for i in self.issues)


   def validate_vessel(vessel: "Vessel") -> List[ValidationIssue]:
       """Validate vessel geometry and return issues."""
       validator = GeometryValidator(vessel)
       return validator.validate()
   ```

**Acceptance Criteria:**
- [ ] ValidationIssue with severity, code, message, component, location
- [ ] ValidationSeverity enum (ERROR, WARNING, INFO)
- [ ] GeometryValidator checks vessel dimensions
- [ ] GeometryValidator checks cargo segment bounds
- [ ] GeometryValidator checks components have required properties
- [ ] GeometryValidator checks referenced profiles exist
- [ ] has_errors() and has_warnings() helper methods
- [ ] validate_vessel() convenience function

---

### Task 23.4: Write Panel and Validation Tests

**Requirements:** Testing
**Dependencies:** Tasks 23.1-23.3
**Difficulty:** Low

**Description:**
Create tests for panel derivation and validation.

**Steps:**

1. Create `tests/python/test_phase23_vessel_panels.py`:
   ```python
   """Tests for Phase 23: Vessel Panel Derivation & Validation."""

   import pytest
   from grillex.vessel import Vessel
   from grillex.vessel.geometry.panels import Panel, PanelOrientation
   from grillex.vessel.geometry.validation import (
       validate_vessel, ValidationSeverity
   )
   from grillex.vessel.reference_vessels import create_standard_barge_100x24


   class TestPanelDerivation:
       """Tests for panel derivation."""

       def test_deck_panels_from_stiffeners(self):
           """Deck panels are derived from stiffener layout."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           segment = vessel.add_cargo_segment(x_start=20, x_end=80)

           deck = segment.add_deck(z=6.0, name="main_deck")
           deck.set_plating(thickness=0.016)
           deck.add_stiffeners(direction="longitudinal", spacing=0.6, section="HP200x10")

           panels = deck.get_panels()

           # Should have panels between stiffeners
           assert len(panels) > 0
           # All panels should be XY orientation
           assert all(p.orientation == PanelOrientation.XY for p in panels)

       def test_panel_thickness(self):
           """Panel thickness matches plate field thickness."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           segment = vessel.add_cargo_segment(x_start=20, x_end=80)

           deck = segment.add_deck(z=6.0)
           deck.set_plating(thickness=0.016)

           panels = deck.get_panels()

           for panel in panels:
               assert panel.thickness == 0.016

       def test_panel_area(self):
           """Panel area is width x height."""
           panel = Panel(
               plate_field=None,
               orientation=PanelOrientation.XY,
               bounds=(0, 2, 0, 3),
               thickness=0.01
           )
           assert panel.width == 2.0
           assert panel.height == 3.0
           assert panel.area == 6.0


   class TestValidation:
       """Tests for geometry validation."""

       def test_validate_valid_vessel(self):
           """Valid vessel has no errors."""
           vessel = create_standard_barge_100x24()
           issues = validate_vessel(vessel)

           errors = [i for i in issues if i.severity == ValidationSeverity.ERROR]
           assert len(errors) == 0

       def test_validate_invalid_dimensions(self):
           """Invalid dimensions are detected."""
           vessel = Vessel(name="Bad", length=-10, beam=24, depth=6)
           issues = validate_vessel(vessel)

           assert any(i.code == "INVALID_LENGTH" for i in issues)

       def test_validate_missing_plating(self):
           """Missing plating is warned."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           segment = vessel.add_cargo_segment(x_start=20, x_end=80)
           segment.add_deck(z=6.0, name="main_deck")  # No plating set

           issues = validate_vessel(vessel)

           assert any(i.code == "MISSING_PLATING" for i in issues)
   ```

**Acceptance Criteria:**
- [ ] Panel derivation tests pass
- [ ] Panel properties (area, thickness) are correct
- [ ] Validation detects invalid dimensions
- [ ] Validation detects missing plating
- [ ] Valid reference vessel passes validation

---

### Summary

| Task | Description | Difficulty | Status |
|------|-------------|------------|--------|
| 23.1 | Panel class | Medium | Pending |
| 23.2 | Panel derivation for components | Medium | Pending |
| 23.3 | Geometry validation | Medium | Pending |
| 23.4 | Panel and validation tests | Low | Pending |

**Total Acceptance Criteria:** 23 items
