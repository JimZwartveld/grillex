## Phase 21: Vessel Geometry - Core Data Structures

### Overview

This phase implements the core data structures for defining vessel geometry, focusing on the cargo deck segment and its supporting structure. The vessel geometry model will later be converted to a Grillex `StructuralModel` for FEM analysis.

**Key Concepts:**
- **Vessel**: Top-level container with frame numbering system
- **CargoSegment**: Bounded region for cargo loading (between transverse bulkheads)
- **Components**: Bulkheads, side shells, decks, webframes, girders, stiffeners
- **PlateField**: Reusable plate definition with thickness regions
- **Stiffener**: Beam element with auto-derived direction

**Scope:**
- Barges and vessels (cargo deck segment focus)
- Deck plating and deck-supporting structure
- Side shells as boundaries
- NOT in scope: Hull shell (bottom, bow/stern geometry)

**Design Principles:**
- Explicit component definition (Option B from discussion)
- Flexible API for any geometry
- Database-ready UUIDs for components
- Prepared for future extraction to separate repository

**Dependencies:** None (new feature)

**Directory Structure:**
```
src/grillex/
└── vessel/                      # Self-contained package
    ├── __init__.py
    ├── geometry/
    │   ├── __init__.py
    │   ├── vessel.py           # Vessel class
    │   ├── cargo_segment.py    # CargoSegment class
    │   ├── components.py       # Bulkheads, Deck, WebFrame, etc.
    │   ├── plate_field.py      # PlateField with thickness regions
    │   └── stiffener.py        # Stiffener with direction logic
    └── ...
```

---

### Task 21.1: Create Vessel Class with Frame System

**Requirements:** New feature
**Dependencies:** None
**Difficulty:** Medium

**Description:**
Create the top-level `Vessel` class that holds vessel dimensions and the frame numbering system.

**Steps:**

1. Create `src/grillex/vessel/__init__.py`:
   ```python
   """Vessel geometry module for offshore structural modeling."""
   from .geometry.vessel import Vessel
   from .geometry.cargo_segment import CargoSegment
   # ... other exports
   ```

2. Create `src/grillex/vessel/geometry/__init__.py`:
   ```python
   from .vessel import Vessel
   from .cargo_segment import CargoSegment
   from .components import (
       TransverseBulkhead,
       LongitudinalBulkhead,
       SideShell,
       Deck,
       WebFrame,
       LongitudinalGirder,
       TransverseGirder,
   )
   from .plate_field import PlateField, ThicknessRegion
   from .stiffener import Stiffener
   ```

3. Create `src/grillex/vessel/geometry/vessel.py`:
   ```python
   from dataclasses import dataclass, field
   from typing import List, Dict, Optional
   from uuid import uuid4, UUID

   @dataclass
   class Vessel:
       """
       Top-level vessel definition with frame numbering system.

       Attributes:
           name: Vessel identifier
           length: Overall length in meters
           beam: Overall beam (width) in meters
           depth: Depth (keel to main deck) in meters
           frame_spacing: Distance between frames in meters
           frame_zero_x: X-coordinate of frame 0 (typically AP) in meters

       Example:
           >>> vessel = Vessel(
           ...     name="Heavy Transport Barge",
           ...     length=120.0,
           ...     beam=32.0,
           ...     depth=7.5,
           ...     frame_spacing=2.4,
           ... )
           >>> vessel.frame_to_x(10)  # Frame 10 position
           24.0
           >>> vessel.x_to_frame(24.0)  # X=24m is at frame...
           10
       """
       name: str
       length: float
       beam: float
       depth: float
       frame_spacing: float = 2.5
       frame_zero_x: float = 0.0

       id: UUID = field(default_factory=uuid4)
       _cargo_segments: Dict[str, "CargoSegment"] = field(default_factory=dict)
       _materials: Dict[str, dict] = field(default_factory=dict)

       def frame_to_x(self, frame: int) -> float:
           """Convert frame number to X-coordinate."""
           return self.frame_zero_x + frame * self.frame_spacing

       def x_to_frame(self, x: float) -> int:
           """Convert X-coordinate to nearest frame number."""
           return round((x - self.frame_zero_x) / self.frame_spacing)

       def add_material(self, name: str, E: float, yield_stress: float,
                       nu: float = 0.3, rho: float = 7.85) -> "Vessel":
           """
           Add material definition.

           Args:
               name: Material identifier (e.g., "S355")
               E: Young's modulus in kN/m²
               yield_stress: Yield stress in kN/m²
               nu: Poisson's ratio (default: 0.3)
               rho: Density in mT/m³ (default: 7.85 for steel)
           """
           self._materials[name] = {
               "E": E, "yield_stress": yield_stress, "nu": nu, "rho": rho
           }
           return self

       def add_cargo_segment(
           self,
           name: str = None,
           frame_start: int = None,
           frame_end: int = None,
           x_start: float = None,
           x_end: float = None,
           y_start: float = None,
           y_end: float = None,
           support_type: str = "bottom"
       ) -> "CargoSegment":
           """
           Add a cargo deck segment bounded by transverse bulkheads.

           Can specify bounds by frame number or X-coordinate (not both).

           Args:
               name: Segment identifier (auto-generated if not provided)
               frame_start: Starting frame number
               frame_end: Ending frame number
               x_start: Starting X-coordinate in meters
               x_end: Ending X-coordinate in meters
               y_start: Port-side Y-coordinate (negative for starboard)
               y_end: Starboard-side Y-coordinate
               support_type: "bottom" (water-supported) or "columns" (SSCV deckbox)

           Returns:
               The created CargoSegment
           """
           # Convert frames to coordinates if provided
           if frame_start is not None:
               x_start = self.frame_to_x(frame_start)
           if frame_end is not None:
               x_end = self.frame_to_x(frame_end)

           # Default to full beam if not specified
           if y_start is None:
               y_start = -self.beam / 2
           if y_end is None:
               y_end = self.beam / 2

           # Auto-generate name
           if name is None:
               name = f"segment_{len(self._cargo_segments) + 1}"

           segment = CargoSegment(
               name=name,
               vessel=self,
               x_start=x_start,
               x_end=x_end,
               y_start=y_start,
               y_end=y_end,
               support_type=support_type
           )
           self._cargo_segments[name] = segment
           return segment

       def get_cargo_segment(self, name: str) -> "CargoSegment":
           """Get cargo segment by name."""
           return self._cargo_segments[name]

       def get_cargo_segments(self) -> List["CargoSegment"]:
           """Get all cargo segments."""
           return list(self._cargo_segments.values())
   ```

**Acceptance Criteria:**
- [ ] Vessel class exists with name, length, beam, depth attributes
- [ ] Frame spacing and frame_zero_x are configurable
- [ ] frame_to_x() converts frame number to X-coordinate
- [ ] x_to_frame() converts X-coordinate to frame number
- [ ] add_material() stores material properties
- [ ] add_cargo_segment() creates and returns CargoSegment
- [ ] Segments can be defined by frame number OR x-coordinate
- [ ] UUID is auto-generated for database compatibility
- [ ] Type hints and docstrings are complete

---

### Task 21.2: Create CargoSegment Class

**Requirements:** New feature
**Dependencies:** Task 21.1
**Difficulty:** Medium

**Description:**
Create the `CargoSegment` class that represents a bounded region of the vessel for cargo loading.

**Steps:**

1. Create `src/grillex/vessel/geometry/cargo_segment.py`:
   ```python
   from dataclasses import dataclass, field
   from typing import List, Dict, Optional, TYPE_CHECKING
   from uuid import uuid4, UUID

   if TYPE_CHECKING:
       from .vessel import Vessel
       from .components import (
           TransverseBulkhead, LongitudinalBulkhead, SideShell,
           Deck, WebFrame, LongitudinalGirder, TransverseGirder
       )

   @dataclass
   class CargoSegment:
       """
       A cargo deck segment bounded by transverse bulkheads.

       Attributes:
           name: Segment identifier
           vessel: Parent vessel reference
           x_start: Aft boundary X-coordinate in meters
           x_end: Forward boundary X-coordinate in meters
           y_start: Starboard boundary (negative Y)
           y_end: Port boundary (positive Y)
           support_type: "bottom" or "columns"
       """
       name: str
       vessel: "Vessel"
       x_start: float
       x_end: float
       y_start: float
       y_end: float
       support_type: str = "bottom"  # "bottom" or "columns"

       id: UUID = field(default_factory=uuid4)

       # Components
       _transverse_bulkhead_aft: "TransverseBulkhead" = None
       _transverse_bulkhead_fwd: "TransverseBulkhead" = None
       _longitudinal_bulkheads: List["LongitudinalBulkhead"] = field(default_factory=list)
       _side_shells: Dict[str, "SideShell"] = field(default_factory=dict)
       _decks: Dict[str, "Deck"] = field(default_factory=dict)
       _webframes: List["WebFrame"] = field(default_factory=list)
       _longitudinal_girders: List["LongitudinalGirder"] = field(default_factory=list)
       _transverse_girders: List["TransverseGirder"] = field(default_factory=list)

       @property
       def frame_start(self) -> int:
           """Starting frame number."""
           return self.vessel.x_to_frame(self.x_start)

       @property
       def frame_end(self) -> int:
           """Ending frame number."""
           return self.vessel.x_to_frame(self.x_end)

       @property
       def length(self) -> float:
           """Segment length in meters."""
           return self.x_end - self.x_start

       @property
       def width(self) -> float:
           """Segment width in meters."""
           return self.y_end - self.y_start

       @property
       def transverse_bulkhead_aft(self) -> "TransverseBulkhead":
           """Aft transverse bulkhead (auto-created if accessed)."""
           if self._transverse_bulkhead_aft is None:
               from .components import TransverseBulkhead
               self._transverse_bulkhead_aft = TransverseBulkhead(
                   segment=self, x=self.x_start
               )
           return self._transverse_bulkhead_aft

       @property
       def transverse_bulkhead_fwd(self) -> "TransverseBulkhead":
           """Forward transverse bulkhead (auto-created if accessed)."""
           if self._transverse_bulkhead_fwd is None:
               from .components import TransverseBulkhead
               self._transverse_bulkhead_fwd = TransverseBulkhead(
                   segment=self, x=self.x_end
               )
           return self._transverse_bulkhead_fwd

       def add_longitudinal_bulkhead(
           self,
           y: float,
           x_start: float = None,
           x_end: float = None
       ) -> "LongitudinalBulkhead":
           """
           Add a longitudinal bulkhead.

           Args:
               y: Y-coordinate (e.g., 0.0 for centerline)
               x_start: Start X (default: segment start)
               x_end: End X (default: segment end)
           """
           from .components import LongitudinalBulkhead
           bhd = LongitudinalBulkhead(
               segment=self,
               y=y,
               x_start=x_start or self.x_start,
               x_end=x_end or self.x_end
           )
           self._longitudinal_bulkheads.append(bhd)
           return bhd

       def add_side_shell(
           self,
           y: float,
           thickness: float = None,
           material: str = None
       ) -> "SideShell":
           """
           Add a side shell (port or starboard boundary).

           Args:
               y: Y-coordinate (negative for starboard, positive for port)
               thickness: Plate thickness in meters
               material: Material name
           """
           from .components import SideShell
           side = "port" if y > 0 else "starboard"
           shell = SideShell(segment=self, y=y, side=side)
           if thickness:
               shell.set_plating(thickness=thickness, material=material)
           self._side_shells[side] = shell
           return shell

       def add_deck(
           self,
           z: float,
           name: str = None
       ) -> "Deck":
           """
           Add a deck at given height.

           Args:
               z: Deck height in meters
               name: Deck name (e.g., "main_deck", "tween_deck")
           """
           from .components import Deck
           if name is None:
               name = f"deck_z{z:.1f}"
           deck = Deck(segment=self, z=z, name=name)
           self._decks[name] = deck
           return deck

       def add_webframe(
           self,
           x: float = None,
           frame: int = None,
           type: str = "beam",
           section: str = None
       ) -> "WebFrame":
           """
           Add a transverse webframe.

           Args:
               x: X-coordinate (or use frame)
               frame: Frame number (or use x)
               type: "beam" (T/L profile) or "plate" (stiffened plate)
               section: Section name (for beam type)
           """
           from .components import WebFrame
           if frame is not None:
               x = self.vessel.frame_to_x(frame)
           wf = WebFrame(segment=self, x=x, type=type, section=section)
           self._webframes.append(wf)
           return wf

       def add_longitudinal_girder(
           self,
           y: float,
           z: float,
           section: str
       ) -> "LongitudinalGirder":
           """
           Add a longitudinal girder.

           Args:
               y: Y-coordinate
               z: Z-coordinate (deck level)
               section: Section name
           """
           from .components import LongitudinalGirder
           girder = LongitudinalGirder(
               segment=self, y=y, z=z, section=section
           )
           self._longitudinal_girders.append(girder)
           return girder

       def add_transverse_girder(
           self,
           x: float = None,
           frame: int = None,
           z: float = None,
           section: str = None
       ) -> "TransverseGirder":
           """
           Add a transverse girder.

           Args:
               x: X-coordinate (or use frame)
               frame: Frame number
               z: Z-coordinate (deck level)
               section: Section name
           """
           from .components import TransverseGirder
           if frame is not None:
               x = self.vessel.frame_to_x(frame)
           girder = TransverseGirder(
               segment=self, x=x, z=z, section=section
           )
           self._transverse_girders.append(girder)
           return girder

       def get_decks(self) -> List["Deck"]:
           """Get all decks."""
           return list(self._decks.values())

       def get_webframes(self) -> List["WebFrame"]:
           """Get all webframes."""
           return list(self._webframes)

       def get_longitudinal_girders(self) -> List["LongitudinalGirder"]:
           """Get all longitudinal girders."""
           return list(self._longitudinal_girders)
   ```

**Acceptance Criteria:**
- [ ] CargoSegment has x_start, x_end, y_start, y_end bounds
- [ ] frame_start and frame_end properties work correctly
- [ ] length and width properties are computed
- [ ] Transverse bulkheads at boundaries are auto-created on access
- [ ] add_longitudinal_bulkhead() creates bulkhead at y-position
- [ ] add_side_shell() creates port/starboard shell
- [ ] add_deck() creates deck at z-level
- [ ] add_webframe() supports frame number OR x-coordinate
- [ ] add_webframe() supports "beam" and "plate" types
- [ ] add_longitudinal_girder() creates girder with section
- [ ] add_transverse_girder() creates girder with section
- [ ] UUID is auto-generated for each segment

---

### Task 21.3: Create PlateField Class

**Requirements:** New feature
**Dependencies:** Task 21.1
**Difficulty:** Medium

**Description:**
Create the `PlateField` class to represent plate regions with optional varying thickness.

**Steps:**

1. Create `src/grillex/vessel/geometry/plate_field.py`:
   ```python
   from dataclasses import dataclass, field
   from typing import List, Optional, Tuple
   from uuid import uuid4, UUID

   @dataclass
   class ThicknessRegion:
       """A region with specific plate thickness."""
       y_start: float
       y_end: float
       z_start: float
       z_end: float
       thickness: float
       material: Optional[str] = None

       def contains(self, y: float, z: float) -> bool:
           """Check if point (y, z) is within this region."""
           return (self.y_start <= y <= self.y_end and
                   self.z_start <= z <= self.z_end)

   @dataclass
   class PlateField:
       """
       A plate region with optional stiffeners and varying thickness.

       Used by bulkheads, side shells, decks, and plate-type webframes.

       Attributes:
           default_thickness: Default plate thickness in meters
           default_material: Default material name
           thickness_regions: Regions with different thickness

       Example:
           >>> plate = PlateField()
           >>> plate.set_default(thickness=0.012, material="S355")
           >>> plate.add_region(y_start=-4, y_end=4, z_start=0, z_end=5,
           ...                  thickness=0.016)  # Thicker center
       """
       default_thickness: Optional[float] = None
       default_material: Optional[str] = None
       thickness_regions: List[ThicknessRegion] = field(default_factory=list)

       id: UUID = field(default_factory=uuid4)
       _stiffeners: List["Stiffener"] = field(default_factory=list)

       def set_default(
           self,
           thickness: float,
           material: str = None
       ) -> "PlateField":
           """Set default thickness and material."""
           self.default_thickness = thickness
           self.default_material = material
           return self

       def add_region(
           self,
           y_start: float,
           y_end: float,
           z_start: float,
           z_end: float,
           thickness: float,
           material: str = None
       ) -> "PlateField":
           """
           Add a region with different thickness.

           Regions override the default thickness in their bounds.
           Later regions take precedence over earlier ones.
           """
           region = ThicknessRegion(
               y_start=y_start, y_end=y_end,
               z_start=z_start, z_end=z_end,
               thickness=thickness,
               material=material
           )
           self.thickness_regions.append(region)
           return self

       def get_thickness_at(self, y: float, z: float) -> float:
           """Get plate thickness at a point (checks regions, falls back to default)."""
           # Check regions in reverse order (later takes precedence)
           for region in reversed(self.thickness_regions):
               if region.contains(y, z):
                   return region.thickness
           return self.default_thickness

       def get_material_at(self, y: float, z: float) -> str:
           """Get material at a point (checks regions, falls back to default)."""
           for region in reversed(self.thickness_regions):
               if region.contains(y, z):
                   if region.material:
                       return region.material
           return self.default_material

       def add_stiffener(
           self,
           position: float,
           section: str,
           start: float = None,
           end: float = None
       ) -> "Stiffener":
           """
           Add a stiffener to this plate field.

           Direction is inferred from which coordinate is fixed:
           - On XY plane (deck): position is Y → stiffener runs in X
           - On YZ plane (trans. bhd): position is Y or Z → runs perpendicular
           - On XZ plane (long. bhd): position is X or Z → runs perpendicular

           Args:
               position: Fixed coordinate (Y for longitudinal, Z for vertical, etc.)
               section: Section/profile name
               start: Start coordinate along stiffener (optional)
               end: End coordinate along stiffener (optional)
           """
           from .stiffener import Stiffener
           stiff = Stiffener(
               plate_field=self,
               position=position,
               section=section,
               start=start,
               end=end
           )
           self._stiffeners.append(stiff)
           return stiff

       def add_stiffeners(
           self,
           spacing: float,
           section: str,
           start_pos: float,
           end_pos: float,
           direction: str = None
       ) -> List["Stiffener"]:
           """
           Add multiple evenly-spaced stiffeners.

           Args:
               spacing: Distance between stiffeners
               section: Section/profile name
               start_pos: Start of stiffener range
               end_pos: End of stiffener range
               direction: Optional explicit direction hint

           Returns:
               List of created stiffeners
           """
           stiffeners = []
           pos = start_pos + spacing  # First stiffener at start + spacing
           while pos < end_pos - spacing/2:  # Don't place at end
               stiff = self.add_stiffener(position=pos, section=section)
               stiffeners.append(stiff)
               pos += spacing
           return stiffeners

       def get_stiffeners(self) -> List["Stiffener"]:
           """Get all stiffeners on this plate field."""
           return list(self._stiffeners)
   ```

**Acceptance Criteria:**
- [ ] PlateField has default_thickness and default_material
- [ ] ThicknessRegion defines rectangular region with thickness
- [ ] add_region() adds override region for varying thickness
- [ ] get_thickness_at() returns correct thickness (region or default)
- [ ] get_material_at() returns correct material
- [ ] add_stiffener() creates single stiffener at position
- [ ] add_stiffeners() creates evenly-spaced stiffeners
- [ ] UUID is auto-generated

---

### Task 21.4: Create Stiffener Class

**Requirements:** New feature
**Dependencies:** Task 21.3
**Difficulty:** Low

**Description:**
Create the `Stiffener` class with automatic direction derivation based on plate orientation.

**Steps:**

1. Create `src/grillex/vessel/geometry/stiffener.py`:
   ```python
   from dataclasses import dataclass, field
   from typing import Optional, TYPE_CHECKING, Literal
   from uuid import uuid4, UUID
   from enum import Enum

   if TYPE_CHECKING:
       from .plate_field import PlateField

   class StiffenerDirection(Enum):
       """Direction a stiffener runs."""
       LONGITUDINAL = "longitudinal"  # Runs in X (fore-aft)
       TRANSVERSE = "transverse"      # Runs in Y (port-starboard)
       VERTICAL = "vertical"          # Runs in Z (up-down)

   @dataclass
   class Stiffener:
       """
       A stiffener attached to a plate field.

       Direction is derived from the position coordinate and plate orientation.

       Attributes:
           plate_field: Parent plate field
           position: Fixed coordinate value (Y, Z, etc.)
           section: Section/profile name
           start: Start coordinate along stiffener
           end: End coordinate along stiffener
       """
       plate_field: "PlateField"
       position: float
       section: str
       start: Optional[float] = None
       end: Optional[float] = None

       id: UUID = field(default_factory=uuid4)
       _direction: Optional[StiffenerDirection] = None

       @property
       def direction(self) -> StiffenerDirection:
           """
           Get stiffener direction.

           If not explicitly set, derived from plate field orientation
           and which coordinate is fixed.
           """
           if self._direction:
               return self._direction
           # Default to longitudinal for now
           # Actual derivation happens in component context
           return StiffenerDirection.LONGITUDINAL

       def set_direction(self, direction: StiffenerDirection) -> "Stiffener":
           """Explicitly set stiffener direction."""
           self._direction = direction
           return self
   ```

**Acceptance Criteria:**
- [ ] Stiffener has position, section, start, end attributes
- [ ] StiffenerDirection enum has LONGITUDINAL, TRANSVERSE, VERTICAL
- [ ] direction property returns direction (explicit or derived)
- [ ] set_direction() allows explicit override
- [ ] UUID is auto-generated

---

### Task 21.5: Create Component Classes

**Requirements:** New feature
**Dependencies:** Tasks 21.2, 21.3, 21.4
**Difficulty:** High

**Description:**
Create the structural component classes: TransverseBulkhead, LongitudinalBulkhead, SideShell, Deck, WebFrame, LongitudinalGirder, TransverseGirder.

**Steps:**

1. Create `src/grillex/vessel/geometry/components.py`:
   ```python
   from dataclasses import dataclass, field
   from typing import Optional, List, TYPE_CHECKING, Literal
   from uuid import uuid4, UUID

   from .plate_field import PlateField
   from .stiffener import Stiffener, StiffenerDirection

   if TYPE_CHECKING:
       from .cargo_segment import CargoSegment


   @dataclass
   class TransverseBulkhead:
       """
       A transverse bulkhead (YZ plane) at a given X-coordinate.

       Transverse bulkheads are watertight vertical walls that bound
       cargo segments and provide transverse strength.
       """
       segment: "CargoSegment"
       x: float

       id: UUID = field(default_factory=uuid4)
       _plate_field: PlateField = field(default_factory=PlateField)

       @property
       def frame(self) -> int:
           """Frame number at this bulkhead."""
           return self.segment.vessel.x_to_frame(self.x)

       @property
       def plate_field(self) -> PlateField:
           """Plate field for this bulkhead."""
           return self._plate_field

       def set_plating(
           self,
           thickness: float,
           material: str = None
       ) -> "TransverseBulkhead":
           """Set default plating thickness and material."""
           self._plate_field.set_default(thickness, material)
           return self

       def set_plating_region(
           self,
           y_start: float,
           y_end: float,
           z_start: float,
           z_end: float,
           thickness: float,
           material: str = None
       ) -> "TransverseBulkhead":
           """Add a region with different thickness."""
           self._plate_field.add_region(
               y_start, y_end, z_start, z_end, thickness, material
           )
           return self

       def add_stiffener(
           self,
           y: float = None,
           z: float = None,
           section: str = None
       ) -> Stiffener:
           """
           Add a stiffener to this bulkhead.

           Specify y for vertical stiffener, z for horizontal stiffener.
           """
           if y is not None and z is None:
               # Vertical stiffener at y position
               stiff = self._plate_field.add_stiffener(
                   position=y, section=section
               )
               stiff.set_direction(StiffenerDirection.VERTICAL)
           elif z is not None and y is None:
               # Horizontal stiffener at z position
               stiff = self._plate_field.add_stiffener(
                   position=z, section=section
               )
               stiff.set_direction(StiffenerDirection.TRANSVERSE)
           else:
               raise ValueError("Specify either y (vertical) or z (horizontal), not both")
           return stiff

       def add_stiffeners(
           self,
           direction: Literal["vertical", "horizontal"],
           spacing: float,
           section: str
       ) -> List[Stiffener]:
           """Add evenly-spaced stiffeners."""
           if direction == "vertical":
               stiffs = self._plate_field.add_stiffeners(
                   spacing=spacing,
                   section=section,
                   start_pos=self.segment.y_start,
                   end_pos=self.segment.y_end
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.VERTICAL)
           else:  # horizontal
               stiffs = self._plate_field.add_stiffeners(
                   spacing=spacing,
                   section=section,
                   start_pos=0.0,  # Bottom
                   end_pos=self.segment.vessel.depth
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.TRANSVERSE)
           return stiffs


   @dataclass
   class LongitudinalBulkhead:
       """
       A longitudinal bulkhead (XZ plane) at a given Y-coordinate.

       Longitudinal bulkheads run fore-aft and provide longitudinal strength.
       Common at centerline (y=0) or at hold boundaries.
       """
       segment: "CargoSegment"
       y: float
       x_start: float
       x_end: float

       id: UUID = field(default_factory=uuid4)
       _plate_field: PlateField = field(default_factory=PlateField)

       @property
       def plate_field(self) -> PlateField:
           return self._plate_field

       def set_plating(
           self,
           thickness: float,
           material: str = None
       ) -> "LongitudinalBulkhead":
           self._plate_field.set_default(thickness, material)
           return self

       def set_plating_region(
           self,
           x_start: float,
           x_end: float,
           z_start: float,
           z_end: float,
           thickness: float,
           material: str = None
       ) -> "LongitudinalBulkhead":
           # Note: using x,z for longitudinal bulkhead regions
           self._plate_field.add_region(
               y_start=x_start, y_end=x_end,  # Reusing y for x in storage
               z_start=z_start, z_end=z_end,
               thickness=thickness, material=material
           )
           return self

       def add_stiffener(
           self,
           x: float = None,
           z: float = None,
           section: str = None
       ) -> Stiffener:
           """Add stiffener: x for vertical, z for longitudinal."""
           if x is not None and z is None:
               stiff = self._plate_field.add_stiffener(position=x, section=section)
               stiff.set_direction(StiffenerDirection.VERTICAL)
           elif z is not None and x is None:
               stiff = self._plate_field.add_stiffener(position=z, section=section)
               stiff.set_direction(StiffenerDirection.LONGITUDINAL)
           else:
               raise ValueError("Specify either x (vertical) or z (horizontal)")
           return stiff

       def add_stiffeners(
           self,
           direction: Literal["vertical", "horizontal"],
           spacing: float,
           section: str
       ) -> List[Stiffener]:
           if direction == "vertical":
               stiffs = self._plate_field.add_stiffeners(
                   spacing=spacing, section=section,
                   start_pos=self.x_start, end_pos=self.x_end
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.VERTICAL)
           else:
               stiffs = self._plate_field.add_stiffeners(
                   spacing=spacing, section=section,
                   start_pos=0.0, end_pos=self.segment.vessel.depth
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.LONGITUDINAL)
           return stiffs


   @dataclass
   class SideShell:
       """
       Side shell (port or starboard boundary).

       Modeled as a vertical plate at the vessel side.
       """
       segment: "CargoSegment"
       y: float
       side: Literal["port", "starboard"]

       id: UUID = field(default_factory=uuid4)
       _plate_field: PlateField = field(default_factory=PlateField)

       @property
       def plate_field(self) -> PlateField:
           return self._plate_field

       def set_plating(
           self,
           thickness: float,
           material: str = None
       ) -> "SideShell":
           self._plate_field.set_default(thickness, material)
           return self

       def add_stiffener(
           self,
           x: float = None,
           z: float = None,
           section: str = None
       ) -> Stiffener:
           """Add stiffener: x for vertical, z for longitudinal."""
           if x is not None:
               stiff = self._plate_field.add_stiffener(position=x, section=section)
               stiff.set_direction(StiffenerDirection.VERTICAL)
           elif z is not None:
               stiff = self._plate_field.add_stiffener(position=z, section=section)
               stiff.set_direction(StiffenerDirection.LONGITUDINAL)
           else:
               raise ValueError("Specify either x or z")
           return stiff

       def add_stiffeners(
           self,
           direction: Literal["vertical", "longitudinal"],
           spacing: float,
           section: str
       ) -> List[Stiffener]:
           if direction == "vertical":
               stiffs = self._plate_field.add_stiffeners(
                   spacing=spacing, section=section,
                   start_pos=self.segment.x_start,
                   end_pos=self.segment.x_end
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.VERTICAL)
           else:
               stiffs = self._plate_field.add_stiffeners(
                   spacing=spacing, section=section,
                   start_pos=0.0, end_pos=self.segment.vessel.depth
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.LONGITUDINAL)
           return stiffs


   @dataclass
   class Deck:
       """
       A horizontal deck at a given Z-level.

       Decks carry cargo loads and are supported by webframes and girders.
       """
       segment: "CargoSegment"
       z: float
       name: str

       id: UUID = field(default_factory=uuid4)
       _plate_field: PlateField = field(default_factory=PlateField)

       @property
       def plate_field(self) -> PlateField:
           return self._plate_field

       def set_plating(
           self,
           thickness: float,
           material: str = None
       ) -> "Deck":
           self._plate_field.set_default(thickness, material)
           return self

       def set_plating_region(
           self,
           x_start: float,
           x_end: float,
           y_start: float,
           y_end: float,
           thickness: float,
           material: str = None
       ) -> "Deck":
           self._plate_field.add_region(
               y_start=x_start, y_end=x_end,  # x maps to first coord
               z_start=y_start, z_end=y_end,  # y maps to second coord
               thickness=thickness, material=material
           )
           return self

       def add_stiffener(
           self,
           x: float = None,
           y: float = None,
           section: str = None
       ) -> Stiffener:
           """Add stiffener: y for longitudinal (runs in X), x for transverse."""
           if y is not None and x is None:
               stiff = self._plate_field.add_stiffener(position=y, section=section)
               stiff.set_direction(StiffenerDirection.LONGITUDINAL)
           elif x is not None and y is None:
               stiff = self._plate_field.add_stiffener(position=x, section=section)
               stiff.set_direction(StiffenerDirection.TRANSVERSE)
           else:
               raise ValueError("Specify either y (longitudinal) or x (transverse)")
           return stiff

       def add_stiffeners(
           self,
           direction: Literal["longitudinal", "transverse"],
           spacing: float,
           section: str
       ) -> List[Stiffener]:
           if direction == "longitudinal":
               stiffs = self._plate_field.add_stiffeners(
                   spacing=spacing, section=section,
                   start_pos=self.segment.y_start,
                   end_pos=self.segment.y_end
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.LONGITUDINAL)
           else:
               stiffs = self._plate_field.add_stiffeners(
                   spacing=spacing, section=section,
                   start_pos=self.segment.x_start,
                   end_pos=self.segment.x_end
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.TRANSVERSE)
           return stiffs


   @dataclass
   class WebFrame:
       """
       A transverse webframe at a given X-coordinate.

       Can be modeled as:
       - "beam": T or L profile (default)
       - "plate": Stiffened plate field
       """
       segment: "CargoSegment"
       x: float
       type: Literal["beam", "plate"] = "beam"
       section: Optional[str] = None  # For beam type

       id: UUID = field(default_factory=uuid4)
       _plate_field: Optional[PlateField] = None  # For plate type

       @property
       def frame(self) -> int:
           """Frame number at this webframe."""
           return self.segment.vessel.x_to_frame(self.x)

       @property
       def plate_field(self) -> PlateField:
           """Plate field (only for plate type)."""
           if self.type != "plate":
               raise ValueError("plate_field only available for plate-type webframes")
           if self._plate_field is None:
               self._plate_field = PlateField()
           return self._plate_field

       def set_plating(
           self,
           thickness: float,
           material: str = None
       ) -> "WebFrame":
           """Set plating (converts to plate type if needed)."""
           if self.type == "beam":
               self.type = "plate"
           self.plate_field.set_default(thickness, material)
           return self

       def add_stiffeners(
           self,
           direction: Literal["vertical", "horizontal"],
           spacing: float,
           section: str
       ) -> List[Stiffener]:
           """Add stiffeners (only for plate type)."""
           if self.type != "plate":
               raise ValueError("Stiffeners only for plate-type webframes")
           if direction == "vertical":
               stiffs = self.plate_field.add_stiffeners(
                   spacing=spacing, section=section,
                   start_pos=self.segment.y_start,
                   end_pos=self.segment.y_end
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.VERTICAL)
           else:
               stiffs = self.plate_field.add_stiffeners(
                   spacing=spacing, section=section,
                   start_pos=0.0, end_pos=self.segment.vessel.depth
               )
               for s in stiffs:
                   s.set_direction(StiffenerDirection.TRANSVERSE)
           return stiffs


   @dataclass
   class LongitudinalGirder:
       """
       A major longitudinal girder (runs fore-aft).

       Modeled as a beam element under the deck.
       """
       segment: "CargoSegment"
       y: float
       z: float
       section: str

       id: UUID = field(default_factory=uuid4)

       @property
       def x_start(self) -> float:
           return self.segment.x_start

       @property
       def x_end(self) -> float:
           return self.segment.x_end


   @dataclass
   class TransverseGirder:
       """
       A major transverse girder (runs port-starboard).

       Modeled as a beam element under the deck.
       """
       segment: "CargoSegment"
       x: float
       z: float
       section: str

       id: UUID = field(default_factory=uuid4)

       @property
       def frame(self) -> int:
           return self.segment.vessel.x_to_frame(self.x)

       @property
       def y_start(self) -> float:
           return self.segment.y_start

       @property
       def y_end(self) -> float:
           return self.segment.y_end
   ```

**Acceptance Criteria:**
- [ ] TransverseBulkhead at x-position with frame number property
- [ ] TransverseBulkhead.set_plating() and set_plating_region() work
- [ ] TransverseBulkhead.add_stiffener() with y (vertical) or z (horizontal)
- [ ] TransverseBulkhead.add_stiffeners() for multiple stiffeners
- [ ] LongitudinalBulkhead at y-position with x_start, x_end
- [ ] LongitudinalBulkhead stiffener methods work
- [ ] SideShell with side property (port/starboard)
- [ ] Deck at z-level with name
- [ ] Deck.add_stiffener() with y (longitudinal) or x (transverse)
- [ ] WebFrame with type "beam" or "plate"
- [ ] WebFrame.set_plating() converts to plate type
- [ ] LongitudinalGirder with y, z, section
- [ ] TransverseGirder with x, z, section
- [ ] All components have auto-generated UUID

---

### Task 21.6: Create Reference Vessel for Testing

**Requirements:** Testing infrastructure
**Dependencies:** Tasks 21.1-21.5
**Difficulty:** Low

**Description:**
Create a reference "Standard Barge 100x24" vessel definition for testing throughout development.

**Steps:**

1. Create `src/grillex/vessel/reference_vessels.py`:
   ```python
   """Reference vessel definitions for testing."""

   from .geometry import Vessel

   def create_standard_barge_100x24() -> Vessel:
       """
       Create a standard 100m x 24m barge for testing.

       Geometry:
       - Length: 100m
       - Beam: 24m
       - Depth: 6m
       - Frame spacing: 2.5m (40 frames)
       - Cargo segment: Frames 10-35 (62.5m length)
       - Main deck at z=6.0m
       - Centerline longitudinal bulkhead
       - Webframes every 2 frames
       - HP200x10 deck stiffeners at 0.6m spacing

       Returns:
           Configured Vessel object
       """
       vessel = Vessel(
           name="Standard Barge 100x24",
           length=100.0,
           beam=24.0,
           depth=6.0,
           frame_spacing=2.5
       )

       # Add material
       vessel.add_material("S355", E=210e6, yield_stress=355e3)

       # Create cargo segment
       segment = vessel.add_cargo_segment(
           name="main_cargo",
           frame_start=10,
           frame_end=35,
           support_type="bottom"
       )

       # Configure transverse bulkheads
       segment.transverse_bulkhead_aft.set_plating(thickness=0.014, material="S355")
       segment.transverse_bulkhead_aft.add_stiffeners(
           direction="vertical", spacing=0.6, section="HP180x10"
       )

       segment.transverse_bulkhead_fwd.set_plating(thickness=0.014, material="S355")
       segment.transverse_bulkhead_fwd.add_stiffeners(
           direction="vertical", spacing=0.6, section="HP180x10"
       )

       # Side shells
       segment.add_side_shell(y=-12.0, thickness=0.012, material="S355")
       segment.add_side_shell(y=12.0, thickness=0.012, material="S355")

       # Centerline longitudinal bulkhead
       cl_bhd = segment.add_longitudinal_bulkhead(y=0.0)
       cl_bhd.set_plating(thickness=0.012, material="S355")
       cl_bhd.add_stiffeners(direction="horizontal", spacing=0.6, section="HP160x9")

       # Main deck
       main_deck = segment.add_deck(z=6.0, name="main_deck")
       main_deck.set_plating(thickness=0.016, material="S355")
       main_deck.add_stiffeners(
           direction="longitudinal", spacing=0.6, section="HP200x10"
       )

       # Longitudinal girders
       segment.add_longitudinal_girder(y=0.0, z=6.0, section="T500x200x12x20")  # CL
       segment.add_longitudinal_girder(y=-6.0, z=6.0, section="T400x150x10x16")
       segment.add_longitudinal_girder(y=6.0, z=6.0, section="T400x150x10x16")

       # Webframes every 2 frames
       for frame in range(12, 35, 2):
           segment.add_webframe(frame=frame, section="T300x150x10x14")

       return vessel
   ```

2. Create `tests/python/test_phase21_vessel_geometry.py`:
   ```python
   """Tests for Phase 21: Vessel Geometry Core Data Structures."""

   import pytest
   from grillex.vessel import Vessel, CargoSegment
   from grillex.vessel.reference_vessels import create_standard_barge_100x24


   class TestVessel:
       """Tests for Vessel class."""

       def test_create_vessel(self):
           """Vessel can be created with dimensions."""
           vessel = Vessel(
               name="Test Barge",
               length=100.0,
               beam=24.0,
               depth=6.0
           )
           assert vessel.name == "Test Barge"
           assert vessel.length == 100.0
           assert vessel.beam == 24.0
           assert vessel.depth == 6.0

       def test_frame_to_x(self):
           """frame_to_x converts frame number to X-coordinate."""
           vessel = Vessel(
               name="Test", length=100, beam=24, depth=6,
               frame_spacing=2.5, frame_zero_x=0.0
           )
           assert vessel.frame_to_x(0) == 0.0
           assert vessel.frame_to_x(10) == 25.0
           assert vessel.frame_to_x(40) == 100.0

       def test_x_to_frame(self):
           """x_to_frame converts X-coordinate to frame number."""
           vessel = Vessel(
               name="Test", length=100, beam=24, depth=6,
               frame_spacing=2.5
           )
           assert vessel.x_to_frame(0.0) == 0
           assert vessel.x_to_frame(25.0) == 10
           assert vessel.x_to_frame(26.0) == 10  # Rounds to nearest

       def test_add_cargo_segment_by_frame(self):
           """Cargo segment can be defined by frame numbers."""
           vessel = Vessel(
               name="Test", length=100, beam=24, depth=6,
               frame_spacing=2.5
           )
           segment = vessel.add_cargo_segment(
               frame_start=10, frame_end=30
           )
           assert segment.x_start == 25.0
           assert segment.x_end == 75.0

       def test_add_cargo_segment_by_coordinate(self):
           """Cargo segment can be defined by X-coordinates."""
           vessel = Vessel(
               name="Test", length=100, beam=24, depth=6
           )
           segment = vessel.add_cargo_segment(
               x_start=20.0, x_end=80.0
           )
           assert segment.x_start == 20.0
           assert segment.x_end == 80.0


   class TestCargoSegment:
       """Tests for CargoSegment class."""

       def test_transverse_bulkheads_auto_created(self):
           """Transverse bulkheads are auto-created on access."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           segment = vessel.add_cargo_segment(x_start=20, x_end=80)

           aft = segment.transverse_bulkhead_aft
           fwd = segment.transverse_bulkhead_fwd

           assert aft.x == 20.0
           assert fwd.x == 80.0

       def test_add_deck(self):
           """Deck can be added at z-level."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           segment = vessel.add_cargo_segment(x_start=20, x_end=80)

           deck = segment.add_deck(z=6.0, name="main_deck")
           assert deck.z == 6.0
           assert deck.name == "main_deck"

       def test_add_webframe_by_frame(self):
           """Webframe can be added by frame number."""
           vessel = Vessel(
               name="Test", length=100, beam=24, depth=6,
               frame_spacing=2.5
           )
           segment = vessel.add_cargo_segment(frame_start=10, frame_end=30)

           wf = segment.add_webframe(frame=15, section="T300x150x10x14")
           assert wf.x == 37.5
           assert wf.frame == 15


   class TestReferenceVessel:
       """Tests for reference vessel."""

       def test_create_standard_barge(self):
           """Standard barge can be created."""
           vessel = create_standard_barge_100x24()

           assert vessel.name == "Standard Barge 100x24"
           assert vessel.length == 100.0
           assert vessel.beam == 24.0

       def test_standard_barge_has_cargo_segment(self):
           """Standard barge has main cargo segment."""
           vessel = create_standard_barge_100x24()

           segment = vessel.get_cargo_segment("main_cargo")
           assert segment is not None
           assert segment.frame_start == 10
           assert segment.frame_end == 35

       def test_standard_barge_has_deck(self):
           """Standard barge has main deck."""
           vessel = create_standard_barge_100x24()
           segment = vessel.get_cargo_segment("main_cargo")

           decks = segment.get_decks()
           assert len(decks) == 1
           assert decks[0].name == "main_deck"
           assert decks[0].z == 6.0

       def test_standard_barge_has_webframes(self):
           """Standard barge has webframes."""
           vessel = create_standard_barge_100x24()
           segment = vessel.get_cargo_segment("main_cargo")

           webframes = segment.get_webframes()
           # Frames 12, 14, 16, ..., 34 = 12 webframes
           assert len(webframes) == 12
   ```

**Acceptance Criteria:**
- [ ] Reference vessel function create_standard_barge_100x24() exists
- [ ] Reference vessel has realistic dimensions and components
- [ ] Reference vessel includes all component types (bulkheads, deck, webframes, girders)
- [ ] Basic tests pass for Vessel, CargoSegment, and reference vessel
- [ ] Tests verify frame-to-coordinate conversion
- [ ] Tests verify component creation methods

---

### Task 21.7: Export from Main Package

**Requirements:** API completeness
**Dependencies:** Tasks 21.1-21.6
**Difficulty:** Low

**Description:**
Ensure all vessel geometry classes are properly exported from the package.

**Steps:**

1. Update `src/grillex/vessel/__init__.py`:
   ```python
   """
   Vessel geometry module for offshore structural modeling.

   This module provides classes for defining vessel geometry that can be
   converted to Grillex StructuralModel for FEM analysis.

   Example:
       >>> from grillex.vessel import Vessel
       >>> vessel = Vessel(
       ...     name="Transport Barge",
       ...     length=100.0,
       ...     beam=24.0,
       ...     depth=6.0,
       ...     frame_spacing=2.5
       ... )
       >>> segment = vessel.add_cargo_segment(frame_start=10, frame_end=35)
       >>> deck = segment.add_deck(z=6.0, name="main_deck")
   """

   from .geometry import (
       Vessel,
       CargoSegment,
       TransverseBulkhead,
       LongitudinalBulkhead,
       SideShell,
       Deck,
       WebFrame,
       LongitudinalGirder,
       TransverseGirder,
       PlateField,
       ThicknessRegion,
       Stiffener,
       StiffenerDirection,
   )
   from .reference_vessels import create_standard_barge_100x24

   __all__ = [
       "Vessel",
       "CargoSegment",
       "TransverseBulkhead",
       "LongitudinalBulkhead",
       "SideShell",
       "Deck",
       "WebFrame",
       "LongitudinalGirder",
       "TransverseGirder",
       "PlateField",
       "ThicknessRegion",
       "Stiffener",
       "StiffenerDirection",
       "create_standard_barge_100x24",
   ]
   ```

2. Update `src/grillex/__init__.py` to include vessel module:
   ```python
   # Add to existing imports
   from . import vessel
   ```

**Acceptance Criteria:**
- [ ] All geometry classes importable from `grillex.vessel`
- [ ] Reference vessel function is exported
- [ ] Module docstring with usage example
- [ ] `__all__` list is complete

---

### Summary

| Task | Description | Difficulty | Status |
|------|-------------|------------|--------|
| 21.1 | Vessel class with frame system | Medium | Pending |
| 21.2 | CargoSegment class | Medium | Pending |
| 21.3 | PlateField class | Medium | Pending |
| 21.4 | Stiffener class | Low | Pending |
| 21.5 | Component classes | High | Pending |
| 21.6 | Reference vessel for testing | Low | Pending |
| 21.7 | Package exports | Low | Pending |

**Total Acceptance Criteria:** 47 items

---

## Notes for Future Extraction

When moving this module to a separate repository:

1. **Dependencies to manage:**
   - Only `conversion/to_structural.py` depends on Grillex core
   - Geometry classes are fully self-contained

2. **Package structure:**
   - Move entire `src/grillex/vessel/` directory
   - Rename to `vessel_geometry/` or similar
   - Add Grillex as optional dependency for conversion

3. **Database integration:**
   - UUIDs on all components enable PostgreSQL storage
   - YAML/JSON I/O (Phase 24) provides portable format
