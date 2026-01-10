## Phase 25: Vessel Geometry - Conversion to StructuralModel

### Overview

This phase implements the conversion of vessel geometry to a Grillex `StructuralModel` for FEM analysis. This is the key integration point between the vessel geometry module and the Grillex FEM core.

**Key Concepts:**
- **Single-element panels**: Initial mesh with one plate element per panel
- **Offset beams**: Stiffeners modeled as beam elements offset from plate
- **Bottom support**: Vertical supports at bottom for water-supported vessels
- **Node merging**: Automatic connection at component intersections
- **Bracket plate elements**: Brackets modeled as plate elements for shear capacity
- **Plate-to-beam coupling**: Rigid links connecting plate nodes to beam nodes

**Dependencies:** Phase 21-24, Grillex core (StructuralModel, plate elements, beam elements)

**Directory Structure:**
```
src/grillex/vessel/
└── conversion/
    ├── __init__.py
    └── to_structural.py    # Vessel → StructuralModel
```

---

### Task 25.1: Create Conversion Framework

**Requirements:** New feature
**Dependencies:** Phase 21-24
**Difficulty:** High

**Description:**
Create the framework for converting vessel geometry to StructuralModel.

**Steps:**

1. Create `src/grillex/vessel/conversion/__init__.py`:
   ```python
   from .to_structural import (
       vessel_to_structural_model,
       VesselConversionOptions,
   )

   __all__ = [
       "vessel_to_structural_model",
       "VesselConversionOptions",
   ]
   ```

2. Create `src/grillex/vessel/conversion/to_structural.py`:
   ```python
   """
   Convert vessel geometry to Grillex StructuralModel.

   This module is the integration point between the vessel geometry
   module and the Grillex FEM core. When this module is extracted
   to a separate repository, it becomes the Grillex integration layer.
   """

   from dataclasses import dataclass, field
   from typing import List, Dict, Optional, Tuple, TYPE_CHECKING
   import numpy as np

   if TYPE_CHECKING:
       from ..geometry import Vessel, CargoSegment, Panel
       from grillex.core import StructuralModel

   @dataclass
   class VesselConversionOptions:
       """
       Options for vessel to StructuralModel conversion.

       Attributes:
           mesh_size: Target mesh size in meters
               - "panel": One element per panel (default)
               - float: Target element size for finer meshing
           stiffener_attachment: How stiffeners connect to plates
               - "offset": Beam elements offset from plate (default)
               - "shared_nodes": Beams share nodes with plate
           include_bottom: Model bottom plating (for SSCV deckbox)
           bottom_support_stiffness: Vertical support stiffness at bottom
               - None: Rigid supports (default for water-supported)
               - float: Spring stiffness in kN/m
       """
       mesh_size: str | float = "panel"
       stiffener_attachment: str = "offset"
       include_bottom: bool = False
       bottom_support_stiffness: Optional[float] = None

       # Advanced options
       node_merge_tolerance: float = 0.001  # meters
       create_rigid_links: bool = True


   class VesselToStructuralConverter:
       """
       Converts vessel geometry to Grillex StructuralModel.

       Example:
           >>> from grillex.vessel import Vessel
           >>> from grillex.vessel.conversion import vessel_to_structural_model
           >>>
           >>> vessel = create_standard_barge_100x24()
           >>> model = vessel_to_structural_model(vessel)
           >>> model.analyze()
       """

       def __init__(self, vessel: "Vessel", options: VesselConversionOptions = None):
           self.vessel = vessel
           self.options = options or VesselConversionOptions()

           # Tracking for node merging
           self._node_map: Dict[Tuple[float, float, float], int] = {}
           self._model: Optional["StructuralModel"] = None

       def convert(self) -> "StructuralModel":
           """
           Perform the conversion.

           Returns:
               Configured StructuralModel ready for analysis
           """
           from grillex.core import StructuralModel

           self._model = StructuralModel(name=self.vessel.name)
           self._node_map = {}

           # Add materials from vessel
           self._add_materials()

           # Add sections from vessel profiles
           self._add_sections()

           # Convert each cargo segment
           for segment in self.vessel.get_cargo_segments():
               self._convert_segment(segment)

           return self._model

       def _add_materials(self) -> None:
           """Add vessel materials to structural model."""
           for name, props in self.vessel._materials.items():
               self._model.add_material(
                   name=name,
                   E=props["E"],
                   nu=props["nu"],
                   rho=props["rho"]
               )

       def _add_sections(self) -> None:
           """Add sections from vessel profile library."""
           for name in self.vessel.profiles.list_profiles():
               profile = self.vessel.profiles.get(name)
               self._model.add_section(
                   name=name,
                   A=profile.A,
                   Iy=profile.Iy,
                   Iz=profile.Iz,
                   J=profile.J
               )

       def _convert_segment(self, segment: "CargoSegment") -> None:
           """Convert a cargo segment to FEM elements."""
           # Convert decks (plate elements)
           for deck in segment.get_decks():
               self._convert_deck(deck, segment)

           # Convert webframes (beam elements for beam type)
           for webframe in segment.get_webframes():
               if webframe.type == "beam":
                   self._convert_webframe_beam(webframe, segment)
               else:
                   self._convert_webframe_plate(webframe, segment)

           # Convert longitudinal girders
           for girder in segment.get_longitudinal_girders():
               self._convert_longitudinal_girder(girder, segment)

           # Convert transverse girders
           for girder in segment.get_transverse_girders():
               self._convert_transverse_girder(girder, segment)

           # Add bottom supports
           self._add_bottom_supports(segment)

       def _convert_deck(self, deck, segment: "CargoSegment") -> None:
           """Convert deck to plate elements."""
           from ..geometry.panels import derive_panels_from_plate_field, PanelOrientation

           panels = derive_panels_from_plate_field(
               plate_field=deck.plate_field,
               orientation=PanelOrientation.XY,
               bounds=(
                   segment.x_start, segment.x_end,
                   segment.y_start, segment.y_end
               )
           )

           for panel in panels:
               self._add_plate_element(panel, deck.z)

           # Add deck stiffeners
           self._add_stiffeners_from_plate_field(
               deck.plate_field, deck.z, segment
           )

       def _convert_webframe_beam(self, webframe, segment: "CargoSegment") -> None:
           """Convert beam-type webframe to beam elements."""
           if webframe.section is None:
               return

           # Create beam along the deck at webframe x position
           for deck in segment.get_decks():
               x = webframe.x
               y_start = segment.y_start
               y_end = segment.y_end
               z = deck.z

               # May need to split at longitudinal bulkheads
               self._model.add_beam_by_coords(
                   start_position=[x, y_start, z],
                   end_position=[x, y_end, z],
                   section=webframe.section,
                   material=deck.plate_field.default_material or "S355"
               )

       def _convert_webframe_plate(self, webframe, segment: "CargoSegment") -> None:
           """Convert plate-type webframe to plate elements."""
           from ..geometry.panels import derive_panels_from_plate_field, PanelOrientation

           panels = derive_panels_from_plate_field(
               plate_field=webframe.plate_field,
               orientation=PanelOrientation.YZ,
               bounds=(
                   segment.y_start, segment.y_end,
                   0.0, self.vessel.depth
               )
           )

           for panel in panels:
               self._add_plate_element_yz(panel, webframe.x)

       def _convert_longitudinal_girder(self, girder, segment: "CargoSegment") -> None:
           """Convert longitudinal girder to beam elements."""
           material = "S355"  # Default, could get from segment

           self._model.add_beam_by_coords(
               start_position=[segment.x_start, girder.y, girder.z],
               end_position=[segment.x_end, girder.y, girder.z],
               section=girder.section,
               material=material
           )

       def _convert_transverse_girder(self, girder, segment: "CargoSegment") -> None:
           """Convert transverse girder to beam elements."""
           material = "S355"

           self._model.add_beam_by_coords(
               start_position=[girder.x, segment.y_start, girder.z],
               end_position=[girder.x, segment.y_end, girder.z],
               section=girder.section,
               material=material
           )

       def _add_plate_element(self, panel: "Panel", z: float) -> None:
           """Add plate element for XY panel at z height."""
           corners = panel.get_corners_3d(z)
           material = panel.material or "S355"

           self._model.add_plate(
               corners=corners,
               thickness=panel.thickness,
               material=material,
               mesh_size=None  # Single element
           )

       def _add_plate_element_yz(self, panel: "Panel", x: float) -> None:
           """Add plate element for YZ panel at x position."""
           corners = panel.get_corners_3d(x)
           material = panel.material or "S355"

           self._model.add_plate(
               corners=corners,
               thickness=panel.thickness,
               material=material,
               mesh_size=None
           )

       def _add_stiffeners_from_plate_field(
           self, plate_field, z: float, segment: "CargoSegment"
       ) -> None:
           """Add beam elements for stiffeners."""
           from ..geometry.stiffener import StiffenerDirection

           material = plate_field.default_material or "S355"

           for stiff in plate_field.get_stiffeners():
               if stiff.direction == StiffenerDirection.LONGITUDINAL:
                   # Runs in X at y = stiff.position
                   start = [segment.x_start, stiff.position, z]
                   end = [segment.x_end, stiff.position, z]
               elif stiff.direction == StiffenerDirection.TRANSVERSE:
                   # Runs in Y at x = stiff.position
                   start = [stiff.position, segment.y_start, z]
                   end = [stiff.position, segment.y_end, z]
               else:
                   continue  # Skip vertical for deck

               # Add offset if configured
               if self.options.stiffener_attachment == "offset":
                   # Get profile height for offset
                   try:
                       profile = self.vessel.profiles.get(stiff.section)
                       offset = -profile.properties.z_cg  # Below plate
                       start[2] += offset
                       end[2] += offset
                   except (KeyError, AttributeError):
                       pass  # Use same z if profile not found

               self._model.add_beam_by_coords(
                   start_position=start,
                   end_position=end,
                   section=stiff.section,
                   material=material
               )

       def _add_bottom_supports(self, segment: "CargoSegment") -> None:
           """Add vertical supports at bottom of segment."""
           if segment.support_type != "bottom":
               return

           # Add supports along bottom edges
           # This is a simplified approach - actual implementation
           # would need to match node positions from plate mesh

           # For now, fix bottom nodes in Z
           # This requires the model to have nodes at z=0
           # TODO: Implement proper bottom support


   def vessel_to_structural_model(
       vessel: "Vessel",
       options: VesselConversionOptions = None
   ) -> "StructuralModel":
       """
       Convert vessel geometry to Grillex StructuralModel.

       Args:
           vessel: Vessel geometry to convert
           options: Conversion options

       Returns:
           StructuralModel ready for analysis

       Example:
           >>> from grillex.vessel import Vessel
           >>> from grillex.vessel.conversion import vessel_to_structural_model
           >>>
           >>> vessel = Vessel(name="Barge", length=100, beam=24, depth=6)
           >>> segment = vessel.add_cargo_segment(frame_start=10, frame_end=35)
           >>> deck = segment.add_deck(z=6.0)
           >>> deck.set_plating(thickness=0.016, material="S355")
           >>>
           >>> model = vessel_to_structural_model(vessel)
           >>> model.analyze()
       """
       converter = VesselToStructuralConverter(vessel, options)
       return converter.convert()
   ```

**Acceptance Criteria:**
- [ ] VesselConversionOptions with mesh_size, stiffener_attachment options
- [ ] VesselToStructuralConverter class structure
- [ ] Materials transferred from vessel to model
- [ ] Sections created from vessel profiles
- [ ] vessel_to_structural_model() convenience function
- [ ] Basic deck conversion to plate elements

---

### Task 25.2: Implement Deck Conversion

**Requirements:** New feature
**Dependencies:** Task 25.1
**Difficulty:** Medium

**Description:**
Implement complete deck conversion including stiffeners.

**Steps:**

1. Enhance _convert_deck() to handle:
   - Panel derivation from stiffener layout
   - Plate element creation for each panel
   - Stiffener beam elements with offset
   - Connection between stiffeners and plates

2. Handle varying plate thickness regions

**Acceptance Criteria:**
- [ ] Deck panels converted to plate elements
- [ ] Stiffeners converted to beam elements
- [ ] Stiffener offset from plate surface (profile height)
- [ ] Varying thickness regions create separate panels
- [ ] Material properties assigned correctly

---

### Task 25.3: Implement Webframe and Girder Conversion

**Requirements:** New feature
**Dependencies:** Task 25.1
**Difficulty:** Medium

**Description:**
Implement webframe and girder conversion.

**Steps:**

1. Beam-type webframes → transverse beam elements
2. Plate-type webframes → plate elements (YZ plane)
3. Longitudinal girders → beam elements running fore-aft
4. Transverse girders → beam elements running port-starboard

**Acceptance Criteria:**
- [ ] Beam webframes create transverse beam elements
- [ ] Plate webframes create plate elements in YZ plane
- [ ] Longitudinal girders span segment length
- [ ] Transverse girders span segment width
- [ ] Sections assigned correctly

---

### Task 25.4: Implement Bulkhead Conversion

**Requirements:** New feature
**Dependencies:** Task 25.1
**Difficulty:** Medium

**Description:**
Implement transverse and longitudinal bulkhead conversion.

**Steps:**

1. TransverseBulkhead → plate elements in YZ plane
2. LongitudinalBulkhead → plate elements in XZ plane
3. Side shells → plate elements in XZ plane
4. Stiffeners on bulkheads → beam elements

**Acceptance Criteria:**
- [ ] Transverse bulkheads converted to YZ plate elements
- [ ] Longitudinal bulkheads converted to XZ plate elements
- [ ] Side shells converted to XZ plate elements
- [ ] Bulkhead stiffeners converted to beam elements
- [ ] Varying thickness handled

---

### Task 25.5: Implement Node Merging and Connections

**Requirements:** New feature
**Dependencies:** Tasks 25.2-25.4
**Difficulty:** High

**Description:**
Implement node merging at component intersections.

**Steps:**

1. Create node registry for coordinate-based lookup
2. Merge nodes within tolerance at:
   - Deck-bulkhead intersections
   - Stiffener-girder intersections
   - Webframe-girder intersections
3. Create rigid links for offset stiffeners (if configured)

**Acceptance Criteria:**
- [ ] Nodes merged at component intersections
- [ ] Merge tolerance configurable
- [ ] Rigid links created for offset stiffeners
- [ ] Connectivity verified (no floating nodes)

---

### Task 25.6: Implement Bottom Supports

**Requirements:** New feature
**Dependencies:** Task 25.5
**Difficulty:** Medium

**Description:**
Implement bottom supports for water-supported vessels.

**Steps:**

1. For support_type="bottom":
   - Add vertical supports at bottom edges of plates
   - Support stiffness configurable (rigid or spring)
2. For support_type="columns":
   - Use column support definitions from vessel
   - Apply point supports at column locations

**Acceptance Criteria:**
- [ ] Bottom supports added for water-supported vessels
- [ ] Support stiffness configurable
- [ ] Column supports for SSCV deckbox case
- [ ] Horizontal restraints at reference points

---

### Task 25.7: Write Conversion Tests

**Requirements:** Testing
**Dependencies:** Tasks 25.1-25.6
**Difficulty:** Medium

**Description:**
Create comprehensive tests for vessel to FEM conversion.

**Steps:**

1. Create `tests/python/test_phase25_vessel_conversion.py`:
   ```python
   """Tests for Phase 25: Vessel to StructuralModel Conversion."""

   import pytest
   from grillex.vessel import Vessel
   from grillex.vessel.conversion import vessel_to_structural_model
   from grillex.vessel.reference_vessels import create_standard_barge_100x24


   class TestBasicConversion:
       """Tests for basic vessel conversion."""

       def test_convert_simple_vessel(self):
           """Simple vessel can be converted."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           vessel.add_material("S355", E=210e6, yield_stress=355e3)

           segment = vessel.add_cargo_segment(x_start=20, x_end=80)
           deck = segment.add_deck(z=6.0)
           deck.set_plating(thickness=0.016, material="S355")

           model = vessel_to_structural_model(vessel)

           assert model is not None
           assert model.name == "Test"

       def test_convert_reference_vessel(self):
           """Reference vessel can be converted."""
           vessel = create_standard_barge_100x24()
           model = vessel_to_structural_model(vessel)

           assert model is not None

       def test_materials_transferred(self):
           """Materials are transferred to model."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           vessel.add_material("S355", E=210e6, yield_stress=355e3)
           segment = vessel.add_cargo_segment(x_start=20, x_end=80)
           segment.add_deck(z=6.0).set_plating(thickness=0.016, material="S355")

           model = vessel_to_structural_model(vessel)

           # Material should exist in model
           # (exact verification depends on model API)


   class TestDeckConversion:
       """Tests for deck conversion."""

       def test_deck_creates_plate_elements(self):
           """Deck creates plate elements."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           vessel.add_material("S355", E=210e6, yield_stress=355e3)

           segment = vessel.add_cargo_segment(x_start=20, x_end=80)
           deck = segment.add_deck(z=6.0)
           deck.set_plating(thickness=0.016, material="S355")

           model = vessel_to_structural_model(vessel)

           # Should have plate elements
           # (verification depends on model API)

       def test_stiffeners_create_beams(self):
           """Deck stiffeners create beam elements."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           vessel.add_material("S355", E=210e6, yield_stress=355e3)
           vessel.profiles.add_hp("HP200x10", height=0.200, thickness=0.010)

           segment = vessel.add_cargo_segment(x_start=20, x_end=80)
           deck = segment.add_deck(z=6.0)
           deck.set_plating(thickness=0.016, material="S355")
           deck.add_stiffeners(
               direction="longitudinal",
               spacing=0.6,
               section="HP200x10"
           )

           model = vessel_to_structural_model(vessel)

           # Should have beam elements for stiffeners
   ```

**Acceptance Criteria:**
- [ ] Simple vessel converts without error
- [ ] Reference vessel converts without error
- [ ] Materials transferred correctly
- [ ] Deck conversion creates plate elements
- [ ] Stiffener conversion creates beam elements
- [ ] All tests pass

---

### Task 25.8: Implement Bracket Conversion with Plate-to-Beam Coupling

**Requirements:** New feature
**Dependencies:** Tasks 25.3, 25.5
**Difficulty:** High

**Description:**
Implement bracket conversion as plate elements with proper coupling to girder beam elements. Brackets are critical for shear capacity at girder-webframe connections and must be modeled with plate elements for accurate stress distribution.

**Background:**
Brackets increase the effective shear area at connections. Unlike stiffeners (which can be modeled as beams), brackets have complex stress distributions that require plate element modeling. The challenge is connecting plate element nodes to the girder beam element nodes.

**Steps:**

1. Add bracket conversion method:
   ```python
   def _convert_brackets(self, segment: "CargoSegment") -> None:
       """
       Convert brackets to plate elements with beam coupling.

       Brackets require:
       1. Plate elements for the bracket itself
       2. Beam element for flange (if present)
       3. Rigid links connecting bracket plate nodes to girder beam nodes
       """
       # Collect all brackets from girders
       for girder in segment.get_longitudinal_girders():
           for bracket in girder.get_brackets():
               self._convert_single_bracket(bracket, girder)

       for girder in segment.get_transverse_girders():
           for bracket in girder.get_brackets():
               self._convert_single_bracket(bracket, girder)

   def _convert_single_bracket(
       self,
       bracket: "Bracket",
       girder: Union["LongitudinalGirder", "TransverseGirder"]
   ) -> None:
       """
       Convert a single bracket to plate element(s) with coupling.

       The bracket geometry creates a triangular or trapezoidal region
       that connects the girder web to the transverse structure.
       """
       # Get bracket vertices
       vertices = bracket.get_vertices()
       material = bracket.material or "S355"

       # Create plate element for bracket
       self._model.add_plate(
           corners=vertices,
           thickness=bracket.thickness,
           material=material,
           mesh_size=None  # Single element initially
       )

       # Add flange beam if bracket has flange
       if bracket.has_flange and bracket.flange_section:
           self._add_bracket_flange(bracket, vertices)

       # Create rigid links from bracket plate nodes to girder beam
       self._add_bracket_to_girder_coupling(bracket, girder, vertices)

   def _add_bracket_flange(
       self,
       bracket: "Bracket",
       vertices: List[Tuple[float, float, float]]
   ) -> None:
       """Add flange beam element along bracket free edge."""
       # The flange runs along the free (outer) edge of the bracket
       # For trapezoidal bracket: along the sloped edge
       # For triangular bracket: along the hypotenuse

       # Determine flange start/end from bracket geometry
       if bracket.type == BracketType.TRAPEZOIDAL:
           # Outer edge is vertices[1] to vertices[2] typically
           start = list(vertices[1])
           end = list(vertices[2])
       else:
           # Triangular: hypotenuse from heel to toe
           start = list(vertices[0])
           end = list(vertices[2])

       self._model.add_beam_by_coords(
           start_position=start,
           end_position=end,
           section=bracket.flange_section,
           material=bracket.material or "S355"
       )

   def _add_bracket_to_girder_coupling(
       self,
       bracket: "Bracket",
       girder: Union["LongitudinalGirder", "TransverseGirder"],
       bracket_vertices: List[Tuple[float, float, float]]
   ) -> None:
       """
       Create rigid links between bracket plate nodes and girder beam.

       This ensures proper load transfer between the plate elements
       (bracket) and beam elements (girder).

       Strategy:
       1. Identify bracket vertices that lie on the girder line
       2. Find corresponding nodes on the girder beam
       3. Create rigid links (MPCs) connecting them
       """
       # Determine which bracket vertices are on the girder
       # (vertices at the heel of the bracket, where it meets girder web)

       girder_connection_vertices = []

       if isinstance(girder, LongitudinalGirder):
           # Girder runs in X at y=girder.y, z=girder.z
           for v in bracket_vertices:
               if abs(v[1] - girder.y) < self.options.node_merge_tolerance:
                   girder_connection_vertices.append(v)
       else:
           # TransverseGirder runs in Y at x=girder.x, z=girder.z
           for v in bracket_vertices:
               if abs(v[0] - girder.x) < self.options.node_merge_tolerance:
                   girder_connection_vertices.append(v)

       # Create rigid links for these vertices
       for vertex in girder_connection_vertices:
           # Find or create node at vertex position
           plate_node = self._get_or_create_node(*vertex)

           # Find closest point on girder beam
           beam_point = self._project_to_girder(vertex, girder)
           beam_node = self._get_or_create_node(*beam_point)

           # Create rigid link (all DOFs)
           if plate_node != beam_node:
               self._model.add_rigid_link(
                   master_node=beam_node,
                   slave_node=plate_node,
                   dofs=[True, True, True, True, True, True]  # All 6 DOFs
               )

   def _project_to_girder(
       self,
       point: Tuple[float, float, float],
       girder: Union["LongitudinalGirder", "TransverseGirder"]
   ) -> Tuple[float, float, float]:
       """Project a point onto the girder line."""
       x, y, z = point

       if isinstance(girder, LongitudinalGirder):
           # Project to y=girder.y, z=girder.z (x unchanged)
           return (x, girder.y, girder.z)
       else:
           # Project to x=girder.x, z=girder.z (y unchanged)
           return (girder.x, y, girder.z)
   ```

2. Update `_convert_segment` to call bracket conversion:
   ```python
   def _convert_segment(self, segment: "CargoSegment") -> None:
       """Convert a cargo segment to FEM elements."""
       # ... existing conversions ...

       # Convert brackets (must be after girders)
       self._convert_brackets(segment)
   ```

3. Handle bracket meshing options:
   ```python
   @dataclass
   class VesselConversionOptions:
       # ... existing options ...

       # Bracket options
       bracket_mesh_size: Optional[float] = None  # None = single element
       bracket_coupling: str = "rigid_links"  # "rigid_links" or "shared_nodes"
   ```

**Plate-to-Beam Coupling Notes:**

The coupling between bracket plate elements and girder beam elements is critical:

1. **Rigid Links (Recommended)**
   - Create MPC constraints linking plate nodes to beam nodes
   - All 6 DOFs coupled (translations and rotations)
   - Works with offset beams
   - Preserves correct load distribution

2. **Shared Nodes (Alternative)**
   - Plate element nodes directly on beam line
   - Simpler but requires beam and plate to share exact coordinates
   - May have issues with plate element quality

3. **Existing Grillex Constraints**
   - Grillex already supports rigid links (Phase 6)
   - Use `add_rigid_link(master, slave, dofs)` method
   - Master node on beam, slave node on plate

**Acceptance Criteria:**
- [ ] Brackets converted to plate elements
- [ ] Bracket vertices computed correctly for triangular/trapezoidal shapes
- [ ] Bracket flanges converted to beam elements (if has_flange=True)
- [ ] Rigid links created between bracket plates and girder beams
- [ ] Coupling preserves all 6 DOFs
- [ ] Mesh refinement in bracket region supported
- [ ] Test: bracket stress distribution matches expected pattern
- [ ] Test: load transfer through bracket connection verified

---

### Task 25.9: Implement Refined Meshing for High-Stress Regions

**Requirements:** New feature
**Dependencies:** Tasks 25.2-25.8
**Difficulty:** Medium

**Description:**
Implement adaptive meshing that uses finer elements in high-stress regions (brackets, girder-webframe connections).

**Background:**
The initial single-element-per-panel approach is efficient for global response but insufficient for accurate stress analysis in connection regions. This task adds:
1. Option to refine mesh in bracket regions
2. Post-analysis identification of high-stress panels for refinement

**Steps:**

1. Add mesh refinement options:
   ```python
   @dataclass
   class VesselConversionOptions:
       # ... existing options ...

       # Mesh refinement
       refine_bracket_regions: bool = True
       bracket_mesh_size: float = 0.1  # meters
       refine_regions: List[Tuple[float, float, float, float, float]] = None
       # Each tuple: (x_min, y_min, x_max, y_max, mesh_size)
   ```

2. Implement region-based meshing:
   ```python
   def _get_mesh_size_at(self, x: float, y: float, z: float) -> Optional[float]:
       """Get target mesh size at a point."""
       # Check if in refined region
       for region in (self.options.refine_regions or []):
           x_min, y_min, x_max, y_max, size = region
           if x_min <= x <= x_max and y_min <= y <= y_max:
               return size

       # Check if near bracket
       if self.options.refine_bracket_regions:
           for segment in self.vessel.get_cargo_segments():
               for girder in segment.get_longitudinal_girders():
                   for bracket in girder.get_brackets():
                       if self._point_near_bracket(x, y, z, bracket):
                           return self.options.bracket_mesh_size

       return None  # Use default (single element)
   ```

**Acceptance Criteria:**
- [ ] Bracket regions automatically refined when option enabled
- [ ] Custom refinement regions supported via refine_regions option
- [ ] Mesh size transitions smoothly between regions
- [ ] Refinement does not break plate-to-beam coupling

---

### Summary

| Task | Description | Difficulty | Status |
|------|-------------|------------|--------|
| 25.1 | Conversion framework | High | Pending |
| 25.2 | Deck conversion | Medium | Pending |
| 25.3 | Webframe and girder conversion | Medium | Pending |
| 25.4 | Bulkhead conversion | Medium | Pending |
| 25.5 | Node merging and connections | High | Pending |
| 25.6 | Bottom supports | Medium | Pending |
| 25.7 | Conversion tests | Medium | Pending |
| 25.8 | Bracket conversion with plate-to-beam coupling | High | Pending |
| 25.9 | Refined meshing for high-stress regions | Medium | Pending |

**Total Acceptance Criteria:** 42 items

---

## Integration Notes

When extracting the vessel module to a separate repository:

1. **This module becomes the integration layer**
   - `conversion/to_structural.py` imports from `grillex.core`
   - Rest of vessel module is independent

2. **Grillex becomes a dependency**
   ```python
   # In separate vessel-geometry repo
   # setup.py or pyproject.toml
   install_requires=[
       "grillex>=2.0.0",
   ]
   ```

3. **Alternative: Protocol-based integration**
   - Define abstract StructuralModel protocol
   - Allow different FEM backends
