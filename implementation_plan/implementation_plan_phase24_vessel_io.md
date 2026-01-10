## Phase 24: Vessel Geometry - I/O (YAML, JSON, Export)

### Overview

This phase implements serialization and deserialization of vessel geometry, including YAML/JSON formats and export placeholders for Sesam Genie and Abaqus.

**Key Concepts:**
- **YAML/JSON I/O**: Human-readable vessel definition format
- **UUID preservation**: Database compatibility through consistent IDs
- **Export formats**: Sesam Genie (.FEM) and Abaqus (.inp) placeholders

**Dependencies:** Phase 21 (Core), Phase 22 (Profiles), Phase 23 (Panels)

**Directory Structure:**
```
src/grillex/vessel/
└── io/
    ├── __init__.py
    ├── yaml_io.py          # YAML load/save
    ├── json_io.py          # JSON load/save
    └── export/
        ├── __init__.py
        ├── sesam_genie.py  # Sesam Genie export (placeholder)
        └── abaqus.py       # Abaqus export (placeholder)
```

---

### Task 24.1: Define YAML Schema

**Requirements:** New feature
**Dependencies:** Phase 21-23
**Difficulty:** Medium

**Description:**
Define the YAML schema for vessel geometry and implement loading.

**Steps:**

1. Create `src/grillex/vessel/io/__init__.py`:
   ```python
   from .yaml_io import load_vessel_from_yaml, save_vessel_to_yaml
   from .json_io import load_vessel_from_json, save_vessel_to_json

   __all__ = [
       "load_vessel_from_yaml",
       "save_vessel_to_yaml",
       "load_vessel_from_json",
       "save_vessel_to_json",
   ]
   ```

2. Create `src/grillex/vessel/io/yaml_io.py`:
   ```python
   """
   YAML I/O for vessel geometry.

   Schema:
   -------
   ```yaml
   name: "Vessel Name"
   length: 100.0          # meters
   beam: 24.0             # meters
   depth: 6.0             # meters
   frame_spacing: 2.5     # meters
   frame_zero_x: 0.0      # meters

   materials:
     - name: S355
       E: 210000000       # kN/m²
       yield_stress: 355000  # kN/m²
       nu: 0.3
       rho: 7.85          # mT/m³

   profiles:
     - name: T400x150x12x20
       type: tee
       web_height: 0.400
       web_thickness: 0.012
       flange_width: 0.150
       flange_thickness: 0.020

     - name: HP200x10
       type: hp_standard   # Lookup from standard database

   cargo_segments:
     - name: main_cargo
       frame_start: 10
       frame_end: 35
       support_type: bottom

       transverse_bulkheads:
         aft:
           thickness: 0.014
           material: S355
           stiffeners:
             - direction: vertical
               spacing: 0.6
               section: HP180x10

         fwd:
           thickness: 0.014
           material: S355

       side_shells:
         - y: -12.0
           thickness: 0.012
           material: S355

         - y: 12.0
           thickness: 0.012
           material: S355

       longitudinal_bulkheads:
         - y: 0.0
           thickness: 0.012
           material: S355
           stiffeners:
             - direction: horizontal
               spacing: 0.6
               section: HP160x9

       decks:
         - z: 6.0
           name: main_deck
           thickness: 0.016
           material: S355
           stiffeners:
             - direction: longitudinal
               spacing: 0.6
               section: HP200x10

       webframes:
         - frames: [12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34]
           type: beam
           section: T300x150x10x14

       longitudinal_girders:
         - y: 0.0
           z: 6.0
           section: T500x200x12x20

         - y: -6.0
           z: 6.0
           section: T400x150x10x16
   ```
   """

   import yaml
   from pathlib import Path
   from typing import Union, Dict, Any, List

   from ..geometry import (
       Vessel, CargoSegment, TransverseBulkhead, LongitudinalBulkhead,
       SideShell, Deck, WebFrame, LongitudinalGirder, TransverseGirder
   )
   from ..profiles import ProfileLibrary


   def load_vessel_from_yaml(path: Union[str, Path]) -> Vessel:
       """
       Load vessel geometry from YAML file.

       Args:
           path: Path to YAML file

       Returns:
           Configured Vessel object
       """
       with open(path, 'r') as f:
           data = yaml.safe_load(f)

       return _build_vessel_from_dict(data)


   def save_vessel_to_yaml(vessel: Vessel, path: Union[str, Path]) -> None:
       """
       Save vessel geometry to YAML file.

       Args:
           vessel: Vessel to save
           path: Path to output file
       """
       data = _vessel_to_dict(vessel)

       with open(path, 'w') as f:
           yaml.dump(data, f, default_flow_style=False, sort_keys=False)


   def _build_vessel_from_dict(data: Dict[str, Any]) -> Vessel:
       """Build vessel from parsed YAML data."""
       vessel = Vessel(
           name=data["name"],
           length=data["length"],
           beam=data["beam"],
           depth=data["depth"],
           frame_spacing=data.get("frame_spacing", 2.5),
           frame_zero_x=data.get("frame_zero_x", 0.0)
       )

       # Load materials
       for mat in data.get("materials", []):
           vessel.add_material(
               name=mat["name"],
               E=mat["E"],
               yield_stress=mat["yield_stress"],
               nu=mat.get("nu", 0.3),
               rho=mat.get("rho", 7.85)
           )

       # Load profiles
       _load_profiles(vessel.profiles, data.get("profiles", []))

       # Load cargo segments
       for seg_data in data.get("cargo_segments", []):
           _load_cargo_segment(vessel, seg_data)

       return vessel


   def _load_profiles(library: ProfileLibrary, profiles: List[Dict]) -> None:
       """Load profile definitions."""
       for prof in profiles:
           name = prof["name"]
           prof_type = prof.get("type", "").lower()

           if prof_type == "hp_standard":
               library.add_standard_hp(name)
           elif prof_type == "hp":
               library.add_hp(name, prof["height"], prof["thickness"])
           elif prof_type == "flat_bar":
               library.add_flat_bar(name, prof["height"], prof["thickness"])
           elif prof_type == "angle":
               library.add_angle(
                   name, prof["height"], prof["width"], prof["thickness"]
               )
           elif prof_type == "tee":
               library.add_tee(
                   name,
                   prof["web_height"],
                   prof["web_thickness"],
                   prof["flange_width"],
                   prof["flange_thickness"]
               )
           elif prof_type == "i_section":
               library.add_i_section(
                   name,
                   prof["height"],
                   prof["flange_width"],
                   prof["web_thickness"],
                   prof["flange_thickness"]
               )


   def _load_cargo_segment(vessel: Vessel, data: Dict) -> CargoSegment:
       """Load a cargo segment from YAML data."""
       segment = vessel.add_cargo_segment(
           name=data.get("name"),
           frame_start=data.get("frame_start"),
           frame_end=data.get("frame_end"),
           x_start=data.get("x_start"),
           x_end=data.get("x_end"),
           support_type=data.get("support_type", "bottom")
       )

       # Transverse bulkheads
       bhd_data = data.get("transverse_bulkheads", {})
       if "aft" in bhd_data:
           _configure_trans_bhd(segment.transverse_bulkhead_aft, bhd_data["aft"])
       if "fwd" in bhd_data:
           _configure_trans_bhd(segment.transverse_bulkhead_fwd, bhd_data["fwd"])

       # Side shells
       for shell_data in data.get("side_shells", []):
           shell = segment.add_side_shell(
               y=shell_data["y"],
               thickness=shell_data.get("thickness"),
               material=shell_data.get("material")
           )

       # Longitudinal bulkheads
       for bhd_data in data.get("longitudinal_bulkheads", []):
           bhd = segment.add_longitudinal_bulkhead(y=bhd_data["y"])
           if "thickness" in bhd_data:
               bhd.set_plating(bhd_data["thickness"], bhd_data.get("material"))
           for stiff in bhd_data.get("stiffeners", []):
               bhd.add_stiffeners(
                   direction=stiff["direction"],
                   spacing=stiff["spacing"],
                   section=stiff["section"]
               )

       # Decks
       for deck_data in data.get("decks", []):
           deck = segment.add_deck(z=deck_data["z"], name=deck_data.get("name"))
           if "thickness" in deck_data:
               deck.set_plating(deck_data["thickness"], deck_data.get("material"))
           for stiff in deck_data.get("stiffeners", []):
               deck.add_stiffeners(
                   direction=stiff["direction"],
                   spacing=stiff["spacing"],
                   section=stiff["section"]
               )

       # Webframes
       for wf_data in data.get("webframes", []):
           frames = wf_data.get("frames", [])
           x_coords = wf_data.get("x_coords", [])
           wf_type = wf_data.get("type", "beam")
           section = wf_data.get("section")

           for frame in frames:
               segment.add_webframe(frame=frame, type=wf_type, section=section)
           for x in x_coords:
               segment.add_webframe(x=x, type=wf_type, section=section)

       # Longitudinal girders
       for girder_data in data.get("longitudinal_girders", []):
           segment.add_longitudinal_girder(
               y=girder_data["y"],
               z=girder_data["z"],
               section=girder_data["section"]
           )

       # Transverse girders
       for girder_data in data.get("transverse_girders", []):
           segment.add_transverse_girder(
               x=girder_data.get("x"),
               frame=girder_data.get("frame"),
               z=girder_data["z"],
               section=girder_data["section"]
           )

       return segment


   def _configure_trans_bhd(bhd: TransverseBulkhead, data: Dict) -> None:
       """Configure transverse bulkhead from YAML data."""
       if "thickness" in data:
           bhd.set_plating(data["thickness"], data.get("material"))
       for stiff in data.get("stiffeners", []):
           bhd.add_stiffeners(
               direction=stiff["direction"],
               spacing=stiff["spacing"],
               section=stiff["section"]
           )


   def _vessel_to_dict(vessel: Vessel) -> Dict[str, Any]:
       """Convert vessel to dictionary for YAML output."""
       data = {
           "name": vessel.name,
           "length": vessel.length,
           "beam": vessel.beam,
           "depth": vessel.depth,
           "frame_spacing": vessel.frame_spacing,
           "frame_zero_x": vessel.frame_zero_x,
       }

       # Materials
       if vessel._materials:
           data["materials"] = [
               {"name": name, **props}
               for name, props in vessel._materials.items()
           ]

       # Cargo segments
       segments = []
       for seg in vessel.get_cargo_segments():
           segments.append(_segment_to_dict(seg))
       if segments:
           data["cargo_segments"] = segments

       return data


   def _segment_to_dict(segment: CargoSegment) -> Dict[str, Any]:
       """Convert cargo segment to dictionary."""
       data = {
           "name": segment.name,
           "frame_start": segment.frame_start,
           "frame_end": segment.frame_end,
           "support_type": segment.support_type,
       }

       # Decks
       decks = []
       for deck in segment.get_decks():
           deck_data = {"z": deck.z, "name": deck.name}
           if deck.plate_field.default_thickness:
               deck_data["thickness"] = deck.plate_field.default_thickness
           if deck.plate_field.default_material:
               deck_data["material"] = deck.plate_field.default_material
           # Stiffeners would be added here
           decks.append(deck_data)
       if decks:
           data["decks"] = decks

       # Webframes
       webframes = segment.get_webframes()
       if webframes:
           # Group by section and type
           wf_groups = {}
           for wf in webframes:
               key = (wf.type, wf.section)
               if key not in wf_groups:
                   wf_groups[key] = []
               wf_groups[key].append(wf.frame)

           data["webframes"] = [
               {"type": k[0], "section": k[1], "frames": frames}
               for k, frames in wf_groups.items()
           ]

       return data
   ```

**Acceptance Criteria:**
- [ ] YAML schema documented with examples
- [ ] load_vessel_from_yaml() parses vessel definition
- [ ] save_vessel_to_yaml() serializes vessel to YAML
- [ ] Materials are loaded correctly
- [ ] Profiles are loaded (custom and standard HP)
- [ ] Cargo segments with all components are loaded
- [ ] Stiffener layouts are preserved
- [ ] Round-trip (load/save/load) preserves vessel geometry

---

### Task 24.2: Implement JSON I/O

**Requirements:** New feature
**Dependencies:** Task 24.1
**Difficulty:** Low

**Description:**
Implement JSON I/O using the same schema structure as YAML.

**Steps:**

1. Create `src/grillex/vessel/io/json_io.py`:
   ```python
   """
   JSON I/O for vessel geometry.

   Uses the same schema as YAML but in JSON format.
   """

   import json
   from pathlib import Path
   from typing import Union

   from ..geometry import Vessel
   from .yaml_io import _build_vessel_from_dict, _vessel_to_dict


   def load_vessel_from_json(path: Union[str, Path]) -> Vessel:
       """
       Load vessel geometry from JSON file.

       Args:
           path: Path to JSON file

       Returns:
           Configured Vessel object
       """
       with open(path, 'r') as f:
           data = json.load(f)

       return _build_vessel_from_dict(data)


   def save_vessel_to_json(
       vessel: Vessel,
       path: Union[str, Path],
       indent: int = 2
   ) -> None:
       """
       Save vessel geometry to JSON file.

       Args:
           vessel: Vessel to save
           path: Path to output file
           indent: JSON indentation (default: 2)
       """
       data = _vessel_to_dict(vessel)

       with open(path, 'w') as f:
           json.dump(data, f, indent=indent)


   def vessel_to_json_string(vessel: Vessel, indent: int = 2) -> str:
       """
       Convert vessel to JSON string.

       Args:
           vessel: Vessel to convert
           indent: JSON indentation

       Returns:
           JSON string representation
       """
       data = _vessel_to_dict(vessel)
       return json.dumps(data, indent=indent)


   def vessel_from_json_string(json_str: str) -> Vessel:
       """
       Create vessel from JSON string.

       Args:
           json_str: JSON string

       Returns:
           Configured Vessel object
       """
       data = json.loads(json_str)
       return _build_vessel_from_dict(data)
   ```

2. Update exports in `__init__.py`:
   ```python
   from .json_io import (
       load_vessel_from_json,
       save_vessel_to_json,
       vessel_to_json_string,
       vessel_from_json_string,
   )
   ```

**Acceptance Criteria:**
- [ ] load_vessel_from_json() loads vessel from JSON file
- [ ] save_vessel_to_json() saves vessel to JSON file
- [ ] vessel_to_json_string() converts to JSON string
- [ ] vessel_from_json_string() creates vessel from string
- [ ] Same schema as YAML

---

### Task 24.3: Create Sesam Genie Export Placeholder

**Requirements:** New feature (placeholder)
**Dependencies:** Phase 21-23
**Difficulty:** Low

**Description:**
Create placeholder for Sesam Genie FEM format export.

**Steps:**

1. Create `src/grillex/vessel/io/export/__init__.py`:
   ```python
   from .sesam_genie import export_to_sesam_genie
   from .abaqus import export_to_abaqus

   __all__ = [
       "export_to_sesam_genie",
       "export_to_abaqus",
   ]
   ```

2. Create `src/grillex/vessel/io/export/sesam_genie.py`:
   ```python
   """
   Sesam Genie FEM format export.

   Sesam Genie is DNV's structural analysis software. The FEM format
   is used for model exchange.

   Status: PLACEHOLDER - To be implemented
   """

   from pathlib import Path
   from typing import Union, TYPE_CHECKING

   if TYPE_CHECKING:
       from ...geometry import Vessel

   class SesamGenieExportNotImplemented(NotImplementedError):
       """Raised when Sesam Genie export is called but not yet implemented."""
       pass


   def export_to_sesam_genie(
       vessel: "Vessel",
       path: Union[str, Path],
       options: dict = None
   ) -> None:
       """
       Export vessel geometry to Sesam Genie FEM format.

       This export allows verification of Grillex models against
       DNV Sesam Genie for quality assurance.

       Args:
           vessel: Vessel to export
           path: Output file path (.FEM extension recommended)
           options: Export options (reserved for future use)
               - mesh_size: Target mesh size in meters
               - include_loads: Whether to include load definitions
               - coordinate_system: "global" or "vessel"

       Raises:
           SesamGenieExportNotImplemented: This feature is not yet implemented

       Future Implementation Notes:
       ----------------------------
       Sesam Genie FEM format structure:
       1. GNODE - Node definitions
       2. GELMNT1 - Element topology
       3. GELREF1 - Element properties reference
       4. GIORH - I-beam/H-beam properties
       5. GBEAMG - General beam properties
       6. GPIPE - Pipe properties
       7. GPLATE - Plate properties
       8. GUSYI - Cross-section property values
       9. MISOSEL - Material properties

       Key considerations:
       - Coordinate system transformation (vessel to Sesam)
       - Property ID mapping
       - Load case definition (BLDEP, BNLOAD, etc.)
       """
       raise SesamGenieExportNotImplemented(
           "Sesam Genie export is not yet implemented. "
           "This is a placeholder for future development. "
           "See docstring for implementation notes."
       )


   def get_sesam_genie_export_info() -> dict:
       """
       Get information about Sesam Genie export capability.

       Returns:
           Dict with export capability info
       """
       return {
           "implemented": False,
           "status": "placeholder",
           "format": "Sesam Genie FEM (.FEM)",
           "description": "Export to DNV Sesam Genie for verification",
           "planned_features": [
               "Node and element export",
               "Section properties",
               "Material definitions",
               "Load case export",
               "Boundary condition export",
           ]
       }
   ```

**Acceptance Criteria:**
- [ ] export_to_sesam_genie() function exists
- [ ] Raises NotImplementedError with helpful message
- [ ] Docstring documents planned implementation
- [ ] get_sesam_genie_export_info() describes capability

---

### Task 24.4: Create Abaqus Export Placeholder

**Requirements:** New feature (placeholder)
**Dependencies:** Phase 21-23
**Difficulty:** Low

**Description:**
Create placeholder for Abaqus INP format export.

**Steps:**

1. Create `src/grillex/vessel/io/export/abaqus.py`:
   ```python
   """
   Abaqus INP format export.

   Abaqus is a commercial FEM software. The INP format is a text-based
   input format for model definition.

   Status: PLACEHOLDER - To be implemented
   """

   from pathlib import Path
   from typing import Union, TYPE_CHECKING

   if TYPE_CHECKING:
       from ...geometry import Vessel

   class AbaqusExportNotImplemented(NotImplementedError):
       """Raised when Abaqus export is called but not yet implemented."""
       pass


   def export_to_abaqus(
       vessel: "Vessel",
       path: Union[str, Path],
       options: dict = None
   ) -> None:
       """
       Export vessel geometry to Abaqus INP format.

       This export allows verification of Grillex models against
       Abaqus for quality assurance and comparison with industry
       standard FEM software.

       Args:
           vessel: Vessel to export
           path: Output file path (.inp extension recommended)
           options: Export options (reserved for future use)
               - mesh_size: Target mesh size in meters
               - element_type: "S4R" (shell) or "B31" (beam)
               - include_loads: Whether to include load definitions

       Raises:
           AbaqusExportNotImplemented: This feature is not yet implemented

       Future Implementation Notes:
       ----------------------------
       Abaqus INP format structure:
       1. *HEADING - Model description
       2. *NODE - Node definitions
       3. *ELEMENT - Element connectivity
       4. *SHELL SECTION - Shell properties
       5. *BEAM SECTION - Beam properties
       6. *MATERIAL - Material definitions
       7. *BOUNDARY - Boundary conditions
       8. *STEP - Analysis step
       9. *CLOAD, *DLOAD - Loads

       Key considerations:
       - Element type selection (S4R for plates, B31 for beams)
       - Section orientation definition
       - Material property units (Abaqus is unit-agnostic)
       - Part/Instance/Assembly structure for complex models
       """
       raise AbaqusExportNotImplemented(
           "Abaqus export is not yet implemented. "
           "This is a placeholder for future development. "
           "See docstring for implementation notes."
       )


   def get_abaqus_export_info() -> dict:
       """
       Get information about Abaqus export capability.

       Returns:
           Dict with export capability info
       """
       return {
           "implemented": False,
           "status": "placeholder",
           "format": "Abaqus INP (.inp)",
           "description": "Export to Abaqus for verification",
           "planned_features": [
               "Node and element export",
               "Shell elements (S4R) for plates",
               "Beam elements (B31) for stiffeners/girders",
               "Material definitions",
               "Section properties with orientation",
               "Boundary conditions",
               "Load case export",
           ]
       }
   ```

**Acceptance Criteria:**
- [ ] export_to_abaqus() function exists
- [ ] Raises NotImplementedError with helpful message
- [ ] Docstring documents planned implementation
- [ ] get_abaqus_export_info() describes capability

---

### Task 24.5: Write I/O Tests

**Requirements:** Testing
**Dependencies:** Tasks 24.1-24.4
**Difficulty:** Low

**Description:**
Create tests for I/O functionality.

**Steps:**

1. Create `tests/python/test_phase24_vessel_io.py`:
   ```python
   """Tests for Phase 24: Vessel I/O."""

   import pytest
   import tempfile
   from pathlib import Path

   from grillex.vessel import Vessel
   from grillex.vessel.io import (
       load_vessel_from_yaml, save_vessel_to_yaml,
       load_vessel_from_json, save_vessel_to_json,
   )
   from grillex.vessel.io.export import (
       export_to_sesam_genie, export_to_abaqus,
       get_sesam_genie_export_info, get_abaqus_export_info,
   )
   from grillex.vessel.reference_vessels import create_standard_barge_100x24


   class TestYAMLIO:
       """Tests for YAML I/O."""

       def test_save_and_load_yaml(self):
           """Vessel can be saved and loaded from YAML."""
           vessel = create_standard_barge_100x24()

           with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
               path = Path(f.name)

           try:
               save_vessel_to_yaml(vessel, path)
               loaded = load_vessel_from_yaml(path)

               assert loaded.name == vessel.name
               assert loaded.length == vessel.length
               assert loaded.beam == vessel.beam
           finally:
               path.unlink()

       def test_yaml_preserves_cargo_segment(self):
           """YAML round-trip preserves cargo segment."""
           vessel = create_standard_barge_100x24()

           with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
               path = Path(f.name)

           try:
               save_vessel_to_yaml(vessel, path)
               loaded = load_vessel_from_yaml(path)

               segments = loaded.get_cargo_segments()
               assert len(segments) == 1
               assert segments[0].name == "main_cargo"
           finally:
               path.unlink()


   class TestJSONIO:
       """Tests for JSON I/O."""

       def test_save_and_load_json(self):
           """Vessel can be saved and loaded from JSON."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           vessel.add_cargo_segment(x_start=20, x_end=80)

           with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
               path = Path(f.name)

           try:
               save_vessel_to_json(vessel, path)
               loaded = load_vessel_from_json(path)

               assert loaded.name == vessel.name
               assert loaded.length == vessel.length
           finally:
               path.unlink()


   class TestExportPlaceholders:
       """Tests for export placeholders."""

       def test_sesam_genie_not_implemented(self):
           """Sesam Genie export raises NotImplementedError."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)

           with pytest.raises(NotImplementedError):
               export_to_sesam_genie(vessel, "test.FEM")

       def test_abaqus_not_implemented(self):
           """Abaqus export raises NotImplementedError."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)

           with pytest.raises(NotImplementedError):
               export_to_abaqus(vessel, "test.inp")

       def test_sesam_genie_export_info(self):
           """Sesam Genie export info is available."""
           info = get_sesam_genie_export_info()
           assert info["implemented"] is False
           assert "FEM" in info["format"]

       def test_abaqus_export_info(self):
           """Abaqus export info is available."""
           info = get_abaqus_export_info()
           assert info["implemented"] is False
           assert "inp" in info["format"]
   ```

**Acceptance Criteria:**
- [ ] YAML save/load round-trip preserves vessel
- [ ] JSON save/load round-trip preserves vessel
- [ ] Export placeholders raise NotImplementedError
- [ ] Export info functions return capability status
- [ ] All tests pass

---

### Summary

| Task | Description | Difficulty | Status |
|------|-------------|------------|--------|
| 24.1 | YAML schema and I/O | Medium | Pending |
| 24.2 | JSON I/O | Low | Pending |
| 24.3 | Sesam Genie export placeholder | Low | Pending |
| 24.4 | Abaqus export placeholder | Low | Pending |
| 24.5 | I/O tests | Low | Pending |

**Total Acceptance Criteria:** 21 items
