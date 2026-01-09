# Phase 17: FEM Reporting Modules

## Overview

This phase implements comprehensive tabular reporting capabilities for the Grillex FEM platform. The goal is to enable extraction and presentation of all model input data, analysis results, and design code check results in structured tabular formats suitable for engineering reports.

### Objectives
1. Provide tabular reports for all model input data (materials, sections, nodes, beams, loads)
2. Enable FEM results reporting (displacements, reactions, internal actions)
3. Support design code check result reporting
4. Create a foundation for eventual PDF/HTML engineering report generation

### Requirements Traceability
- **R-DATA-004**: Structured result export
- **R-LLM-003**: Machine-readable output formats
- **R-RES-001**: Results accessibility

---

## Task 17.1: Create Reporting Framework and Base Classes
**Requirements:** R-DATA-004, R-LLM-003
**Dependencies:** Phase 4 (Python wrapper), Phase 7 (Internal actions)
**Difficulty:** Medium

**Description:**
Create the foundational reporting framework with base classes for tabular data representation and export.

**Steps:**
1. Create `src/grillex/reporting/__init__.py` module structure
2. Create `src/grillex/reporting/tables.py` with base classes:
   ```python
   from dataclasses import dataclass, field
   from typing import List, Dict, Any, Optional, Union
   from enum import Enum

   class ColumnType(Enum):
       """Column data types for formatting and validation."""
       TEXT = "text"
       INTEGER = "integer"
       FLOAT = "float"
       SCIENTIFIC = "scientific"
       PERCENTAGE = "percentage"
       STATUS = "status"  # PASS/FAIL/WARNING

   @dataclass
   class Column:
       """Definition of a table column."""
       name: str
       key: str  # Key to extract from row data
       type: ColumnType = ColumnType.TEXT
       unit: Optional[str] = None
       precision: int = 3
       width: Optional[int] = None

   @dataclass
   class Table:
       """Tabular data container."""
       title: str
       columns: List[Column]
       rows: List[Dict[str, Any]]
       metadata: Dict[str, Any] = field(default_factory=dict)

       def to_dict(self) -> Dict[str, Any]:
           """Convert to dictionary for serialization."""
           ...

       def to_dataframe(self):
           """Convert to pandas DataFrame if available."""
           ...

       def to_csv(self, path: str) -> None:
           """Export to CSV file."""
           ...

       def to_markdown(self) -> str:
           """Render as Markdown table."""
           ...

       def to_text(self, max_width: int = 120) -> str:
           """Render as formatted text table."""
           ...
   ```

3. Create `src/grillex/reporting/formatters.py`:
   ```python
   class ValueFormatter:
       """Format values based on column type."""

       @staticmethod
       def format_float(value: float, precision: int = 3) -> str:
           ...

       @staticmethod
       def format_scientific(value: float, precision: int = 3) -> str:
           ...

       @staticmethod
       def format_status(status: str) -> str:
           """Format PASS/FAIL/WARNING status."""
           ...
   ```

4. Create `src/grillex/reporting/report.py`:
   ```python
   @dataclass
   class Report:
       """Collection of tables forming a complete report."""
       title: str
       tables: List[Table]
       summary: Optional[str] = None
       metadata: Dict[str, Any] = field(default_factory=dict)

       def add_table(self, table: Table) -> None:
           ...

       def to_dict(self) -> Dict[str, Any]:
           ...

       def to_json(self, path: str, indent: int = 2) -> None:
           ...

       def to_markdown(self) -> str:
           ...

       def to_text(self) -> str:
           ...
   ```

**Acceptance Criteria:**
- [ ] Table class supports all column types (text, int, float, scientific, percentage, status)
- [ ] Tables can be exported to CSV format
- [ ] Tables can be exported to Markdown format
- [ ] Tables can be rendered as formatted text
- [ ] Report class can combine multiple tables
- [ ] All exports include units where applicable

---

## Task 17.2: Input Data Reports - Materials and Sections
**Requirements:** R-DATA-004
**Dependencies:** Task 17.1, Task 1.2 (Materials), Task 1.3 (Sections)
**Difficulty:** Low

**Description:**
Create reporters for material and section properties.

**Steps:**
1. Create `src/grillex/reporting/input_reports.py`:
   ```python
   class MaterialTableBuilder:
       """Build material property tables."""

       @staticmethod
       def build(model: "StructuralModel") -> Table:
           """Create material properties table."""
           columns = [
               Column("Name", "name", ColumnType.TEXT),
               Column("E", "E", ColumnType.SCIENTIFIC, unit="kN/m²"),
               Column("ν", "nu", ColumnType.FLOAT),
               Column("ρ", "rho", ColumnType.FLOAT, unit="mT/m³"),
               Column("fy", "fy", ColumnType.FLOAT, unit="kN/m²"),
               Column("fu", "fu", ColumnType.FLOAT, unit="kN/m²"),
           ]
           rows = []
           for name, mat in model._materials.items():
               rows.append({
                   "name": name,
                   "E": mat.E,
                   "nu": mat.nu,
                   "rho": mat.rho,
                   "fy": getattr(mat, 'fy', None),
                   "fu": getattr(mat, 'fu', None),
               })
           return Table("Material Properties", columns, rows)

   class SectionTableBuilder:
       """Build section property tables."""

       @staticmethod
       def build(model: "StructuralModel") -> Table:
           """Create section properties table."""
           columns = [
               Column("Name", "name", ColumnType.TEXT),
               Column("A", "A", ColumnType.SCIENTIFIC, unit="m²"),
               Column("Iy", "Iy", ColumnType.SCIENTIFIC, unit="m⁴"),
               Column("Iz", "Iz", ColumnType.SCIENTIFIC, unit="m⁴"),
               Column("J", "J", ColumnType.SCIENTIFIC, unit="m⁴"),
               Column("Iw", "Iw", ColumnType.SCIENTIFIC, unit="m⁶"),
               Column("Asy", "Asy", ColumnType.SCIENTIFIC, unit="m²"),
               Column("Asz", "Asz", ColumnType.SCIENTIFIC, unit="m²"),
           ]
           rows = []
           for name, sec in model._sections.items():
               rows.append({
                   "name": name,
                   "A": sec.A,
                   "Iy": sec.Iy,
                   "Iz": sec.Iz,
                   "J": sec.J,
                   "Iw": getattr(sec, 'Iw', 0.0),
                   "Asy": getattr(sec, 'Asy', 0.0),
                   "Asz": getattr(sec, 'Asz', 0.0),
               })
           return Table("Section Properties", columns, rows)
   ```

2. Add convenience methods to `StructuralModel`:
   ```python
   def get_material_table(self) -> Table:
       """Get tabular report of material properties."""
       from grillex.reporting import MaterialTableBuilder
       return MaterialTableBuilder.build(self)

   def get_section_table(self) -> Table:
       """Get tabular report of section properties."""
       from grillex.reporting import SectionTableBuilder
       return SectionTableBuilder.build(self)
   ```

**Acceptance Criteria:**
- [ ] Material table includes E, nu, rho, fy, fu with units
- [ ] Section table includes A, Iy, Iz, J, Iw, Asy, Asz with units
- [ ] Empty/None values handled gracefully
- [ ] Tables accessible via model.get_material_table() and model.get_section_table()

---

## Task 17.3: Input Data Reports - Nodes and Elements
**Requirements:** R-DATA-004
**Dependencies:** Task 17.1, Task 4.1 (Model wrapper)
**Difficulty:** Low

**Description:**
Create reporters for node and beam element geometry.

**Steps:**
1. Add to `src/grillex/reporting/input_reports.py`:
   ```python
   class NodeTableBuilder:
       """Build node coordinate tables."""

       @staticmethod
       def build(model: "StructuralModel") -> Table:
           columns = [
               Column("ID", "id", ColumnType.INTEGER),
               Column("X", "x", ColumnType.FLOAT, unit="m", precision=3),
               Column("Y", "y", ColumnType.FLOAT, unit="m", precision=3),
               Column("Z", "z", ColumnType.FLOAT, unit="m", precision=3),
               Column("DOFs", "dofs", ColumnType.TEXT),
           ]
           rows = []
           for pos, node in model._node_map.items():
               rows.append({
                   "id": node.id,
                   "x": pos[0],
                   "y": pos[1],
                   "z": pos[2],
                   "dofs": _format_dof_status(node),
               })
           return Table("Node Coordinates", columns, rows)

   class BeamTableBuilder:
       """Build beam element tables."""

       @staticmethod
       def build(model: "StructuralModel") -> Table:
           columns = [
               Column("ID", "beam_id", ColumnType.INTEGER),
               Column("Name", "name", ColumnType.TEXT),
               Column("Start", "start", ColumnType.TEXT),
               Column("End", "end", ColumnType.TEXT),
               Column("Length", "length", ColumnType.FLOAT, unit="m", precision=3),
               Column("Section", "section", ColumnType.TEXT),
               Column("Material", "material", ColumnType.TEXT),
               Column("Elements", "n_elements", ColumnType.INTEGER),
           ]
           rows = []
           for beam in model.beams:
               rows.append({
                   "beam_id": beam.beam_id,
                   "name": beam.name or f"Beam_{beam.beam_id}",
                   "start": _format_coords(beam.start_pos),
                   "end": _format_coords(beam.end_pos),
                   "length": beam.length,
                   "section": beam.section.name,
                   "material": beam.material.name,
                   "n_elements": len(beam.elements),
               })
           return Table("Beam Elements", columns, rows)
   ```

2. Add helper functions:
   ```python
   def _format_coords(pos: List[float]) -> str:
       """Format coordinates as string."""
       return f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"

   def _format_dof_status(node) -> str:
       """Format DOF active status."""
       dof_names = ["UX", "UY", "UZ", "RX", "RY", "RZ", "W"]
       active = [name for i, name in enumerate(dof_names) if node.dof_active[i]]
       return ", ".join(active)
   ```

3. Add convenience methods to `StructuralModel`:
   ```python
   def get_node_table(self) -> Table:
       """Get tabular report of node coordinates."""
       ...

   def get_beam_table(self) -> Table:
       """Get tabular report of beam elements."""
       ...
   ```

**Acceptance Criteria:**
- [ ] Node table includes ID, coordinates (X, Y, Z), active DOFs
- [ ] Beam table includes ID, name, start/end coordinates, length, section, material
- [ ] Beam table shows number of FE elements per beam
- [ ] Coordinates formatted consistently with units

---

## Task 17.4: Input Data Reports - Boundary Conditions and Loads
**Requirements:** R-DATA-004
**Dependencies:** Task 17.1, Task 3.3 (BC handler), Task 5.1 (Loads)
**Difficulty:** Medium

**Description:**
Create reporters for boundary conditions and applied loads.

**Steps:**
1. Add to `src/grillex/reporting/input_reports.py`:
   ```python
   class BoundaryConditionTableBuilder:
       """Build boundary condition tables."""

       @staticmethod
       def build(model: "StructuralModel") -> Table:
           columns = [
               Column("Node", "node", ColumnType.TEXT),
               Column("DOF", "dof", ColumnType.TEXT),
               Column("Type", "type", ColumnType.TEXT),
               Column("Value", "value", ColumnType.FLOAT),
           ]
           # Extract BCs from cpp model
           ...
           return Table("Boundary Conditions", columns, rows)

   class LoadTableBuilder:
       """Build load tables per load case."""

       @staticmethod
       def build_nodal_loads(model: "StructuralModel",
                             load_case: str) -> Table:
           columns = [
               Column("Node", "node", ColumnType.TEXT),
               Column("DOF", "dof", ColumnType.TEXT),
               Column("Value", "value", ColumnType.FLOAT),
               Column("Unit", "unit", ColumnType.TEXT),
           ]
           ...
           return Table(f"Nodal Loads - {load_case}", columns, rows)

       @staticmethod
       def build_line_loads(model: "StructuralModel",
                           load_case: str) -> Table:
           columns = [
               Column("Beam", "beam", ColumnType.TEXT),
               Column("Type", "type", ColumnType.TEXT),
               Column("Direction", "direction", ColumnType.TEXT),
               Column("Start", "start_value", ColumnType.FLOAT, unit="kN/m"),
               Column("End", "end_value", ColumnType.FLOAT, unit="kN/m"),
           ]
           ...
           return Table(f"Line Loads - {load_case}", columns, rows)

       @staticmethod
       def build_accelerations(model: "StructuralModel",
                              load_case: str) -> Table:
           columns = [
               Column("Direction", "direction", ColumnType.TEXT),
               Column("Value", "value", ColumnType.FLOAT, unit="m/s²"),
           ]
           ...
           return Table(f"Accelerations - {load_case}", columns, rows)

   class LoadCombinationTableBuilder:
       """Build load combination tables."""

       @staticmethod
       def build(model: "StructuralModel") -> Table:
           columns = [
               Column("Name", "name", ColumnType.TEXT),
               Column("Type", "type", ColumnType.TEXT),
               Column("Factors", "factors", ColumnType.TEXT),
           ]
           ...
           return Table("Load Combinations", columns, rows)
   ```

2. Add convenience methods to `StructuralModel`:
   ```python
   def get_boundary_condition_table(self) -> Table:
       ...

   def get_load_table(self, load_case: Optional[str] = None) -> List[Table]:
       """Get load tables for specified or all load cases."""
       ...

   def get_load_combination_table(self) -> Table:
       ...
   ```

**Acceptance Criteria:**
- [ ] BC table shows node, DOF, type (fixed/prescribed), value
- [ ] Nodal load table shows node, DOF, value with appropriate units
- [ ] Line load table shows beam, type, direction, start/end values
- [ ] Acceleration table shows direction and value
- [ ] Load combination table shows name, type, and factor summary

---

## Task 17.4a: Input Data Reports - Cargo and Vessel Motions
**Requirements:** R-DATA-004, R-CARGO-001
**Dependencies:** Task 17.1, Phase 9 (Cargo), Phase 20 (Vessel Motions)
**Difficulty:** Medium

**Description:**
Create reporters for cargo definitions and vessel motion parameters.

**Steps:**
1. Add to `src/grillex/reporting/input_reports.py`:
   ```python
   class CargoTableBuilder:
       """Build cargo summary and connection tables."""

       @staticmethod
       def build_summary(model: "StructuralModel") -> Table:
           """Create cargo summary table."""
           columns = [
               Column("Name", "name", ColumnType.TEXT),
               Column("CoG X", "cog_x", ColumnType.FLOAT, unit="m"),
               Column("CoG Y", "cog_y", ColumnType.FLOAT, unit="m"),
               Column("CoG Z", "cog_z", ColumnType.FLOAT, unit="m"),
               Column("Mass", "mass", ColumnType.FLOAT, unit="mT"),
               Column("Ixx", "ixx", ColumnType.FLOAT, unit="mT·m²"),
               Column("Iyy", "iyy", ColumnType.FLOAT, unit="mT·m²"),
               Column("Izz", "izz", ColumnType.FLOAT, unit="mT·m²"),
               Column("Connections", "n_connections", ColumnType.INTEGER),
           ]
           rows = []
           for cargo in model.cargos:
               rows.append({
                   "name": cargo.name,
                   "cog_x": cargo.cog[0],
                   "cog_y": cargo.cog[1],
                   "cog_z": cargo.cog[2],
                   "mass": cargo.mass,
                   "ixx": cargo.inertia[0] if cargo.inertia else 0.0,
                   "iyy": cargo.inertia[1] if cargo.inertia else 0.0,
                   "izz": cargo.inertia[2] if cargo.inertia else 0.0,
                   "n_connections": len(cargo.connections),
               })
           return Table("Cargo Summary", columns, rows)

       @staticmethod
       def build_connections(model: "StructuralModel",
                            cargo_name: Optional[str] = None) -> Table:
           """Create cargo connections table."""
           columns = [
               Column("Cargo", "cargo", ColumnType.TEXT),
               Column("Connection", "connection", ColumnType.TEXT),
               Column("Position X", "pos_x", ColumnType.FLOAT, unit="m"),
               Column("Position Y", "pos_y", ColumnType.FLOAT, unit="m"),
               Column("Position Z", "pos_z", ColumnType.FLOAT, unit="m"),
               Column("kx", "kx", ColumnType.SCIENTIFIC, unit="kN/m"),
               Column("ky", "ky", ColumnType.SCIENTIFIC, unit="kN/m"),
               Column("kz", "kz", ColumnType.SCIENTIFIC, unit="kN/m"),
               Column("krx", "krx", ColumnType.SCIENTIFIC, unit="kNm/rad"),
               Column("kry", "kry", ColumnType.SCIENTIFIC, unit="kNm/rad"),
               Column("krz", "krz", ColumnType.SCIENTIFIC, unit="kNm/rad"),
               Column("Condition", "condition", ColumnType.TEXT),
           ]
           rows = []
           for cargo in model.cargos:
               if cargo_name and cargo.name != cargo_name:
                   continue
               for conn in cargo.connections:
                   rows.append({
                       "cargo": cargo.name,
                       "connection": conn.name,
                       "pos_x": conn.structural_position[0],
                       "pos_y": conn.structural_position[1],
                       "pos_z": conn.structural_position[2],
                       "kx": conn.stiffness[0],
                       "ky": conn.stiffness[1],
                       "kz": conn.stiffness[2],
                       "krx": conn.stiffness[3],
                       "kry": conn.stiffness[4],
                       "krz": conn.stiffness[5],
                       "condition": conn.loading_condition,
                   })
           return Table("Cargo Connections", columns, rows)

   class VesselMotionTableBuilder:
       """Build vessel motion tables."""

       @staticmethod
       def build_summary(model: "StructuralModel") -> Table:
           """Create vessel motion summary table."""
           columns = [
               Column("Name", "name", ColumnType.TEXT),
               Column("Center X", "center_x", ColumnType.FLOAT, unit="m"),
               Column("Center Y", "center_y", ColumnType.FLOAT, unit="m"),
               Column("Center Z", "center_z", ColumnType.FLOAT, unit="m"),
               Column("Components", "n_components", ColumnType.INTEGER),
               Column("Description", "description", ColumnType.TEXT),
           ]
           rows = []
           for vm in model._vessel_motion_generators:
               rows.append({
                   "name": vm.name,
                   "center_x": vm.motion_center[0],
                   "center_y": vm.motion_center[1],
                   "center_z": vm.motion_center[2],
                   "n_components": len(vm.components),
                   "description": vm.description or "",
               })
           return Table("Vessel Motions", columns, rows)

       @staticmethod
       def build_components(model: "StructuralModel",
                           motion_name: Optional[str] = None) -> Table:
           """Create vessel motion components table."""
           columns = [
               Column("Motion", "motion", ColumnType.TEXT),
               Column("Type", "type", ColumnType.TEXT),
               Column("Amplitude", "amplitude", ColumnType.FLOAT),
               Column("Unit", "unit", ColumnType.TEXT),
               Column("Phase", "phase", ColumnType.FLOAT, unit="rad"),
           ]
           rows = []
           for vm in model._vessel_motion_generators:
               if motion_name and vm.name != motion_name:
                   continue
               for comp in vm.components:
                   # Determine unit based on motion type
                   is_rotation = comp.motion_type.value in ("roll", "pitch", "yaw")
                   unit = "rad/s²" if is_rotation else "m/s²"
                   rows.append({
                       "motion": vm.name,
                       "type": comp.motion_type.value.capitalize(),
                       "amplitude": comp.amplitude,
                       "unit": unit,
                       "phase": comp.phase,
                   })
           return Table("Vessel Motion Components", columns, rows)

       @staticmethod
       def build_accelerations_matrix(model: "StructuralModel") -> Table:
           """Create acceleration matrix showing all motions and their 6-DOF accelerations."""
           columns = [
               Column("Motion", "motion", ColumnType.TEXT),
               Column("Surge", "surge", ColumnType.FLOAT, unit="m/s²"),
               Column("Sway", "sway", ColumnType.FLOAT, unit="m/s²"),
               Column("Heave", "heave", ColumnType.FLOAT, unit="m/s²"),
               Column("Roll", "roll", ColumnType.FLOAT, unit="rad/s²"),
               Column("Pitch", "pitch", ColumnType.FLOAT, unit="rad/s²"),
               Column("Yaw", "yaw", ColumnType.FLOAT, unit="rad/s²"),
           ]
           rows = []
           for vm in model._vessel_motion_generators:
               accel, _ = vm.get_acceleration_field()
               rows.append({
                   "motion": vm.name,
                   "surge": accel[0],
                   "sway": accel[1],
                   "heave": accel[2],
                   "roll": accel[3],
                   "pitch": accel[4],
                   "yaw": accel[5],
               })
           return Table("Vessel Motion Accelerations", columns, rows)
   ```

2. Add convenience methods to `StructuralModel`:
   ```python
   def get_cargo_table(self) -> Table:
       """Get tabular summary of all cargo items."""
       from grillex.reporting import CargoTableBuilder
       return CargoTableBuilder.build_summary(self)

   def get_cargo_connections_table(self, cargo_name: Optional[str] = None) -> Table:
       """Get tabular report of cargo connections."""
       from grillex.reporting import CargoTableBuilder
       return CargoTableBuilder.build_connections(self, cargo_name)

   def get_vessel_motion_table(self) -> Table:
       """Get tabular summary of vessel motions."""
       from grillex.reporting import VesselMotionTableBuilder
       return VesselMotionTableBuilder.build_summary(self)

   def get_vessel_motion_accelerations_table(self) -> Table:
       """Get acceleration matrix for all vessel motions."""
       from grillex.reporting import VesselMotionTableBuilder
       return VesselMotionTableBuilder.build_accelerations_matrix(self)
   ```

**Acceptance Criteria:**
- [ ] Cargo summary table includes name, CoG position, mass, inertia, connection count
- [ ] Cargo connections table includes position, 6-DOF stiffnesses, loading condition
- [ ] Cargo connections can be filtered by cargo name
- [ ] Vessel motion summary includes name, motion center, component count
- [ ] Vessel motion components table shows type, amplitude with correct units
- [ ] Vessel motion accelerations matrix shows all 6-DOF accelerations
- [ ] Empty cargo/vessel motion lists handled gracefully

---

## Task 17.5: FEM Results Reports - Displacements
**Requirements:** R-DATA-004, R-RES-001
**Dependencies:** Task 17.1, Task 3.4 (Solver), Task 4.3 (Results access)
**Difficulty:** Medium

**Description:**
Create reporters for nodal displacement results.

**Steps:**
1. Create `src/grillex/reporting/result_reports.py`:
   ```python
   class DisplacementTableBuilder:
       """Build displacement result tables."""

       @staticmethod
       def build(model: "StructuralModel",
                load_case: Optional[str] = None,
                load_combination: Optional[str] = None,
                nodes: Optional[List[int]] = None) -> Table:
           """
           Build displacement table.

           Args:
               model: Analyzed structural model
               load_case: Specific load case name (optional)
               load_combination: Specific combination name (optional)
               nodes: Filter to specific node IDs (optional)
           """
           columns = [
               Column("Node", "node_id", ColumnType.INTEGER),
               Column("X", "x", ColumnType.FLOAT, unit="m"),
               Column("Y", "y", ColumnType.FLOAT, unit="m"),
               Column("Z", "z", ColumnType.FLOAT, unit="m"),
               Column("UX", "ux", ColumnType.SCIENTIFIC, unit="m"),
               Column("UY", "uy", ColumnType.SCIENTIFIC, unit="m"),
               Column("UZ", "uz", ColumnType.SCIENTIFIC, unit="m"),
               Column("RX", "rx", ColumnType.SCIENTIFIC, unit="rad"),
               Column("RY", "ry", ColumnType.SCIENTIFIC, unit="rad"),
               Column("RZ", "rz", ColumnType.SCIENTIFIC, unit="rad"),
           ]
           rows = []
           # Extract displacements from analysis results
           for pos, node in model._node_map.items():
               if nodes and node.id not in nodes:
                   continue
               row = {
                   "node_id": node.id,
                   "x": pos[0], "y": pos[1], "z": pos[2],
               }
               for dof_idx, dof_name in enumerate(["ux", "uy", "uz", "rx", "ry", "rz"]):
                   row[dof_name] = model.get_displacement_at(
                       list(pos), dof_idx, load_case=load_case
                   )
               rows.append(row)

           title = "Nodal Displacements"
           if load_case:
               title += f" - {load_case}"
           elif load_combination:
               title += f" - {load_combination}"

           return Table(title, columns, rows)

       @staticmethod
       def build_summary(model: "StructuralModel",
                        load_case: Optional[str] = None) -> Table:
           """Build displacement summary (max values)."""
           columns = [
               Column("DOF", "dof", ColumnType.TEXT),
               Column("Max Value", "max_value", ColumnType.SCIENTIFIC),
               Column("Node", "node_id", ColumnType.INTEGER),
               Column("Location", "location", ColumnType.TEXT),
               Column("Unit", "unit", ColumnType.TEXT),
           ]
           # Find maximum displacements per DOF
           ...
           return Table("Displacement Summary", columns, rows)
   ```

2. Add convenience methods to `StructuralModel`:
   ```python
   def get_displacement_table(self, load_case: Optional[str] = None,
                              load_combination: Optional[str] = None,
                              nodes: Optional[List[int]] = None) -> Table:
       """Get tabular displacement results."""
       ...

   def get_displacement_summary(self, load_case: Optional[str] = None) -> Table:
       """Get summary of maximum displacements."""
       ...
   ```

**Acceptance Criteria:**
- [ ] Displacement table includes all 6 DOFs with correct units
- [ ] Results can be filtered by load case or load combination
- [ ] Results can be filtered to specific nodes
- [ ] Summary table shows maximum values per DOF with locations
- [ ] Scientific notation used for small values

---

## Task 17.6: FEM Results Reports - Reactions
**Requirements:** R-DATA-004, R-RES-001
**Dependencies:** Task 17.1, Task 3.4 (Solver)
**Difficulty:** Medium

**Description:**
Create reporters for support reaction results.

**Steps:**
1. Add to `src/grillex/reporting/result_reports.py`:
   ```python
   class ReactionTableBuilder:
       """Build reaction result tables."""

       @staticmethod
       def build(model: "StructuralModel",
                load_case: Optional[str] = None,
                load_combination: Optional[str] = None) -> Table:
           columns = [
               Column("Node", "node_id", ColumnType.INTEGER),
               Column("X", "x", ColumnType.FLOAT, unit="m"),
               Column("Y", "y", ColumnType.FLOAT, unit="m"),
               Column("Z", "z", ColumnType.FLOAT, unit="m"),
               Column("FX", "fx", ColumnType.FLOAT, unit="kN"),
               Column("FY", "fy", ColumnType.FLOAT, unit="kN"),
               Column("FZ", "fz", ColumnType.FLOAT, unit="kN"),
               Column("MX", "mx", ColumnType.FLOAT, unit="kNm"),
               Column("MY", "my", ColumnType.FLOAT, unit="kNm"),
               Column("MZ", "mz", ColumnType.FLOAT, unit="kNm"),
           ]
           # Extract reactions at support nodes
           ...
           return Table("Support Reactions", columns, rows)

       @staticmethod
       def build_totals(model: "StructuralModel",
                       load_case: Optional[str] = None) -> Table:
           """Build reaction totals table."""
           columns = [
               Column("Component", "component", ColumnType.TEXT),
               Column("Total", "total", ColumnType.FLOAT),
               Column("Unit", "unit", ColumnType.TEXT),
           ]
           # Sum reactions
           ...
           return Table("Reaction Totals", columns, rows)
   ```

2. Add convenience methods to `StructuralModel`:
   ```python
   def get_reaction_table(self, load_case: Optional[str] = None,
                          load_combination: Optional[str] = None) -> Table:
       """Get tabular reaction results at supports."""
       ...

   def get_reaction_totals(self, load_case: Optional[str] = None) -> Table:
       """Get sum of all reactions for equilibrium check."""
       ...
   ```

**Acceptance Criteria:**
- [ ] Reaction table includes all 6 components with correct units
- [ ] Only support nodes (with BCs) are included
- [ ] Totals table shows sum of forces and moments
- [ ] Results can be filtered by load case or combination

---

## Task 17.7: FEM Results Reports - Internal Actions
**Requirements:** R-DATA-004, R-RES-001
**Dependencies:** Task 17.1, Task 7.1 (Internal actions)
**Difficulty:** Medium

**Description:**
Create reporters for beam internal action results.

**Steps:**
1. Add to `src/grillex/reporting/result_reports.py`:
   ```python
   class InternalActionTableBuilder:
       """Build internal action tables."""

       @staticmethod
       def build_at_locations(model: "StructuralModel",
                             beam: "Beam",
                             positions: List[float],
                             load_case: Optional[str] = None) -> Table:
           """
           Build internal actions at specified positions along beam.

           Args:
               model: Analyzed model
               beam: Beam to report
               positions: List of x positions [m] along beam
               load_case: Specific load case (optional)
           """
           columns = [
               Column("x", "x", ColumnType.FLOAT, unit="m"),
               Column("x/L", "x_norm", ColumnType.FLOAT),
               Column("N", "N", ColumnType.FLOAT, unit="kN"),
               Column("Vy", "Vy", ColumnType.FLOAT, unit="kN"),
               Column("Vz", "Vz", ColumnType.FLOAT, unit="kN"),
               Column("Mx", "Mx", ColumnType.FLOAT, unit="kNm"),
               Column("My", "My", ColumnType.FLOAT, unit="kNm"),
               Column("Mz", "Mz", ColumnType.FLOAT, unit="kNm"),
           ]
           rows = []
           for x in positions:
               actions = beam.get_internal_actions_at(x, model, load_case)
               rows.append({
                   "x": x,
                   "x_norm": x / beam.length,
                   "N": actions.N,
                   "Vy": actions.Vy,
                   "Vz": actions.Vz,
                   "Mx": actions.Mx,
                   "My": actions.My,
                   "Mz": actions.Mz,
               })

           title = f"Internal Actions - {beam.name or f'Beam {beam.beam_id}'}"
           if load_case:
               title += f" - {load_case}"
           return Table(title, columns, rows)

       @staticmethod
       def build_at_check_locations(model: "StructuralModel",
                                    beam: "Beam",
                                    load_case: Optional[str] = None) -> Table:
           """Build internal actions at beam check locations."""
           positions = [loc * beam.length for loc in beam.check_locations]
           return InternalActionTableBuilder.build_at_locations(
               model, beam, positions, load_case
           )

       @staticmethod
       def build_extrema(model: "StructuralModel",
                        beam: "Beam",
                        load_case: Optional[str] = None) -> Table:
           """Build table of internal action extrema."""
           columns = [
               Column("Component", "component", ColumnType.TEXT),
               Column("Max", "max_value", ColumnType.FLOAT),
               Column("x (max)", "x_max", ColumnType.FLOAT, unit="m"),
               Column("Min", "min_value", ColumnType.FLOAT),
               Column("x (min)", "x_min", ColumnType.FLOAT, unit="m"),
               Column("Unit", "unit", ColumnType.TEXT),
           ]
           # Extract extrema for each component
           ...
           return Table("Internal Action Extrema", columns, rows)

       @staticmethod
       def build_all_beams_summary(model: "StructuralModel",
                                   load_case: Optional[str] = None) -> Table:
           """Build summary table of max internal actions for all beams."""
           columns = [
               Column("Beam", "beam", ColumnType.TEXT),
               Column("|N|max", "N_max", ColumnType.FLOAT, unit="kN"),
               Column("|Vy|max", "Vy_max", ColumnType.FLOAT, unit="kN"),
               Column("|Vz|max", "Vz_max", ColumnType.FLOAT, unit="kN"),
               Column("|Mx|max", "Mx_max", ColumnType.FLOAT, unit="kNm"),
               Column("|My|max", "My_max", ColumnType.FLOAT, unit="kNm"),
               Column("|Mz|max", "Mz_max", ColumnType.FLOAT, unit="kNm"),
           ]
           rows = []
           for beam in model.beams:
               # Get extrema for each beam
               ...
           return Table("Internal Actions Summary - All Beams", columns, rows)
   ```

2. Add convenience methods to `Beam` class:
   ```python
   def get_internal_action_table(self, model: "StructuralModel",
                                 positions: Optional[List[float]] = None,
                                 load_case: Optional[str] = None) -> Table:
       """Get tabular internal actions at specified positions."""
       ...

   def get_internal_action_extrema_table(self, model: "StructuralModel",
                                         load_case: Optional[str] = None) -> Table:
       """Get table of internal action extrema."""
       ...
   ```

3. Add convenience methods to `StructuralModel`:
   ```python
   def get_internal_action_summary(self, load_case: Optional[str] = None) -> Table:
       """Get summary of internal actions for all beams."""
       ...
   ```

**Acceptance Criteria:**
- [ ] Internal action table includes N, Vy, Vz, Mx, My, Mz with units
- [ ] Tables can be generated at arbitrary positions or check locations
- [ ] Extrema table shows max/min values with locations
- [ ] Summary table covers all beams in model
- [ ] Warping actions (B, Mx_sv, Mx_w) included for 14-DOF elements

---

## Task 17.8: Design Code Check Reports
**Requirements:** R-DATA-004, R-CODE-001
**Dependencies:** Task 17.1, Phase 10 (Design codes)
**Difficulty:** Medium

**Description:**
Create reporters for design code check results.

**Steps:**
1. Create `src/grillex/reporting/code_check_reports.py`:
   ```python
   class CodeCheckTableBuilder:
       """Build design code check result tables."""

       @staticmethod
       def build_beam_checks(results: List["CheckResult"],
                            beam_name: str) -> Table:
           """Build check result table for single beam."""
           columns = [
               Column("Location", "location", ColumnType.FLOAT),
               Column("Check", "check_name", ColumnType.TEXT),
               Column("Utilization", "utilization", ColumnType.PERCENTAGE),
               Column("Status", "status", ColumnType.STATUS),
               Column("Load Combo", "load_combination", ColumnType.TEXT),
               Column("Governing", "governing", ColumnType.TEXT),
           ]
           rows = []
           for result in results:
               rows.append({
                   "location": result.location,
                   "check_name": result.check_name,
                   "utilization": result.utilization * 100,
                   "status": result.status,
                   "load_combination": result.load_combination,
                   "governing": "✓" if result.governing else "",
               })
           return Table(f"Design Checks - {beam_name}", columns, rows)

       @staticmethod
       def build_summary(design_code: "DesignCode",
                        all_results: Dict[str, List["CheckResult"]]) -> Table:
           """Build summary table across all elements."""
           columns = [
               Column("Beam", "beam", ColumnType.TEXT),
               Column("Checks", "n_checks", ColumnType.INTEGER),
               Column("Max Util", "max_util", ColumnType.PERCENTAGE),
               Column("Critical Check", "critical_check", ColumnType.TEXT),
               Column("Status", "status", ColumnType.STATUS),
           ]
           rows = []
           for beam_name, results in all_results.items():
               summary = design_code.get_summary(results)
               max_result = max(results, key=lambda r: r.utilization)
               rows.append({
                   "beam": beam_name,
                   "n_checks": summary["total"],
                   "max_util": summary["max_utilization"] * 100,
                   "critical_check": max_result.check_name,
                   "status": "PASS" if summary["failed"] == 0 else "FAIL",
               })
           return Table(f"Design Check Summary - {design_code.name}", columns, rows)

       @staticmethod
       def build_governing_checks(design_code: "DesignCode",
                                  results: List["CheckResult"]) -> Table:
           """Build table of only governing checks."""
           governing = design_code.get_governing_results(results)
           columns = [
               Column("Beam", "beam", ColumnType.TEXT),
               Column("Location", "location", ColumnType.FLOAT),
               Column("Check", "check_name", ColumnType.TEXT),
               Column("Utilization", "utilization", ColumnType.PERCENTAGE),
               Column("Status", "status", ColumnType.STATUS),
               Column("Load Combo", "load_combination", ColumnType.TEXT),
           ]
           rows = []
           for result in governing:
               rows.append({
                   "beam": result.element_id,
                   "location": result.location,
                   "check_name": result.check_name,
                   "utilization": result.utilization * 100,
                   "status": result.status,
                   "load_combination": result.load_combination,
               })
           return Table("Governing Design Checks", columns, rows)
   ```

2. Add convenience methods to `DesignCode` base class:
   ```python
   def get_results_table(self, results: List[CheckResult],
                        element_id: Optional[str] = None) -> Table:
       """Get tabular check results."""
       ...

   def get_summary_table(self, results: List[CheckResult]) -> Table:
       """Get summary table of all checks."""
       ...

   def get_governing_table(self, results: List[CheckResult]) -> Table:
       """Get table of only governing checks."""
       ...
   ```

**Acceptance Criteria:**
- [ ] Check result table includes location, check name, utilization, status
- [ ] Summary table shows max utilization per element
- [ ] Governing checks table filters to critical results
- [ ] Status column shows PASS/FAIL formatting
- [ ] Utilization shown as percentage

---

## Task 17.9: Model Input Summary Report
**Requirements:** R-DATA-004
**Dependencies:** Tasks 17.2-17.4
**Difficulty:** Low

**Description:**
Create a comprehensive model input summary combining all input tables.

**Steps:**
1. Add to `src/grillex/reporting/input_reports.py`:
   ```python
   class ModelInputReportBuilder:
       """Build complete model input report."""

       @staticmethod
       def build(model: "StructuralModel") -> Report:
           """Create comprehensive input report."""
           report = Report(
               title=f"Model Input Report - {model.name}",
               metadata={
                   "model_name": model.name,
                   "generated": datetime.now().isoformat(),
               }
           )

           # Model overview
           overview = Table(
               title="Model Overview",
               columns=[
                   Column("Property", "property", ColumnType.TEXT),
                   Column("Value", "value", ColumnType.TEXT),
               ],
               rows=[
                   {"property": "Model Name", "value": model.name},
                   {"property": "Total Nodes", "value": str(len(model._node_map))},
                   {"property": "Total Beams", "value": str(len(model.beams))},
                   {"property": "Total Materials", "value": str(len(model._materials))},
                   {"property": "Total Sections", "value": str(len(model._sections))},
                   {"property": "Load Cases", "value": str(model.get_load_case_count())},
                   {"property": "Load Combinations", "value": str(len(model._load_combinations))},
               ]
           )
           report.add_table(overview)

           # Add all input tables
           report.add_table(MaterialTableBuilder.build(model))
           report.add_table(SectionTableBuilder.build(model))
           report.add_table(NodeTableBuilder.build(model))
           report.add_table(BeamTableBuilder.build(model))
           report.add_table(BoundaryConditionTableBuilder.build(model))

           # Add load tables for each load case
           for lc_name in model.get_load_case_names():
               report.add_table(LoadTableBuilder.build_nodal_loads(model, lc_name))

           if model._load_combinations:
               report.add_table(LoadCombinationTableBuilder.build(model))

           return report
   ```

2. Add convenience method to `StructuralModel`:
   ```python
   def get_input_report(self) -> Report:
       """Get comprehensive input data report."""
       from grillex.reporting import ModelInputReportBuilder
       return ModelInputReportBuilder.build(self)
   ```

**Acceptance Criteria:**
- [ ] Report includes model overview table
- [ ] Report includes all input data tables
- [ ] Report can be exported to JSON, Markdown, text
- [ ] Metadata includes generation timestamp

---

## Task 17.10: Analysis Results Report
**Requirements:** R-DATA-004
**Dependencies:** Tasks 17.5-17.7
**Difficulty:** Low

**Description:**
Create comprehensive analysis results report combining all result tables.

**Steps:**
1. Create `src/grillex/reporting/result_reports.py` (add to existing):
   ```python
   class AnalysisResultsReportBuilder:
       """Build complete analysis results report."""

       @staticmethod
       def build(model: "StructuralModel",
                load_case: Optional[str] = None,
                load_combination: Optional[str] = None,
                include_all_beams: bool = True) -> Report:
           """Create comprehensive results report."""
           report = Report(
               title=f"Analysis Results - {model.name}",
               metadata={
                   "model_name": model.name,
                   "load_case": load_case,
                   "load_combination": load_combination,
                   "generated": datetime.now().isoformat(),
               }
           )

           # Displacement summary and details
           report.add_table(DisplacementTableBuilder.build_summary(model, load_case))
           report.add_table(DisplacementTableBuilder.build(model, load_case))

           # Reactions
           report.add_table(ReactionTableBuilder.build(model, load_case))
           report.add_table(ReactionTableBuilder.build_totals(model, load_case))

           # Internal actions summary
           report.add_table(InternalActionTableBuilder.build_all_beams_summary(
               model, load_case
           ))

           # Detailed internal actions per beam (optional)
           if include_all_beams:
               for beam in model.beams:
                   report.add_table(InternalActionTableBuilder.build_at_check_locations(
                       model, beam, load_case
                   ))

           return report
   ```

2. Add convenience method to `StructuralModel`:
   ```python
   def get_results_report(self, load_case: Optional[str] = None,
                          load_combination: Optional[str] = None,
                          include_all_beams: bool = True) -> Report:
       """Get comprehensive analysis results report."""
       from grillex.reporting import AnalysisResultsReportBuilder
       return AnalysisResultsReportBuilder.build(
           self, load_case, load_combination, include_all_beams
       )
   ```

**Acceptance Criteria:**
- [ ] Report includes displacement summary and details
- [ ] Report includes reactions with totals
- [ ] Report includes internal action summary
- [ ] Optional detailed internal actions per beam
- [ ] Report can be exported to JSON, Markdown, text

---

## Task 17.11: Design Check Report
**Requirements:** R-DATA-004, R-CODE-001
**Dependencies:** Task 17.8, Phase 10
**Difficulty:** Low

**Description:**
Create comprehensive design code check report.

**Steps:**
1. Add to `src/grillex/reporting/code_check_reports.py`:
   ```python
   class DesignCheckReportBuilder:
       """Build complete design check report."""

       @staticmethod
       def build(design_code: "DesignCode",
                all_results: Dict[str, List["CheckResult"]]) -> Report:
           """Create comprehensive design check report."""
           report = Report(
               title=f"Design Check Report - {design_code.name}",
               metadata={
                   "design_code": design_code.name,
                   "version": design_code.version,
                   "generated": datetime.now().isoformat(),
               }
           )

           # Overall summary
           all_checks = [r for results in all_results.values() for r in results]
           summary = design_code.get_summary(all_checks)

           overview = Table(
               title="Design Check Overview",
               columns=[
                   Column("Metric", "metric", ColumnType.TEXT),
                   Column("Value", "value", ColumnType.TEXT),
               ],
               rows=[
                   {"metric": "Design Code", "value": design_code.name},
                   {"metric": "Total Checks", "value": str(summary["total"])},
                   {"metric": "Passed", "value": str(summary["passed"])},
                   {"metric": "Failed", "value": str(summary["failed"])},
                   {"metric": "Max Utilization", "value": f"{summary['max_utilization']*100:.1f}%"},
                   {"metric": "Overall Status", "value": "PASS" if summary["failed"] == 0 else "FAIL"},
               ]
           )
           report.add_table(overview)

           # Summary by element
           report.add_table(CodeCheckTableBuilder.build_summary(
               design_code, all_results
           ))

           # Governing checks
           report.add_table(CodeCheckTableBuilder.build_governing_checks(
               design_code, all_checks
           ))

           # Detailed checks per element
           for beam_name, results in all_results.items():
               report.add_table(CodeCheckTableBuilder.build_beam_checks(
                   results, beam_name
               ))

           return report
   ```

2. Add convenience method to `DesignCode`:
   ```python
   def get_report(self, all_results: Dict[str, List[CheckResult]]) -> Report:
       """Get comprehensive design check report."""
       from grillex.reporting import DesignCheckReportBuilder
       return DesignCheckReportBuilder.build(self, all_results)
   ```

**Acceptance Criteria:**
- [ ] Report includes design check overview
- [ ] Report includes summary by element
- [ ] Report includes governing checks table
- [ ] Report includes detailed checks per element
- [ ] Overall pass/fail status clearly shown

---

## Task 17.12: Export to Multiple Formats
**Requirements:** R-DATA-004
**Dependencies:** Task 17.1
**Difficulty:** Medium

**Description:**
Implement export functionality for various formats.

**Steps:**
1. Add to `src/grillex/reporting/exporters.py`:
   ```python
   class CSVExporter:
       """Export tables to CSV files."""

       @staticmethod
       def export_table(table: Table, path: str) -> None:
           """Export single table to CSV."""
           ...

       @staticmethod
       def export_report(report: Report, directory: str) -> List[str]:
           """Export report as multiple CSV files."""
           ...

   class MarkdownExporter:
       """Export to Markdown format."""

       @staticmethod
       def export_table(table: Table) -> str:
           """Render table as Markdown."""
           ...

       @staticmethod
       def export_report(report: Report, path: str) -> None:
           """Export report as Markdown file."""
           ...

   class JSONExporter:
       """Export to JSON format."""

       @staticmethod
       def export_table(table: Table) -> str:
           """Render table as JSON."""
           ...

       @staticmethod
       def export_report(report: Report, path: str, indent: int = 2) -> None:
           """Export report as JSON file."""
           ...

   class TextExporter:
       """Export to formatted text."""

       @staticmethod
       def export_table(table: Table, max_width: int = 120) -> str:
           """Render table as formatted text."""
           ...

       @staticmethod
       def export_report(report: Report, path: Optional[str] = None) -> str:
           """Export report as formatted text."""
           ...

   class ExcelExporter:
       """Export to Excel format (optional, requires openpyxl)."""

       @staticmethod
       def export_report(report: Report, path: str) -> None:
           """Export report as Excel workbook with multiple sheets."""
           try:
               import openpyxl
           except ImportError:
               raise ImportError("openpyxl required for Excel export: pip install openpyxl")
           ...
   ```

2. Add export methods to `Report` class:
   ```python
   def to_csv(self, directory: str) -> List[str]:
       """Export all tables to CSV files in directory."""
       ...

   def to_markdown(self, path: Optional[str] = None) -> str:
       """Export to Markdown. If path provided, writes to file."""
       ...

   def to_json(self, path: Optional[str] = None, indent: int = 2) -> str:
       """Export to JSON. If path provided, writes to file."""
       ...

   def to_text(self, path: Optional[str] = None) -> str:
       """Export to formatted text. If path provided, writes to file."""
       ...

   def to_excel(self, path: str) -> None:
       """Export to Excel workbook (requires openpyxl)."""
       ...
   ```

**Acceptance Criteria:**
- [ ] CSV export works for single tables and full reports
- [ ] Markdown export produces valid GitHub-flavored Markdown
- [ ] JSON export produces valid, indented JSON
- [ ] Text export produces readable ASCII tables
- [ ] Excel export optional (graceful error if openpyxl not installed)
- [ ] All exports preserve units in headers/metadata

---

## Task 17.13: LLM Tool Integration
**Requirements:** R-LLM-003
**Dependencies:** Task 17.1-17.12, Phase 12 (LLM tooling)
**Difficulty:** Medium

**Description:**
Add LLM tool schemas for report generation.

**Steps:**
1. Add to `src/grillex/llm/tools.py`:
   ```python
   # Report generation tools
   {
       "name": "get_material_table",
       "description": "Get tabular report of all material properties in the model",
       "input_schema": {
           "type": "object",
           "properties": {
               "format": {
                   "type": "string",
                   "description": "Output format: 'text', 'markdown', 'json', 'csv'",
                   "enum": ["text", "markdown", "json", "csv"],
                   "default": "text"
               }
           }
       }
   },
   {
       "name": "get_displacement_table",
       "description": "Get tabular report of nodal displacements after analysis",
       "input_schema": {
           "type": "object",
           "properties": {
               "load_case": {
                   "type": "string",
                   "description": "Name of load case to report (optional)"
               },
               "nodes": {
                   "type": "array",
                   "items": {"type": "integer"},
                   "description": "Filter to specific node IDs (optional)"
               },
               "format": {
                   "type": "string",
                   "enum": ["text", "markdown", "json"],
                   "default": "text"
               }
           }
       }
   },
   {
       "name": "get_internal_action_table",
       "description": "Get tabular report of internal actions (N, V, M) along a beam",
       "input_schema": {
           "type": "object",
           "properties": {
               "beam_id": {
                   "type": "integer",
                   "description": "ID of beam to report"
               },
               "positions": {
                   "type": "array",
                   "items": {"type": "number"},
                   "description": "Positions along beam in meters (optional, defaults to check locations)"
               },
               "load_case": {
                   "type": "string",
                   "description": "Name of load case (optional)"
               },
               "format": {
                   "type": "string",
                   "enum": ["text", "markdown", "json"],
                   "default": "text"
               }
           },
           "required": ["beam_id"]
       }
   },
   {
       "name": "get_design_check_table",
       "description": "Get tabular report of design code check results",
       "input_schema": {
           "type": "object",
           "properties": {
               "beam_id": {
                   "type": "integer",
                   "description": "ID of beam to report (optional, all beams if not specified)"
               },
               "governing_only": {
                   "type": "boolean",
                   "description": "Only show governing checks",
                   "default": false
               },
               "format": {
                   "type": "string",
                   "enum": ["text", "markdown", "json"],
                   "default": "text"
               }
           }
       }
   },
   {
       "name": "generate_input_report",
       "description": "Generate comprehensive model input report",
       "input_schema": {
           "type": "object",
           "properties": {
               "format": {
                   "type": "string",
                   "enum": ["text", "markdown", "json"],
                   "default": "markdown"
               },
               "output_path": {
                   "type": "string",
                   "description": "File path to save report (optional)"
               }
           }
       }
   },
   {
       "name": "generate_results_report",
       "description": "Generate comprehensive analysis results report",
       "input_schema": {
           "type": "object",
           "properties": {
               "load_case": {
                   "type": "string",
                   "description": "Load case to report (optional)"
               },
               "include_beam_details": {
                   "type": "boolean",
                   "description": "Include detailed internal actions per beam",
                   "default": true
               },
               "format": {
                   "type": "string",
                   "enum": ["text", "markdown", "json"],
                   "default": "markdown"
               },
               "output_path": {
                   "type": "string",
                   "description": "File path to save report (optional)"
               }
           }
       }
   }
   ```

2. Add handlers in `ToolExecutor`:
   ```python
   def _execute_get_material_table(self, params: Dict) -> str:
       ...

   def _execute_get_displacement_table(self, params: Dict) -> str:
       ...

   def _execute_generate_input_report(self, params: Dict) -> str:
       ...
   ```

**Acceptance Criteria:**
- [ ] All table builders have corresponding LLM tools
- [ ] Report generators have LLM tools
- [ ] Tools support multiple output formats
- [ ] Tool descriptions are clear for LLM understanding

---

## Task 17.14: Unit Tests
**Requirements:** Testing
**Dependencies:** Tasks 17.1-17.13
**Difficulty:** Medium

**Description:**
Create comprehensive test suite for reporting modules.

**Steps:**
1. Create `tests/python/test_phase17_reporting.py`:
   - Test Table class creation and methods
   - Test Column type formatting
   - Test each TableBuilder
   - Test Report class
   - Test export to all formats
   - Test integration with StructuralModel

2. Test cases:
   ```python
   class TestTable:
       def test_table_creation(self):
           ...

       def test_to_markdown(self):
           ...

       def test_to_csv(self):
           ...

       def test_to_dict(self):
           ...

   class TestMaterialTableBuilder:
       def test_build_with_materials(self):
           ...

       def test_empty_model(self):
           ...

   class TestDisplacementTableBuilder:
       def test_build_after_analysis(self):
           ...

       def test_filter_by_nodes(self):
           ...

   class TestInternalActionTableBuilder:
       def test_build_at_check_locations(self):
           ...

       def test_extrema_table(self):
           ...

   class TestCodeCheckTableBuilder:
       def test_build_beam_checks(self):
           ...

       def test_summary_table(self):
           ...

   class TestModelInputReport:
       def test_complete_report(self):
           ...

       def test_export_formats(self):
           ...

   class TestAnalysisResultsReport:
       def test_complete_report(self):
           ...
   ```

**Acceptance Criteria:**
- [ ] >90% code coverage for reporting module
- [ ] All table builders tested
- [ ] All export formats tested
- [ ] Integration with model wrapper tested
- [ ] Edge cases (empty models, missing data) handled

---

## Task 17.15: Documentation
**Requirements:** R-DOC-001
**Dependencies:** Tasks 17.1-17.14
**Difficulty:** Low

**Description:**
Create documentation for reporting module.

**Steps:**
1. Create `docs/user/reporting.rst`:
   - Overview of reporting capabilities
   - Table and Report classes
   - Available table builders
   - Export formats
   - Usage examples with doctests

2. Update `docs/index.rst` to include reporting documentation

3. Add docstrings to all public classes and methods

**Acceptance Criteria:**
- [ ] User documentation with examples
- [ ] Doctests pass
- [ ] API reference complete
- [ ] Examples show all export formats

---

## Summary

### Dependencies Graph
```
17.1 Framework ─┬─> 17.2 Materials/Sections ─┐
                ├─> 17.3 Nodes/Elements ─────┤
                ├─> 17.4 BCs/Loads ──────────┼─> 17.9 Input Report
                ├─> 17.5 Displacements ──────┤
                ├─> 17.6 Reactions ──────────┼─> 17.10 Results Report
                ├─> 17.7 Internal Actions ───┤
                └─> 17.8 Code Checks ────────┴─> 17.11 Check Report
                                                      │
                                            17.12 Exporters
                                                      │
                                            17.13 LLM Tools
                                                      │
                                        17.14 Tests ──┴─> 17.15 Docs
```

### Estimated Task Complexity
| Task | Complexity | Priority |
|------|------------|----------|
| 17.1 Framework | Medium | High |
| 17.2 Materials/Sections | Low | High |
| 17.3 Nodes/Elements | Low | High |
| 17.4 BCs/Loads | Medium | High |
| 17.5 Displacements | Medium | High |
| 17.6 Reactions | Medium | High |
| 17.7 Internal Actions | Medium | High |
| 17.8 Code Checks | Medium | Medium |
| 17.9 Input Report | Low | High |
| 17.10 Results Report | Low | High |
| 17.11 Check Report | Low | Medium |
| 17.12 Exporters | Medium | High |
| 17.13 LLM Tools | Medium | Medium |
| 17.14 Tests | Medium | High |
| 17.15 Documentation | Low | Medium |

### Future Extensions (Beyond Phase 17)
- **HTML Report Generation**: Interactive HTML reports with collapsible sections
- **PDF Report Generation**: Professional engineering report PDFs with headers/footers
- **Plot Integration**: Embed moment/shear diagrams in reports
- **Template System**: Customizable report templates
- **Comparison Reports**: Compare results between load cases or model versions
