## Phase 4: Python Front-End & I/O

### Task 4.1: Create Python Model API
**Requirements:** R-MOD-001, R-MOD-002, R-ARCH-003
**Dependencies:** Task 3.5
**Difficulty:** Medium

**Description:**
Create Pythonic wrapper around C++ Model class.

**Steps:**
1. Create `grillex/core/model.py`:
   ```python
   from grillex._grillex_cpp import Model as _CppModel

   class Beam:
       """Represents a beam in the model."""
       def __init__(self, end_a_position, end_b_position, section, material, **kwargs):
           self.end_a_position = end_a_position
           self.end_b_position = end_b_position
           # ... store other properties

   class Model:
       """High-level Python model interface."""

       def __init__(self, name: str = "Unnamed Model"):
           self._cpp_model = _CppModel()
           self.name = name
           self.beams: list[Beam] = []

       def add_beam(self, end_a_position: list[float], end_b_position: list[float],
                    section: str, material: str, **kwargs) -> Beam:
           """Add a beam using endpoint coordinates."""
           beam = Beam(end_a_position, end_b_position, section, material, **kwargs)
           self.beams.append(beam)
           return beam

       def analyze(self) -> "ResultCase":
           """Run linear static analysis."""
           # Build C++ model from Python objects
           # Run analysis
           # Return results
   ```

2. Support both coordinate-based and node-reference-based beam creation

**Acceptance Criteria:**
- [ ] Beams can be created with coordinate lists
- [ ] Model can be analyzed from Python
- [ ] Results are accessible from Python

---

### Task 4.2: Implement YAML Input Parser
**Requirements:** R-DATA-001, R-DATA-002, R-DATA-003, R-DATA-004
**Dependencies:** Task 4.1
**Difficulty:** Medium

**Description:**
Implement YAML loading for model input.

**Steps:**
1. Create `grillex/io/yaml_loader.py`:
   ```python
   import yaml
   from typing import Any
   from grillex.core.model import Model

   def load_model_from_yaml(file_path: str) -> Model:
       """Load a model from YAML file."""
       with open(file_path, 'r') as f:
           data = yaml.safe_load(f)
       return build_model_from_dict(data)

   def build_model_from_dict(data: dict[str, Any]) -> Model:
       """Build model from dictionary (YAML structure)."""
       model = Model(name=data.get('name', 'Unnamed'))

       # Load materials
       for mat_data in data.get('Material', []):
           model.add_material(**mat_data)

       # Load sections
       for sec_data in data.get('Section', []):
           model.add_section(**sec_data)

       # Load beams
       for beam_data in data.get('Beam', []):
           model.add_beam(**beam_data)

       # ... etc for other entity types
       return model
   ```

2. Define the YAML schema (document expected structure):
   ```yaml
   # Example model.yaml
   name: "Simple Beam"

   Material:
     - name: Steel
       E: 210000000  # kN/m^2
       nu: 0.3
       rho: 7.85     # mT/m^3

   Section:
     - name: IPE300
       A: 0.00538
       Iy: 0.0000836
       Iz: 0.00000604
       J: 0.000000201

   Beam:
     - EndAPosition: [0, 0, 0]
       EndBPosition: [6, 0, 0]
       Section: IPE300
       Material: Steel
   ```

**Acceptance Criteria:**
- [ ] Valid YAML files load without error
- [ ] All entity types are supported
- [ ] Clear error messages for invalid YAML

---

### Task 4.3: Implement Result Output (JSON)
**Requirements:** R-DATA-006, R-RES-001
**Dependencies:** Task 4.1
**Difficulty:** Medium

**Description:**
Implement JSON output for analysis results.

**Steps:**
1. Create `grillex/io/result_writer.py`:
   ```python
   import json
   from dataclasses import dataclass, asdict

   @dataclass
   class NodeResult:
       node_id: int
       displacements: list[float]  # [ux, uy, uz, rx, ry, rz]
       reactions: list[float] | None = None

   @dataclass
   class ElementResult:
       element_id: int
       end_forces_i: list[float]  # [N, Vy, Vz, Mx, My, Mz] at end i
       end_forces_j: list[float]

   @dataclass
   class ResultCase:
       name: str
       load_case: str
       nodes: list[NodeResult]
       elements: list[ElementResult]

       def to_json(self, file_path: str) -> None:
           with open(file_path, 'w') as f:
               json.dump(asdict(self), f, indent=2)
   ```

2. Format is LLM-friendly with clear structure

**Acceptance Criteria:**
- [ ] Results export to valid JSON
- [ ] JSON structure is human-readable
- [ ] All result types are included

---

### Task 4.4: Implement Beam Subdivision at Internal Nodes
**Requirements:** R-MOD-005
**Dependencies:** Task 4.1
**Difficulty:** High

**Description:**
Automatically subdivide beams when internal nodes exist along their length.

**Steps:**
1. Add to Model class:
   ```python
   def _subdivide_beams(self) -> None:
       """Find and subdivide beams that have internal nodes."""
       for beam in self.beams[:]:  # Copy list to allow modification
           internal_nodes = self._find_internal_nodes(beam)
           if internal_nodes:
               self._split_beam_at_nodes(beam, internal_nodes)
   ```

2. Algorithm:
   ```
   For each beam from A to B:
       For each node N in registry:
           If N is not A or B:
               If N lies on line segment A-B (within tolerance):
                   Add N to internal_nodes list with distance from A

       Sort internal_nodes by distance
       Create sub-beams: A-N1, N1-N2, ..., Nn-B
       Remove original beam
   ```

**Acceptance Criteria:**
- [ ] Beam with one internal node becomes two elements
- [ ] Beam with multiple internal nodes becomes multiple elements
- [ ] Section/material properties propagate to sub-beams

---

