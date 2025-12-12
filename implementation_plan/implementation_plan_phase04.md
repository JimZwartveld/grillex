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
   from dataclasses import dataclass
   import numpy as np
   from grillex._grillex_cpp import Model as _CppModel, BeamElement, InternalActions

   @dataclass
   class BeamResultLine:
       """Results along a beam for plotting.

       This dataclass provides a convenient structure for storing and
       plotting internal action results along a beam.
       """
       x_positions: np.ndarray      # Positions along beam [m]
       values: np.ndarray           # Values (moment, shear, force, etc.)
       component: str               # 'Mz', 'My', 'Vy', 'Vz', 'N', 'Mx'
       units: str                   # 'kN⋅m', 'kN', etc.
       extrema: list[tuple[float, float]] = None  # [(x, value), ...] for extrema
       element_boundaries: list[float] = None     # x-positions of element joints
       discontinuities: list[float] = None        # x-positions of jumps

       def __post_init__(self):
           """Initialize optional fields."""
           if self.extrema is None:
               self.extrema = []
           if self.element_boundaries is None:
               self.element_boundaries = []
           if self.discontinuities is None:
               self.discontinuities = []

       def plot(self, ax=None, **kwargs):
           """Convenience method to plot this line.

           Args:
               ax: Matplotlib axes object (creates new if None)
               **kwargs: Additional arguments passed to ax.plot()

           Returns:
               The matplotlib axes object
           """
           if ax is None:
               import matplotlib.pyplot as plt
               fig, ax = plt.subplots()

           # Plot main line
           ax.plot(self.x_positions, self.values, **kwargs)

           # Mark extrema
           for x, val in self.extrema:
               ax.plot(x, val, 'ro', markersize=8)
               ax.annotate(f'{val:.2f}',
                          xy=(x, val),
                          xytext=(5, 5),
                          textcoords='offset points')

           # Mark element boundaries
           for x_bound in self.element_boundaries:
               ax.axvline(x_bound, color='gray', linestyle=':', alpha=0.5)

           # Formatting
           ax.grid(True, alpha=0.3)
           ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
           ax.set_xlabel('Position along beam [m]')
           ax.set_ylabel(f'{self.component} [{self.units}]')
           ax.set_title(f'{self.component} diagram')

           return ax

   class Beam:
       """Represents a beam in the model.

       A Beam is a Python-level abstraction that can consist of multiple
       BeamElement objects (C++ FE elements). This allows for continuous
       internal action plots across element boundaries.
       """
       def __init__(self, end_a_position, end_b_position, section, material, **kwargs):
           self.end_a_position = end_a_position
           self.end_b_position = end_b_position
           self.section = section
           self.material = material
           self.elements: list[BeamElement] = []  # Underlying C++ elements
           self.length: float = 0.0
           # ... store other properties

       def get_internal_actions_at(self, x_global: float, model: 'Model') -> 'InternalActions':
           """Query internal actions at any position along the entire beam.

           Args:
               x_global: Position along entire Beam [0, L_total] in meters
               model: Model object containing analysis results

           Returns:
               InternalActions object with N, Vy, Vz, Mx, My, Mz at position x
           """
           # Find which element contains x_global
           element, x_local = self._find_element_at_position(x_global)

           # Query that element using C++ method
           return element.get_internal_actions(
               x_local,
               model.get_displacements(),
               model.get_dof_handler()
           )

       def get_moment_line(
           self,
           axis: str,
           model: 'Model',
           num_points: int = 100
       ) -> tuple[np.ndarray, np.ndarray]:
           """Get moment diagram data for plotting.

           Args:
               axis: 'y' or 'z' - bending axis
               model: Model with analysis results
               num_points: Number of sample points along beam

           Returns:
               (x_positions, moments): Arrays for plotting
           """
           x_positions = np.linspace(0, self.length, num_points)
           moments = []

           for x in x_positions:
               actions = self.get_internal_actions_at(x, model)
               moment = actions.My if axis == 'y' else actions.Mz
               moments.append(moment)

           return x_positions, np.array(moments)

       def get_shear_line(
           self,
           axis: str,
           model: 'Model',
           num_points: int = 100
       ) -> tuple[np.ndarray, np.ndarray]:
           """Get shear diagram data for plotting."""
           x_positions = np.linspace(0, self.length, num_points)
           shears = []

           for x in x_positions:
               actions = self.get_internal_actions_at(x, model)
               shear = actions.Vy if axis == 'y' else actions.Vz
               shears.append(shear)

           return x_positions, np.array(shears)

       def get_normal_force_line(
           self,
           model: 'Model',
           num_points: int = 100
       ) -> tuple[np.ndarray, np.ndarray]:
           """Get normal force diagram data for plotting."""
           x_positions = np.linspace(0, self.length, num_points)
           forces = []

           for x in x_positions:
               actions = self.get_internal_actions_at(x, model)
               forces.append(actions.N)

           return x_positions, np.array(forces)

       def plot_internal_actions(
           self,
           model: 'Model',
           components: list[str] = ['Mz', 'Vy', 'N'],
           figsize: tuple[float, float] | None = None
       ) -> 'matplotlib.figure.Figure':
           """Create matplotlib plots of internal actions along beam.

           Args:
               model: Model with analysis results
               components: List of components to plot ('Mz', 'My', 'Vy', 'Vz', 'N', 'Mx')
               figsize: Figure size (width, height) in inches

           Returns:
               matplotlib Figure object
           """
           import matplotlib.pyplot as plt

           if figsize is None:
               figsize = (10, 3 * len(components))

           fig, axes = plt.subplots(len(components), 1, figsize=figsize)
           if len(components) == 1:
               axes = [axes]

           for ax, component in zip(axes, components):
               x, values = self._get_line_data(component, model)
               ax.plot(x, values, 'b-', linewidth=2, label=component)
               ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
               ax.grid(True, alpha=0.3)
               ax.set_xlabel('Position along beam [m]')
               ax.set_ylabel(f'{component} [{self._get_units(component)}]')
               ax.set_title(f'{component} diagram')

               # Mark element boundaries
               self._mark_element_boundaries(ax)

               # Mark extrema
               extrema = self.find_extrema(component, model)
               for x_ext, val_ext in extrema:
                   ax.plot(x_ext, val_ext, 'ro', markersize=8)
                   ax.annotate(f'{val_ext:.2f}',
                              xy=(x_ext, val_ext),
                              xytext=(5, 5),
                              textcoords='offset points')

           plt.tight_layout()
           return fig

       def find_extrema(
           self,
           component: str,
           model: 'Model'
       ) -> list[tuple[float, float]]:
           """Find extrema (local maxima/minima) for a component.

           Args:
               component: 'Mz', 'My', 'Vy', 'Vz', 'N', 'Mx'
               model: Model with analysis results

           Returns:
               List of (x_position, value) tuples for extrema
           """
           extrema = []
           cumulative_length = 0.0

           for element in self.elements:
               # Find extrema within this element
               elem_extrema = element.find_component_extrema(component, model)

               # Convert to global coordinates
               for x_local, value in elem_extrema:
                   x_global = cumulative_length + x_local
                   extrema.append((x_global, value))

               cumulative_length += element.length()

           return self._filter_true_extrema(extrema)

       def _find_element_at_position(
           self,
           x_global: float
       ) -> tuple['BeamElement', float]:
           """Find which element contains x_global and return local position."""
           cumulative_length = 0.0

           for element in self.elements:
               element_length = element.length()
               if x_global <= cumulative_length + element_length:
                   x_local = x_global - cumulative_length
                   return element, x_local
               cumulative_length += element_length

           # Handle edge case: x_global == total length
           return self.elements[-1], self.elements[-1].length()

       def _get_line_data(
           self,
           component: str,
           model: 'Model',
           num_points: int = 100
       ) -> tuple[np.ndarray, np.ndarray]:
           """Get line data for a specific component."""
           if component in ['Mz', 'My']:
               axis = 'z' if component == 'Mz' else 'y'
               return self.get_moment_line(axis, model, num_points)
           elif component in ['Vy', 'Vz']:
               axis = 'y' if component == 'Vy' else 'z'
               return self.get_shear_line(axis, model, num_points)
           elif component == 'N':
               return self.get_normal_force_line(model, num_points)
           elif component == 'Mx':
               # Torsion - to be implemented
               raise NotImplementedError("Torsion plotting not yet implemented")
           else:
               raise ValueError(f"Unknown component: {component}")

       def _get_units(self, component: str) -> str:
           """Get units string for a component."""
           if component in ['Mz', 'My', 'Mx']:
               return 'kN⋅m'
           elif component in ['Vy', 'Vz', 'N']:
               return 'kN'
           else:
               return ''

       def _mark_element_boundaries(self, ax) -> None:
           """Draw vertical lines at element boundaries."""
           cumulative_length = 0.0
           for i, element in enumerate(self.elements[:-1]):
               cumulative_length += element.length()
               ax.axvline(cumulative_length,
                         color='gray',
                         linestyle=':',
                         alpha=0.5,
                         label='Element boundary' if i == 0 else '')

       def _filter_true_extrema(
           self,
           candidates: list[tuple[float, float]]
       ) -> list[tuple[float, float]]:
           """Filter candidate points to find true local extrema."""
           if len(candidates) < 3:
               return candidates

           extrema = []
           for i in range(1, len(candidates) - 1):
               prev_val = candidates[i-1][1]
               curr_val = candidates[i][1]
               next_val = candidates[i+1][1]

               # Local maximum or minimum
               if (curr_val > prev_val and curr_val > next_val) or \
                  (curr_val < prev_val and curr_val < next_val):
                   extrema.append(candidates[i])

           return extrema

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

       def get_beam_internal_actions(
           self,
           beam_id: int,
           x: float,
           component: str | None = None
       ) -> 'InternalActions' | float:
           """Get internal actions at a position along a beam.

           Args:
               beam_id: ID of the beam to query
               x: Position along beam [m]
               component: Optional specific component ('N', 'Vy', 'Vz', 'Mx', 'My', 'Mz')

           Returns:
               InternalActions object or float value if component specified
           """
           beam = self.beams[beam_id]
           actions = beam.get_internal_actions_at(x, self)

           if component:
               return getattr(actions, component)
           return actions

       def plot_beam_results(
           self,
           beam_id: int,
           components: list[str] = ['Mz', 'Vy', 'N'],
           show: bool = True
       ) -> 'matplotlib.figure.Figure':
           """Plot internal action diagrams for a beam.

           Args:
               beam_id: ID of the beam to plot
               components: List of components to plot
               show: Whether to display the plot immediately

           Returns:
               matplotlib Figure object
           """
           beam = self.beams[beam_id]
           fig = beam.plot_internal_actions(self, components)

           if show:
               import matplotlib.pyplot as plt
               plt.show()

           return fig

       def export_beam_results(
           self,
           beam_id: int,
           filename: str,
           num_points: int = 100
       ) -> None:
           """Export beam internal actions to CSV file.

           Args:
               beam_id: ID of the beam to export
               filename: Output CSV filename
               num_points: Number of sample points along beam
           """
           import csv
           import numpy as np

           beam = self.beams[beam_id]
           x_positions = np.linspace(0, beam.length, num_points)

           with open(filename, 'w', newline='') as csvfile:
               writer = csv.writer(csvfile)
               writer.writerow(['x', 'N', 'Vy', 'Vz', 'Mx', 'My', 'Mz'])

               for x in x_positions:
                   actions = beam.get_internal_actions_at(x, self)
                   writer.writerow([
                       x, actions.N, actions.Vy, actions.Vz,
                       actions.Mx, actions.My, actions.Mz
                   ])

       def get_displacements(self) -> np.ndarray:
           """Get global displacement vector from C++ model."""
           return self._cpp_model.get_displacements()

       def get_dof_handler(self) -> 'DOFHandler':
           """Get DOF handler from C++ model."""
           return self._cpp_model.get_dof_handler()
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

### Task 4.5: Expose C++ BeamElement Internal Actions Methods
**Requirements:** R-RES-001, R-ARCH-003 (for Phase 7 support)
**Dependencies:** Task 4.1, Phase 7 (Task 7.2)
**Difficulty:** Medium

**Description:**
Create Python bindings to expose C++ BeamElement methods for computing internal actions along elements. This enables multi-element beam plotting and continuous internal action diagrams.

**Background:**
Phase 7 implements internal action computation in C++ using differential equations. The Python API needs to expose these methods so that the Python `Beam` class can aggregate results from multiple `BeamElement` objects.

**Steps:**
1. Add Python bindings in `python/bindings.cpp` using pybind11:
   ```cpp
   #include <pybind11/pybind11.h>
   #include <pybind11/eigen.h>
   #include <pybind11/stl.h>
   #include "grillex/internal_actions.hpp"
   #include "grillex/beam_element.hpp"

   namespace py = pybind11;

   void bind_internal_actions(py::module& m) {
       // Expose InternalActions struct
       py::class_<grillex::InternalActions>(m, "InternalActions")
           .def(py::init<>())
           .def_readwrite("x", &grillex::InternalActions::x, "Position along element [m]")
           .def_readwrite("N", &grillex::InternalActions::N, "Axial force [kN]")
           .def_readwrite("Vy", &grillex::InternalActions::Vy, "Shear force y-axis [kN]")
           .def_readwrite("Vz", &grillex::InternalActions::Vz, "Shear force z-axis [kN]")
           .def_readwrite("Mx", &grillex::InternalActions::Mx, "Torsion [kN⋅m]")
           .def_readwrite("My", &grillex::InternalActions::My, "Bending moment y-axis [kN⋅m]")
           .def_readwrite("Mz", &grillex::InternalActions::Mz, "Bending moment z-axis [kN⋅m]")
           .def("__repr__", [](const grillex::InternalActions& ia) {
               return "<InternalActions at x=" + std::to_string(ia.x) +
                      " m: N=" + std::to_string(ia.N) +
                      " kN, Mz=" + std::to_string(ia.Mz) + " kN⋅m>";
           });

       // Expose EndForces struct
       py::class_<grillex::EndForces>(m, "EndForces")
           .def(py::init<>())
           .def_readwrite("forces_i", &grillex::EndForces::forces_i,
                         "End forces at node i [N, Vy, Vz, Mx, My, Mz]")
           .def_readwrite("forces_j", &grillex::EndForces::forces_j,
                         "End forces at node j [N, Vy, Vz, Mx, My, Mz]");

       // Expose DisplacementLine struct (for Task 7.2c)
       py::class_<grillex::DisplacementLine>(m, "DisplacementLine")
           .def(py::init<>())
           .def_readwrite("x", &grillex::DisplacementLine::x)
           .def_readwrite("u", &grillex::DisplacementLine::u, "Axial displacement [m]")
           .def_readwrite("v", &grillex::DisplacementLine::v, "Lateral displacement y [m]")
           .def_readwrite("w", &grillex::DisplacementLine::w, "Lateral displacement z [m]")
           .def_readwrite("theta_x", &grillex::DisplacementLine::theta_x, "Twist rotation [rad]")
           .def_readwrite("theta_y", &grillex::DisplacementLine::theta_y, "Bending rotation y [rad]")
           .def_readwrite("theta_z", &grillex::DisplacementLine::theta_z, "Bending rotation z [rad]");
   }

   void bind_beam_element_extensions(py::module& m) {
       // Extend BeamElement binding with internal action methods
       py::class_<grillex::BeamElement>(m, "BeamElement")
           // ... existing bindings ...

           .def("get_internal_actions",
                &grillex::BeamElement::get_internal_actions,
                py::arg("x"),
                py::arg("global_displacements"),
                py::arg("dof_handler"),
                "Compute internal actions at position x along element.\n\n"
                "Args:\n"
                "    x: Position along element [0, L] in meters\n"
                "    global_displacements: Global displacement vector from analysis\n"
                "    dof_handler: DOF handler from model\n\n"
                "Returns:\n"
                "    InternalActions object with N, Vy, Vz, Mx, My, Mz")

           .def("compute_end_forces",
                &grillex::BeamElement::compute_end_forces,
                py::arg("global_displacements"),
                py::arg("dof_handler"),
                "Compute element end forces from global displacements.\n\n"
                "Returns:\n"
                "    EndForces object with forces_i and forces_j")

           .def("get_displacements_at",
                &grillex::BeamElement::get_displacements_at,
                py::arg("x"),
                py::arg("global_displacements"),
                py::arg("dof_handler"),
                "Compute displacements/rotations at position x along element.\n\n"
                "Args:\n"
                "    x: Position along element [0, L] in meters\n"
                "    global_displacements: Global displacement vector\n"
                "    dof_handler: DOF handler\n\n"
                "Returns:\n"
                "    DisplacementLine object with u, v, w, θx, θy, θz")

           .def("find_component_extrema",
                &grillex::BeamElement::find_component_extrema,
                py::arg("component"),
                py::arg("model"),
                "Find extrema (local max/min) for a component along element.\n\n"
                "Args:\n"
                "    component: 'N', 'Vy', 'Vz', 'Mx', 'My', or 'Mz'\n"
                "    model: Model with analysis results\n\n"
                "Returns:\n"
                "    List of (x_local, value) tuples for extrema within element")

           .def("length",
                &grillex::BeamElement::length,
                "Get element length [m]");
   }

   PYBIND11_MODULE(_grillex_cpp, m) {
       m.doc() = "Grillex C++ core bindings";

       // ... existing bindings ...

       bind_internal_actions(m);
       bind_beam_element_extensions(m);
   }
   ```

2. Update `Beam` class in `grillex/core/model.py` to use these methods (already done in Task 4.1):
   - `get_internal_actions_at()` calls `element.get_internal_actions()`
   - `find_extrema()` calls `element.find_component_extrema()`
   - Helper methods use `element.length()` for position calculations

3. Add type hints in `python/grillex.pyi` (stub file for IDE support):
   ```python
   from typing import List, Tuple
   import numpy as np

   class InternalActions:
       x: float
       N: float
       Vy: float
       Vz: float
       Mx: float
       My: float
       Mz: float

   class EndForces:
       forces_i: np.ndarray  # shape (6,)
       forces_j: np.ndarray  # shape (6,)

   class DisplacementLine:
       x: float
       u: float
       v: float
       w: float
       theta_x: float
       theta_y: float
       theta_z: float

   class BeamElement:
       def get_internal_actions(
           self,
           x: float,
           global_displacements: np.ndarray,
           dof_handler: DOFHandler
       ) -> InternalActions: ...

       def compute_end_forces(
           self,
           global_displacements: np.ndarray,
           dof_handler: DOFHandler
       ) -> EndForces: ...

       def get_displacements_at(
           self,
           x: float,
           global_displacements: np.ndarray,
           dof_handler: DOFHandler
       ) -> DisplacementLine: ...

       def find_component_extrema(
           self,
           component: str,
           model: Model
       ) -> List[Tuple[float, float]]: ...

       def length(self) -> float: ...
   ```

4. Add example usage in documentation:
   ```python
   from grillex import Model, DOFIndex
   import matplotlib.pyplot as plt
   import numpy as np

   # Create and analyze model
   model = Model()
   # ... build model ...
   model.analyze()

   # Access a beam (which may consist of multiple BeamElements)
   beam = model.beams[0]

   # Get moment diagram data
   x, Mz = beam.get_moment_line('z', model, num_points=200)

   # Plot
   plt.figure(figsize=(10, 6))
   plt.plot(x, Mz, 'b-', linewidth=2)
   plt.xlabel('Position along beam [m]')
   plt.ylabel('Moment Mz [kN⋅m]')
   plt.title('Bending Moment Diagram')
   plt.grid(True, alpha=0.3)
   plt.axhline(0, color='k', linestyle='--', linewidth=0.5)

   # Mark extrema
   extrema = beam.find_extrema('Mz', model)
   for x_ext, M_ext in extrema:
       plt.plot(x_ext, M_ext, 'ro', markersize=8)
       plt.annotate(f'{M_ext:.2f} kN⋅m',
                   xy=(x_ext, M_ext),
                   xytext=(5, 5),
                   textcoords='offset points')

   plt.show()

   # Or use convenience method
   fig = model.plot_beam_results(beam_id=0, components=['Mz', 'Vy', 'N'])
   plt.show()

   # Export to CSV
   model.export_beam_results(beam_id=0, filename='beam_results.csv')
   ```

**Acceptance Criteria:**
- [ ] InternalActions, EndForces, and DisplacementLine structs are accessible from Python
- [ ] BeamElement methods (get_internal_actions, compute_end_forces, get_displacements_at, find_component_extrema) are callable from Python
- [ ] Methods accept Eigen arrays and return appropriate types
- [ ] Type hints are provided for IDE support
- [ ] Example usage demonstrates multi-element beam plotting
- [ ] Unit tests verify Python bindings work correctly:
  ```python
  def test_beam_element_internal_actions():
      """Test that BeamElement.get_internal_actions() returns valid results"""
      model = Model()
      # ... create simple cantilever ...
      model.analyze()

      element = model.beams[0].elements[0]
      actions = element.get_internal_actions(
          element.length() / 2,  # Midpoint
          model.get_displacements(),
          model.get_dof_handler()
      )

      assert isinstance(actions, InternalActions)
      assert actions.x == element.length() / 2
      assert isinstance(actions.N, float)
      assert isinstance(actions.Mz, float)
  ```

**Notes:**
- This task depends on Phase 7 (Task 7.2) being implemented first in C++
- The Python bindings simply expose existing C++ functionality
- The Beam class (Task 4.1) aggregates these element-level methods
- For beams with single elements, Beam methods delegate directly to BeamElement
- For multi-element beams, Beam stitches together results from multiple BeamElements

---

