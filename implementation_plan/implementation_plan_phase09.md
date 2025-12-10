## Phase 9: Cargo Modelling

### Task 9.1: Implement Cargo Abstraction
**Requirements:** R-CARGO-001, R-CARGO-002, R-CARGO-003
**Dependencies:** Tasks 8.1, 8.2, 6.2
**Difficulty:** Medium

**Description:**
Create the Cargo Python-level abstraction.

**Steps:**
1. Create `grillex/core/cargo.py`:
   ```python
   class Cargo:
       """
       Cargo modelled as point mass + spring connections.
       The C++ core only sees the constituent elements.
       """
       def __init__(self, name: str):
           self.name = name
           self.cog_position: list[float] = [0, 0, 0]
           self.mass: float = 0
           self.inertia: list[float] = [0, 0, 0, 0, 0, 0]  # Ixx, Iyy, Izz, Ixy, Ixz, Iyz
           self.connections: list[CargoConnection] = []

       def add_connection(self, structural_node: int, stiffness: list[float],
                         cargo_offset: list[float] = None) -> None:
           """Add spring connection to structure."""
           self.connections.append(CargoConnection(...))

       def generate_elements(self, model: "Model") -> None:
           """Generate the actual FE elements for this cargo."""
           # 1. Create node at CoG
           # 2. Create point mass at CoG node
           # 3. For each connection, create spring (and rigid link if offset)
   ```

2. The `generate_elements()` method creates:
   - One node at the CoG
   - One point mass element
   - Spring elements to structural nodes
   - Rigid links if there are offsets

**Acceptance Criteria:**
- [ ] Cargo definition is simple and clear
- [ ] Generated elements correctly represent the cargo
- [ ] Cargo mass contributes to inertial loads under acceleration

---

