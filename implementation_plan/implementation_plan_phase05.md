## Phase 5: Loads & Load Cases

### Task 5.1: Implement Load Case Structure
**Requirements:** R-LOAD-001, R-LOAD-002, R-LOAD-010
**Dependencies:** Task 4.1
**Difficulty:** Medium

**Description:**
Create the load case and load combination structures.

**Steps:**
1. Create `cpp/include/grillex/loads.hpp`:
   ```cpp
   enum class LoadCaseType { Permanent, Variable, Environmental, Accidental };

   struct NodalLoad {
       int node_id;
       Eigen::Vector<double, 6> forces;  // [Fx, Fy, Fz, Mx, My, Mz]
   };

   struct LineLoad {
       int element_id;
       Eigen::Vector3d w_start;  // Distributed load at start [wx, wy, wz] in global
       Eigen::Vector3d w_end;    // Distributed load at end
   };

   class LoadCase {
   public:
       int id;
       std::string name;
       LoadCaseType type;
       std::vector<NodalLoad> nodal_loads;
       std::vector<LineLoad> line_loads;

       // Acceleration field (optional)
       Eigen::Vector<double, 6> acceleration = Eigen::Vector<double, 6>::Zero();
       Eigen::Vector3d acceleration_ref_point = Eigen::Vector3d::Zero();

       Eigen::VectorXd assemble_load_vector(const Model& model) const;
   };
   ```

2. Create `grillex/core/loads.py` with Python wrapper

**Acceptance Criteria:**
- [ ] Nodal loads can be applied
- [ ] Line loads can be applied to beams
- [ ] Load cases have type classification

---

### Task 5.2: Implement Line Load Equivalent Nodal Forces
**Requirements:** R-LOAD-001
**Dependencies:** Task 5.1
**Difficulty:** Medium
**Note:** Phase 7 (Task 7.2) internal actions computation depends on this task being complete

**Description:**
Compute equivalent nodal forces for distributed beam loads.

**Important:** This task is **critical for Phase 7** internal actions computation. Phase 7 uses the differential equation approach which requires knowledge of distributed loads along elements to compute accurate internal actions (moment, shear, normal force). Without this infrastructure, Phase 7 will be limited to computing end forces only and using simple linear interpolation for internal actions.

**Steps:**
1. Add to BeamElement:
   ```cpp
   Eigen::Vector<double, 12> equivalent_nodal_forces(
       const Eigen::Vector3d& w_start,
       const Eigen::Vector3d& w_end) const;
   ```

2. For uniform load w in local z:
   ```
   f_z_i = wL/2
   m_y_i = wL²/12
   f_z_j = wL/2
   m_y_j = -wL²/12

   Transform to global and return as 12x1 vector
   ```

3. Handle trapezoidal loads (linear variation)

4. **Coordinate with Phase 7:** Ensure LineLoad structure is accessible from BeamElement:
   ```cpp
   // BeamElement should be able to query its distributed loads
   class BeamElement {
       // ...
       DistributedLoad get_distributed_load_y() const;
       DistributedLoad get_distributed_load_z() const;
       DistributedLoad get_distributed_load_axial() const;
       // These query the current load case for loads applied to this element
   };
   ```

   The `DistributedLoad` struct should match the format expected by Phase 7:
   ```cpp
   struct DistributedLoad {
       double q_start;  // Load intensity at element start [kN/m]
       double q_end;    // Load intensity at element end [kN/m]
       // Linear interpolation: q(x) = q_start + (q_end - q_start) * x / L
   };
   ```

**Acceptance Criteria:**
- [ ] Uniform load produces correct reactions
- [ ] Fixed-end moments match theory
- [ ] Trapezoidal loads work correctly
- [ ] BeamElement can query its distributed loads for Phase 7 internal actions computation
- [ ] DistributedLoad structure is compatible with Phase 7 differential equation approach

---

### Task 5.3: Implement Acceleration Field Loads
**Requirements:** R-LOAD-003, R-LOAD-004, R-LOAD-005, R-LOAD-006, R-LOAD-007
**Dependencies:** Tasks 5.1, 2.3
**Difficulty:** High

**Description:**
Implement inertial loads from acceleration fields.

**Steps:**
1. Add acceleration computation:
   ```cpp
   Eigen::Vector<double, 6> compute_acceleration_at_point(
       const Eigen::Vector3d& point,
       const Eigen::Vector<double, 6>& accel_field,
       const Eigen::Vector3d& ref_point) const;
   ```

   Using: a(P) = a(ref) + α × r + ω × (ω × r)
   For quasi-static: a(P) = a(ref) + α × r

2. Compute inertial load vector:
   ```cpp
   Eigen::VectorXd compute_inertial_load(
       const BeamElement& elem,
       const LoadCase& lc) const {
       // Get nodal accelerations
       auto a_i = compute_acceleration_at_point(elem.node_i->position(), ...);
       auto a_j = compute_acceleration_at_point(elem.node_j->position(), ...);

       // Stack into element acceleration vector
       Eigen::Vector<double, 12> a_e;
       a_e << a_i, a_j;

       // Inertial forces: f = -M * a
       return -elem.global_mass_matrix() * a_e;
   }
   ```

3. Account for offsets when computing acceleration at beam ends

**Acceptance Criteria:**
- [ ] Gravity load (az = -9.81) produces correct weight forces
- [ ] Rotational acceleration produces centrifugal effects
- [ ] Results match: 1 mT/m beam with gravity → 9.81 kN/m load

---

### Task 5.4: Implement Load Combinations
**Requirements:** R-LOAD-010, R-LOAD-011
**Dependencies:** Task 5.1
**Difficulty:** Low

**Description:**
Implement load combination definitions.

**Steps:**
1. Create load combination structure:
   ```cpp
   struct LoadCombinationTerm {
       LoadCase* load_case;
       double factor;
   };

   class LoadCombination {
   public:
       int id;
       std::string name;
       std::vector<LoadCombinationTerm> terms;

       Eigen::VectorXd get_combined_load_vector(const Model& model) const;
   };
   ```

2. Combined load = sum of (factor * individual load case loads)

**Acceptance Criteria:**
- [ ] Combinations sum loads correctly
- [ ] Factors are applied correctly
- [ ] Multiple load cases combine properly

---

## Dependencies and Impact on Other Phases

### Phase 7 Dependency: Internal Actions Computation

Phase 7 (Internal Actions and Results) has a **critical dependency** on Phase 5, specifically Task 5.2 (Line Load Equivalent Nodal Forces).

**Why this dependency exists:**

Phase 7 uses a **differential equation approach** (similar to pystructeng) to compute internal actions (moment, shear, normal force) along beam elements. This approach requires knowledge of the distributed loads q(x) along each element. The differential equations are:

```
Axial:       dN/dx + q_x = 0
Bending-Y:   dV_y/dx + q_y = 0,  dM_z/dx - V_y = 0
Bending-Z:   dV_z/dx + q_z = 0,  dM_y/dx - V_z = 0
```

Without distributed load information, Phase 7 can only:
- Compute element end forces using `f_local = K_local * u_local`
- Use simple linear interpolation for internal actions between end points
- This is **inaccurate** for beams with distributed loads

With Phase 5 complete, Phase 7 can:
- Use analytical closed-form solutions for each release combination
- Account for distributed loads in moment/shear diagrams
- Provide accurate results matching hand calculations

**Implementation Phasing:**

To allow independent development, Phase 7 is split into sub-phases:

1. **Phase 7a (Minimum Viable):** Can be implemented WITHOUT Phase 5
   - Task 7.1: Element end forces using `f = K*u`
   - Simple linear interpolation for internal actions
   - **Limitation:** Inaccurate for distributed loads
   - **Acceptance:** End forces match reactions for point-loaded beams

2. **Phase 7b (Enhanced):** Requires Phase 5 (Tasks 5.1-5.2)
   - Update Task 7.1 to subtract fixed-end forces
   - **Acceptance:** End forces correct even with distributed loads present

3. **Phase 7c (Full Differential Equation Approach):** Requires Phase 5 complete
   - Task 7.2: All release combination formulas
   - Accurate internal actions accounting for q(x)
   - **Acceptance:** Internal actions match analytical solutions

**Coordination Requirements:**

1. **Data Structure Compatibility:**
   - Phase 5's `LineLoad` must be convertible to Phase 7's `DistributedLoad`
   - Both use trapezoidal (linearly varying) load representation
   - Load intensities in same units (kN/m or kN/m²)

2. **Load Querying Interface:**
   - BeamElement needs methods to query current distributed loads:
     ```cpp
     DistributedLoad BeamElement::get_distributed_load_y() const;
     DistributedLoad BeamElement::get_distributed_load_z() const;
     DistributedLoad BeamElement::get_distributed_load_axial() const;
     ```
   - These query the active LoadCase for loads applied to this element

3. **Global vs Local Coordinates:**
   - Phase 5 stores loads in **global coordinates** (user-friendly input)
   - Phase 7 needs loads in **local element coordinates** (for differential equations)
   - Transformation handled by BeamElement::get_distributed_load_*() methods

**Testing Strategy:**

Test the dependency by creating a cantilever with uniform load:

```python
# Test case: cantilever with UDL
model = Model()
n1 = model.get_or_create_node(0, 0, 0)
n2 = model.get_or_create_node(6, 0, 0)
mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
sec = model.create_section("Test", 0.01, 1e-5, 2e-5, 1e-5)
beam = model.create_beam(n1, n2, mat, sec)

model.boundary_conditions.fix_node(n1.id)

# Add distributed load (Phase 5)
load_case = model.create_load_case("DL")
load_case.add_line_load(beam.id, w_start=[0, -10, 0], w_end=[0, -10, 0])

model.analyze(load_case)

# Query internal actions (Phase 7)
# Should get: M(x) = w*x²/2 - w*L*x
actions_mid = beam.get_internal_actions_at(3.0, model)
M_expected = -10 * (3.0**2 / 2 - 6.0 * 3.0)  # = 45 kN⋅m
assert abs(actions_mid.Mz - M_expected) / abs(M_expected) < 0.01
```

**Recommendation:**

Implement Phase 5 (at least Tasks 5.1-5.2) **before** Phase 7 to enable full functionality. If Phase 7 is needed urgently, implement Phase 7a as a stopgap, but clearly document its limitations for beams with distributed loads.

---

