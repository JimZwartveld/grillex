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

**Description:**
Compute equivalent nodal forces for distributed beam loads.

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

**Acceptance Criteria:**
- [ ] Uniform load produces correct reactions
- [ ] Fixed-end moments match theory
- [ ] Trapezoidal loads work correctly

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

