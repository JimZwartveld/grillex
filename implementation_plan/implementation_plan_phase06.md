## Phase 6: MPC & Rigid Links

### Task 6.1: Implement Transformation Matrix for MPC
**Requirements:** R-MPC-001, R-MPC-002
**Dependencies:** Task 3.2
**Difficulty:** High

**Description:**
Implement the constraint transformation matrix T.

**Steps:**
1. Create `cpp/include/grillex/constraints.hpp`:
   ```cpp
   class ConstraintHandler {
   public:
       // Add master-slave equality constraint
       void add_equality_constraint(int slave_node, int slave_dof,
                                    int master_node, int master_dof);

       // Add rigid link constraint
       void add_rigid_link(int slave_node, int master_node,
                          const Eigen::Vector3d& offset);

       // Build transformation matrix
       Eigen::SparseMatrix<double> build_transformation_matrix(
           const DOFHandler& dof_handler) const;

       // Apply to system
       void reduce_system(
           Eigen::SparseMatrix<double>& K,
           Eigen::VectorXd& F,
           const DOFHandler& dof_handler) const;

   private:
       std::vector<EqualityConstraint> equalities_;
       std::vector<RigidLink> rigid_links_;
   };
   ```

2. Build T matrix:
   ```
   T relates full DOFs to reduced (independent) DOFs:
   u_full = T * u_reduced

   For equality u_slave = u_master:
   Row for slave DOF has 1 in column for master DOF

   For rigid link with offset r:
   u_S = u_M + θ_M × r
   θ_S = θ_M
   ```

**Acceptance Criteria:**
- [x] Simple equality constraints work
- [x] Rigid links transfer forces correctly
- [x] T is correctly formed (correct dimensions and entries)

**Execution Summary (2025-12-17):**

**Implementation completed successfully.** All acceptance criteria verified through comprehensive tests.

**Files Created:**
1. `cpp/include/grillex/constraints.hpp` - Header with:
   - `EqualityConstraint` struct for simple DOF equality (u_slave = u_master)
   - `RigidLink` struct with skew-symmetric matrix computation for rigid body kinematics
   - `ConstraintHandler` class with full MPC implementation
   - `ReducedSystem` struct for returning reduced K, F, and transformation matrix T

2. `cpp/src/constraints.cpp` - Implementation with:
   - `add_equality_constraint()` - Adds slave=master DOF constraint with validation
   - `add_rigid_link()` - Adds rigid link with offset and 6-DOF coupling
   - `build_transformation_matrix()` - Builds sparse T matrix (n_full × n_reduced)
   - `reduce_system()` - Computes K_reduced = T^T * K * T, F_reduced = T^T * F
   - `expand_displacements()` - Recovers u_full = T * u_reduced
   - `build_constraint_map()` - Internal helper for constraint coefficient mapping
   - `get_slave_dofs()` - Identifies dependent DOFs

3. `cpp/bindings/bindings.cpp` - Added Python bindings for all new types

4. Updated Python exports in:
   - `src/grillex/core/data_types.py`
   - `src/grillex/core/__init__.py`

5. `tests/python/test_phase6_constraints.py` - 23 comprehensive tests covering:
   - Constraint creation and validation
   - Skew-symmetric matrix correctness
   - Transformation matrix dimensions and entries
   - System reduction mechanics
   - Displacement expansion
   - Complete analysis with equality constraints (two cantilevers tied together)
   - Complete analysis with rigid links (rotation-translation coupling verified)
   - All acceptance criteria

**Implementation Details:**
- Transformation matrix T maps reduced (independent) DOFs to full DOFs: u_full = T * u_reduced
- For equality constraints: T[slave_row, master_col] = 1.0
- For rigid links with offset r = [rx, ry, rz]:
  - Translation coupling: u_slave = u_master + θ_master × r
  - Rotation coupling: θ_slave = θ_master
- System reduction: K_reduced = T^T * K * T, F_reduced = T^T * F

**Issues Encountered and Fixes:**
1. **Test Pattern Issue:** Initial tests used `model.nodes` which isn't exposed in Python bindings. Fixed by using `NodeRegistry` directly, consistent with other test files.
2. **Exception Type:** C++ `std::invalid_argument` maps to Python `ValueError` (not `RuntimeError`). Updated test expectations.

**All 23 tests pass.**

---

### Task 6.2: Implement Rigid Link Kinematics
**Requirements:** R-MPC-003
**Dependencies:** Task 6.1
**Difficulty:** Medium

**Description:**
Implement the rigid link transformation in detail.

**Steps:**
1. For rigid link from master M to slave S with offset r_MS:
   ```
   In matrix form:
   [u_S]   [I   R] [u_M]
   [θ_S] = [0   I] [θ_M]

   Where R is skew-symmetric matrix of r_MS:
   R = [ 0   -rz   ry]
       [ rz   0   -rx]
       [-ry  rx    0 ]
   ```

2. Build 6x6 transformation block per rigid link

**Acceptance Criteria:**
- [x] Slave node moves correctly with master
- [x] Rotation at master produces translation at slave
- [x] Forces transfer correctly through rigid link

**Execution Summary (2025-12-17):**

**Implementation completed successfully.** All acceptance criteria verified through comprehensive tests.

**Changes Made:**
1. Added `transformation_block_6x6()` method to `RigidLink` struct in `constraints.hpp`:
   - Returns the full 6x6 transformation matrix:
     ```
     T = [I  R]
         [0  I]
     ```
   - Where I is 3x3 identity, R is skew-symmetric matrix, 0 is zero matrix

2. Added Python binding for `transformation_block_6x6()` in `bindings.cpp`

3. Added 6 new tests in `TestTask62RigidLinkKinematics` class:
   - `test_transformation_block_6x6_structure`: Verifies [I R; 0 I] structure
   - `test_transformation_block_zero_offset`: Zero offset gives identity matrix
   - `test_ac1_slave_moves_with_master_translation`: Translation coupling verified
   - `test_ac2_rotation_produces_translation`: Rotation-translation coupling (u_sx = u_mx + ry*θmz)
   - `test_ac3_force_transfer_through_rigid_link`: Force applied to slave transfers to master
   - `test_full_6dof_rigid_link_coupling`: All 6 DOF couplings verified with 3D offset

**Implementation Details:**
The 6x6 transformation block relates slave DOFs to master DOFs:
```
[u_S]   [I   R] [u_M]
[θ_S] = [0   I] [θ_M]

Where R is the skew-symmetric matrix of offset r:
R = [ 0   -rz   ry]
    [ rz   0   -rx]
    [-ry  rx    0 ]
```

This yields the kinematic relations:
- u_sx = u_mx + (-rz*θmy + ry*θmz)
- u_sy = u_my + (rz*θmx - rx*θmz)
- u_sz = u_mz + (-ry*θmx + rx*θmy)
- θ_sx = θ_mx, θ_sy = θ_my, θ_sz = θ_mz

**All 29 tests pass (23 from Task 6.1 + 6 from Task 6.2).**

---

### Task 6.3: Apply MPC to Global System
**Requirements:** R-MPC-004
**Dependencies:** Tasks 6.1, 6.2
**Difficulty:** Medium

**Description:**
Apply the transformation to reduce the global system.

**Steps:**
1. Implement system reduction:
   ```cpp
   void ConstraintHandler::reduce_system(K, F, dof_handler) {
       auto T = build_transformation_matrix(dof_handler);

       // K_reduced = T^T * K * T
       K = T.transpose() * K * T;

       // F_reduced = T^T * F
       F = T.transpose() * F;
   }
   ```

2. After solving for u_reduced, recover full displacements:
   ```cpp
   Eigen::VectorXd expand_displacements(
       const Eigen::VectorXd& u_reduced) const {
       return T * u_reduced;
   }
   ```

**Acceptance Criteria:**
- [ ] Reduced system is smaller than original
- [ ] Full displacements are recovered correctly
- [ ] Constrained DOFs satisfy constraint equations

---

