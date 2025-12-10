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
- [ ] Simple equality constraints work
- [ ] Rigid links transfer forces correctly
- [ ] T is correctly formed (correct dimensions and entries)

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
- [ ] Slave node moves correctly with master
- [ ] Rotation at master produces translation at slave
- [ ] Forces transfer correctly through rigid link

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

