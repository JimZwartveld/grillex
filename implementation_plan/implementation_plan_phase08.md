## Phase 8: Additional Element Types

### Task 8.1: Implement Spring Element
**Requirements:** R-MOD-007, R-ELEM-007
**Dependencies:** Task 3.2
**Difficulty:** Medium

**Description:**
Create spring element connecting two nodes.

**Steps:**
1. Create `cpp/include/grillex/spring_element.hpp`:
   ```cpp
   class SpringElement {
   public:
       int id;
       Node* node_i;
       Node* node_j;

       // Stiffness values
       double kx = 0;   // Translational x
       double ky = 0;   // Translational y
       double kz = 0;   // Translational z
       double krx = 0;  // Rotational x
       double kry = 0;  // Rotational y
       double krz = 0;  // Rotational z

       // Optional: eccentricity offset
       Eigen::Vector3d offset_i = Eigen::Vector3d::Zero();
       Eigen::Vector3d offset_j = Eigen::Vector3d::Zero();

       Eigen::Matrix<double, 12, 12> global_stiffness_matrix() const;
   };
   ```

2. Stiffness matrix (diagonal for uncoupled spring):
   ```
   K = [+k   -k]
       [-k   +k]

   For 6-DOF: 12x12 block diagonal
   ```

**Acceptance Criteria:**
- [x] Spring provides correct stiffness between nodes
- [x] Uncoupled DOFs are independent
- [x] Eccentricity can be handled via rigid links

---

### Task 8.2: Implement Point Mass Element
**Requirements:** R-MOD-008, R-ELEM-008
**Dependencies:** Task 3.2
**Difficulty:** Low

**Description:**
Create point mass/inertia element at a node.

**Steps:**
1. Create `cpp/include/grillex/point_mass.hpp`:
   ```cpp
   class PointMass {
   public:
       int id;
       Node* node;

       // Mass and inertia
       double mass;      // Translational mass
       double Ixx, Iyy, Izz;  // Moments of inertia
       double Ixy = 0, Ixz = 0, Iyz = 0;  // Products of inertia

       // Return 6x6 mass matrix
       Eigen::Matrix<double, 6, 6> mass_matrix() const;
   };
   ```

2. Mass matrix:
   ```
   M = [m  0  0  0  0  0]
       [0  m  0  0  0  0]
       [0  0  m  0  0  0]
       [0  0  0 Ixx Ixy Ixz]
       [0  0  0 Ixy Iyy Iyz]
       [0  0  0 Ixz Iyz Izz]
   ```

**Acceptance Criteria:**
- [x] Point mass contributes to global mass matrix
- [x] Inertia tensor is correctly represented
- [x] Off-diagonal terms work for asymmetric masses

---

### Task 8.3: Implement Plate Element (Basic)
**Requirements:** R-MOD-006, R-ELEM-006
**Dependencies:** Task 3.2
**Difficulty:** High

**Description:**
Implement a basic 4-node plate/shell element.

**Steps:**
1. Create `cpp/include/grillex/plate_element.hpp`:
   ```cpp
   class PlateElement {
   public:
       int id;
       std::array<Node*, 4> nodes;  // 4 corner nodes
       double thickness;
       Material* material;

       // Local coordinate system
       LocalAxes local_axes;

       // Stiffness matrix (24x24 for 4 nodes × 6 DOFs)
       Eigen::Matrix<double, 24, 24> global_stiffness_matrix() const;
       Eigen::Matrix<double, 24, 24> global_mass_matrix() const;
   };
   ```

2. Use Mindlin plate theory (MITC4 or similar) for bending
3. Start with bending-only; membrane can be added later

**Acceptance Criteria:**
- [ ] Plate deflects under pressure load
- [ ] Simple plate matches analytical solution
- [ ] Mesh refinement converges

**Note:** This is a complex task. Consider implementing in sub-tasks.

---

