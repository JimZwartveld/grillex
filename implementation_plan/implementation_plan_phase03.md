## Phase 3: Assembly & Solver

**⚠️ UPDATED FOR WARPING DOF SUPPORT ⚠️**

The following tasks have been updated to handle both 12-DOF and 14-DOF elements.

### Task 3.1: Implement DOF Numbering System
**Requirements:** R-DOF-005, R-DOF-001, R-DOF-002, R-DOF-007 (warping)
**Dependencies:** Task 1.2, Task 2.7 (for warping support)
**Difficulty:** Medium

**Description:**
Create the global DOF numbering system with support for optional 7th DOF (warping).

**Steps:**
1. Create `cpp/include/grillex/dof_handler.hpp`:
   ```cpp
   class DOFHandler {
   public:
       // Assign global DOF numbers to all nodes
       // Handles both 6-DOF and 7-DOF nodes automatically
       void number_dofs(NodeRegistry& registry);

       // Get total number of DOFs
       int total_dofs() const;

       // Get global DOF number for a node/local_dof pair
       // local_dof: 0-5 for standard, 6 for warping
       int get_global_dof(int node_id, int local_dof) const;

       // Get location array for an element (maps local to global DOFs)
       // Handles both 12-DOF and 14-DOF elements
       std::vector<int> get_location_array(const BeamElementBase& elem) const;

       // Check if any node has warping DOF active
       bool has_warping_dofs() const;

   private:
       int total_dofs_ = 0;
       bool has_warping_ = false;
       std::map<std::pair<int,int>, int> dof_map_;  // (node_id, local_dof) -> global_dof
   };
   ```

2. Updated numbering algorithm for 7 DOFs:
   ```
   global_dof_counter = 0
   for each node in registry:
       for local_dof in [0..6]:  // UX, UY, UZ, RX, RY, RZ, WARP
           if node.dof_active[local_dof]:
               dof_map[(node.id, local_dof)] = global_dof_counter++
               if local_dof == 6:
                   has_warping_ = true
   ```

3. Location array generation for variable-size elements:
   ```cpp
   std::vector<int> get_location_array(const BeamElementBase& elem) const {
       int n_dofs = elem.num_dofs();  // 12 or 14
       int dofs_per_node = n_dofs / 2;  // 6 or 7
       std::vector<int> loc(n_dofs);

       // Node i DOFs
       for (int d = 0; d < dofs_per_node; ++d) {
           loc[d] = get_global_dof(elem.node_i->id, d);
       }
       // Node j DOFs
       for (int d = 0; d < dofs_per_node; ++d) {
           loc[dofs_per_node + d] = get_global_dof(elem.node_j->id, d);
       }
       return loc;
   }
   ```

**Acceptance Criteria:**
- [ ] Each active DOF gets a unique global number
- [ ] Location arrays correctly map element DOFs to global DOFs (12 or 14)
- [ ] Inactive DOFs are not numbered
- [ ] 7th DOF (warping) numbered only when active on node
- [ ] Mixed models (some elements 12-DOF, some 14-DOF) work correctly

---

### Task 3.2: Implement Global Matrix Assembly
**Requirements:** R-ASM-001, R-ASM-002
**Dependencies:** Tasks 2.2, 2.3, 3.1, (2.7 for warping support)
**Difficulty:** Medium

**Description:**
Implement sparse matrix assembly for K and M. Must handle variable-size element matrices (12×12 or 14×14).

**Steps:**
1. Create `cpp/include/grillex/assembler.hpp`:
   ```cpp
   class Assembler {
   public:
       Assembler(DOFHandler& dof_handler);

       // Assemble global stiffness matrix
       // Handles mixed 12-DOF and 14-DOF elements automatically
       Eigen::SparseMatrix<double> assemble_stiffness(
           const std::vector<BeamElementBase*>& elements) const;

       // Assemble global mass matrix
       Eigen::SparseMatrix<double> assemble_mass(
           const std::vector<BeamElementBase*>& elements) const;

   private:
       DOFHandler& dof_handler_;

       // Add element matrix to global using location array
       // Works with any size element matrix via Eigen::MatrixXd
       void add_element_matrix(
           std::vector<Eigen::Triplet<double>>& triplets,
           const Eigen::MatrixXd& element,
           const std::vector<int>& loc_array) const;
   };
   ```

2. Use triplet list for efficient sparse assembly (handles variable sizes):
   ```cpp
   std::vector<Eigen::Triplet<double>> triplets;
   for (auto* elem : elements) {
       // Returns 12×12 or 14×14 depending on element type
       Eigen::MatrixXd K_e = elem->global_stiffness_matrix();

       // Returns vector of size 12 or 14
       std::vector<int> loc = dof_handler.get_location_array(*elem);

       // Add all non-zero entries
       for (int i = 0; i < K_e.rows(); ++i) {
           for (int j = 0; j < K_e.cols(); ++j) {
               if (loc[i] >= 0 && loc[j] >= 0 && K_e(i,j) != 0.0) {
                   triplets.push_back({loc[i], loc[j], K_e(i,j)});
               }
           }
       }
   }
   K_global.setFromTriplets(triplets);
   ```

**Acceptance Criteria:**
- [ ] Assembled matrix is sparse
- [ ] Assembled matrix is symmetric
- [ ] Single 12-DOF element assembly matches element stiffness matrix
- [ ] Single 14-DOF element assembly matches element stiffness matrix
- [ ] Mixed assembly (12-DOF and 14-DOF elements) works correctly

---

### Task 3.3: Implement Boundary Conditions
**Requirements:** R-DOF-006, R-DOF-007 (warping BCs)
**Dependencies:** Task 3.2
**Difficulty:** Medium

**Description:**
Implement fixed DOFs and prescribed displacements. Support warping-specific boundary conditions.

**Warping Boundary Condition Types:**
- **Warping restrained (WARP=0):** Fixed end, built-in support - warping is prevented
- **Warping free (WARP unrestrained):** Fork support, simple support - free to warp
- **Warping continuous:** At beam connections (automatic - not a BC)

**Steps:**
1. Create `cpp/include/grillex/boundary_condition.hpp`:
   ```cpp
   // DOF index constants for clarity
   enum DOFIndex {
       UX = 0, UY = 1, UZ = 2,
       RX = 3, RY = 4, RZ = 5,
       WARP = 6  // Warping DOF (7th DOF)
   };

   struct FixedDOF {
       int node_id;
       int local_dof;  // 0-5 for standard, 6 for warping
       double value = 0.0;  // Prescribed value
   };

   class BCHandler {
   public:
       // Add single fixed DOF
       void add_fixed_dof(int node_id, int local_dof, double value = 0.0);

       // Convenience: fix all DOFs at a node (full fixity)
       void fix_node(int node_id);

       // Convenience: fix all DOFs including warping at a node
       void fix_node_with_warping(int node_id);

       // Convenience: pin support (fix translations only)
       void pin_node(int node_id);

       // Convenience: fork support (fix translations, free rotations and warping)
       void fork_support(int node_id);

       // Apply BCs using penalty method
       void apply_to_system(
           Eigen::SparseMatrix<double>& K,
           Eigen::VectorXd& F,
           const DOFHandler& dof_handler) const;

       std::vector<int> get_fixed_global_dofs(const DOFHandler& dof_handler) const;

   private:
       std::vector<FixedDOF> fixed_dofs_;
   };
   ```

2. Implement convenience methods:
   ```cpp
   void BCHandler::fix_node_with_warping(int node_id) {
       for (int dof = 0; dof <= 6; ++dof) {  // Include warping
           add_fixed_dof(node_id, dof, 0.0);
       }
   }

   void BCHandler::fork_support(int node_id) {
       // Fix translations only (UX, UY, UZ)
       // Leave rotations (RX, RY, RZ) and warping (WARP) free
       add_fixed_dof(node_id, UX, 0.0);
       add_fixed_dof(node_id, UY, 0.0);
       add_fixed_dof(node_id, UZ, 0.0);
   }
   ```

3. Penalty method implementation (unchanged, works for any DOF):
   ```
   For fixed DOF at global index i with value u_prescribed:
   K(i,i) = K(i,i) + BIG_NUMBER
   F(i) = BIG_NUMBER * u_prescribed
   ```

**Acceptance Criteria:**
- [ ] Fixed DOFs result in zero (or prescribed) displacement
- [ ] Reactions can be recovered from K * u - F
- [ ] System remains solvable after BC application
- [ ] Warping DOF (index 6) can be fixed or left free
- [ ] Fork support correctly leaves warping free
- [ ] Built-in support with warping correctly restrains warping

---

### Task 3.4: Implement Linear Solver
**Requirements:** R-ASM-004, R-ASM-005, R-NFR-001
**Dependencies:** Task 3.3
**Difficulty:** Medium

**Description:**
Implement the linear system solver.

**Steps:**
1. Create `cpp/include/grillex/solver.hpp`:
   ```cpp
   class LinearSolver {
   public:
       enum class Method { SparseLU, SimplicialLDLT, ConjugateGradient };

       LinearSolver(Method method = Method::SimplicialLDLT);

       // Solve K * u = F
       Eigen::VectorXd solve(
           const Eigen::SparseMatrix<double>& K,
           const Eigen::VectorXd& F);

       // Check if system is singular (detect rigid body modes)
       bool is_singular() const;
       std::string get_error_message() const;

   private:
       Method method_;
       bool singular_ = false;
       std::string error_msg_;
   };
   ```

2. Use Eigen's sparse solvers:
   ```cpp
   Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
   solver.compute(K);
   if (solver.info() != Eigen::Success) {
       singular_ = true;
       // Detect cause...
   }
   return solver.solve(F);
   ```

**Acceptance Criteria:**
- [ ] Solver returns correct displacement for simple problems
- [ ] Singular systems are detected and reported
- [ ] Performance is acceptable for sparse systems (1000+ DOFs)

---

### Task 3.5: Implement Model Class (Orchestration)
**Requirements:** R-ARCH-003
**Dependencies:** Tasks 3.1-3.4
**Difficulty:** Medium

**Description:**
Create the top-level Model class that orchestrates analysis.

**Steps:**
1. Create `cpp/include/grillex/model.hpp`:
   ```cpp
   class Model {
   public:
       NodeRegistry nodes;
       std::vector<std::unique_ptr<Material>> materials;
       std::vector<std::unique_ptr<Section>> sections;
       std::vector<std::unique_ptr<BeamElement>> elements;
       BCHandler boundary_conditions;

       // Create entities
       Material* create_material(const std::string& name, double E, double nu, double rho);
       Section* create_section(const std::string& name, double A, double Iy, double Iz, double J);
       BeamElement* create_beam(Node* node_i, Node* node_j, Material* mat, Section* sec);

       // Run analysis
       bool analyze();

       // Results
       Eigen::VectorXd displacements;
       Eigen::VectorXd reactions;

   private:
       DOFHandler dof_handler_;
       Assembler assembler_;
       LinearSolver solver_;
   };
   ```

2. Implement `analyze()`:
   ```
   1. Number DOFs
   2. Assemble K
   3. Create F (initially zeros)
   4. Apply boundary conditions
   5. Solve
   6. Store displacements
   7. Compute reactions
   ```

**Acceptance Criteria:**
- [ ] Complete analysis workflow runs without errors
- [ ] Results match hand calculations for simple models
- [ ] Error handling for invalid models

---

