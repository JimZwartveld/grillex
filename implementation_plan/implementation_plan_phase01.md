## Phase 1: Core Data Structures (C++)

### Task 1.1: Implement Node Class (C++)
**Requirements:** R-MOD-003, R-DOF-001
**Dependencies:** Task 0.2
**Difficulty:** Medium

**Description:**
Create the C++ Node class representing a point in the structural model.

**Steps:**
1. Create `cpp/include/grillex/node.hpp`:
   ```cpp
   class Node {
   public:
       int id;
       double x, y, z;
       // DOF flags (which DOFs are active)
       std::array<bool, 6> dof_active = {true, true, true, true, true, true};
       // Global DOF numbers (-1 = not assigned)
       std::array<int, 6> global_dof_numbers = {-1, -1, -1, -1, -1, -1};

       Node(int id, double x, double y, double z);
       Eigen::Vector3d position() const;
   };
   ```

2. Create `cpp/src/node.cpp` with implementation

3. Add Eigen dependency to CMakeLists.txt (use FetchContent)

**Acceptance Criteria:**
- [x] Node can be constructed with id and coordinates
- [x] Position can be retrieved as Eigen::Vector3d
- [x] DOF arrays are correctly initialized

---

### Task 1.2: Implement Node Registry (C++)
**Requirements:** R-MOD-003, R-MOD-004, R-UNITS-005
**Dependencies:** Task 1.1
**Difficulty:** Medium

**Description:**
Create a registry that manages nodes and handles node merging based on tolerance.

**Steps:**
1. Create `cpp/include/grillex/node_registry.hpp`:
   ```cpp
   class NodeRegistry {
   public:
       NodeRegistry(double tolerance = 1e-6);

       // Returns existing node if within tolerance, else creates new
       Node* get_or_create_node(double x, double y, double z);
       Node* get_node_by_id(int id);
       const std::vector<std::unique_ptr<Node>>& all_nodes() const;

       void set_tolerance(double tol);
       double get_tolerance() const;

   private:
       double tolerance_;
       std::vector<std::unique_ptr<Node>> nodes_;
       int next_id_ = 1;
   };
   ```

2. Implement node lookup using distance calculation:
   - For each existing node, compute Euclidean distance
   - If distance < tolerance, return existing node
   - Otherwise create new node with next_id

**Acceptance Criteria:**
- [x] Creating node at (0,0,0) twice returns same node
- [x] Creating node at (0,0,1e-9) with tolerance 1e-6 returns existing node
- [x] Creating node at (0,0,1) with tolerance 1e-6 creates new node
- [x] Node IDs are sequential and unique

---

### Task 1.3: Implement Material Class (C++)
**Requirements:** R-ARCH-005
**Dependencies:** Task 0.2
**Difficulty:** Low

**Description:**
Create a Material class for storing material properties.

**Steps:**
1. Create `cpp/include/grillex/material.hpp`:
   ```cpp
   class Material {
   public:
       int id;
       std::string name;
       double E;      // Young's modulus (kN/m^2)
       double G;      // Shear modulus (kN/m^2)
       double nu;     // Poisson's ratio
       double rho;    // Density (mT/m^3)

       Material(int id, std::string name, double E, double nu, double rho);

       // Compute G from E and nu if not provided
       static double compute_G(double E, double nu);
   };
   ```

2. Implement with automatic G calculation: G = E / (2 * (1 + nu))

**Acceptance Criteria:**
- [x] Material can be constructed with standard properties
- [x] G is correctly computed from E and nu
- [x] All units are documented in header comments

---

### Task 1.4: Implement Section Class (C++)
**Requirements:** R-ARCH-005, R-ELEM-001
**Dependencies:** Task 0.2
**Difficulty:** Medium

**Description:**
Create Section class for beam cross-section properties.

**Steps:**
1. Create `cpp/include/grillex/section.hpp`:
   ```cpp
   class Section {
   public:
       int id;
       std::string name;
       double A;      // Area (m^2)
       double Iy;     // Second moment of area about local y (m^4)
       double Iz;     // Second moment of area about local z (m^4)
       double J;      // Torsional constant (m^4)
       double Iw;     // Warping constant (m^6) - optional
       double Asy;    // Shear area y-direction (m^2)
       double Asz;    // Shear area z-direction (m^2)

       // For stress calculation
       double zy_top, zy_bot;   // Distance to extreme fibres (y-axis)
       double zz_top, zz_bot;   // Distance to extreme fibres (z-axis)

       Section(int id, std::string name, double A, double Iy, double Iz, double J);
   };
   ```

2. Implement with sensible defaults for optional parameters

**Acceptance Criteria:**
- [x] Section can be constructed with basic properties
- [x] Optional properties default to 0 or computed values
- [x] Section can store fibre distances for stress calculation

---

### Task 1.5: Create Python Bindings for Data Structures
**Requirements:** R-ARCH-004
**Dependencies:** Tasks 1.1-1.4
**Difficulty:** Medium

**Description:**
Expose Node, NodeRegistry, Material, and Section to Python via pybind11.

**Steps:**
1. Update `cpp/bindings/bindings.cpp`:
   ```cpp
   #include <pybind11/pybind11.h>
   #include <pybind11/stl.h>
   #include <pybind11/eigen.h>

   #include "grillex/node.hpp"
   #include "grillex/node_registry.hpp"
   #include "grillex/material.hpp"
   #include "grillex/section.hpp"

   namespace py = pybind11;

   PYBIND11_MODULE(_grillex_cpp, m) {
       py::class_<Node>(m, "Node")
           .def(py::init<int, double, double, double>())
           .def_readonly("id", &Node::id)
           .def_readonly("x", &Node::x)
           // ... etc
       ;
       // Similar for other classes
   }
   ```

2. Create `grillex/core/data_types.py` that imports and re-exports from `_grillex_cpp`

**Acceptance Criteria:**
- [x] `from grillex.core import Node, Material, Section` works
- [x] Can create instances from Python
- [x] Properties are accessible

### Execution Notes (Completed 2025-12-08)

**Steps Taken:**
1. Created C++ header and implementation files for all core data structures:
   - `cpp/include/grillex/node.hpp` and `cpp/src/node.cpp`
   - `cpp/include/grillex/node_registry.hpp` and `cpp/src/node_registry.cpp`
   - `cpp/include/grillex/material.hpp` and `cpp/src/material.cpp`
   - `cpp/include/grillex/section.hpp` and `cpp/src/section.cpp`

2. Updated `cpp/CMakeLists.txt` to include new source files in the build

3. Created comprehensive Python bindings in `cpp/bindings/bindings.cpp`:
   - Added pybind11/eigen.h for Eigen::Vector3d support
   - Exposed all four classes with full property access
   - Added __repr__ methods for user-friendly output
   - Used lambda for NodeRegistry.all_nodes() to handle unique_ptr vector

4. Created Python wrapper module `src/grillex/core/data_types.py`
   - Re-exports C++ classes for clean Python API
   - Updated `src/grillex/core/__init__.py` for easy imports

5. Created comprehensive test suite `tests/python/test_phase1_data_structures.py`:
   - 24 tests covering all acceptance criteria
   - Tests organized by task (Node, NodeRegistry, Material, Section, Bindings)
   - All tests passing ✓

**Problems Encountered:**
- **Issue**: pybind11 could not cast vector of unique_ptr directly
  - **Error**: `binding reference of type 'unique_ptr<...>' to value of type 'const unique_ptr<...>' drops 'const' qualifier`
  - **Solution**: Used lambda in binding to convert unique_ptr vector to list of raw pointers
  ```cpp
  .def("all_nodes", [](const grillex::NodeRegistry &reg) {
      py::list result;
      for (const auto& node : reg.all_nodes()) {
          result.append(node.get());
      }
      return result;
  }, ...)
  ```

**Verification:**
- All 31 tests passing (24 new Phase 1 tests + 7 existing)
- Build successful with no warnings
- All acceptance criteria met:
  - Node class: construction, position vector, DOF initialization ✓
  - NodeRegistry: node merging, tolerance handling, sequential IDs ✓
  - Material: construction, automatic G computation, documented units ✓
  - Section: construction, optional parameters, fibre distances ✓
  - Python bindings: imports work, instance creation, property access ✓

**Code Coverage:** 98% (45 statements, 1 miss)

**Key Design Decisions:**
1. Used raw pointers for node references (lifetime managed by NodeRegistry)
2. Automatic shear modulus calculation in Material constructor
3. Comprehensive docstrings with units in C++ headers
4. Python-friendly __repr__ methods for all classes

**Files Created:**
- C++ Headers: 4 files in `cpp/include/grillex/`
- C++ Source: 4 files in `cpp/src/`
- Python Wrapper: `src/grillex/core/data_types.py`
- Tests: `tests/python/test_phase1_data_structures.py`
- Updated: `cpp/CMakeLists.txt`, `cpp/bindings/bindings.cpp`, `src/grillex/core/__init__.py`

**Time Taken:** ~45 minutes

---

