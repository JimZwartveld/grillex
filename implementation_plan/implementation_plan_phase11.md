## Phase 11: Error Handling & Diagnostics

### Task 11.1: Implement Error Code System
**Requirements:** R-ERR-001, R-ERR-002
**Dependencies:** Task 3.4
**Difficulty:** Medium

**Description:**
Create structured error reporting.

**Steps:**
1. Create `cpp/include/grillex/errors.hpp`:
   ```cpp
   enum class ErrorCode {
       OK,
       UNCONSTRAINED_SYSTEM,
       SINGULAR_MATRIX,
       INVALID_PROPERTY,
       INVALID_ELEMENT,
       INVALID_NODE_REFERENCE,
       // ...
   };

   struct GrillexError {
       ErrorCode code;
       std::string message;
       std::vector<int> involved_dofs;
       std::vector<int> involved_elements;

       static GrillexError unconstrained(const std::vector<int>& dofs);
       static GrillexError singular(const std::string& details);
   };
   ```

2. Return errors from analysis instead of throwing exceptions
3. Python wrapper converts to Python exceptions or error objects

**Acceptance Criteria:**
- [x] Errors have machine-readable codes
- [x] Errors have human-readable messages
- [x] Diagnostic info (DOFs, elements) is included

### Execution Notes (Completed 2025-12-20)

**Steps Taken:**
1. Created `cpp/include/grillex/errors.hpp` with:
   - `ErrorCode` enum with categorized error codes (100s structural, 200s elements, 300s loads, 400s model, 500s solver)
   - `GrillexError` struct with code, message, involved_dofs, involved_elements, involved_nodes, details map, and suggestion
   - Factory methods: `unconstrained()`, `singular()`, `invalid_element()`, `invalid_node()`, `empty_model()`, `not_analyzed()`
   - Helper functions: `error_code_to_string()`, `is_ok()`, `is_error()`, `to_string()`
2. Added Python bindings in `cpp/bindings/bindings.cpp`
3. Updated Python exports in `src/grillex/core/data_types.py` and `__init__.py`
4. Created comprehensive test suite in `tests/python/test_phase11_error_handling.py` (55 tests)
5. Created Sphinx documentation at `docs/errors.rst`

**Error Code Categories:**
- 0: OK
- 100-199: Structural/constraint errors (UNCONSTRAINED_SYSTEM, SINGULAR_MATRIX, etc.)
- 200-299: Element errors (INVALID_ELEMENT, INVALID_MATERIAL, INVALID_SECTION, etc.)
- 300-399: Load errors (INVALID_LOAD_NODE, EMPTY_LOAD_CASE, etc.)
- 400-499: Model errors (EMPTY_MODEL, NOT_ANALYZED, etc.)
- 500-599: Solver errors (SOLVER_CONVERGENCE_FAILED, NUMERICAL_OVERFLOW, etc.)
- 999: UNKNOWN_ERROR

**Problems Encountered:**
- None

**Verification:**
- 55 unit tests passing for error/warning system ✓
- All error codes are unique
- Factory methods work correctly
- to_string() produces well-formatted output

---

### Task 11.2: Implement Warning System
**Requirements:** R-ERR-003, R-ERR-004
**Dependencies:** Task 11.1
**Difficulty:** Medium

**Description:**
Create warning system for questionable models.

**Steps:**
1. Create warning structure:
   ```cpp
   enum class WarningCode {
       EXTREME_ASPECT_RATIO,
       NEAR_SINGULARITY,
       STIFFNESS_CONTRAST,
       SMALL_ELEMENT,
       // ...
   };

   enum class WarningSeverity { Low, Medium, High };

   struct GrillexWarning {
       WarningCode code;
       WarningSeverity severity;
       std::string message;
       std::map<std::string, std::string> details;
   };
   ```

2. Add model validation checks:
   - Beam aspect ratio (length/depth)
   - Stiffness ratios between adjacent elements
   - Very short elements
   - Near-zero properties

**Acceptance Criteria:**
- [x] Warnings don't block analysis
- [x] Severity levels are assigned appropriately
- [x] LLM can parse warning codes

### Execution Notes (Completed 2025-12-20)

**Steps Taken:**
1. Created `cpp/include/grillex/warnings.hpp` with:
   - `WarningCode` enum with categorized codes (100s geometry, 200s stiffness, 300s property, 400s load, 500s analysis)
   - `WarningSeverity` enum (Low=0, Medium=1, High=2)
   - `GrillexWarning` struct with code, severity, message, involved_elements, involved_nodes, details map, and suggestion
   - Factory methods: `extreme_aspect_ratio()`, `small_element()`, `stiffness_contrast()`, `near_singularity()`, `large_displacement()`, `near_zero_property()`
   - `WarningList` class for collecting warnings with filtering by severity
2. Added Python bindings in `cpp/bindings/bindings.cpp`
3. Updated Python exports in `src/grillex/core/data_types.py` and `__init__.py`
4. Added tests in `tests/python/test_phase11_error_handling.py`
5. Added to Sphinx documentation at `docs/errors.rst`

**Warning Code Categories:**
- 100-199: Geometry warnings (EXTREME_ASPECT_RATIO, SMALL_ELEMENT, LARGE_ELEMENT, NON_COLLINEAR_WARPING)
- 200-299: Stiffness warnings (STIFFNESS_CONTRAST, NEAR_SINGULARITY, VERY_STIFF_SPRING, VERY_SOFT_SPRING)
- 300-399: Property warnings (NEAR_ZERO_PROPERTY, POSSIBLE_UNIT_ERROR, INCONSISTENT_SECTION)
- 400-499: Load warnings (LARGE_LOAD, LOAD_AT_FREE_NODE, ACCELERATION_WITHOUT_MASS)
- 500-599: Analysis warnings (LARGE_DISPLACEMENT, HIGH_STRESS, SOLVER_REFINEMENT)

**WarningList Features:**
- `add()` - Add warnings
- `has_warnings()` - Check if any warnings exist
- `count()` - Total warning count
- `count_by_severity()` - Count by severity level
- `get_by_min_severity()` - Filter warnings by minimum severity
- `clear()` - Clear all warnings
- `summary()` - Get formatted summary string

**Problems Encountered:**
- None

**Verification:**
- All warning tests passing ✓
- Severity levels work correctly
- WarningList filtering works as expected

---

### Task 11.3: Detect Rigid Body Modes
**Requirements:** R-VAL-003, R-ERR-002
**Dependencies:** Task 3.4, Task 16.3 (Eigenvalue Solver)
**Difficulty:** High

**Description:**
Detect and report unconstrained rigid body modes using eigenvalue analysis.

**Steps:**
1. After assembly, check for singularity:
   ```cpp
   std::vector<int> find_unconstrained_dofs(
       const Eigen::SparseMatrix<double>& K) {
       // Compute eigenvalues near zero
       // Identify DOFs associated with near-zero modes
       // Return list of unconstrained DOFs
   }
   ```

2. Report which nodes/DOFs are unconstrained
3. Suggest which supports to add

**Acceptance Criteria:**
- [x] Free-floating model detected as singular
- [x] Specific unconstrained DOFs identified
- [x] Helpful diagnostic message generated

### Execution Notes (Completed 2025-12-22)

**Steps Taken:**
1. Created `cpp/include/grillex/singularity_diagnostics.hpp` with:
   - `RigidBodyModeType` enum (TranslationX/Y/Z, RotationX/Y/Z, Warping, Mixed)
   - `RigidBodyModeInfo` struct with mode number, eigenvalue, type, involved nodes/DOFs
   - `DOFParticipation` struct tracking DOF participation in rigid body modes
   - `SingularityDiagnostics` result struct with is_singular flag, mode info, messages
   - `SingularityAnalyzerSettings` struct for configuration
   - `SingularityAnalyzer` class using eigenvalue decomposition

2. Created `cpp/src/singularity_diagnostics.cpp` implementing:
   - Eigenvalue analysis to detect near-zero eigenvalues (rigid body modes)
   - Mode type identification from eigenvector patterns
   - DOF participation extraction and ranking
   - Fix suggestion generation based on mode type
   - JSON and string output formats

3. Added reverse lookup methods to `DOFHandler`:
   - `is_warping_dof(global_dof)`
   - `get_node_from_global_dof(global_dof)`
   - `get_local_dof_from_global_dof(global_dof)`
   - `get_element_from_warping_dof(global_dof)`

4. Added Python bindings for all singularity diagnostics types
5. Updated Python exports in `data_types.py` and `__init__.py`
6. Created comprehensive test suite `tests/python/test_phase11_singularity_diagnostics.py` (24 tests)

**Implementation Features:**
- Uses eigenvalue analysis from Phase 16 to detect rigid body modes
- Identifies specific unconstrained DOFs with participation levels
- Classifies mode types (translation, rotation, warping, mixed)
- Generates actionable fix suggestions (e.g., "Add X-direction support at node 1")
- Provides both human-readable and machine-readable (JSON) output
- `is_singular()` quick check method for fast detection
- `count_rigid_body_modes()` for mode counting

**Problems Encountered:**
- **Issue**: C++ default member initializers in nested struct caused compilation error
  - **Error**: "default member initializer required before end of enclosing class"
  - **Solution**: Moved `SingularityAnalyzerSettings` to top-level struct before `SingularityAnalyzer` class

- **Issue**: Test fixtures used outdated API signatures
  - **Solution**: Updated Material/Section constructors to include name parameter, Assembler to pass dof_handler and use assemble_stiffness([beam]) pattern

**Verification:**
- 24 unit tests passing ✓
- Free-floating beam correctly detected with 6+ rigid body modes
- Well-constrained cantilever correctly detected as non-singular
- Suggested fixes include specific DOFs and directions
- JSON output is machine-parseable

**Key Learnings:**
- Eigenvalue analysis is essential for proper rigid body mode detection
- Penalty BC method masks singularity (by design), so diagnostics should run on raw matrices
- Mode type identification from eigenvectors requires participation threshold tuning

---

