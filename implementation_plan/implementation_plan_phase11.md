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
- [ ] Errors have machine-readable codes
- [ ] Errors have human-readable messages
- [ ] Diagnostic info (DOFs, elements) is included

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
- [ ] Warnings don't block analysis
- [ ] Severity levels are assigned appropriately
- [ ] LLM can parse warning codes

---

### Task 11.3: Detect Rigid Body Modes
**Requirements:** R-VAL-003, R-ERR-002
**Dependencies:** Task 3.4
**Difficulty:** High

**Description:**
Detect and report unconstrained rigid body modes.

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
- [ ] Free-floating model detected as singular
- [ ] Specific unconstrained DOFs identified
- [ ] Helpful diagnostic message generated

---

