# Phase 15: Nonlinear Springs (Tension/Compression-Only)

## Overview

This phase implements support for tension-only and compression-only spring elements, which requires an iterative nonlinear solver. This is a significant architectural addition because spring stiffness depends on the displacement state, creating a chicken-and-egg problem that requires iteration to resolve.

### Why This Is Nonlinear

In a linear system, the stiffness matrix K is constant:
```
K * u = F  →  u = K⁻¹ * F
```

With tension/compression-only springs, K depends on whether each spring is in tension or compression:
```
K(u) * u = F  →  Requires iteration
```

The spring state (active/inactive) is unknown until we solve, but the solution depends on which springs are active.

### Load Combination Implications

**Critical Change**: Currently, load combinations work by linear superposition:
```python
u_combined = factor1 * u_loadcase1 + factor2 * u_loadcase2
```

With nonlinear springs, this approach is **invalid** because:
- Load case 1 alone might have spring A in tension (active)
- Load case 2 alone might have spring A in compression (inactive)
- The combination might have spring A active or inactive depending on the net effect
- You cannot determine the combined spring state from individual results

**Solution**: Each load combination must be solved as a separate nonlinear analysis, not as post-processed linear superposition.

---

## Task 15.1: Spring Element State Tracking

**Requirements:** R-ELEM-010 (extended)
**Dependencies:** Task 8.1 (Spring Element)
**Difficulty:** Medium

**Description:**
Extend SpringElement to support tension-only and compression-only behavior with state tracking.

**Steps:**

1. Add spring behavior enum to `spring_element.hpp`:
   ```cpp
   enum class SpringBehavior {
       Linear = 0,       // Always active (default, current behavior)
       TensionOnly = 1,  // Active only when elongated (Δu > 0)
       CompressionOnly = 2  // Active only when compressed (Δu < 0)
   };
   ```

2. Add state tracking to SpringElement:
   ```cpp
   class SpringElement {
   public:
       // Existing members...

       // Per-DOF behavior (allows mixed, e.g., tension-only in Z, linear in X)
       std::array<SpringBehavior, 6> behavior = {
           SpringBehavior::Linear, SpringBehavior::Linear,
           SpringBehavior::Linear, SpringBehavior::Linear,
           SpringBehavior::Linear, SpringBehavior::Linear
       };

       // State tracking (updated during iteration)
       std::array<bool, 6> is_active = {true, true, true, true, true, true};

       // Elongation/compression for each DOF (u_j - u_i)
       std::array<double, 6> deformation = {0, 0, 0, 0, 0, 0};

       // Methods
       void update_state(const Eigen::VectorXd& displacements,
                        const DOFHandler& dof_handler);
       bool state_changed() const;  // Did state change in last update?
       Eigen::Matrix<double, 12, 12> current_stiffness_matrix() const;
   };
   ```

3. Implement state update logic:
   ```cpp
   void SpringElement::update_state(const Eigen::VectorXd& u,
                                    const DOFHandler& dof_handler) {
       std::array<bool, 6> previous_state = is_active;

       // Get DOF indices for both nodes
       auto dofs_i = dof_handler.get_node_dofs(node_i->id);
       auto dofs_j = dof_handler.get_node_dofs(node_j->id);

       for (int i = 0; i < 6; ++i) {
           // Compute deformation: u_j - u_i
           double u_i = (dofs_i[i] >= 0) ? u(dofs_i[i]) : 0.0;
           double u_j = (dofs_j[i] >= 0) ? u(dofs_j[i]) : 0.0;
           deformation[i] = u_j - u_i;

           // Update active state based on behavior
           switch (behavior[i]) {
               case SpringBehavior::Linear:
                   is_active[i] = true;
                   break;
               case SpringBehavior::TensionOnly:
                   is_active[i] = (deformation[i] > -gap_tolerance_);
                   break;
               case SpringBehavior::CompressionOnly:
                   is_active[i] = (deformation[i] < gap_tolerance_);
                   break;
           }
       }

       state_changed_ = (previous_state != is_active);
   }
   ```

4. Implement current stiffness matrix (respects active state):
   ```cpp
   Eigen::Matrix<double, 12, 12> SpringElement::current_stiffness_matrix() const {
       Eigen::Matrix<double, 12, 12> K = Eigen::Matrix<double, 12, 12>::Zero();
       std::array<double, 6> k_values = {kx, ky, kz, krx, kry, krz};

       for (int i = 0; i < 6; ++i) {
           if (!is_active[i]) continue;  // Skip inactive DOFs

           double k = k_values[i];
           if (std::abs(k) > 1e-20) {
               K(i, i) = k;
               K(i, i + 6) = -k;
               K(i + 6, i) = -k;
               K(i + 6, i + 6) = k;
           }
       }
       return K;
   }
   ```

**Acceptance Criteria:**
- [ ] SpringBehavior enum added with Linear, TensionOnly, CompressionOnly values
- [ ] Per-DOF behavior can be set independently
- [ ] State tracking correctly identifies tension vs compression
- [ ] current_stiffness_matrix() returns zero contribution for inactive DOFs
- [ ] state_changed() correctly detects state transitions
- [ ] Gap tolerance prevents chattering near zero deformation

---

## Task 15.2: Iterative Nonlinear Solver

**Requirements:** New
**Dependencies:** Task 15.1, Task 3.3 (LinearSolver)
**Difficulty:** High

**Description:**
Implement an iterative solver that handles spring state changes until convergence.

**Algorithm: Modified Newton-Raphson with State Update**

```
1. Initialize: Assume all springs active, solve linear system
2. Loop:
   a. Update spring states based on current displacement
   b. If no state changed → CONVERGED, exit
   c. Reassemble stiffness matrix with new states
   d. Solve: K_new * u_new = F
   e. If iteration > max_iterations → FAILED, exit
   f. Go to step 2a
```

**Steps:**

1. Create `cpp/include/grillex/nonlinear_solver.hpp`:
   ```cpp
   #pragma once
   #include "grillex/solver.hpp"
   #include "grillex/spring_element.hpp"
   #include <vector>

   namespace grillex {

   struct NonlinearSolverResult {
       Eigen::VectorXd displacements;
       bool converged = false;
       int iterations = 0;
       std::string message;

       // Spring states at convergence
       std::vector<std::pair<int, std::array<bool, 6>>> spring_states;
   };

   struct NonlinearSolverSettings {
       int max_iterations = 50;
       double displacement_tolerance = 1e-6;  // Relative change in u
       double gap_tolerance = 1e-10;          // Tolerance for zero deformation
       bool allow_all_springs_inactive = false;  // Allow solution with no springs?
       LinearSolver::Method linear_method = LinearSolver::Method::SimplicialLDLT;
   };

   class NonlinearSolver {
   public:
       explicit NonlinearSolver(const NonlinearSolverSettings& settings = {});

       /**
        * @brief Solve system with nonlinear springs
        * @param base_K Stiffness matrix WITHOUT spring contributions
        * @param F Force vector
        * @param springs Vector of spring elements (will be modified for state)
        * @param dof_handler DOF handler for DOF indexing
        * @return Solver result with displacements and convergence info
        */
       NonlinearSolverResult solve(
           const Eigen::SparseMatrix<double>& base_K,
           const Eigen::VectorXd& F,
           std::vector<SpringElement*>& springs,
           const DOFHandler& dof_handler);

       const NonlinearSolverSettings& settings() const { return settings_; }
       void set_settings(const NonlinearSolverSettings& s) { settings_ = s; }

   private:
       NonlinearSolverSettings settings_;
       LinearSolver linear_solver_;

       Eigen::SparseMatrix<double> assemble_spring_stiffness(
           const std::vector<SpringElement*>& springs,
           const DOFHandler& dof_handler,
           int total_dofs) const;

       bool check_displacement_convergence(
           const Eigen::VectorXd& u_old,
           const Eigen::VectorXd& u_new) const;
   };

   } // namespace grillex
   ```

2. Implement solver in `cpp/src/nonlinear_solver.cpp`:
   ```cpp
   NonlinearSolverResult NonlinearSolver::solve(
       const Eigen::SparseMatrix<double>& base_K,
       const Eigen::VectorXd& F,
       std::vector<SpringElement*>& springs,
       const DOFHandler& dof_handler)
   {
       NonlinearSolverResult result;
       int n = base_K.rows();

       // Check if any springs are nonlinear
       bool has_nonlinear = false;
       for (auto* spring : springs) {
           for (int i = 0; i < 6; ++i) {
               if (spring->behavior[i] != SpringBehavior::Linear) {
                   has_nonlinear = true;
                   break;
               }
           }
           if (has_nonlinear) break;
       }

       // If all springs are linear, solve directly
       if (!has_nonlinear) {
           auto K_springs = assemble_spring_stiffness(springs, dof_handler, n);
           Eigen::SparseMatrix<double> K_total = base_K + K_springs;
           result.displacements = linear_solver_.solve(K_total, F);
           result.converged = !linear_solver_.is_singular();
           result.iterations = 1;
           result.message = result.converged ? "Linear solve (no nonlinear springs)"
                                             : linear_solver_.get_error_message();
           return result;
       }

       // Initialize: all springs active
       for (auto* spring : springs) {
           spring->is_active.fill(true);
       }

       Eigen::VectorXd u = Eigen::VectorXd::Zero(n);

       for (int iter = 0; iter < settings_.max_iterations; ++iter) {
           result.iterations = iter + 1;

           // Assemble current stiffness
           auto K_springs = assemble_spring_stiffness(springs, dof_handler, n);
           Eigen::SparseMatrix<double> K_total = base_K + K_springs;

           // Solve
           Eigen::VectorXd u_new = linear_solver_.solve(K_total, F);

           if (linear_solver_.is_singular()) {
               result.converged = false;
               result.message = "Singular system at iteration " +
                               std::to_string(iter + 1);
               return result;
           }

           // Update spring states
           bool any_changed = false;
           for (auto* spring : springs) {
               spring->update_state(u_new, dof_handler);
               if (spring->state_changed()) {
                   any_changed = true;
               }
           }

           // Check convergence: no state changes
           if (!any_changed) {
               result.displacements = u_new;
               result.converged = true;
               result.message = "Converged after " +
                               std::to_string(iter + 1) + " iterations";

               // Store final spring states
               for (auto* spring : springs) {
                   result.spring_states.emplace_back(
                       spring->id, spring->is_active);
               }
               return result;
           }

           u = u_new;
       }

       // Failed to converge
       result.displacements = u;
       result.converged = false;
       result.message = "Failed to converge after " +
                       std::to_string(settings_.max_iterations) + " iterations";
       return result;
   }
   ```

3. Add oscillation detection and damping:
   ```cpp
   // Track state history to detect oscillation
   std::vector<std::vector<bool>> state_history;

   // If same state pattern repeats, apply damping:
   // - Use weighted average of solutions
   // - Or use partial stiffness for transitioning springs
   ```

**Acceptance Criteria:**
- [ ] NonlinearSolver class implemented with settings struct
- [ ] Iterative solve correctly updates spring states
- [ ] Convergence detected when no spring states change
- [ ] Maximum iteration limit prevents infinite loops
- [ ] Linear-only springs bypass iteration (optimization)
- [ ] Oscillation detection prevents flip-flopping states
- [ ] Singular system during iteration handled gracefully
- [ ] Result includes final spring states for reporting

---

## Task 15.3: Model Integration for Nonlinear Analysis

**Requirements:** New
**Dependencies:** Task 15.2, Task 3.4 (Model)
**Difficulty:** Medium

**Description:**
Integrate nonlinear solver into Model class, handling load cases and combinations properly.

**Key Changes:**

1. Separate stiffness assembly:
   - Base stiffness (beams, plates) - constant
   - Spring stiffness - may change during iteration

2. Load combination behavior:
   - Linear springs: Can still use superposition
   - Nonlinear springs: Must solve each combination separately

**Steps:**

1. Add nonlinear analysis methods to Model:
   ```cpp
   class Model {
   public:
       // Existing...

       // Check if model has nonlinear springs
       bool has_nonlinear_springs() const;

       // Analyze with nonlinear spring support
       void analyze_nonlinear();

       // Analyze specific load combination (required for nonlinear)
       LoadCombinationResult analyze_combination(
           const LoadCombination& combo,
           const NonlinearSolverSettings& settings = {});

       // Get settings
       NonlinearSolverSettings& nonlinear_settings() { return nl_settings_; }

   private:
       NonlinearSolverSettings nl_settings_;

       // Separate assembly
       Eigen::SparseMatrix<double> assemble_base_stiffness();  // No springs
       Eigen::SparseMatrix<double> assemble_spring_stiffness(); // Springs only
   };
   ```

2. Implement `analyze_nonlinear()`:
   ```cpp
   void Model::analyze_nonlinear() {
       if (!has_nonlinear_springs()) {
           // Fall back to linear analysis
           analyze();
           return;
       }

       // Assemble base stiffness once
       auto base_K = assemble_base_stiffness();

       NonlinearSolver solver(nl_settings_);

       // Solve each load case
       for (auto& lc : load_cases_) {
           auto F = assemble_force_vector(lc);
           F = apply_accelerations(F, lc);

           auto result = solver.solve(base_K, F, springs_, dof_handler_);

           LoadCaseResult lc_result;
           lc_result.displacements = result.displacements;
           lc_result.success = result.converged;
           lc_result.iterations = result.iterations;
           lc_result.message = result.message;
           lc_result.spring_states = result.spring_states;

           // Compute reactions
           auto K_total = base_K + assemble_current_spring_stiffness();
           lc_result.reactions = compute_reactions(K_total, result.displacements, F);

           results_[lc.id()] = lc_result;
       }
   }
   ```

3. Implement `analyze_combination()`:
   ```cpp
   LoadCombinationResult Model::analyze_combination(
       const LoadCombination& combo,
       const NonlinearSolverSettings& settings)
   {
       // Cannot use superposition with nonlinear springs
       // Must solve the combined load directly

       auto base_K = assemble_base_stiffness();
       Eigen::VectorXd F_combined = Eigen::VectorXd::Zero(dof_handler_.total_dofs());

       // Combine force vectors with factors
       for (const auto& term : combo.get_terms()) {
           auto F_case = assemble_force_vector(*term.load_case);
           F_case = apply_accelerations(F_case, *term.load_case);
           F_combined += term.factor * F_case;
       }

       NonlinearSolver solver(settings);
       auto result = solver.solve(base_K, F_combined, springs_, dof_handler_);

       LoadCombinationResult combo_result;
       combo_result.displacements = result.displacements;
       combo_result.converged = result.converged;
       combo_result.iterations = result.iterations;
       combo_result.spring_states = result.spring_states;

       return combo_result;
   }
   ```

**Acceptance Criteria:**
- [ ] has_nonlinear_springs() correctly identifies nonlinear springs in model
- [ ] analyze_nonlinear() uses iterative solver for load cases
- [ ] analyze_combination() solves combined loads directly (not superposition)
- [ ] Linear models still use efficient linear solver
- [ ] LoadCaseResult extended with iteration count and spring states
- [ ] Reactions computed correctly with final spring stiffness

---

## Task 15.4: Python API Updates

**Requirements:** R-LLM-001 (Type hints)
**Dependencies:** Task 15.3
**Difficulty:** Medium

**Description:**
Expose nonlinear spring functionality through Python API.

**Steps:**

1. Add pybind11 bindings for new types:
   ```cpp
   // SpringBehavior enum
   py::enum_<grillex::SpringBehavior>(m, "SpringBehavior",
       "Spring behavior type for nonlinear analysis")
       .value("Linear", grillex::SpringBehavior::Linear,
              "Always active (default)")
       .value("TensionOnly", grillex::SpringBehavior::TensionOnly,
              "Active only when elongated")
       .value("CompressionOnly", grillex::SpringBehavior::CompressionOnly,
              "Active only when compressed")
       .export_values();

   // NonlinearSolverSettings
   py::class_<grillex::NonlinearSolverSettings>(m, "NonlinearSolverSettings")
       .def(py::init<>())
       .def_readwrite("max_iterations", &grillex::NonlinearSolverSettings::max_iterations)
       .def_readwrite("gap_tolerance", &grillex::NonlinearSolverSettings::gap_tolerance)
       // ...

   // NonlinearSolverResult
   py::class_<grillex::NonlinearSolverResult>(m, "NonlinearSolverResult")
       .def_readonly("displacements", &grillex::NonlinearSolverResult::displacements)
       .def_readonly("converged", &grillex::NonlinearSolverResult::converged)
       .def_readonly("iterations", &grillex::NonlinearSolverResult::iterations)
       .def_readonly("message", &grillex::NonlinearSolverResult::message)
       .def_readonly("spring_states", &grillex::NonlinearSolverResult::spring_states);

   // SpringElement updates
   py::class_<grillex::SpringElement>(...)
       // Existing...
       .def_readwrite("behavior", &grillex::SpringElement::behavior)
       .def_readonly("is_active", &grillex::SpringElement::is_active)
       .def_readonly("deformation", &grillex::SpringElement::deformation)
       .def("set_behavior", [](SpringElement& s, int dof, SpringBehavior b) {
           s.behavior[dof] = b;
       }, py::arg("dof"), py::arg("behavior"))
       .def("set_all_behavior", [](SpringElement& s, SpringBehavior b) {
           s.behavior.fill(b);
       }, py::arg("behavior"));
   ```

2. Update Python wrapper in `model_wrapper.py`:
   ```python
   def add_spring(
       self,
       node1: Union[List[float], Node],
       node2: Union[List[float], Node],
       kx: float = 0.0,
       ky: float = 0.0,
       kz: float = 0.0,
       krx: float = 0.0,
       kry: float = 0.0,
       krz: float = 0.0,
       behavior: SpringBehavior = SpringBehavior.Linear,
       behavior_per_dof: Optional[Dict[int, SpringBehavior]] = None,
   ) -> SpringElement:
       """
       Add a spring element between two nodes.

       Args:
           node1: First node or [x, y, z] coordinates.
           node2: Second node or [x, y, z] coordinates.
           kx, ky, kz: Translational stiffness [kN/m].
           krx, kry, krz: Rotational stiffness [kN·m/rad].
           behavior: Default behavior for all DOFs.
           behavior_per_dof: Optional per-DOF behavior override.
               Keys are DOF indices (0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ).

       Returns:
           The created SpringElement.

       Example:
           # Tension-only vertical spring (bearing pad)
           spring = model.add_spring(
               cargo_node, deck_node,
               kz=10000.0,  # 10 MN/m vertical stiffness
               behavior=SpringBehavior.CompressionOnly
           )

           # Mixed behavior: compression in Z, linear in X/Y
           spring = model.add_spring(
               node1, node2,
               kx=1000.0, ky=1000.0, kz=5000.0,
               behavior=SpringBehavior.Linear,
               behavior_per_dof={2: SpringBehavior.CompressionOnly}
           )
       """
       # Implementation...
   ```

3. Add analysis methods:
   ```python
   def analyze_with_nonlinear_springs(
       self,
       settings: Optional[NonlinearSolverSettings] = None
   ) -> Dict[int, LoadCaseResult]:
       """
       Run analysis with support for nonlinear (tension/compression-only) springs.

       If no nonlinear springs exist, falls back to efficient linear analysis.

       Args:
           settings: Optional solver settings (max iterations, tolerances).

       Returns:
           Dictionary mapping load case ID to results.
       """

   def analyze_load_combination(
       self,
       combination: LoadCombination,
       settings: Optional[NonlinearSolverSettings] = None
   ) -> LoadCombinationResult:
       """
       Analyze a specific load combination with nonlinear spring support.

       Unlike linear analysis where combinations can use superposition,
       nonlinear springs require solving each combination directly.

       Args:
           combination: The load combination to analyze.
           settings: Optional solver settings.

       Returns:
           Results for this specific combination.
       """
   ```

**Acceptance Criteria:**
- [ ] SpringBehavior enum exported to Python
- [ ] NonlinearSolverSettings exposed with all fields
- [ ] NonlinearSolverResult exposed with spring_states
- [ ] SpringElement.behavior accessible per-DOF from Python
- [ ] StructuralModel.add_spring() accepts behavior parameter
- [ ] analyze_with_nonlinear_springs() method added
- [ ] analyze_load_combination() method added
- [ ] All new types have complete docstrings with units

---

## Task 15.5: Results Reporting for Nonlinear Springs

**Requirements:** R-RES-002 (extended)
**Dependencies:** Task 15.4
**Difficulty:** Low

**Description:**
Extend results reporting to include spring states and convergence info.

**Steps:**

1. Extend LoadCaseResult:
   ```cpp
   struct LoadCaseResult {
       // Existing...
       Eigen::VectorXd displacements;
       Eigen::VectorXd reactions;
       bool success;

       // New for nonlinear
       int iterations = 1;  // 1 for linear solve
       std::string solver_message;

       // Spring states: vector of (spring_id, active_states[6])
       std::vector<std::pair<int, std::array<bool, 6>>> spring_states;

       // Spring forces at convergence
       std::vector<std::pair<int, std::array<double, 6>>> spring_forces;
   };
   ```

2. Add spring force calculation:
   ```cpp
   std::array<double, 6> SpringElement::compute_forces() const {
       std::array<double, 6> forces;
       std::array<double, 6> k_values = {kx, ky, kz, krx, kry, krz};

       for (int i = 0; i < 6; ++i) {
           if (is_active[i]) {
               forces[i] = k_values[i] * deformation[i];
           } else {
               forces[i] = 0.0;  // No force when inactive
           }
       }
       return forces;
   }
   ```

3. Add Python methods for querying spring results:
   ```python
   def get_spring_state(
       self,
       spring_id: int,
       load_case: Optional[str] = None
   ) -> Dict[str, bool]:
       """Get active/inactive state for each DOF of a spring."""

   def get_spring_force(
       self,
       spring_id: int,
       load_case: Optional[str] = None
   ) -> Dict[str, float]:
       """Get spring forces for each DOF [kN or kN·m]."""

   def get_spring_summary(
       self,
       load_case: Optional[str] = None
   ) -> pd.DataFrame:
       """Get summary of all spring states and forces."""
   ```

**Acceptance Criteria:**
- [ ] LoadCaseResult includes iterations and solver_message
- [ ] Spring states stored in results
- [ ] Spring forces computed and stored
- [ ] Python API can query individual spring states
- [ ] Summary DataFrame shows all springs with states and forces
- [ ] Units documented (kN for force, kN·m for moment)

---

## Task 15.6: Convergence Enhancements

**Requirements:** New
**Dependencies:** Task 15.2
**Difficulty:** Medium

**Description:**
Implement advanced convergence strategies to handle difficult cases.

**Challenges:**

1. **Oscillation**: Spring alternates between active/inactive each iteration
2. **Multiple solutions**: Some configurations have multiple valid equilibria
3. **Near-zero deformation**: Numerical precision issues at boundaries

**Strategies:**

1. **State history tracking**:
   ```cpp
   // Detect cycles in state history
   if (state_seen_before(current_states, history)) {
       // Apply averaging or damping
       use_partial_stiffness = true;
   }
   ```

2. **Partial stiffness for oscillating springs**:
   ```cpp
   // Instead of 0 or k, use 0.5*k for oscillating springs
   double effective_k = oscillating ? 0.5 * k : (active ? k : 0);
   ```

3. **Gap tolerance with hysteresis**:
   ```cpp
   // Different thresholds for activation vs deactivation
   bool should_activate = deformation > gap_tolerance_;
   bool should_deactivate = deformation < -gap_tolerance_;
   // Maintain current state if in hysteresis band
   ```

4. **Line search for displacement updates**:
   ```cpp
   // Damped update: u_new = u + alpha * (u_trial - u)
   // Reduce alpha if states oscillate
   ```

**Acceptance Criteria:**
- [ ] Oscillation detection implemented
- [ ] Partial stiffness option for oscillating springs
- [ ] Hysteresis band prevents rapid state changes
- [ ] Line search damping available as option
- [ ] Clear warning messages for convergence issues
- [ ] Settings expose all convergence parameters

---

## Task 15.7: Validation Tests

**Requirements:** R-VAL-002 (extended)
**Dependencies:** Task 15.1-15.6
**Difficulty:** Medium

**Description:**
Create comprehensive tests for nonlinear spring behavior.

**Test Cases:**

1. **Simple tension-only spring**:
   ```python
   def test_tension_only_spring_active():
       """Tension-only spring with positive displacement should be active."""
       # Pull on spring → should behave like normal spring

   def test_tension_only_spring_inactive():
       """Tension-only spring with compression should be inactive."""
       # Push on spring → zero stiffness, free displacement
   ```

2. **Compression-only spring**:
   ```python
   def test_compression_only_bearing():
       """Bearing pad under gravity - compression only."""
       # Cargo sitting on deck - spring active

   def test_compression_only_liftoff():
       """Bearing pad with uplift load - liftoff."""
       # Upward load exceeds weight - spring inactive, cargo lifts
   ```

3. **Load reversal**:
   ```python
   def test_load_reversal_iteration():
       """Spring state changes during iteration."""
       # Initial guess wrong, must iterate to correct state
   ```

4. **Multiple springs with mixed states**:
   ```python
   def test_cargo_with_four_pads():
       """Cargo on four bearing pads, one lifts off under moment."""
       # Overturning moment → one pad inactive
   ```

5. **Load combination with nonlinear springs**:
   ```python
   def test_combination_differs_from_superposition():
       """Combined analysis gives different result than linear superposition."""
       # Demonstrate that direct combination solve is required
   ```

6. **Convergence tests**:
   ```python
   def test_convergence_reported():
       """Solver reports iteration count and convergence status."""

   def test_max_iterations_respected():
       """Solver stops after max iterations if not converged."""
   ```

**Acceptance Criteria:**
- [ ] Tension-only spring tests pass
- [ ] Compression-only spring tests pass
- [ ] Load reversal iteration test demonstrates state changes
- [ ] Multi-spring test shows partial liftoff
- [ ] Load combination test proves superposition invalidity
- [ ] Convergence reporting verified
- [ ] Edge cases (near-zero deformation) handled

---

## Task 15.8: Documentation and Examples

**Requirements:** R-LLM-001
**Dependencies:** Task 15.7
**Difficulty:** Low

**Description:**
Document nonlinear spring usage with practical examples.

**Documentation Sections:**

1. **User Guide: Nonlinear Springs**
   - When to use tension/compression-only springs
   - Setting up bearing pads (compression-only)
   - Setting up tie-downs/seafastening (tension-only)
   - Understanding iteration and convergence

2. **Technical Reference**
   - Algorithm description
   - Solver settings and their effects
   - Convergence criteria explained
   - Troubleshooting non-convergence

3. **Examples**:
   ```python
   # Example: Cargo on Bearing Pads
   """
   Model a cargo item resting on four bearing pads.
   Under overturning moment, one or more pads may lift off.
   """
   model = StructuralModel("Cargo Liftoff Example")

   # ... create structure ...

   # Add bearing pads as compression-only springs
   for corner in cargo_corners:
       model.add_spring(
           cargo_node, deck_node,
           kz=10000.0,  # 10 MN/m vertical stiffness
           behavior=SpringBehavior.CompressionOnly
       )

   # Analyze with nonlinear solver
   results = model.analyze_with_nonlinear_springs()

   # Check which pads lifted off
   for spring in model.springs:
       state = model.get_spring_state(spring.id)
       if not state['kz']:
           print(f"Spring {spring.id} lifted off!")
   ```

**Acceptance Criteria:**
- [ ] User guide section added to docs
- [ ] Technical reference documents algorithm
- [ ] At least 2 complete examples with code
- [ ] Troubleshooting section for common issues
- [ ] All docstrings complete with units

---

## Task 15.9: Performance Optimization

**Requirements:** New
**Dependencies:** Task 15.3
**Difficulty:** Medium

**Description:**
Optimize nonlinear solver for practical performance.

**Optimizations:**

1. **Sparse spring stiffness update**:
   ```cpp
   // Don't reassemble full matrix each iteration
   // Only update spring contributions
   K_total = base_K;  // Copy base
   add_spring_contributions(K_total, active_springs);
   ```

2. **Factorization reuse** (when possible):
   ```cpp
   // If only a few springs change state, use rank-1 updates
   // instead of full refactorization
   ```

3. **Parallel spring state update**:
   ```cpp
   #pragma omp parallel for
   for (auto* spring : springs) {
       spring->update_state(u, dof_handler);
   }
   ```

4. **Early termination for purely linear models**:
   ```cpp
   if (!has_nonlinear_springs()) {
       return linear_solve();  // Skip iteration overhead
   }
   ```

**Acceptance Criteria:**
- [ ] Sparse matrix updates avoid full reassembly
- [ ] Linear models have no performance penalty
- [ ] Iteration count logged for performance analysis
- [ ] Benchmark shows acceptable performance for 100+ springs

---

## Summary: Phase 15 Dependencies

```
Task 15.1 (State Tracking)
    ↓
Task 15.2 (Nonlinear Solver) ← Task 15.6 (Convergence Enhancements)
    ↓
Task 15.3 (Model Integration)
    ↓
Task 15.4 (Python API)
    ↓
Task 15.5 (Results Reporting)
    ↓
Task 15.7 (Validation Tests)
    ↓
Task 15.8 (Documentation)

Task 15.9 (Performance) - Can run in parallel with 15.7-15.8
```

## Estimated Effort

| Task | Difficulty | Estimated Effort |
|------|------------|------------------|
| 15.1 | Medium | 2-3 hours |
| 15.2 | High | 4-6 hours |
| 15.3 | Medium | 2-3 hours |
| 15.4 | Medium | 2-3 hours |
| 15.5 | Low | 1-2 hours |
| 15.6 | Medium | 2-3 hours |
| 15.7 | Medium | 2-3 hours |
| 15.8 | Low | 1-2 hours |
| 15.9 | Medium | 2-3 hours |

**Total: ~18-28 hours**

## Key Design Decisions

1. **Per-DOF behavior**: Springs can have different behavior per DOF (e.g., compression-only in Z, linear in X/Y)

2. **State-based iteration**: Uses spring state changes as convergence criterion rather than displacement residuals

3. **Load combinations require direct solve**: Cannot use superposition with nonlinear springs

4. **Gap tolerance**: Small tolerance prevents chattering near zero deformation

5. **Existing linear path preserved**: Models without nonlinear springs use efficient linear solver
