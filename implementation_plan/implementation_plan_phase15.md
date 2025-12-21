# Phase 15: Nonlinear Springs (Tension/Compression-Only and Gap Springs)

## Overview

This phase implements support for tension-only, compression-only, and gap spring elements, which requires an iterative nonlinear solver. This is a significant architectural addition because spring stiffness depends on the displacement state, creating a chicken-and-egg problem that requires iteration to resolve.

### Spring Types Covered

| Type | Behavior | Use Case |
|------|----------|----------|
| **Tension-Only** | Active only when elongated (δ > 0) | Cables, tie-downs, seafastening |
| **Compression-Only** | Active only when compressed (δ < 0) | Bearing pads, contact surfaces |
| **Tension Gap** | Active when elongation exceeds gap (δ > gap) | Hooks with slack, loose bolts |
| **Compression Gap** | Active when compression exceeds gap (δ < -gap) | Contact with clearance, bumpers |

### Force-Displacement Relationships

```
Tension-Only (gap=0):        Compression-Only (gap=0):

    F ↑      ╱                  F ↑
      │    ╱                      │
      │  ╱                        │
──────┼╱─────→ δ            ──────┼──────→ δ
      │                         ╱ │
      │                       ╱   │
                            ╱     ↓

Tension Gap:                 Compression Gap:

    F ↑        ╱                F ↑
      │      ╱                    │
      │    ╱                      │
──────┼──•╱──→ δ            ──────┼──────→ δ
      │  ↑gap                   ╲ │↑gap
      │                       ╲  •│
                            ╲    ↓
```

### Why This Is Nonlinear

In a linear system, the stiffness matrix K is constant:
```
K * u = F  →  u = K⁻¹ * F
```

With nonlinear springs, K depends on whether each spring is active:
```
K(u) * u = F + F_gap(u)  →  Requires iteration
```

The spring state (active/inactive) is unknown until we solve, but the solution depends on which springs are active.

### Gap Springs Add Additional Complexity

Gap springs introduce **gap closure forces** (F_gap). When a gap spring activates:
- The stiffness k is added to K (same as tension/compression-only)
- An additional force term accounts for the gap offset

**Force calculation for active gap spring:**
```
F = k × (δ - gap)      for tension gap (δ > gap)
F = k × (δ + gap)      for compression gap (δ < -gap)
```

This can be rewritten as:
```
F = k × δ - k × gap    (tension)
F = k × δ + k × gap    (compression)
```

The `±k×gap` term is the **gap closure force** that must be added to the right-hand side of the equation. This makes the system:
```
K(u) × u = F_external + F_gap(u)
```

Where F_gap contains contributions from all active gap springs.

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

### Static → Dynamic Load Sequencing

**Physical Reality**: In cargo/offshore operations, loads are applied in a specific sequence:

1. **Static phase**: Cargo is set down onto deck under gravity (Permanent loads)
   - Gaps close as cargo settles
   - Contact pattern is established
   - This is the "baseline" equilibrium state

2. **Dynamic phase**: Environmental/operational loads are applied (Variable, Environmental, Accidental)
   - Applied ON TOP of the settled static state
   - Springs may activate (new contact) or deactivate (liftoff) relative to static state
   - The static state is the starting point, not zero displacement

**Why This Matters**:

```
Example: Cargo on bearing pads with 5mm air gap

WRONG approach (solve combined from u=0):
┌─────────────────────────────────────────────────────────┐
│ Combined: Gravity + Roll Moment                         │
│ Starting from: u=0, all springs initially active        │
│ Problem: May find non-physical equilibrium or diverge   │
└─────────────────────────────────────────────────────────┘

CORRECT approach (static first, then add dynamic):
┌─────────────────────────────────────────────────────────┐
│ Step 1: Solve static (gravity only)                     │
│ → Gaps close, cargo settles, contact established        │
│ → u_static, spring_states_static                        │
├─────────────────────────────────────────────────────────┤
│ Step 2: Solve combined, starting FROM static state      │
│ → Initial guess: u_static, spring_states_static         │
│ → Add dynamic loads                                     │
│ → Iterate from this physical starting point             │
│ → Some pads may lift off (state changes)                │
└─────────────────────────────────────────────────────────┘
```

**Algorithm for Load Combination Analysis**:

```
1. Identify the "static base" combination:
   - All Permanent load cases with their factors
   - This represents the set-down/gravity state

2. Solve static base combination:
   - Start from u=0, all springs initially active
   - Iterate until converged
   - Store: u_static, spring_states_static

3. For each full load combination (static + dynamic):
   - Initialize from: u_static, spring_states_static
   - Add dynamic load contributions (Variable, Environmental, Accidental)
   - Iterate until converged
   - This may result in:
     - Springs that were active becoming inactive (liftoff)
     - Springs that were inactive becoming active (new contact)
     - But starting from physical baseline
```

**Implementation Requirement**: The NonlinearSolver must accept an optional initial state:
- `initial_displacement`: Starting displacement vector (from static solve)
- `initial_spring_states`: Starting spring active/inactive states

This ensures the dynamic analysis starts from the physically correct baseline.

---

## Task 15.1: Spring Element State Tracking

**Requirements:** R-ELEM-010 (extended)
**Dependencies:** Task 8.1 (Spring Element)
**Difficulty:** Medium

**Description:**
Extend SpringElement to support tension-only, compression-only, and gap spring behavior with state tracking.

**Steps:**

1. Add spring behavior enum to `spring_element.hpp`:
   ```cpp
   enum class SpringBehavior {
       Linear = 0,         // Always active (default, current behavior)
       TensionOnly = 1,    // Active only when elongated (δ > gap)
       CompressionOnly = 2 // Active only when compressed (δ < -gap)
   };
   ```

2. Add state tracking and gap values to SpringElement:
   ```cpp
   class SpringElement {
   public:
       // Existing members...
       int id;
       Node* node_i;
       Node* node_j;
       double kx, ky, kz, krx, kry, krz;
       LoadingCondition loading_condition;

       // === NEW: Nonlinear behavior ===

       // Per-DOF behavior (allows mixed, e.g., compression-only in Z, linear in X)
       std::array<SpringBehavior, 6> behavior = {
           SpringBehavior::Linear, SpringBehavior::Linear,
           SpringBehavior::Linear, SpringBehavior::Linear,
           SpringBehavior::Linear, SpringBehavior::Linear
       };

       // Per-DOF gap values [m for translation, rad for rotation]
       // Gap must be overcome before spring activates
       std::array<double, 6> gap = {0, 0, 0, 0, 0, 0};

       // State tracking (updated during iteration)
       std::array<bool, 6> is_active = {true, true, true, true, true, true};

       // Current deformation for each DOF: δ = u_j - u_i
       std::array<double, 6> deformation = {0, 0, 0, 0, 0, 0};

       // Methods
       void update_state(const Eigen::VectorXd& displacements,
                        const DOFHandler& dof_handler);
       bool state_changed() const;
       bool has_gap() const;  // Any DOF has non-zero gap?
       bool is_nonlinear() const;  // Any DOF is non-Linear behavior?

       Eigen::Matrix<double, 12, 12> current_stiffness_matrix() const;
       std::array<double, 6> compute_forces() const;
       Eigen::Matrix<double, 12, 1> compute_gap_forces(
           const DOFHandler& dof_handler) const;

   private:
       bool state_changed_ = false;
       double gap_tolerance_ = 1e-10;  // Prevent chattering
   };
   ```

3. Implement state update logic with gap support:
   ```cpp
   void SpringElement::update_state(const Eigen::VectorXd& u,
                                    const DOFHandler& dof_handler) {
       std::array<bool, 6> previous_state = is_active;

       // Get DOF indices for both nodes
       auto dofs_i = dof_handler.get_node_dofs(node_i->id);
       auto dofs_j = dof_handler.get_node_dofs(node_j->id);

       for (int i = 0; i < 6; ++i) {
           // Compute deformation: δ = u_j - u_i
           double u_i = (dofs_i[i] >= 0) ? u(dofs_i[i]) : 0.0;
           double u_j = (dofs_j[i] >= 0) ? u(dofs_j[i]) : 0.0;
           deformation[i] = u_j - u_i;

           double g = gap[i];
           double delta = deformation[i];

           // Update active state based on behavior and gap
           switch (behavior[i]) {
               case SpringBehavior::Linear:
                   // Linear with gap: active when |δ| > gap (rare but supported)
                   is_active[i] = (g <= gap_tolerance_) ||
                                  (std::abs(delta) > g - gap_tolerance_);
                   break;

               case SpringBehavior::TensionOnly:
                   // Active when elongation exceeds gap: δ > gap
                   is_active[i] = (delta > g - gap_tolerance_);
                   break;

               case SpringBehavior::CompressionOnly:
                   // Active when compression exceeds gap: δ < -gap
                   is_active[i] = (delta < -g + gap_tolerance_);
                   break;
           }
       }

       state_changed_ = (previous_state != is_active);
   }
   ```

4. Implement force calculation with gap offset:
   ```cpp
   std::array<double, 6> SpringElement::compute_forces() const {
       std::array<double, 6> forces = {0, 0, 0, 0, 0, 0};
       std::array<double, 6> k_values = {kx, ky, kz, krx, kry, krz};

       for (int i = 0; i < 6; ++i) {
           if (!is_active[i]) {
               forces[i] = 0.0;
               continue;
           }

           double k = k_values[i];
           double delta = deformation[i];
           double g = gap[i];

           switch (behavior[i]) {
               case SpringBehavior::Linear:
                   if (g <= gap_tolerance_) {
                       forces[i] = k * delta;  // No gap
                   } else if (delta > g) {
                       forces[i] = k * (delta - g);  // Tension beyond gap
                   } else if (delta < -g) {
                       forces[i] = k * (delta + g);  // Compression beyond gap
                   } else {
                       forces[i] = 0.0;  // Within gap
                   }
                   break;

               case SpringBehavior::TensionOnly:
                   // F = k × (δ - gap)
                   forces[i] = k * (delta - g);
                   break;

               case SpringBehavior::CompressionOnly:
                   // F = k × (δ + gap), where δ is negative
                   forces[i] = k * (delta + g);
                   break;
           }
       }
       return forces;
   }
   ```

5. Implement gap force vector for solver:
   ```cpp
   Eigen::Matrix<double, 12, 1> SpringElement::compute_gap_forces(
       const DOFHandler& dof_handler) const
   {
       // Returns the gap closure forces to add to F vector
       // F_gap = ±k×gap for active gap springs

       Eigen::Matrix<double, 12, 1> F_gap = Eigen::Matrix<double, 12, 1>::Zero();
       std::array<double, 6> k_values = {kx, ky, kz, krx, kry, krz};

       for (int i = 0; i < 6; ++i) {
           if (!is_active[i] || gap[i] <= gap_tolerance_) continue;

           double k = k_values[i];
           double g = gap[i];
           double delta = deformation[i];

           double f_gap = 0.0;
           switch (behavior[i]) {
               case SpringBehavior::TensionOnly:
                   // Gap force pulls nodes together: -k×gap at node_i, +k×gap at node_j
                   f_gap = -k * g;  // Force direction for tension gap
                   break;

               case SpringBehavior::CompressionOnly:
                   // Gap force pushes nodes apart: +k×gap at node_i, -k×gap at node_j
                   f_gap = k * g;  // Force direction for compression gap
                   break;

               case SpringBehavior::Linear:
                   // Bidirectional gap - determine based on current state
                   if (delta > g) {
                       f_gap = -k * g;  // Tension
                   } else if (delta < -g) {
                       f_gap = k * g;   // Compression
                   }
                   break;
           }

           // Apply to DOF indices
           F_gap(i) = -f_gap;      // Force on node_i
           F_gap(i + 6) = f_gap;   // Force on node_j (opposite)
       }

       return F_gap;
   }
   ```

6. Implement stiffness matrix (unchanged from no-gap version):
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
- [ ] Per-DOF gap values can be set independently
- [ ] State tracking correctly identifies when gap is closed
- [ ] current_stiffness_matrix() returns zero contribution for inactive DOFs
- [ ] compute_forces() correctly applies gap offset to force calculation
- [ ] compute_gap_forces() returns correct gap closure forces for solver
- [ ] state_changed() correctly detects state transitions
- [ ] Gap tolerance prevents chattering near threshold
- [ ] has_gap() and is_nonlinear() helper methods work correctly

---

## Task 15.2: Iterative Nonlinear Solver

**Requirements:** New
**Dependencies:** Task 15.1, Task 3.3 (LinearSolver)
**Difficulty:** High

**Description:**
Implement an iterative solver that handles spring state changes until convergence.

**Algorithm: Modified Newton-Raphson with State Update and Gap Forces**

```
1. Initialize: Assume all springs active, solve linear system
2. Loop:
   a. Update spring states based on current displacement
   b. If no state changed AND displacement converged → CONVERGED, exit
   c. Reassemble stiffness matrix K with active spring states
   d. Compute gap forces F_gap for active gap springs
   e. Solve: K * u_new = F_external + F_gap
   f. If iteration > max_iterations → FAILED, exit
   g. Go to step 2a
```

**Note on Gap Forces:**
Gap springs require an additional force term because the spring force is:
```
F_spring = k × (δ - gap)    not    F_spring = k × δ
```
The `-k×gap` term (gap closure force) must be added to the RHS of the equation.

---

### Complete Header File: `cpp/include/grillex/nonlinear_solver.hpp`

```cpp
#pragma once

#include "grillex/solver.hpp"
#include "grillex/spring_element.hpp"
#include "grillex/dof_handler.hpp"
#include <Eigen/Sparse>
#include <vector>
#include <map>
#include <string>
#include <functional>

namespace grillex {

// Forward declarations
class SpringElement;
class DOFHandler;

/**
 * @brief Result from nonlinear spring solver
 */
struct NonlinearSolverResult {
    /// Solution displacement vector [m, rad]
    Eigen::VectorXd displacements;

    /// True if solver converged
    bool converged = false;

    /// Number of iterations performed
    int iterations = 0;

    /// Descriptive message (convergence info or error)
    std::string message;

    /// Final spring states: (spring_id, active_state[6])
    std::vector<std::pair<int, std::array<bool, 6>>> spring_states;

    /// Final spring forces: (spring_id, forces[6]) [kN or kN·m]
    std::vector<std::pair<int, std::array<double, 6>>> spring_forces;

    /// History of state changes per iteration (for diagnostics)
    std::vector<int> state_changes_per_iteration;
};

/**
 * @brief Initial state for starting nonlinear iteration
 *
 * Used to start from a known state (e.g., static solution) rather than zero.
 * This is essential for static→dynamic load sequencing where the static
 * (gravity) solution establishes the baseline contact pattern.
 */
struct NonlinearInitialState {
    /// Initial displacement vector (empty = start from zero)
    Eigen::VectorXd displacement;

    /// Initial spring states: (spring_id, active_states[6])
    /// If empty, all springs start as active
    std::vector<std::pair<int, std::array<bool, 6>>> spring_states;

    /// Check if initial state is provided
    bool has_initial_state() const {
        return displacement.size() > 0;
    }
};

/**
 * @brief Settings for nonlinear spring solver
 */
struct NonlinearSolverSettings {
    /// Maximum iterations before giving up
    int max_iterations = 50;

    /// Tolerance for spring activation threshold [m]
    /// Prevents chattering when deformation is very close to gap
    double gap_tolerance = 1e-10;

    /// Relative displacement tolerance for convergence check
    /// Used as secondary check: ||u_new - u_old|| / ||u_new|| < tol
    double displacement_tolerance = 1e-8;

    /// Allow solution where all nonlinear springs are inactive?
    /// If false, solver warns when this happens
    bool allow_all_springs_inactive = false;

    /// Enable oscillation detection and damping
    bool enable_oscillation_damping = true;

    /// Number of iterations to look back for oscillation detection
    int oscillation_history_size = 4;

    /// Damping factor when oscillation detected (0 < alpha < 1)
    /// u_damped = alpha * u_new + (1 - alpha) * u_old
    double oscillation_damping_factor = 0.5;

    /// Linear solver method to use
    LinearSolver::Method linear_method = LinearSolver::Method::SimplicialLDLT;

    /// Optional callback for iteration progress monitoring
    /// Parameters: (iteration, num_state_changes, displacement_norm)
    std::function<void(int, int, double)> progress_callback = nullptr;
};

/**
 * @brief Iterative solver for systems with nonlinear springs
 *
 * Handles tension-only, compression-only, and gap springs through
 * an iterative state-update algorithm. Each iteration:
 * 1. Assembles stiffness from currently active springs
 * 2. Computes gap closure forces for active gap springs
 * 3. Solves the linear system
 * 4. Updates spring states based on new displacements
 * 5. Checks for convergence (no state changes)
 *
 * **Static→Dynamic Sequencing**:
 * For proper cargo analysis, solve static loads first, then use that
 * solution as the initial state for dynamic combinations:
 *
 *   // Step 1: Solve static (gravity) to get baseline
 *   auto static_result = solver.solve(base_K, F_static, springs, dof_handler);
 *
 *   // Step 2: Create initial state from static solution
 *   NonlinearInitialState init;
 *   init.displacement = static_result.displacements;
 *   init.spring_states = static_result.spring_states;
 *
 *   // Step 3: Solve combined (static + dynamic) starting from static state
 *   auto combined_result = solver.solve(base_K, F_combined, springs, dof_handler, init);
 */
class NonlinearSolver {
public:
    /**
     * @brief Construct solver with settings
     * @param settings Solver configuration
     */
    explicit NonlinearSolver(const NonlinearSolverSettings& settings = {});

    /**
     * @brief Solve system with nonlinear springs
     *
     * @param base_K Base stiffness matrix (beams, plates - excludes springs)
     * @param F External force vector [kN]
     * @param springs Vector of spring elements (states will be updated)
     * @param dof_handler DOF handler for global DOF indexing
     * @param initial_state Optional initial state from previous solve (e.g., static)
     * @return Solver result with displacements and convergence info
     *
     * The springs vector is modified: after solve(), each spring's
     * is_active and deformation arrays contain the final converged state.
     *
     * **Initial State for Static→Dynamic Sequencing**:
     * When analyzing dynamic load combinations, provide the static solution
     * as initial_state. This ensures:
     * - Iteration starts from physically correct baseline (cargo settled)
     * - Gap springs that closed under static load start as active
     * - Springs that were inactive stay inactive unless dynamic loads change them
     */
    NonlinearSolverResult solve(
        const Eigen::SparseMatrix<double>& base_K,
        const Eigen::VectorXd& F,
        std::vector<SpringElement*>& springs,
        const DOFHandler& dof_handler,
        const NonlinearInitialState& initial_state = {});

    /// Get current settings
    const NonlinearSolverSettings& settings() const { return settings_; }

    /// Update settings
    void set_settings(const NonlinearSolverSettings& s) { settings_ = s; }

private:
    NonlinearSolverSettings settings_;
    LinearSolver linear_solver_;

    /**
     * @brief Assemble spring stiffness matrix from active springs
     *
     * Only includes contributions from springs where is_active[dof] == true.
     * Uses sparse triplet assembly for efficiency.
     */
    Eigen::SparseMatrix<double> assemble_spring_stiffness(
        const std::vector<SpringElement*>& springs,
        const DOFHandler& dof_handler,
        int total_dofs) const;

    /**
     * @brief Assemble gap closure forces for active gap springs
     *
     * For each active spring with gap > 0, computes the force offset
     * that accounts for the gap in the force-displacement relationship.
     */
    Eigen::VectorXd assemble_gap_forces(
        const std::vector<SpringElement*>& springs,
        const DOFHandler& dof_handler,
        int total_dofs) const;

    /**
     * @brief Check for oscillation in state history
     *
     * Detects if spring states are cycling between the same patterns,
     * indicating the solver is not converging.
     *
     * @param history State snapshots from recent iterations
     * @return true if oscillation detected
     */
    bool detect_oscillation(
        const std::vector<std::vector<bool>>& history) const;

    /**
     * @brief Encode current spring states as a flat boolean vector
     */
    std::vector<bool> encode_spring_states(
        const std::vector<SpringElement*>& springs) const;
};

} // namespace grillex
```

---

### Complete Implementation: `cpp/src/nonlinear_solver.cpp`

```cpp
#include "grillex/nonlinear_solver.hpp"
#include <cmath>
#include <sstream>
#include <algorithm>
#include <set>

namespace grillex {

NonlinearSolver::NonlinearSolver(const NonlinearSolverSettings& settings)
    : settings_(settings)
    , linear_solver_(settings.linear_method)
{}

NonlinearSolverResult NonlinearSolver::solve(
    const Eigen::SparseMatrix<double>& base_K,
    const Eigen::VectorXd& F,
    std::vector<SpringElement*>& springs,
    const DOFHandler& dof_handler,
    const NonlinearInitialState& initial_state)
{
    NonlinearSolverResult result;
    const int n = base_K.rows();

    // =========================================================================
    // Step 1: Check if we actually need nonlinear iteration
    // =========================================================================
    bool has_nonlinear = false;
    bool has_gaps = false;

    for (const auto* spring : springs) {
        if (spring->is_nonlinear()) {
            has_nonlinear = true;
        }
        if (spring->has_gap()) {
            has_gaps = true;
        }
        if (has_nonlinear && has_gaps) break;
    }

    // Fast path: purely linear springs with no gaps
    if (!has_nonlinear && !has_gaps) {
        // All springs are always active, assemble once and solve
        for (auto* spring : springs) {
            spring->is_active.fill(true);
            spring->deformation.fill(0.0);
        }

        Eigen::SparseMatrix<double> K_springs =
            assemble_spring_stiffness(springs, dof_handler, n);
        Eigen::SparseMatrix<double> K_total = base_K + K_springs;

        result.displacements = linear_solver_.solve(K_total, F);
        result.converged = !linear_solver_.is_singular();
        result.iterations = 1;
        result.message = result.converged
            ? "Linear solve (no nonlinear springs)"
            : "Linear solve failed: " + linear_solver_.get_error_message();

        // Update spring deformations for reporting
        if (result.converged) {
            for (auto* spring : springs) {
                spring->update_state(result.displacements, dof_handler);
                result.spring_states.emplace_back(spring->id, spring->is_active);
                result.spring_forces.emplace_back(spring->id, spring->compute_forces());
            }
        }
        return result;
    }

    // =========================================================================
    // Step 2: Initialize for iterative solve
    // =========================================================================

    Eigen::VectorXd u;
    Eigen::VectorXd u_prev;

    if (initial_state.has_initial_state()) {
        // Starting from provided initial state (e.g., static solution)
        // This is essential for static→dynamic sequencing where the static
        // (gravity) solution establishes the baseline contact pattern

        u = initial_state.displacement;
        u_prev = u;

        // Apply initial spring states from previous solve
        if (!initial_state.spring_states.empty()) {
            // Create a map for quick lookup
            std::map<int, std::array<bool, 6>> state_map;
            for (const auto& [spring_id, states] : initial_state.spring_states) {
                state_map[spring_id] = states;
            }

            // Apply states to springs
            for (auto* spring : springs) {
                auto it = state_map.find(spring->id);
                if (it != state_map.end()) {
                    spring->is_active = it->second;
                } else {
                    // Spring not in initial state - start as active
                    spring->is_active.fill(true);
                }
                // Update deformations from displacement
                spring->update_state(u, dof_handler);
            }
        } else {
            // No spring states provided - update from initial displacement
            for (auto* spring : springs) {
                spring->update_state(u, dof_handler);
            }
        }
    } else {
        // No initial state - start from zero with all springs active
        // (optimistic initial guess)
        u = Eigen::VectorXd::Zero(n);
        u_prev = u;

        for (auto* spring : springs) {
            spring->is_active.fill(true);
            spring->deformation.fill(0.0);
        }
    }

    // State history for oscillation detection
    std::vector<std::vector<bool>> state_history;

    // =========================================================================
    // Step 3: Iteration loop
    // =========================================================================
    for (int iter = 0; iter < settings_.max_iterations; ++iter) {
        result.iterations = iter + 1;

        // ---------------------------------------------------------------------
        // 3a. Assemble stiffness matrix with current active springs
        // ---------------------------------------------------------------------
        Eigen::SparseMatrix<double> K_springs =
            assemble_spring_stiffness(springs, dof_handler, n);
        Eigen::SparseMatrix<double> K_total = base_K + K_springs;

        // ---------------------------------------------------------------------
        // 3b. Compute gap forces for active gap springs
        // ---------------------------------------------------------------------
        Eigen::VectorXd F_gap = assemble_gap_forces(springs, dof_handler, n);
        Eigen::VectorXd F_total = F + F_gap;

        // ---------------------------------------------------------------------
        // 3c. Solve linear system: K * u = F + F_gap
        // ---------------------------------------------------------------------
        Eigen::VectorXd u_new = linear_solver_.solve(K_total, F_total);

        if (linear_solver_.is_singular()) {
            result.converged = false;
            result.displacements = u;
            std::ostringstream oss;
            oss << "Singular system at iteration " << (iter + 1)
                << ": " << linear_solver_.get_error_message();
            result.message = oss.str();

            // Store current spring states for diagnostics
            for (auto* spring : springs) {
                result.spring_states.emplace_back(spring->id, spring->is_active);
            }
            return result;
        }

        // ---------------------------------------------------------------------
        // 3d. Update spring states based on new displacements
        // ---------------------------------------------------------------------
        int num_state_changes = 0;
        for (auto* spring : springs) {
            spring->update_state(u_new, dof_handler);
            if (spring->state_changed()) {
                // Count how many DOFs changed
                for (int d = 0; d < 6; ++d) {
                    // We'd need to track previous state per-DOF to count precisely
                    // For now, just count springs that changed
                }
                num_state_changes++;
            }
        }

        result.state_changes_per_iteration.push_back(num_state_changes);

        // Progress callback
        if (settings_.progress_callback) {
            double u_norm = u_new.norm();
            settings_.progress_callback(iter + 1, num_state_changes, u_norm);
        }

        // ---------------------------------------------------------------------
        // 3e. Check convergence: no state changes
        // ---------------------------------------------------------------------
        if (num_state_changes == 0) {
            // Also check displacement convergence as secondary criterion
            double du_norm = (u_new - u).norm();
            double u_norm = u_new.norm();
            bool displacement_converged =
                (u_norm < 1e-15) || (du_norm / u_norm < settings_.displacement_tolerance);

            if (displacement_converged) {
                result.displacements = u_new;
                result.converged = true;

                std::ostringstream oss;
                oss << "Converged after " << (iter + 1) << " iteration"
                    << ((iter + 1) > 1 ? "s" : "");
                result.message = oss.str();

                // Store final spring states and forces
                for (auto* spring : springs) {
                    result.spring_states.emplace_back(spring->id, spring->is_active);
                    result.spring_forces.emplace_back(spring->id, spring->compute_forces());
                }

                // Warning if all nonlinear springs are inactive
                if (!settings_.allow_all_springs_inactive) {
                    bool any_nonlinear_active = false;
                    for (const auto* spring : springs) {
                        if (!spring->is_nonlinear()) continue;
                        for (int d = 0; d < 6; ++d) {
                            if (spring->behavior[d] != SpringBehavior::Linear &&
                                spring->is_active[d]) {
                                any_nonlinear_active = true;
                                break;
                            }
                        }
                        if (any_nonlinear_active) break;
                    }
                    if (!any_nonlinear_active && has_nonlinear) {
                        result.message += " (WARNING: all nonlinear springs inactive)";
                    }
                }

                return result;
            }
        }

        // ---------------------------------------------------------------------
        // 3f. Oscillation detection and damping
        // ---------------------------------------------------------------------
        if (settings_.enable_oscillation_damping) {
            state_history.push_back(encode_spring_states(springs));

            // Keep history bounded
            if (static_cast<int>(state_history.size()) > settings_.oscillation_history_size) {
                state_history.erase(state_history.begin());
            }

            if (detect_oscillation(state_history)) {
                // Apply damping: blend with previous solution
                double alpha = settings_.oscillation_damping_factor;
                u_new = alpha * u_new + (1.0 - alpha) * u_prev;

                // Re-update spring states with damped displacement
                for (auto* spring : springs) {
                    spring->update_state(u_new, dof_handler);
                }
            }
        }

        u_prev = u;
        u = u_new;
    }

    // =========================================================================
    // Step 4: Failed to converge
    // =========================================================================
    result.displacements = u;
    result.converged = false;

    std::ostringstream oss;
    oss << "Failed to converge after " << settings_.max_iterations << " iterations";

    // Analyze why: still oscillating?
    if (!result.state_changes_per_iteration.empty()) {
        int last_changes = result.state_changes_per_iteration.back();
        if (last_changes > 0) {
            oss << " (still " << last_changes << " spring state changes)";
        }
    }

    result.message = oss.str();

    // Store final states anyway for diagnostics
    for (auto* spring : springs) {
        result.spring_states.emplace_back(spring->id, spring->is_active);
        result.spring_forces.emplace_back(spring->id, spring->compute_forces());
    }

    return result;
}

Eigen::SparseMatrix<double> NonlinearSolver::assemble_spring_stiffness(
    const std::vector<SpringElement*>& springs,
    const DOFHandler& dof_handler,
    int total_dofs) const
{
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(springs.size() * 24);  // Up to 12*2 entries per spring

    for (const auto* spring : springs) {
        // Get element stiffness (respects is_active flags)
        Eigen::Matrix<double, 12, 12> K_elem = spring->current_stiffness_matrix();

        // Build location array
        std::array<int, 12> loc;
        for (int d = 0; d < 6; ++d) {
            loc[d] = dof_handler.get_global_dof(spring->node_i->id, d);
            loc[d + 6] = dof_handler.get_global_dof(spring->node_j->id, d);
        }

        // Scatter to global
        for (int i = 0; i < 12; ++i) {
            if (loc[i] < 0) continue;
            for (int j = 0; j < 12; ++j) {
                if (loc[j] < 0) continue;
                double val = K_elem(i, j);
                if (std::abs(val) > 1e-20) {
                    triplets.emplace_back(loc[i], loc[j], val);
                }
            }
        }
    }

    Eigen::SparseMatrix<double> K(total_dofs, total_dofs);
    K.setFromTriplets(triplets.begin(), triplets.end());
    return K;
}

Eigen::VectorXd NonlinearSolver::assemble_gap_forces(
    const std::vector<SpringElement*>& springs,
    const DOFHandler& dof_handler,
    int total_dofs) const
{
    Eigen::VectorXd F_gap = Eigen::VectorXd::Zero(total_dofs);

    for (const auto* spring : springs) {
        if (!spring->has_gap()) continue;

        // Get element gap forces
        Eigen::Matrix<double, 12, 1> f_elem = spring->compute_gap_forces(dof_handler);

        // Build location array
        std::array<int, 12> loc;
        for (int d = 0; d < 6; ++d) {
            loc[d] = dof_handler.get_global_dof(spring->node_i->id, d);
            loc[d + 6] = dof_handler.get_global_dof(spring->node_j->id, d);
        }

        // Scatter to global
        for (int i = 0; i < 12; ++i) {
            if (loc[i] >= 0) {
                F_gap(loc[i]) += f_elem(i);
            }
        }
    }

    return F_gap;
}

std::vector<bool> NonlinearSolver::encode_spring_states(
    const std::vector<SpringElement*>& springs) const
{
    std::vector<bool> state;
    state.reserve(springs.size() * 6);
    for (const auto* spring : springs) {
        for (int d = 0; d < 6; ++d) {
            state.push_back(spring->is_active[d]);
        }
    }
    return state;
}

bool NonlinearSolver::detect_oscillation(
    const std::vector<std::vector<bool>>& history) const
{
    if (history.size() < 3) return false;

    // Check if current state matches any previous state
    const auto& current = history.back();

    for (size_t i = 0; i < history.size() - 1; ++i) {
        if (history[i] == current) {
            // We've seen this state before - oscillation!
            return true;
        }
    }

    // Also check for 2-cycle: A -> B -> A -> B
    if (history.size() >= 4) {
        size_t n = history.size();
        if (history[n-1] == history[n-3] && history[n-2] == history[n-4]) {
            return true;
        }
    }

    return false;
}

} // namespace grillex
```

---

### Key Implementation Details

**1. Two-Phase Approach:**
- First check if iteration is needed (fast path for linear springs)
- Only enter iteration loop if nonlinear behavior or gaps exist

**2. Gap Force Computation:**
The gap forces are computed by `SpringElement::compute_gap_forces()` which returns:
```cpp
// For tension gap spring (active when δ > gap):
//   F = k × (δ - gap) = k×δ - k×gap
//   The k×δ part is in the stiffness matrix
//   The -k×gap part is the gap force (added to RHS)
f_gap = -k * gap;  // Applied as: +f on node_i, -f on node_j

// For compression gap spring (active when δ < -gap):
//   F = k × (δ + gap) = k×δ + k×gap
//   The k×δ part is in the stiffness matrix
//   The +k×gap part is the gap force
f_gap = +k * gap;  // Applied as: -f on node_i, +f on node_j
```

**3. Convergence Criteria:**
- Primary: No spring state changes between iterations
- Secondary: Displacement norm converged (for gap springs where force changes even without state changes)

**4. Oscillation Detection:**
- Track state history over last N iterations
- Detect if same state pattern repeats (indicates cycling)
- Apply damping: `u = α × u_new + (1-α) × u_old`

**5. Diagnostics:**
- `state_changes_per_iteration`: Track convergence progress
- `spring_states` and `spring_forces`: Final state for post-processing
- `progress_callback`: Optional real-time monitoring

**Acceptance Criteria:**
- [ ] NonlinearSolver class implemented with settings struct
- [ ] NonlinearInitialState struct implemented for static→dynamic sequencing
- [ ] solve() accepts optional initial_state parameter
- [ ] When initial_state provided, iteration starts from that displacement/state
- [ ] When initial_state not provided, starts from zero with all springs active
- [ ] Iterative solve correctly updates spring states
- [ ] Gap forces computed and added to RHS for gap springs
- [ ] Convergence detected when no spring states change
- [ ] Maximum iteration limit prevents infinite loops
- [ ] Linear-only springs without gaps bypass iteration (optimization)
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

3. Implement `analyze_combination()` with static→dynamic sequencing:
   ```cpp
   LoadCombinationResult Model::analyze_combination(
       const LoadCombination& combo,
       const NonlinearSolverSettings& settings)
   {
       // Cannot use superposition with nonlinear springs
       // Must solve the combined load directly
       //
       // IMPORTANT: Static→Dynamic Sequencing
       // For proper physical behavior, we:
       // 1. First solve the "static base" (Permanent loads only)
       // 2. Then solve the full combination starting from static state
       // This ensures gaps close under gravity before dynamic loads are applied

       auto base_K = assemble_base_stiffness();
       NonlinearSolver solver(settings);

       // =====================================================================
       // Step 1: Identify and solve static base (Permanent loads only)
       // =====================================================================
       Eigen::VectorXd F_static = Eigen::VectorXd::Zero(dof_handler_.total_dofs());
       bool has_permanent = false;

       for (const auto& term : combo.get_terms()) {
           if (term.load_case->type() == LoadCaseType::Permanent) {
               auto F_case = assemble_force_vector(*term.load_case);
               F_case = apply_accelerations(F_case, *term.load_case);
               F_static += term.factor * F_case;
               has_permanent = true;
           }
       }

       NonlinearInitialState initial_state;

       if (has_permanent && has_nonlinear_springs()) {
           // Solve static base to establish baseline contact pattern
           auto static_result = solver.solve(base_K, F_static, springs_, dof_handler_);

           if (!static_result.converged) {
               // Static solve failed - return error
               LoadCombinationResult combo_result;
               combo_result.converged = false;
               combo_result.message = "Static base solve failed: " + static_result.message;
               return combo_result;
           }

           // Use static solution as starting point for full combination
           initial_state.displacement = static_result.displacements;
           initial_state.spring_states = static_result.spring_states;
       }

       // =====================================================================
       // Step 2: Solve full combination starting from static state
       // =====================================================================
       Eigen::VectorXd F_combined = Eigen::VectorXd::Zero(dof_handler_.total_dofs());

       // Combine ALL force vectors (including Permanent again, for full load)
       for (const auto& term : combo.get_terms()) {
           auto F_case = assemble_force_vector(*term.load_case);
           F_case = apply_accelerations(F_case, *term.load_case);
           F_combined += term.factor * F_case;
       }

       auto result = solver.solve(base_K, F_combined, springs_, dof_handler_, initial_state);

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
- [ ] analyze_combination() implements static→dynamic sequencing:
  - Permanent loads solved first to establish baseline contact pattern
  - Full combination solved starting from static state (initial_state)
  - Static solve failure returns meaningful error message
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
       .def_readwrite("gap", &grillex::SpringElement::gap)
       .def_readonly("is_active", &grillex::SpringElement::is_active)
       .def_readonly("deformation", &grillex::SpringElement::deformation)
       .def("set_behavior", [](SpringElement& s, int dof, SpringBehavior b) {
           s.behavior[dof] = b;
       }, py::arg("dof"), py::arg("behavior"))
       .def("set_all_behavior", [](SpringElement& s, SpringBehavior b) {
           s.behavior.fill(b);
       }, py::arg("behavior"))
       .def("set_gap", [](SpringElement& s, int dof, double g) {
           s.gap[dof] = g;
       }, py::arg("dof"), py::arg("gap"))
       .def("set_all_gaps", [](SpringElement& s, double g) {
           s.gap.fill(g);
       }, py::arg("gap"))
       .def("has_gap", &grillex::SpringElement::has_gap)
       .def("is_nonlinear", &grillex::SpringElement::is_nonlinear)
       .def("compute_forces", &grillex::SpringElement::compute_forces);
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
       gap: float = 0.0,
       gap_per_dof: Optional[Dict[int, float]] = None,
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
           gap: Default gap for all DOFs [m for translation, rad for rotation].
               Spring only activates after deformation exceeds this gap.
           gap_per_dof: Optional per-DOF gap override.
               Keys are DOF indices (0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ).

       Returns:
           The created SpringElement.

       Example:
           # Compression-only vertical spring (bearing pad)
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

           # Compression gap spring (contact with 10mm clearance)
           spring = model.add_spring(
               node1, node2,
               kz=50000.0,  # High contact stiffness
               behavior=SpringBehavior.CompressionOnly,
               gap=0.010  # 10mm gap before contact
           )

           # Tension gap spring (hook with 5mm slack)
           spring = model.add_spring(
               cargo_node, deck_node,
               kz=20000.0,
               behavior=SpringBehavior.TensionOnly,
               gap=0.005  # 5mm slack before spring engages
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
- [ ] SpringElement.gap accessible per-DOF from Python
- [ ] StructuralModel.add_spring() accepts behavior and gap parameters
- [ ] set_gap() and set_all_gaps() methods work correctly
- [ ] has_gap() and is_nonlinear() methods exposed
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

7. **Gap spring tests**:
   ```python
   def test_compression_gap_open():
       """Compression gap spring with gap not closed remains inactive."""
       # Small compression < gap → no spring force

   def test_compression_gap_closed():
       """Compression gap spring activates when gap closes."""
       # Compression > gap → spring active with offset force

   def test_compression_gap_force():
       """Verify spring force uses gap offset: F = k × (δ + gap)."""
       # Force should be proportional to excess compression beyond gap

   def test_tension_gap_slack():
       """Tension gap spring with slack remains inactive."""
       # Elongation < gap → spring loose, no force

   def test_tension_gap_engaged():
       """Tension gap spring engages when slack taken up."""
       # Elongation > gap → spring pulls with offset force

   def test_gap_spring_iteration():
       """Gap spring state changes during iteration as gap closes."""
       # Initial guess may not close gap, iteration should converge

   def test_contact_with_clearance():
       """Practical case: cargo landing on deck with 5mm air gap."""
       # Cargo drops under gravity, gap closes, contact force develops

   def test_hook_with_slack():
       """Practical case: lifting with slack in cables."""
       # Upward acceleration takes up slack, then transmits force
   ```

8. **Analytical verification**:
   ```python
   def test_gap_spring_analytical():
       """Verify gap spring matches analytical solution."""
       # Simple system: spring with gap under known load
       # Analytical: if F > k*gap, then δ = F/k, else δ = F_free
       # Compare computed vs analytical displacement
   ```

9. **Static→Dynamic sequencing tests**:
   ```python
   def test_static_dynamic_sequencing():
       """Verify static loads establish baseline before dynamic loads applied."""
       # Cargo on gap springs with initial clearance
       # Step 1: Static (gravity) should close gaps, establish contact
       # Step 2: Dynamic (roll moment) applied FROM static state
       # Verify contact pattern differs from solving combined from u=0

   def test_static_baseline_used_for_combination():
       """Verify combination solver uses static solution as initial state."""
       # Create combination with Permanent + Variable load cases
       # Permanent load should be solved first
       # Combined result should start from Permanent solution

   def test_liftoff_from_static_contact():
       """Test spring liftoff when dynamic load reverses static contact."""
       # Cargo settled under gravity (all pads in contact)
       # Apply roll moment that lifts one side
       # Verify correct pads lift off starting from settled state

   def test_no_permanent_loads():
       """Test behavior when combination has no Permanent loads."""
       # Only Variable/Environmental loads in combination
       # Should solve from u=0 (no static baseline)

   def test_initial_state_carries_spring_states():
       """Verify spring states from static solve are preserved."""
       # Static solve with some springs inactive
       # Dynamic solve should start with those states
       # Compare iterations vs solving from all-active initial guess
   ```

**Acceptance Criteria:**
- [ ] Tension-only spring tests pass
- [ ] Compression-only spring tests pass
- [ ] Load reversal iteration test demonstrates state changes
- [ ] Multi-spring test shows partial liftoff
- [ ] Load combination test proves superposition invalidity
- [ ] Gap spring open/closed state tests pass
- [ ] Gap spring force offset verified (F = k × (δ - gap))
- [ ] Gap closure iteration test passes
- [ ] Contact with clearance practical test passes
- [ ] Hook with slack practical test passes
- [ ] Analytical verification test passes
- [ ] Static→dynamic sequencing test verifies Permanent loads solved first
- [ ] Liftoff from static contact test passes
- [ ] Initial state preserves spring states from static solve
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
   - Understanding gap springs (contact with clearance)
   - Understanding iteration and convergence

2. **Technical Reference**
   - Algorithm description (including gap forces)
   - Force-displacement relationships for each spring type
   - Solver settings and their effects
   - Convergence criteria explained
   - Troubleshooting non-convergence

3. **Examples**:
   ```python
   # Example 1: Cargo on Bearing Pads
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

   ```python
   # Example 2: Contact with Clearance (Gap Spring)
   """
   Model cargo lowering onto deck with initial air gap.
   The 10mm gap must close before contact forces develop.
   """
   model = StructuralModel("Gap Contact Example")

   # ... create structure ...

   # Add gap springs representing contact with 10mm clearance
   for pad_location in pad_locations:
       model.add_spring(
           cargo_node, deck_node,
           kz=50000.0,  # High contact stiffness [kN/m]
           behavior=SpringBehavior.CompressionOnly,
           gap=0.010  # 10mm air gap [m]
       )

   # Analyze - solver will iterate to find which gaps close
   results = model.analyze_with_nonlinear_springs()

   # Check gap closure and contact forces
   for spring in model.springs:
       forces = model.get_spring_force(spring.id)
       if forces['kz'] != 0:
           print(f"Spring {spring.id}: contact force = {forces['kz']:.1f} kN")
       else:
           print(f"Spring {spring.id}: gap still open")
   ```

   ```python
   # Example 3: Lifting with Slack Cables
   """
   Cargo suspended by cables with initial slack.
   Under upward acceleration, slack is taken up before load transfer.
   """
   model = StructuralModel("Slack Cable Example")

   # ... create cargo and crane structure ...

   # Add tension gap springs representing cables with slack
   for cable_point in cable_attachment_points:
       model.add_spring(
           cargo_node, crane_node,
           kz=100000.0,  # Cable axial stiffness [kN/m]
           behavior=SpringBehavior.TensionOnly,
           gap=0.020  # 20mm cable slack [m]
       )

   # Add upward acceleration (crane lifting)
   load_case.add_acceleration([0, 0, 1.5 * 9.81])  # 1.5g upward

   # Analyze
   results = model.analyze_with_nonlinear_springs()

   # Check which cables are taut
   for spring in model.springs:
       state = model.get_spring_state(spring.id)
       if state['kz']:
           print(f"Cable {spring.id} is taut")
       else:
           print(f"Cable {spring.id} still has slack")
   ```

**Acceptance Criteria:**
- [ ] User guide section added to docs (including gap springs)
- [ ] Technical reference documents algorithm and gap forces
- [ ] At least 3 complete examples with code (bearing pads, gap contact, slack cables)
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

1. **Per-DOF behavior and gap**: Springs can have different behavior AND gap per DOF (e.g., compression-only with 10mm gap in Z, linear in X/Y)

2. **Gap forces on RHS**: Gap springs contribute additional "gap closure forces" to the load vector, not just stiffness changes

3. **State-based iteration**: Uses spring state changes as convergence criterion rather than displacement residuals

4. **Load combinations require direct solve**: Cannot use superposition with nonlinear springs

5. **Static→Dynamic load sequencing**: For load combinations:
   - Permanent (gravity) loads solved first to establish baseline contact pattern
   - Full combination solved starting from static state via `initial_state` parameter
   - This ensures physically correct behavior where cargo settles before dynamic loads are applied
   - Essential for gap springs where gaps must close under gravity before dynamic analysis

6. **Gap tolerance**: Small tolerance prevents chattering near gap threshold

7. **Existing linear path preserved**: Models without nonlinear springs or gaps use efficient linear solver

8. **Force offset for gaps**: Spring force is proportional to excess deformation beyond gap: F = k × (δ - gap)

9. **Initial state support**: Solver accepts optional `NonlinearInitialState` with:
   - Initial displacement vector
   - Initial spring active/inactive states
   - Enables chaining of analysis phases (static → dynamic)
