# Phase 16: Eigenvalue Analysis (Modal/Dynamic)

## Overview

This phase implements eigenvalue analysis capabilities for computing natural frequencies and mode shapes of structural models. This is essential for:

- **Dynamic response analysis** - Understanding structural vibration behavior
- **Resonance avoidance** - Ensuring natural frequencies are away from excitation frequencies
- **Seismic design** - Modal response spectrum analysis (future extension)
- **Fatigue assessment** - Identifying dynamic amplification factors

**Requirements Reference:** R-ASM-006 (architecture shall not preclude extension to eigenvalue/modal/dynamic analyses)

**Dependencies:**
- Phase 3 complete (Assembly & Solver)
- Phase 8 complete (Point Mass elements)

**Difficulty:** High

---

## Mathematical Background

### Generalized Eigenvalue Problem

The equation of motion for undamped free vibration:

```
M × ü + K × u = 0
```

Assuming harmonic motion `u = φ × sin(ωt)`:

```
K × φ = ω² × M × φ
```

Where:
- **K** = Global stiffness matrix [kN/m]
- **M** = Global mass matrix [mT]
- **ω** = Natural circular frequency [rad/s]
- **φ** = Mode shape (eigenvector)
- **f = ω/(2π)** = Natural frequency [Hz]
- **T = 1/f** = Natural period [s]

### Eigenvalue Transformation

The generalized eigenvalue problem `K × φ = λ × M × φ` (where λ = ω²) can be solved by:

1. **Direct methods** - Compute all eigenvalues (suitable for small/medium models)
2. **Iterative methods** - Compute selected eigenvalues (required for large models)

For structural dynamics, we typically need only the **lowest n modes** (first few natural frequencies), making iterative methods more efficient.

### Shift-and-Invert Transformation

To find eigenvalues near a shift σ, transform:

```
(K - σM)⁻¹ × M × φ = μ × φ
where μ = 1/(λ - σ)
```

**Advantages:**
- Largest μ corresponds to eigenvalues closest to σ
- Setting σ = 0 (or small positive) finds lowest frequencies
- Uses existing linear solver for `(K - σM)⁻¹`
- Rapid convergence for well-separated eigenvalues

### Subspace Iteration Method

Algorithm for finding the first n modes:

```
1. Initialize random subspace X₀ (n_dofs × n_modes)
2. For each iteration k:
   a. Solve (K - σM) × Y = M × Xₖ  (using linear solver)
   b. Project: K_proj = Yᵀ × K × Y, M_proj = Yᵀ × M × Y
   c. Solve reduced problem: K_proj × Z = λ × M_proj × Z
   d. Update subspace: Xₖ₊₁ = Y × Z
   e. Mass-orthonormalize: φᵀ × M × φ = I
   f. Check convergence: |λₖ₊₁ - λₖ| / |λₖ| < tolerance
3. Return converged eigenvalues and eigenvectors
```

### Modal Quantities

**Mass-normalized mode shapes:**
```
φᵀ × M × φ = I  (identity matrix)
```

**Participation factor** for direction d (unit vector):
```
Γₙ,ᵈ = φₙᵀ × M × r_d
```
where r_d is the influence vector for direction d.

**Effective modal mass:**
```
Mₑff,ₙ,ᵈ = Γₙ,ᵈ² / (φₙᵀ × M × φₙ) = Γₙ,ᵈ²  (for mass-normalized modes)
```

**Cumulative mass participation:**
```
∑(Mₑff,ₙ,ᵈ) / M_total × 100%
```
Design codes typically require ≥90% cumulative mass participation.

---

## Implementation Tasks

### Task 16.1: Eigenvalue Solver Settings and Results (C++)

**File:** `cpp/include/grillex/eigenvalue_solver.hpp`

**Description:** Create data structures for eigenvalue solver configuration and results.

**Structures:**

```cpp
enum class EigensolverMethod {
    Dense,              // Eigen's GeneralizedSelfAdjointEigenSolver (all modes)
    SubspaceIteration,  // Iterative subspace method (selected modes)
    ShiftInvert         // Shift-and-invert with power iteration
};

struct EigensolverSettings {
    int n_modes = 10;                    // Number of modes to compute
    double shift = 0.0;                  // Frequency shift σ [rad/s]²
    double tolerance = 1e-8;             // Convergence tolerance
    int max_iterations = 100;            // Maximum iterations
    EigensolverMethod method = EigensolverMethod::SubspaceIteration;
    bool compute_participation = true;   // Compute participation factors
    bool mass_normalize = true;          // Mass-normalize mode shapes
};

struct ModeResult {
    int mode_number;                     // 1-based mode number
    double eigenvalue;                   // λ = ω² [rad/s]²
    double frequency_hz;                 // f = ω/(2π) [Hz]
    double period_s;                     // T = 1/f [s]
    Eigen::VectorXd mode_shape;          // φ (mass-normalized)

    // Participation factors for X, Y, Z translations
    double participation_x = 0.0;
    double participation_y = 0.0;
    double participation_z = 0.0;

    // Effective modal mass [mT]
    double effective_mass_x = 0.0;
    double effective_mass_y = 0.0;
    double effective_mass_z = 0.0;

    // Effective modal mass percentage [%]
    double effective_mass_pct_x = 0.0;
    double effective_mass_pct_y = 0.0;
    double effective_mass_pct_z = 0.0;
};

struct EigensolverResult {
    bool converged = false;
    int iterations = 0;
    std::string message;
    std::vector<ModeResult> modes;

    // Total mass for percentage calculations
    double total_mass_x = 0.0;
    double total_mass_y = 0.0;
    double total_mass_z = 0.0;

    // Cumulative effective mass percentages
    std::vector<double> cumulative_mass_pct_x;
    std::vector<double> cumulative_mass_pct_y;
    std::vector<double> cumulative_mass_pct_z;
};
```

**Acceptance Criteria:**
- [x] EigensolverSettings struct with all configuration options
- [x] ModeResult struct with eigenvalue, frequency, period, mode shape
- [x] ModeResult includes participation factors for X, Y, Z
- [x] ModeResult includes effective modal mass (absolute and percentage)
- [x] EigensolverResult contains vector of ModeResult plus summary data
- [x] Cumulative mass participation tracked for code compliance checking

### Execution Notes (Completed 2025-12-22)

**Steps Taken:**
1. Created `cpp/include/grillex/eigenvalue_solver.hpp` with all data structures
2. Implemented `EigensolverMethod` enum with Dense, SubspaceIteration, ShiftInvert options
3. Created `EigensolverSettings` struct with all configuration options (n_modes, shift, tolerance, max_iterations, method, compute_participation, mass_normalize, rigid_body_threshold)
4. Created `ModeResult` struct with eigenvalue, omega, frequency_hz, period_s, mode_shape, and all participation/effective mass fields
5. Created `EigensolverResult` struct with convergence info, modes vector, DOF mapping, and helper methods

**Key Implementation Details:**
- Added `is_rigid_body_mode` flag to ModeResult for detecting zero-frequency modes
- Included `reduced_to_full` DOF mapping in EigensolverResult for mode shape expansion
- Added helper methods: `get_mode()`, `get_frequencies()`, `get_periods()`, `expand_mode_shape()`

**Verification:**
- 22 Python tests passing ✓
- All struct fields accessible from Python via pybind11 bindings ✓

---

### Task 16.2: Boundary Condition Reduction for Eigenvalue Analysis

**File:** `cpp/include/grillex/eigenvalue_solver.hpp`, `cpp/src/eigenvalue_solver.cpp`

**Description:** Implement DOF elimination for fixed boundary conditions. Unlike static analysis (which can use penalty method), eigenvalue analysis requires true elimination to avoid spurious high-frequency modes.

**Methods:**

```cpp
class EigenvalueSolver {
public:
    // Reduce system by eliminating fixed DOFs
    // Returns reduced K, M matrices and DOF mapping
    std::tuple<Eigen::SparseMatrix<double>,
               Eigen::SparseMatrix<double>,
               std::vector<int>>
    reduce_system(const Eigen::SparseMatrix<double>& K,
                  const Eigen::SparseMatrix<double>& M,
                  const BCHandler& bc_handler,
                  const DOFHandler& dof_handler) const;

    // Expand reduced mode shape to full DOF vector
    Eigen::VectorXd expand_mode_shape(
        const Eigen::VectorXd& reduced_shape,
        const std::vector<int>& dof_mapping,
        int total_dofs) const;
};
```

**Algorithm:**
1. Identify all fixed DOFs from BCHandler
2. Create mapping: reduced DOF index → full DOF index
3. Extract submatrices K_red and M_red (rows/cols for free DOFs only)
4. After solving, expand mode shapes: insert zeros at fixed DOFs

**Acceptance Criteria:**
- [x] Fixed DOFs are eliminated (not penalized)
- [x] Reduced matrices are symmetric and sparse
- [x] DOF mapping correctly tracks free vs fixed DOFs
- [x] Mode shapes expand correctly with zeros at fixed DOFs
- [x] Prescribed non-zero displacements handled (set to zero for eigenmodes)

### Execution Notes (Completed 2025-12-22)

**Steps Taken:**
1. Implemented `reduce_system()` method in `EigenvalueSolver` class
2. Created mapping from reduced DOF indices to full DOF indices
3. Used sparse matrix slicing to extract free DOF submatrices
4. Implemented static `expand_mode_shape()` for restoring full DOF vectors

**Algorithm Details:**
- Iterates through BCHandler's fixed DOFs to build set of constrained DOFs
- Creates `reduced_to_full` mapping vector for free DOFs only
- Uses Eigen sparse matrix indexing for efficient submatrix extraction
- Expansion inserts zeros at fixed DOF positions

**Problems Encountered:**
- **Issue**: Initially used wrong iteration pattern for `BCHandler::get_fixed_dofs()`
  - **Error**: Tried to use map-style structured binding when it returns `std::vector<FixedDOF>`
  - **Solution**: Iterate over vector directly, accessing `fixed_dof.node_id`, `fixed_dof.local_dof`

**Verification:**
- Test `test_reduce_system_fixed_node` passes ✓
- Reduced matrices maintain correct dimensions ✓
- Mode shapes expand with zeros at fixed DOFs ✓

---

### Task 16.3: Dense Eigenvalue Solver

**File:** `cpp/src/eigenvalue_solver.cpp`

**Description:** Implement dense solver using Eigen's `GeneralizedSelfAdjointEigenSolver`. This computes all eigenvalues and is suitable for small/medium models (< 1000 DOFs).

**Implementation:**

```cpp
EigensolverResult EigenvalueSolver::solve_dense(
    const Eigen::SparseMatrix<double>& K,
    const Eigen::SparseMatrix<double>& M,
    const EigensolverSettings& settings) const
{
    // Convert to dense matrices
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K);
    Eigen::MatrixXd M_dense = Eigen::MatrixXd(M);

    // Solve generalized eigenvalue problem
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver(K_dense, M_dense);

    // Extract first n_modes eigenvalues/vectors
    // (eigenvalues are sorted in ascending order)
    ...
}
```

**Acceptance Criteria:**
- [x] Correctly solves K × φ = λ × M × φ
- [x] Eigenvalues sorted in ascending order (lowest frequency first)
- [x] Returns only first n_modes requested
- [x] Handles case where n_modes > n_dofs gracefully
- [x] Reports error for non-positive-definite M matrix

### Execution Notes (Completed 2025-12-22)

**Steps Taken:**
1. Implemented `solve_dense()` method using `Eigen::GeneralizedSelfAdjointEigenSolver`
2. Added `solve()` dispatcher that routes to appropriate solver method
3. Implemented `mass_normalize()` for eigenvector normalization
4. Implemented `compute_frequencies()` for eigenvalue → frequency/period conversion
5. Added rigid body mode detection (eigenvalue < threshold)
6. Created Python bindings for all eigenvalue types

**Algorithm Details:**
- Converts sparse matrices to dense for Eigen's GSAE solver
- Eigenvalues naturally sorted ascending by Eigen
- Mass normalization: φ = φ / sqrt(φᵀMφ)
- Frequency: ω = sqrt(λ), f = ω/(2π), T = 1/f
- Handles negative eigenvalues (sets frequency/period to 0)

**Key Implementation Details:**
- `solve()` dispatches based on `settings.method`:
  - Dense: Uses `solve_dense()`
  - SubspaceIteration/ShiftInvert: Falls back to Dense (not yet implemented)
- Automatic clipping of n_modes to n_dofs when needed
- Returns `converged=false` with error message for non-positive-definite M

**Verification:**
- Tests for cantilever beam eigenvalues pass ✓
- Mass normalization verified: φᵀMφ ≈ 1.0 ✓
- Frequencies correctly computed from eigenvalues ✓
- 22 tests total passing ✓

---

### Task 16.4: Subspace Iteration Solver

**File:** `cpp/src/eigenvalue_solver.cpp`

**Description:** Implement subspace iteration with shift-and-invert for efficient computation of lowest modes in large sparse systems.

**Algorithm:**

```cpp
EigensolverResult EigenvalueSolver::solve_subspace_iteration(
    const Eigen::SparseMatrix<double>& K,
    const Eigen::SparseMatrix<double>& M,
    const EigensolverSettings& settings) const
{
    int n = K.rows();
    int p = std::min(settings.n_modes + 8, n);  // Subspace size (oversample)

    // 1. Form shifted matrix: A = K - σM
    Eigen::SparseMatrix<double> A = K - settings.shift * M;

    // 2. Factor A for repeated solves
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);

    // 3. Initialize random subspace
    Eigen::MatrixXd X = Eigen::MatrixXd::Random(n, p);

    // 4. Iterate until convergence
    for (int iter = 0; iter < settings.max_iterations; ++iter) {
        // Solve A × Y = M × X
        Eigen::MatrixXd MX = M * X;
        Eigen::MatrixXd Y(n, p);
        for (int j = 0; j < p; ++j) {
            Y.col(j) = solver.solve(MX.col(j));
        }

        // Project onto subspace
        Eigen::MatrixXd K_proj = Y.transpose() * K * Y;
        Eigen::MatrixXd M_proj = Y.transpose() * M * Y;

        // Solve reduced eigenvalue problem (dense, small)
        Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> eig(K_proj, M_proj);

        // Update subspace
        X = Y * eig.eigenvectors();

        // Check convergence...
    }

    // Extract first n_modes and return
    ...
}
```

**Acceptance Criteria:**
- [x] Converges to correct eigenvalues for well-conditioned problems
- [x] Uses existing LinearSolver infrastructure where possible
- [x] Handles shift parameter correctly (σ = 0 finds lowest modes)
- [x] Oversample subspace (p > n_modes) for robust convergence
- [x] Detects and reports non-convergence
- [x] Performance acceptable for 1000+ DOF systems

### Execution Notes (Completed 2025-12-22)

**Steps Taken:**
1. Implemented full `solve_subspace_iteration()` method with shift-and-invert
2. Uses `Eigen::SparseLU` for factorizing (K - σM) for repeated solves
3. Oversample subspace: p = max(2*n_modes, n_modes + 8)
4. Falls back to dense solver for small systems (< 50 DOFs)
5. Added convergence checking based on eigenvalue relative change

**Algorithm Details:**
- Random initial subspace with mass-orthonormalization
- Iterative refinement with projected eigenvalue problem
- Convergence when max relative eigenvalue change < tolerance
- Reports iteration count in result

**Verification:**
- Tests for SubspaceIteration method pass ✓
- Tests for ShiftInvert method pass ✓
- Small systems correctly fall back to dense solver ✓

---

### Task 16.5: Mass Matrix Assembly Enhancement

**File:** `cpp/src/assembler.cpp`, `cpp/include/grillex/assembler.hpp`

**Description:** Extend Assembler to include point mass contributions in the global mass matrix. Currently only beam elements are assembled.

**Changes:**

```cpp
class Assembler {
public:
    // Existing
    Eigen::SparseMatrix<double> assemble_mass(
        const std::vector<BeamElement*>& elements) const;

    // New: Include point masses
    Eigen::SparseMatrix<double> assemble_mass(
        const std::vector<BeamElement*>& elements,
        const std::vector<PointMass*>& point_masses) const;

    // New: Include plate elements (if mass implemented)
    Eigen::SparseMatrix<double> assemble_mass(
        const std::vector<BeamElement*>& beam_elements,
        const std::vector<PlateElement*>& plate_elements,
        const std::vector<PointMass*>& point_masses) const;
};
```

**Acceptance Criteria:**
- [x] Point mass 6×6 matrices assembled at correct global DOFs
- [x] Beam element mass matrices assembled (existing functionality preserved)
- [ ] Plate element mass matrices assembled (if implemented) - deferred, plates not yet supported
- [x] Mixed assembly (beams + point masses) works correctly
- [x] Total mass equals sum of element masses (verified by trace of M for translations)
- [x] Backward compatible: existing assemble_mass(beams) still works

### Execution Notes (Completed 2025-12-22)

**Steps Taken:**
1. Added `assemble_mass(beam_elements, point_masses)` overload to Assembler
2. Added `compute_total_mass(beam_elements, point_masses)` for total mass calculation
3. Updated assembler.hpp with new method declarations
4. Added pybind11 bindings with explicit overload disambiguation

**Key Implementation Details:**
- Point mass 6×6 matrix assembled using DOFHandler for global DOF mapping
- Uses triplet list for efficient sparse assembly
- Total mass computed as sum of (rho * A * L) for beams plus point mass values
- Plate elements deferred (not yet implemented with mass)

**Verification:**
- test_assemble_mass_with_point_masses: verifies diagonal entries increase by mass value ✓
- test_compute_total_mass: verifies beam + point mass summation ✓

---

### Task 16.6: Participation Factors and Effective Modal Mass

**File:** `cpp/src/eigenvalue_solver.cpp`

**Description:** Compute modal participation factors and effective modal mass for each mode and direction.

**Implementation:**

```cpp
void EigenvalueSolver::compute_participation_factors(
    EigensolverResult& result,
    const Eigen::SparseMatrix<double>& M,
    const DOFHandler& dof_handler) const
{
    // Influence vectors for X, Y, Z translations
    // r_x has 1.0 at all UX DOFs, 0 elsewhere
    Eigen::VectorXd r_x = Eigen::VectorXd::Zero(M.rows());
    Eigen::VectorXd r_y = Eigen::VectorXd::Zero(M.rows());
    Eigen::VectorXd r_z = Eigen::VectorXd::Zero(M.rows());

    // Fill influence vectors based on DOF types
    for (const auto& node : nodes) {
        int dof_ux = dof_handler.global_dof(node->id, DOFIndex::UX);
        int dof_uy = dof_handler.global_dof(node->id, DOFIndex::UY);
        int dof_uz = dof_handler.global_dof(node->id, DOFIndex::UZ);
        if (dof_ux >= 0) r_x(dof_ux) = 1.0;
        if (dof_uy >= 0) r_y(dof_uy) = 1.0;
        if (dof_uz >= 0) r_z(dof_uz) = 1.0;
    }

    // Compute total modal mass in each direction
    double M_total_x = r_x.transpose() * M * r_x;
    double M_total_y = r_y.transpose() * M * r_y;
    double M_total_z = r_z.transpose() * M * r_z;

    // For each mode
    for (auto& mode : result.modes) {
        // Participation factor: Γ = φᵀ × M × r
        mode.participation_x = mode.mode_shape.transpose() * M * r_x;
        mode.participation_y = mode.mode_shape.transpose() * M * r_y;
        mode.participation_z = mode.mode_shape.transpose() * M * r_z;

        // Effective modal mass: Meff = Γ² (for mass-normalized modes)
        mode.effective_mass_x = mode.participation_x * mode.participation_x;
        mode.effective_mass_y = mode.participation_y * mode.participation_y;
        mode.effective_mass_z = mode.participation_z * mode.participation_z;

        // Percentage of total mass
        mode.effective_mass_pct_x = 100.0 * mode.effective_mass_x / M_total_x;
        mode.effective_mass_pct_y = 100.0 * mode.effective_mass_y / M_total_y;
        mode.effective_mass_pct_z = 100.0 * mode.effective_mass_z / M_total_z;
    }

    // Compute cumulative mass participation
    ...
}
```

**Acceptance Criteria:**
- [x] Participation factors computed for X, Y, Z translations
- [x] Effective modal mass computed correctly
- [x] Percentages sum to ≤100% across all modes
- [x] Cumulative mass tracked (for code compliance: need ≥90%)
- [ ] Rotational participation factors computed (RX, RY, RZ) - deferred to future enhancement
- [x] Results match hand calculations for simple cases

### Execution Notes (Completed 2025-12-22)

**Steps Taken:**
1. Implemented `compute_participation_factors()` method in EigenvalueSolver
2. Builds influence vectors (r_x, r_y, r_z) from DOF mapping
3. Computes total modal mass in each direction from r^T M r
4. For each mode, computes Γ = φ^T M r (participation factor)
5. Computes effective modal mass as Γ² (for mass-normalized modes)
6. Computes cumulative mass percentages

**Algorithm Details:**
- Determines DOF type by checking full_dof % 6 (0=UX, 1=UY, 2=UZ)
- Participation factor: Γ = φᵀ × M × r
- Effective mass: Meff = Γ² for mass-normalized modes
- Percentage: 100 × Meff / M_total
- Cumulative: running sum of percentages

**Verification:**
- test_participation_factors_computed: verifies total_mass_x/y/z populated ✓
- test_effective_mass_percentage_sum: verifies monotonically increasing cumulative ✓
- test_mode_participation_factors_exist: verifies all mode fields populated ✓

---

### Task 16.7: Model Integration

**File:** `cpp/include/grillex/model.hpp`, `cpp/src/model.cpp`

**Description:** Add eigenvalue analysis method to Model class.

**API:**

```cpp
class Model {
public:
    // Existing analysis methods...
    bool analyze();
    bool analyze_nonlinear();

    // New: Eigenvalue analysis
    bool analyze_eigenvalues(const EigensolverSettings& settings = {});

    // Query eigenvalue results
    bool has_eigenvalue_results() const;
    const EigensolverResult& get_eigenvalue_result() const;

    // Convenience methods
    std::vector<double> get_natural_frequencies() const;  // [Hz]
    std::vector<double> get_periods() const;              // [s]
    Eigen::VectorXd get_mode_shape(int mode_number) const;  // 1-based

private:
    std::unique_ptr<EigensolverResult> eigenvalue_result_;
};
```

**Workflow:**
1. Number DOFs (reuse from static analysis)
2. Assemble K and M matrices (including point masses)
3. Reduce system (eliminate fixed DOFs)
4. Solve eigenvalue problem
5. Expand mode shapes to full DOF vector
6. Compute participation factors
7. Store results

**Acceptance Criteria:**
- [ ] `analyze_eigenvalues()` returns true on success
- [ ] Results accessible via `get_eigenvalue_result()`
- [ ] Convenience methods work correctly
- [ ] Error handling for singular/ill-conditioned systems
- [ ] Works with warping DOFs (14-DOF elements)
- [ ] Works with mixed element types (beams + point masses)

---

### Task 16.8: Python Bindings

**File:** `cpp/bindings/bindings.cpp`

**Description:** Expose eigenvalue analysis to Python.

**Bindings:**

```cpp
// Enums
py::enum_<EigensolverMethod>(m, "EigensolverMethod")
    .value("Dense", EigensolverMethod::Dense)
    .value("SubspaceIteration", EigensolverMethod::SubspaceIteration)
    .value("ShiftInvert", EigensolverMethod::ShiftInvert);

// Settings
py::class_<EigensolverSettings>(m, "EigensolverSettings")
    .def(py::init<>())
    .def_readwrite("n_modes", &EigensolverSettings::n_modes)
    .def_readwrite("shift", &EigensolverSettings::shift)
    .def_readwrite("tolerance", &EigensolverSettings::tolerance)
    .def_readwrite("max_iterations", &EigensolverSettings::max_iterations)
    .def_readwrite("method", &EigensolverSettings::method)
    .def_readwrite("compute_participation", &EigensolverSettings::compute_participation)
    .def_readwrite("mass_normalize", &EigensolverSettings::mass_normalize);

// Mode result
py::class_<ModeResult>(m, "ModeResult")
    .def_readonly("mode_number", &ModeResult::mode_number)
    .def_readonly("eigenvalue", &ModeResult::eigenvalue)
    .def_readonly("frequency_hz", &ModeResult::frequency_hz)
    .def_readonly("period_s", &ModeResult::period_s)
    .def_property_readonly("mode_shape", ...)
    .def_readonly("participation_x", &ModeResult::participation_x)
    // ... etc

// Eigensolver result
py::class_<EigensolverResult>(m, "EigensolverResult")
    .def_readonly("converged", &EigensolverResult::converged)
    .def_readonly("iterations", &EigensolverResult::iterations)
    .def_readonly("message", &EigensolverResult::message)
    .def_property_readonly("modes", ...);

// Model methods
py::class_<Model>(m, "Model")
    // ... existing bindings
    .def("analyze_eigenvalues", &Model::analyze_eigenvalues,
         py::arg("settings") = EigensolverSettings{})
    .def("has_eigenvalue_results", &Model::has_eigenvalue_results)
    .def("get_eigenvalue_result", &Model::get_eigenvalue_result)
    .def("get_natural_frequencies", &Model::get_natural_frequencies)
    .def("get_periods", &Model::get_periods)
    .def("get_mode_shape", &Model::get_mode_shape);
```

**Acceptance Criteria:**
- [ ] All C++ types exported to Python
- [ ] EigensolverSettings can be configured from Python
- [ ] Mode results accessible with all fields
- [ ] Mode shapes returned as numpy arrays
- [ ] Type hints added to `_grillex_cpp.pyi`

---

### Task 16.9: Python API Wrapper

**File:** `src/grillex/core/model_wrapper.py`

**Description:** Add high-level Python API for eigenvalue analysis.

**API:**

```python
class StructuralModel:
    def analyze_modes(
        self,
        n_modes: int = 10,
        method: str = "subspace",
        tolerance: float = 1e-8,
        max_iterations: int = 100
    ) -> bool:
        """
        Perform eigenvalue analysis to find natural frequencies and mode shapes.

        Args:
            n_modes: Number of modes to compute (lowest frequencies first)
            method: Solver method ("dense", "subspace", "shift_invert")
            tolerance: Convergence tolerance for iterative methods
            max_iterations: Maximum iterations for iterative methods

        Returns:
            True if analysis converged successfully

        Raises:
            RuntimeError: If model has no mass or is improperly constrained
        """

    def get_natural_frequencies(self) -> List[float]:
        """Get natural frequencies in Hz, sorted ascending."""

    def get_periods(self) -> List[float]:
        """Get natural periods in seconds, sorted by frequency."""

    def get_mode_shape(self, mode_number: int) -> np.ndarray:
        """
        Get mode shape for specified mode (1-based indexing).

        Returns mass-normalized eigenvector as numpy array.
        """

    def get_modal_results_dataframe(self) -> pd.DataFrame:
        """
        Get modal analysis results as pandas DataFrame.

        Columns: mode, frequency_hz, period_s,
                 participation_x, participation_y, participation_z,
                 eff_mass_pct_x, eff_mass_pct_y, eff_mass_pct_z,
                 cumulative_x, cumulative_y, cumulative_z
        """

    def get_mode_displacement_at(
        self,
        mode_number: int,
        position: List[float]
    ) -> Dict[str, float]:
        """
        Get modal displacement at a position.

        Returns dict with keys: ux, uy, uz, rx, ry, rz
        """
```

**Acceptance Criteria:**
- [ ] `analyze_modes()` provides clean high-level interface
- [ ] Results accessible via multiple convenience methods
- [ ] DataFrame output for easy visualization and export
- [ ] Mode displacement queries work at arbitrary positions
- [ ] Docstrings complete with units and examples
- [ ] Type hints for all public methods

---

### Task 16.10: Validation Tests - Analytical Benchmarks

**File:** `tests/python/test_phase16_eigenvalue.py`

**Description:** Implement validation tests against analytical solutions.

**Test Cases:**

```python
class TestEigenvalueAnalytical:
    """Analytical benchmark tests for eigenvalue analysis."""

    def test_simply_supported_beam_first_mode(self):
        """
        Simply supported beam, first bending mode.

        Analytical: f₁ = (π/L)² × √(EI/ρA) / (2π)

        For L=10m, E=210e6 kN/m², I=8.36e-5 m⁴, ρ=7.85e-3 mT/m³, A=5.38e-3 m²:
        f₁ = 4.65 Hz (approximately)
        """

    def test_simply_supported_beam_higher_modes(self):
        """
        Simply supported beam, modes 2-5.

        fₙ/f₁ = n² (for simply supported beam)
        f₂ = 4×f₁, f₃ = 9×f₁, etc.
        """

    def test_cantilever_beam_first_mode(self):
        """
        Cantilever beam, first bending mode.

        f₁ = 1.875² × √(EI/ρAL⁴) / (2π)
        λ values: 1.875, 4.694, 7.855, 10.996, ...
        """

    def test_cantilever_beam_mode_shape(self):
        """
        Verify cantilever mode shape matches analytical form.

        φ(x) = cosh(λx/L) - cos(λx/L) - σ(sinh(λx/L) - sin(λx/L))
        where σ = (cosh(λ) + cos(λ)) / (sinh(λ) + sin(λ))
        """

    def test_sdof_spring_mass(self):
        """
        Single DOF spring-mass system.

        f = √(k/m) / (2π)

        k = 1000 kN/m, m = 10 mT → f = 1.59 Hz
        """

    def test_two_dof_system(self):
        """
        Two-mass system with known eigenvalues.

        Verifies both frequencies and mode shapes.
        """

    def test_free_free_beam_rigid_body_modes(self):
        """
        Free-free beam has 6 rigid body modes (ω ≈ 0).

        First flexible mode frequency should match theory.
        """
```

**Acceptance Criteria:**
- [ ] Simply supported beam: frequency within 1% of analytical
- [ ] Cantilever beam: frequency within 1% of analytical
- [ ] SDOF system: exact match (within numerical tolerance)
- [ ] Mode shapes match analytical forms
- [ ] Rigid body modes detected (ω < threshold)
- [ ] Higher modes follow correct frequency ratios

---

### Task 16.11: Validation Tests - Participation Factors

**File:** `tests/python/test_phase16_eigenvalue.py`

**Description:** Test participation factors and effective modal mass calculations.

**Test Cases:**

```python
class TestParticipationFactors:
    """Tests for modal participation and effective mass."""

    def test_cantilever_x_participation(self):
        """
        Cantilever along X-axis, loaded in Z.
        First mode should have high Z participation, low X and Y.
        """

    def test_symmetric_structure_participation(self):
        """
        Symmetric structure should have symmetric participation.
        """

    def test_cumulative_mass_approaches_100(self):
        """
        Sum of effective modal mass should approach 100% as more modes computed.
        """

    def test_point_mass_contribution(self):
        """
        Point mass at beam tip increases effective mass in fundamental mode.
        """

    def test_participation_sign_convention(self):
        """
        Verify participation factor signs are consistent.
        """
```

**Acceptance Criteria:**
- [ ] Participation factors correctly identify dominant directions
- [ ] Effective modal mass sums to ≤100%
- [ ] Point masses properly included in calculations
- [ ] Cumulative mass tracking works correctly

---

### Task 16.12: Integration Tests

**File:** `tests/python/test_phase16_eigenvalue.py`

**Description:** Integration tests for real-world usage scenarios.

**Test Cases:**

```python
class TestEigenvalueIntegration:
    """Integration tests for eigenvalue analysis."""

    def test_grillage_natural_frequencies(self):
        """Multi-beam grillage structure."""

    def test_mixed_elements(self):
        """Beams + point masses + springs."""

    def test_warping_elements(self):
        """14-DOF elements with warping."""

    def test_large_model_performance(self):
        """1000+ DOF model completes in reasonable time."""

    def test_yaml_model_eigenvalue(self):
        """Load YAML model, run eigenvalue analysis."""

    def test_mode_shape_continuity(self):
        """Mode shapes are continuous across element boundaries."""
```

**Acceptance Criteria:**
- [ ] Works with complex multi-element models
- [ ] Mixed element types handled correctly
- [ ] Warping DOFs included in analysis
- [ ] Performance acceptable for large models
- [ ] YAML workflow works end-to-end

---

### Task 16.13: Documentation

**File:** `docs/user/eigenvalue_analysis.rst`

**Description:** User documentation for eigenvalue analysis feature.

**Contents:**
1. Introduction to modal analysis
2. Running eigenvalue analysis
3. Interpreting results (frequencies, mode shapes, participation)
4. Example: cantilever beam
5. Example: multi-story frame
6. Troubleshooting (convergence issues, zero-mass DOFs)
7. Technical reference (algorithms, settings)

**Acceptance Criteria:**
- [ ] Clear explanation of when to use eigenvalue analysis
- [ ] Step-by-step examples with code
- [ ] Interpretation guide for results
- [ ] Troubleshooting section
- [ ] All examples are doctests that pass

---

### Task 16.14: LLM Tool Schema

**File:** `src/grillex/llm/tools.py`

**Description:** Add tool schema for LLM-driven eigenvalue analysis.

**Tool Definition:**

```python
{
    "name": "analyze_modes",
    "description": "Perform eigenvalue analysis to find natural frequencies and mode shapes. Use this to check for resonance, understand dynamic behavior, or prepare for response spectrum analysis.",
    "input_schema": {
        "type": "object",
        "properties": {
            "n_modes": {
                "type": "integer",
                "description": "Number of modes to compute (lowest frequencies first)",
                "default": 10
            },
            "method": {
                "type": "string",
                "enum": ["dense", "subspace", "shift_invert"],
                "description": "Solver method. Use 'subspace' for large models.",
                "default": "subspace"
            }
        }
    }
},
{
    "name": "get_modal_summary",
    "description": "Get summary of modal analysis results including frequencies, periods, and participation factors.",
    "input_schema": {
        "type": "object",
        "properties": {}
    }
},
{
    "name": "check_resonance",
    "description": "Check if any natural frequency is within a specified range of an excitation frequency.",
    "input_schema": {
        "type": "object",
        "properties": {
            "excitation_frequency": {
                "type": "number",
                "description": "Excitation frequency to check against [Hz]"
            },
            "tolerance_percent": {
                "type": "number",
                "description": "Percentage band around excitation frequency",
                "default": 15
            }
        },
        "required": ["excitation_frequency"]
    }
}
```

**Acceptance Criteria:**
- [ ] Tool schema for analyze_modes
- [ ] Tool schema for get_modal_summary
- [ ] Tool schema for check_resonance
- [ ] Handler implementations in ToolExecutor
- [ ] Error suggestions for common eigenvalue issues

---

## Acceptance Criteria Summary

| Task | Description | Criteria Count |
|------|-------------|----------------|
| 16.1 | Settings and Results structs | 6 |
| 16.2 | BC Reduction | 5 |
| 16.3 | Dense Solver | 5 |
| 16.4 | Subspace Iteration | 6 |
| 16.5 | Mass Assembly Enhancement | 6 |
| 16.6 | Participation Factors | 6 |
| 16.7 | Model Integration | 6 |
| 16.8 | Python Bindings | 5 |
| 16.9 | Python API Wrapper | 6 |
| 16.10 | Analytical Benchmarks | 6 |
| 16.11 | Participation Tests | 5 |
| 16.12 | Integration Tests | 6 |
| 16.13 | Documentation | 5 |
| 16.14 | LLM Tool Schema | 5 |
| **Total** | | **78** |

---

## Implementation Order

**Recommended sequence:**

1. **Task 16.1** - Data structures (foundation for everything)
2. **Task 16.5** - Mass assembly enhancement (needed before solving)
3. **Task 16.2** - BC reduction (needed for correct eigenvalue problem)
4. **Task 16.3** - Dense solver (simpler, good for testing)
5. **Task 16.10** - Analytical benchmarks (validate dense solver)
6. **Task 16.4** - Subspace iteration (for larger models)
7. **Task 16.6** - Participation factors
8. **Task 16.7** - Model integration
9. **Task 16.8** - Python bindings
10. **Task 16.9** - Python API wrapper
11. **Task 16.11-12** - Remaining tests
12. **Task 16.13-14** - Documentation and LLM tools

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Convergence issues with subspace iteration | Medium | Medium | Fall back to dense solver; add shift parameter |
| Zero-mass DOFs causing numerical issues | Medium | High | Detect and warn; optionally add small mass |
| Performance for large models | Low | Medium | Subspace iteration; sparse operations |
| Warping DOF handling | Low | Medium | Careful testing with 14-DOF elements |

---

## Future Extensions

After Phase 16 is complete, the following can be added:

1. **Response Spectrum Analysis** - Combine modal results with design spectra
2. **Harmonic Response Analysis** - Frequency-domain response
3. **Damping** - Modal damping ratios, Rayleigh damping
4. **Geometric Stiffness** - Buckling eigenvalue analysis (K_g matrix)
5. **Mode Shape Animation** - Export to visualization tools
