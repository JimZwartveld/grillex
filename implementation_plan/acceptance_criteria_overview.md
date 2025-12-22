# Grillex 2.0 - Acceptance Criteria Overview

This document provides a comprehensive overview of all acceptance criteria across implementation phases. It is automatically updated when tasks are completed.

**Last Updated:** 2025-12-22

## Summary Statistics

| Phase | Description | Total Criteria | Met | Unmet | Progress |
|-------|-------------|----------------|-----|-------|----------|
| 0 | Project Setup & Infrastructure | 8 | 8 | 0 | 100% |
| 1 | Core Data Structures | 13 | 13 | 0 | 100% |
| 2 | Beam Element Foundation | 32 | 32 | 0 | 100% |
| 3 | Assembly & Solver | 15 | 15 | 0 | 100% |
| 4 | Python Front-End & I/O | 16 | 16 | 0 | 100% |
| 5 | Loads & Load Cases | 15 | 15 | 0 | 100% |
| 6 | MPC & Rigid Links | 9 | 9 | 0 | 100% |
| 7 | Internal Actions & Results | 35 | 35 | 0 | 100% |
| 8 | Additional Element Types | 9 | 9 | 0 | 100% |
| 9 | Cargo Modeling | 20 | 20 | 0 | 100% |
| 10 | Design Codes | 47 | 6 | 41 | 13% |
| 11 | Error Handling | 9 | 6 | 3 | 67% |
| 12 | LLM Tooling | 10 | 9 | 1 | 90% |
| 13 | Validation Benchmarks | 12 | 12 | 0 | 100% |
| 14 | DevOps | 4 | 0 | 4 | 0% |
| 15 | Nonlinear Springs | 79 | 77 | 2 | 97% |
| **Total** | | **321** | **282** | **39** | **88%** |

---

## Phase 0: Project Setup & Infrastructure

### Task 0.1: Initialize Project Structure
- [x] Directory structure exists
- [x] `pip install -e .` runs without error (even if no C++ yet)
- [x] `import grillex` works

### Task 0.2: Setup C++ Build Infrastructure
- [x] `cmake -B build && cmake --build build` succeeds
- [x] Python can import the compiled module
- [x] Test function is callable from Python

### Task 0.3: Setup Testing Infrastructure
- [x] `pytest` runs and reports 1 passing test
- [x] Test discovery works correctly

---

## Phase 1: Core Data Structures (C++)

### Task 1.1: Implement Node Class
- [x] Node can be constructed with id and coordinates
- [x] Position can be retrieved as Eigen::Vector3d
- [x] DOF arrays are correctly initialized

### Task 1.2: Implement Node Registry
- [x] Creating node at (0,0,0) twice returns same node
- [x] Creating node at (0,0,1e-9) with tolerance 1e-6 returns existing node
- [x] Creating node at (0,0,1) with tolerance 1e-6 creates new node
- [x] Node IDs are sequential and unique

### Task 1.3: Implement Material Class
- [x] Material can be constructed with standard properties
- [x] G is correctly computed from E and nu
- [x] All units are documented in header comments

### Task 1.4: Implement Section Class
- [x] Section can be constructed with basic properties
- [x] Optional properties default to 0 or computed values
- [x] Section can store fibre distances for stress calculation

### Task 1.5: Create Python Bindings
- [x] `from grillex.core import Node, Material, Section` works
- [x] Can create instances from Python
- [x] Properties are accessible

---

## Phase 2: Beam Element Foundation (C++)

### Task 2.1: Implement Local Coordinate System
- [x] Horizontal beam along global X has local z pointing up (global Z)
- [x] Vertical beam has well-defined local axes (no NaN)
- [x] Roll angle rotates y and z about x
- [x] Rotation matrix is orthonormal (R^T * R = I)

### Task 2.2: Implement Beam Element Stiffness Matrix
- [x] Stiffness matrix is symmetric
- [x] Stiffness matrix is positive semi-definite (6 zero eigenvalues for rigid body modes)
- [x] Simple cantilever deflection matches: δ = PL³/(3EI)

### Task 2.3: Implement Beam Element Mass Matrix
- [x] Mass matrix is symmetric
- [x] Mass matrix is positive semi-definite
- [x] Total mass equals ρAL when integrated

### Task 2.4: Implement End Offsets
- [x] Beam with offsets has correct effective length
- [x] Stiffness matrix accounts for eccentric connection
- [x] Simple offset beam matches reference solution

### Task 2.5: Implement End Releases
- [x] Simply supported beam (moment releases at both ends) gives correct deflection
- [x] Pinned-fixed beam gives correct reactions
- [x] Released DOFs don't appear in global equations (conceptually)
- [x] Axial release creates sliding connection (no axial force transfer)
- [x] Torsion release creates torsion hinge (no torque transfer)
- [x] Warping release at beam end gives B=0 (bimoment-free connection)
- [x] Multiple simultaneous releases work correctly
- [x] Condensation preserves matrix symmetry

### Task 2.6: Implement Timoshenko Beam Element
- [x] For very slender beams (L/d > 20), Timoshenko ≈ Euler-Bernoulli
- [x] For deep beams (L/d < 5), Timoshenko gives smaller stiffness (larger deflections)
- [x] Shear locking is avoided (φ → 0 recovers Euler-Bernoulli)
- [x] Stiffness and mass matrices remain symmetric and positive semi-definite

### Task 2.7: Implement Warping Element (7th DOF)
- [x] 14×14 stiffness matrix is symmetric
- [x] For Iw = 0 (no warping capacity), reduces to standard 12×12 behavior
- [x] Cantilever I-beam with torque: warping-restrained end has higher stiffness
- [x] Warping DOF can be enabled/disabled per node
- [x] Warping increases torsional stiffness compared to St. Venant alone

### Task 2.8: Unified Beam Element Factory
- [x] Factory correctly creates Euler-Bernoulli, Timoshenko, or Warping elements
- [x] num_dofs() returns correct value (12 or 14)
- [x] Existing code continues to work with default config

### Task 2.9: Warping DOF Decoupling at Non-Collinear Connections
- [x] Collinearity detection correctly identifies parallel elements
- [x] Non-collinear elements have independent warping DOFs
- [x] Collinear elements share warping DOFs (continuous warping)
- [x] Boundary conditions work for element-specific warping DOFs
- [x] T-joint with torque shows no warping coupling between orthogonal beams
- [x] Continuous beam shows warping continuity at internal nodes
- [x] User can override automatic coupling detection
- [x] Backward compatible: models without warping unchanged

---

## Phase 3: Assembly & Solver

### Task 3.1: Implement DOF Handler
- [x] DOFs are numbered correctly
- [x] Location arrays map element to global DOFs
- [x] Warping DOFs are handled for 14-DOF elements

### Task 3.2: Implement Global Matrix Assembler
- [x] Assembled matrix is sparse
- [x] Assembled matrix is symmetric
- [x] Single 12-DOF element assembly matches element stiffness matrix
- [x] Single 14-DOF element assembly matches element stiffness matrix
- [x] Mixed assembly (12-DOF and 14-DOF elements) works correctly

### Task 3.3: Implement Boundary Condition Handler
- [x] Fixed DOFs result in zero (or prescribed) displacement
- [x] Reactions can be recovered from K * u - F
- [x] System remains solvable after BC application
- [x] Warping DOF (index 6) can be fixed or left free
- [x] Fork support correctly leaves warping free
- [x] Built-in support with warping correctly restrains warping

### Task 3.4: Implement Linear Solver
- [x] Solver returns correct displacement for simple problems
- [x] Singular systems are detected and reported
- [x] Performance is acceptable for sparse systems (1000+ DOFs)

### Task 3.5: Integrate Analysis Pipeline
- [x] Complete analysis workflow runs without errors
- [x] Results match hand calculations for simple models
- [x] Error handling for invalid models

---

## Phase 4: Python Front-End & I/O

### Task 4.1: Implement Python Model Wrapper
- [x] Beams can be created with coordinate lists
- [x] Model can be analyzed from Python
- [x] Results are accessible from Python

### Task 4.2: Implement YAML Loader
- [x] Valid YAML files load without error
- [x] All entity types are supported
- [x] Clear error messages for invalid YAML

### Task 4.3: Implement Result Writer
- [x] Results export to valid JSON
- [x] JSON structure is human-readable
- [x] All result types are included

### Task 4.4: Implement Beam Subdivision
- [x] Beam with one internal node becomes two elements
- [x] Beam with multiple internal nodes becomes multiple elements
- [x] Section/material properties propagate to sub-beams

### Task 4.5: Python Bindings for Internal Actions
- [x] InternalActions, EndForces, and DisplacementLine structs are accessible from Python
- [x] BeamElement methods (get_internal_actions, compute_end_forces, get_displacements_at, find_component_extrema) are callable from Python
- [x] Methods accept Eigen arrays and return appropriate types
- [x] Type hints are provided for IDE support
- [x] Example usage demonstrates multi-element beam plotting
- [x] Unit tests verify Python bindings work correctly

---

## Phase 5: Loads & Load Cases

### Task 5.1: Implement Load Types
- [x] Nodal loads can be applied
- [x] Line loads can be applied to beams
- [x] Load cases have type classification

### Task 5.2: Implement Distributed Loads
- [x] Uniform load produces correct reactions
- [x] Fixed-end moments match theory
- [x] Trapezoidal loads work correctly
- [x] BeamElement can query its distributed loads for Phase 7 internal actions computation
- [x] DistributedLoad structure is compatible with Phase 7 differential equation approach

### Task 5.3: Implement Acceleration Loads
- [x] Gravity load (az = -9.81) produces correct weight forces
- [x] Rotational acceleration produces centrifugal effects
- [x] Results match: 1 mT/m beam with gravity → 9.81 kN/m load

### Task 5.4: Implement Load Combinations
- [x] Combinations sum loads correctly
- [x] Factors are applied correctly
- [x] Multiple load cases combine properly

---

## Phase 6: MPC & Rigid Links

### Task 6.1: Implement Constraint Handler
- [x] Simple equality constraints work
- [x] Rigid links transfer forces correctly
- [x] T is correctly formed (correct dimensions and entries)

### Task 6.2: Implement Rigid Links
- [x] Slave node moves correctly with master
- [x] Rotation at master produces translation at slave
- [x] Forces transfer correctly through rigid link

### Task 6.3: Implement Reduced System
- [x] Reduced system is smaller than original
- [x] Full displacements are recovered correctly
- [x] Constrained DOFs satisfy constraint equations

---

## Phase 7: Internal Actions & Results

### Task 7.0: Distributed Load Query Methods
- [x] `get_distributed_load_y()` returns correct local y-component
- [x] `get_distributed_load_z()` returns correct local z-component
- [x] `get_distributed_load_axial()` returns correct local x-component
- [x] Multiple line loads on same element are summed correctly
- [x] Global-to-local transformation is handled properly
- [x] Returns zero `DistributedLoad` if no line loads on element

### Task 7.1: Compute Element End Forces
- [x] End forces match support reactions for simple cases (cantilever, simply supported)
- [x] Sign convention is consistent across all force/moment components
- [x] Forces satisfy equilibrium: sum(F) = 0, sum(M) = 0
- [x] End releases properly zero out forces at released DOFs
- [x] Works for both 12-DOF and 14-DOF (warping) elements

### Task 7.2: Internal Action Functions Along Beam
- [x] Simply supported beam with UDL: M_max = wL²/8 at midspan (within 0.1%)
- [x] Cantilever with tip load: M_max = PL at support (exact)
- [x] Cantilever with UDL: M_max = wL²/2 at support, M(L/2) = wL²/8 (within 0.1%)
- [x] Fixed-fixed beam with UDL: M_ends = wL²/12, M_mid = wL²/24 (within 0.1%)
- [x] All 16 release combinations produce physically correct results
- [x] Shear and moment satisfy dM/dx = V at all points
- [x] Extrema are found correctly (analytical vs numerical agreement)
- [x] Euler-Bernoulli and Timoshenko results agree for slender beams (L/h > 20)
- [x] Timoshenko shows increased deflection for short, deep beams

### Task 7.2b: Warping Results (Bimoments)
- [x] Bimoment at warping-restrained end matches analytical solution for uniform torsion
- [x] Warping-free end has B ≈ 0 (within numerical tolerance)
- [x] For two-span continuous beam under torsion, bimoment is continuous at support
- [x] Total normal stress σ_total = σ_axial + σ_bending + σ_warping is computed correctly
- [x] For section with Iw = 0 (closed sections), bimoment results are zero
- [x] Sign convention consistent with standard references (Kollbrunner & Hajdin)
- [x] Comparison with analytical solution for cantilever I-beam under torsion
- [x] All 16 boundary condition combinations produce correct results

### Task 7.2c: Displacement/Rotation Lines
- [x] Displacements at element ends match nodal values exactly
- [x] Deflection shape for cantilever with tip load matches analytical curve
- [x] Rotation φ_z = dw/dy for Euler-Bernoulli beams
- [x] For Timoshenko, φ_z ≠ dw/dy (shear deformation included)

### Task 7.2f: Multi-Element Beam Plotting
- [x] Continuous moment diagram across 3-element beam matches hand calculation
- [x] Element boundaries are clearly marked in plots
- [x] Concentrated loads cause visible shear discontinuities
- [x] Extrema are found and marked correctly across element boundaries
- [x] Deflection diagram is smooth and continuous
- [x] Works with beams of varying element counts (2 to 10+ elements)

### Task 7.3: Python Bindings for Internal Actions
- [x] InternalActions, EndForces, and DisplacementLine structs are accessible from Python
- [x] BeamElement methods are callable from Python
- [x] Methods accept Eigen arrays and return appropriate types
- [x] Type hints are provided for IDE support
- [x] Example usage demonstrates multi-element beam plotting
- [x] Unit tests verify Python bindings work correctly

### Task 7.4: Implement Check Locations
- [x] Check locations can be added at arbitrary normalized positions
- [x] Standard check locations (0, 0.25, 0.5, 0.75, 1) can be set automatically
- [x] Internal actions are computed correctly at check locations
- [x] Check locations persist across multiple analyses

---

## Phase 8: Additional Element Types

### Task 8.1: Implement Spring Element
- [x] Spring provides correct stiffness between nodes
- [x] Uncoupled DOFs are independent
- [x] Eccentricity can be handled via rigid links

### Task 8.2: Implement Point Mass Element
- [x] Point mass contributes to global mass matrix
- [x] Inertia tensor is correctly represented
- [x] Off-diagonal terms work for asymmetric masses

### Task 8.3: Implement Plate Element
- [x] Plate deflects under pressure load
- [x] Simple plate matches analytical solution
- [x] Mesh refinement converges

---

## Phase 9: Cargo Modeling

### Task 9.1: Implement Cargo Definition
- [x] Cargo definition is simple and clear
- [x] Generated elements correctly represent the cargo
- [x] Cargo mass contributes to inertial loads under acceleration

### Task 9.2: Static vs Dynamic Cargo Connections
- [x] CargoConnection has loading_condition attribute with values "all", "static", "dynamic"
- [x] add_connection accepts loading_condition parameter
- [x] Validation rejects invalid loading_condition values
- [x] Default loading_condition="all" maintains backward compatibility

### Task 9.3: Add Loading Condition to Spring Element (C++)
- [x] SpringElement has loading_condition property
- [x] is_active_for_load_case correctly maps load case types to connection activity
- [x] Python can get/set loading_condition on spring elements
- [x] LoadingCondition enum is accessible from Python

### Task 9.4: Filter Springs in Stiffness Matrix Assembly
- [x] Static load cases (Permanent) use K matrix without dynamic springs
- [x] Dynamic load cases (Variable/Environmental) use K matrix with all springs
- [x] Existing tests pass (backward compatibility with loading_condition="all")
- [x] Performance is acceptable (no regression for models without conditional springs)

### Task 9.5: Tests for Static/Dynamic Cargo Connections
- [x] Static connections carry load only in Permanent load cases
- [x] Dynamic connections carry load only in Variable/Environmental load cases
- [x] Connections with loading_condition="all" work for all load case types
- [x] Combined reaction magnitudes match expected values from hand calculations
- [x] Test coverage includes all loading_condition values

---

## Phase 10: Design Codes

### Task 10.1: Design Code Base Classes
- [x] Base classes are defined
- [x] Plugin structure allows multiple codes
- [x] Check results include all required info

### Task 10.2: Implement Basic Checks
- [x] At least basic checks are implemented
- [x] Utilization is computed correctly
- [x] Governing check is identified

### Task 10.3: BeamDesignSettings Class
- [ ] BeamDesignSettings stores all buckling parameters
- [ ] Effective length computation works with factors and absolute lengths
- [ ] Moment gradient factors can be specified or computed
- [ ] Settings can be attached to Beam elements

### Task 10.4: EC3BeamSettings Class
- [ ] EC3BeamSettings stores all EC3-specific parameters
- [ ] Buckling curves can be specified or auto-selected
- [ ] Section class can override automatic classification
- [ ] Imperfection factors computed correctly

### Task 10.5: Section Classification (EC3 Table 5.2)
- [ ] Flange classification correct for I-sections
- [ ] Web classification correct for pure bending
- [ ] Web classification correct for combined N+M
- [ ] Section class = max(flange_class, web_class)
- [ ] Returns Class 4 when limits exceeded
- [ ] Epsilon factor computed correctly from fy

### Task 10.6: Flexural Buckling Check (EC3 6.3.1)
- [ ] Slenderness computed correctly for y and z axes
- [ ] Chi reduction factor matches tabulated values (within 1%)
- [ ] Buckling curves a0, a, b, c, d all work correctly
- [ ] Check performs for both axes, reports governing
- [ ] Class 4 effective area supported

### Task 10.7: Lateral-Torsional Buckling Check (EC3 6.3.2)
- [ ] Mcr computed correctly for I-sections
- [ ] C1 factors work for standard moment diagrams
- [ ] χLT matches EC3 tables (within 1%)
- [ ] Both general and rolled section methods available
- [ ] Short beams (λ̄LT ≤ 0.4) skip LTB check (χLT = 1.0)

### Task 10.8: Member Buckling Interaction (EC3 6.3.3)
- [ ] Both interaction equations (y-y and z-z) checked
- [ ] Interaction factors kyy, kyz, kzy, kzz computed correctly
- [ ] Cm factors for different moment diagrams
- [ ] ΔM shift moments for Class 4 sections
- [ ] Reports governing interaction equation

### Task 10.9: Additional Cross-Section Checks
- [ ] Torsion stress computed correctly
- [ ] Shear-moment interaction reduces capacity when V > 0.5Vpl
- [ ] Combined N+V+M interaction implemented
- [ ] Warping stresses included for open sections

### Task 10.10: Integrate Settings with Beam and Model
- [ ] Settings can be attached to individual beams
- [ ] Default settings can be set at model level
- [ ] EC3 checks use beam settings when available
- [ ] Falls back to defaults when no settings specified

### Task 10.11: Comprehensive Tests for Extended EC3
- [ ] At least 50 new tests for extended EC3
- [ ] Buckling reduction factors within 1% of tabulated values
- [ ] All edge cases handled (λ̄ < 0.2, λ̄ > 3.0, etc.)
- [ ] Tests cover all buckling curves

---

## Phase 11: Error Handling & Diagnostics

### Task 11.1: Structured Error Codes
- [x] Errors have machine-readable codes
- [x] Errors have human-readable messages
- [x] Diagnostic info (DOFs, elements) is included

### Task 11.2: Warning System
- [x] Warnings don't block analysis
- [x] Severity levels are assigned appropriately
- [x] LLM can parse warning codes

### Task 11.3: Singularity Diagnostics
- [ ] Free-floating model detected as singular
- [ ] Specific unconstrained DOFs identified
- [ ] Helpful diagnostic message generated

---

## Phase 12: LLM Tooling

### Task 12.1: Docstrings and Type Hints
- [x] All public functions have docstrings
- [x] All parameters have type hints
- [x] Units are documented
- [ ] mypy passes (deferred to CI/CD setup)

### Task 12.2: Tool Schemas
- [x] Tool schemas are valid JSON Schema
- [x] Schemas are self-documenting
- [x] Tools cover main modelling operations

### Task 12.3: Error Recovery Suggestions
- [x] Each error type has fix suggestions
- [x] Suggestions are actionable tool calls
- [x] LLM can execute suggestions to fix model

---

## Phase 13: Validation Benchmarks

### Task 13.1: Simply Supported Beam Benchmark
- [x] Tests pass
- [x] Uniform load deflection matches 5wL⁴/(384EI)
- [x] Point load deflection matches PL³/(48EI)

### Task 13.2: Cantilever Beam Validation
- [x] Tests pass
- [x] Deflection matches PL³/(3EI)
- [x] Reaction moment matches PL

### Task 13.3: Offset Beam Validation
- [x] Offset beam gives correct deflection
- [x] Forces transfer correctly through offset

### Task 13.4: Cargo on Springs Validation
- [x] Vertical reactions equal cargo weight
- [x] Forces balance correctly

### Task 13.5: Singularity Detection Validation
- [x] Unconstrained system returns analysis failure
- [x] Partially constrained system detected

### Task 13.6: 14-DOF Warping Element Validation
- [x] Cantilever I-beam bimoment matches analytical solution for uniform torsion
- [x] Two-span continuous beam shows bimoment continuity at internal support

---

## Phase 14: DevOps

### Task 14.1: CI/CD Pipeline
- [ ] Tests run on push
- [ ] Coverage reported
- [ ] Build artifacts published

### Task 14.2: Documentation
- [ ] Sphinx docs build
- [ ] API reference complete
- [ ] Examples included

---

## Phase 15: Nonlinear Springs (Tension/Compression-Only and Gap Springs)

### Task 15.1: Spring Element State Tracking
- [x] SpringBehavior enum added with Linear, TensionOnly, CompressionOnly values
- [x] Per-DOF behavior can be set independently
- [x] Per-DOF gap values can be set independently
- [x] State tracking correctly identifies when gap is closed
- [x] current_stiffness_matrix() returns zero contribution for inactive DOFs
- [x] compute_forces() correctly applies gap offset to force calculation
- [x] compute_gap_forces() returns correct gap closure forces for solver
- [x] state_changed() correctly detects state transitions
- [x] Gap tolerance prevents chattering near threshold
- [x] has_gap() and is_nonlinear() helper methods work correctly

### Task 15.2: Iterative Nonlinear Solver
- [x] NonlinearSolver class implemented with settings struct
- [x] NonlinearInitialState struct implemented for static→dynamic sequencing
- [x] solve() accepts optional initial_state parameter
- [x] When initial_state provided, iteration starts from that displacement/state
- [x] When initial_state not provided, starts from zero with all springs active
- [x] Iterative solve correctly updates spring states
- [x] Gap forces computed and added to RHS for gap springs
- [x] Convergence detected when no spring states change
- [x] Maximum iteration limit prevents infinite loops
- [x] Linear-only springs without gaps bypass iteration (optimization)
- [x] Oscillation detection prevents flip-flopping states
- [x] Singular system during iteration handled gracefully
- [x] Result includes final spring states for reporting

### Task 15.3: Model Integration for Nonlinear Analysis
- [x] has_nonlinear_springs() correctly identifies nonlinear springs in model
- [x] analyze_nonlinear() uses iterative solver for load cases
- [x] analyze_combination() solves combined loads directly (not superposition)
- [x] analyze_combination() implements static→dynamic sequencing:
  - Permanent loads solved first to establish baseline contact pattern
  - Full combination solved starting from static state (initial_state)
  - Static solve failure returns meaningful error message
- [x] Linear models still use efficient linear solver
- [x] LoadCaseResult extended with iteration count and spring states
- [x] Reactions computed correctly with final spring stiffness

### Task 15.4: Python API Updates
- [x] SpringBehavior enum exported to Python
- [x] NonlinearSolverSettings exposed with all fields
- [x] NonlinearSolverResult exposed with spring_states
- [x] SpringElement.behavior accessible per-DOF from Python
- [x] SpringElement.gap accessible per-DOF from Python
- [x] StructuralModel.add_spring() accepts behavior and gap parameters
- [x] set_gap() and set_all_gaps() methods work correctly
- [x] has_gap() and is_nonlinear() methods exposed
- [x] analyze_with_nonlinear_springs() method added
- [x] analyze_load_combination() method added
- [x] All new types have complete docstrings with units

### Task 15.5: Results Reporting for Nonlinear Springs
- [x] LoadCaseResult includes iterations and solver_message
- [x] Spring states stored in results
- [x] Spring forces computed and stored
- [x] Python API can query individual spring states
- [x] Summary DataFrame shows all springs with states and forces
- [x] Units documented (kN for force, kN·m for moment)

### Task 15.6: Convergence Enhancements
- [x] Oscillation detection implemented
- [x] Partial stiffness option for oscillating springs
- [x] Hysteresis band prevents rapid state changes
- [ ] Line search damping available as option
- [x] Clear warning messages for convergence issues
- [x] Settings expose all convergence parameters

### Task 15.7: Validation Tests
- [x] Tension-only spring tests pass
- [x] Compression-only spring tests pass
- [x] Load reversal iteration test demonstrates state changes
- [x] Multi-spring test shows partial liftoff
- [x] Load combination test proves superposition invalidity
- [x] Gap spring open/closed state tests pass
- [x] Gap spring force offset verified (F = k × (δ - gap))
- [x] Gap closure iteration test passes
- [x] Contact with clearance practical test passes
- [x] Hook with slack practical test passes
- [x] Analytical verification test passes
- [x] Static→dynamic sequencing test verifies Permanent loads solved first
- [x] Liftoff from static contact test passes
- [x] Initial state preserves spring states from static solve
- [x] Convergence reporting verified
- [x] Edge cases (near-zero deformation) handled

### Task 15.8: Documentation and Examples
- [x] User guide section added to docs (including gap springs)
- [x] Technical reference documents algorithm and gap forces
- [x] At least 3 complete examples with code (bearing pads, gap contact, slack cables)
- [x] Troubleshooting section for common issues
- [x] All docstrings complete with units

### Task 15.9: Performance Optimization
- [x] Sparse matrix updates avoid full reassembly
- [x] Linear models have no performance penalty
- [x] Iteration count logged for performance analysis
- [x] Benchmark shows acceptable performance for 100+ springs

---

## Maintenance Notes

### Updating This Document

When a task is completed and acceptance criteria are verified:

1. Update the checkbox from `[ ]` to `[x]`
2. Update the summary statistics table at the top
3. Update the "Last Updated" date
4. Commit with message: `Update acceptance criteria: [Phase X.Y Task Name]`

### Verification Process

Each acceptance criterion should be verified by:
1. A passing automated test (preferred)
2. Manual verification documented in execution notes
3. Code review confirmation

### Related Documents

- `grillex_requirements.md` - Full requirements specification
- `implementation_plan_overview.md` - Phase overview
- `implementation_plan_phase*.md` - Detailed phase implementation plans
- `implementation_plan_phase15.md` - Nonlinear springs (tension/compression-only, gap springs)
- `CLAUDE.md` - AI assistant guidelines
