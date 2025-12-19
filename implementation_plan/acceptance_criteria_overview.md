# Grillex 2.0 - Acceptance Criteria Overview

This document provides a comprehensive overview of all acceptance criteria across implementation phases. It is automatically updated when tasks are completed.

**Last Updated:** 2025-12-19

## Summary Statistics

| Phase | Description | Total Criteria | Met | Unmet | Progress |
|-------|-------------|----------------|-----|-------|----------|
| 0 | Project Setup & Infrastructure | 8 | 8 | 0 | 100% |
| 1 | Core Data Structures | 13 | 13 | 0 | 100% |
| 2 | Beam Element Foundation | 32 | 29 | 3 | 91% |
| 3 | Assembly & Solver | 15 | 15 | 0 | 100% |
| 4 | Python Front-End & I/O | 16 | 10 | 6 | 63% |
| 5 | Loads & Load Cases | 15 | 0 | 15 | 0% |
| 6 | MPC & Rigid Links | 9 | 9 | 0 | 100% |
| 7 | Internal Actions & Results | 31 | 23 | 8 | 74% |
| 8 | Additional Element Types | 9 | 0 | 9 | 0% |
| 9 | Cargo Modeling | 3 | 0 | 3 | 0% |
| 10 | Design Codes | 6 | 0 | 6 | 0% |
| 11 | Error Handling | 9 | 0 | 9 | 0% |
| 12 | LLM Tooling | 9 | 0 | 9 | 0% |
| 13 | Validation Benchmarks | 10 | 0 | 10 | 0% |
| 14 | DevOps | 4 | 0 | 4 | 0% |
| **Total** | | **179** | **107** | **72** | **60%** |

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
- [ ] Simply supported beam (moment releases at both ends) gives correct deflection
- [ ] Pinned-fixed beam gives correct reactions
- [ ] Released DOFs don't appear in global equations (conceptually)
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
- [ ] Boundary conditions work for element-specific warping DOFs
- [ ] T-joint with torque shows no warping coupling between orthogonal beams
- [ ] Continuous beam shows warping continuity at internal nodes
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
- [ ] InternalActions, EndForces, and DisplacementLine structs are accessible from Python
- [ ] BeamElement methods (get_internal_actions, compute_end_forces, get_displacements_at, find_component_extrema) are callable from Python
- [ ] Methods accept Eigen arrays and return appropriate types
- [ ] Type hints are provided for IDE support
- [ ] Example usage demonstrates multi-element beam plotting
- [ ] Unit tests verify Python bindings work correctly

---

## Phase 5: Loads & Load Cases

### Task 5.1: Implement Load Types
- [ ] Nodal loads can be applied
- [ ] Line loads can be applied to beams
- [ ] Load cases have type classification

### Task 5.2: Implement Distributed Loads
- [ ] Uniform load produces correct reactions
- [ ] Fixed-end moments match theory
- [ ] Trapezoidal loads work correctly
- [ ] BeamElement can query its distributed loads for Phase 7 internal actions computation
- [ ] DistributedLoad structure is compatible with Phase 7 differential equation approach

### Task 5.3: Implement Acceleration Loads
- [ ] Gravity load (az = -9.81) produces correct weight forces
- [ ] Rotational acceleration produces centrifugal effects
- [ ] Results match: 1 mT/m beam with gravity → 9.81 kN/m load

### Task 5.4: Implement Load Combinations
- [ ] Combinations sum loads correctly
- [ ] Factors are applied correctly
- [ ] Multiple load cases combine properly

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
- [ ] For two-span continuous beam under torsion, bimoment is continuous at support
- [x] Total normal stress σ_total = σ_axial + σ_bending + σ_warping is computed correctly
- [x] For section with Iw = 0 (closed sections), bimoment results are zero
- [x] Sign convention consistent with standard references (Kollbrunner & Hajdin)
- [ ] Comparison with analytical solution for cantilever I-beam under torsion
- [x] All 16 boundary condition combinations produce correct results

### Task 7.2c: Displacement/Rotation Lines
- [ ] Displacements at element ends match nodal values exactly
- [ ] Deflection shape for cantilever with tip load matches analytical curve
- [ ] Rotation φ_z = dw/dy for Euler-Bernoulli beams
- [ ] For Timoshenko, φ_z ≠ dw/dy (shear deformation included)

### Task 7.2f: Multi-Element Beam Plotting
- [ ] Continuous moment diagram across 3-element beam matches hand calculation
- [ ] Element boundaries are clearly marked in plots
- [ ] Concentrated loads cause visible shear discontinuities
- [ ] Extrema are found and marked correctly across element boundaries
- [ ] Deflection diagram is smooth and continuous
- [ ] Works with beams of varying element counts (2 to 10+ elements)

### Task 7.3: Python Bindings for Internal Actions
- [x] InternalActions, EndForces, and DisplacementLine structs are accessible from Python
- [x] BeamElement methods are callable from Python
- [x] Methods accept Eigen arrays and return appropriate types
- [x] Type hints are provided for IDE support
- [x] Example usage demonstrates multi-element beam plotting
- [x] Unit tests verify Python bindings work correctly

---

## Phase 8: Additional Element Types

### Task 8.1: Implement Spring Element
- [ ] Spring provides correct stiffness between nodes
- [ ] Uncoupled DOFs are independent
- [ ] Eccentricity can be handled via rigid links

### Task 8.2: Implement Point Mass Element
- [ ] Point mass contributes to global mass matrix
- [ ] Inertia tensor is correctly represented
- [ ] Off-diagonal terms work for asymmetric masses

### Task 8.3: Implement Plate Element
- [ ] Plate deflects under pressure load
- [ ] Simple plate matches analytical solution
- [ ] Mesh refinement converges

---

## Phase 9: Cargo Modeling

### Task 9.1: Implement Cargo Definition
- [ ] Cargo definition is simple and clear
- [ ] Generated elements correctly represent the cargo
- [ ] Cargo mass contributes to inertial loads under acceleration

---

## Phase 10: Design Codes

### Task 10.1: Design Code Base Classes
- [ ] Base classes are defined
- [ ] Plugin structure allows multiple codes
- [ ] Check results include all required info

### Task 10.2: Implement Basic Checks
- [ ] At least basic checks are implemented
- [ ] Utilization is computed correctly
- [ ] Governing check is identified

---

## Phase 11: Error Handling & Diagnostics

### Task 11.1: Structured Error Codes
- [ ] Errors have machine-readable codes
- [ ] Errors have human-readable messages
- [ ] Diagnostic info (DOFs, elements) is included

### Task 11.2: Warning System
- [ ] Warnings don't block analysis
- [ ] Severity levels are assigned appropriately
- [ ] LLM can parse warning codes

### Task 11.3: Singularity Diagnostics
- [ ] Free-floating model detected as singular
- [ ] Specific unconstrained DOFs identified
- [ ] Helpful diagnostic message generated

---

## Phase 12: LLM Tooling

### Task 12.1: Docstrings and Type Hints
- [ ] All public functions have docstrings
- [ ] All parameters have type hints
- [ ] Units are documented
- [ ] mypy passes

### Task 12.2: Tool Schemas
- [ ] Tool schemas are valid JSON Schema
- [ ] Schemas are self-documenting
- [ ] Tools cover main modelling operations

### Task 12.3: Error Recovery Suggestions
- [ ] Each error type has fix suggestions
- [ ] Suggestions are actionable tool calls
- [ ] LLM can execute suggestions to fix model

---

## Phase 13: Validation Benchmarks

### Task 13.1: Basic Analytical Benchmarks
- [ ] Test passes
- [ ] Results within tolerance of analytical
- [ ] Clear reference to formula

### Task 13.2: Cantilever Beam Validation
- [ ] Tests pass
- [ ] Deflection matches PL³/(3EI)
- [ ] Reaction moment matches PL

### Task 13.3: Offset Beam Validation
- [ ] Offset beam gives correct deflection
- [ ] Forces transfer correctly through offset

### Task 13.4: Spring-Mass Validation
- [ ] Spring force = mass × g
- [ ] Forces balance correctly

### Task 13.5: Singularity Detection Validation
- [ ] UNCONSTRAINED_SYSTEM error returned
- [ ] Unconstrained DOFs identified

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
- `CLAUDE.md` - AI assistant guidelines
