# Grillex 2.0 – Implementation Plan

## Overview

This document provides a detailed, step-by-step implementation plan for building Grillex 2.0, a structural analysis and design platform. The plan is structured so that individual tasks can be executed by agents with varying levels of expertise.

**Key Principles:**
- Each task has clear inputs, outputs, and acceptance criteria
- Tasks reference specific requirements (R-ID)
- Dependencies between tasks are explicitly stated
- Tasks are ordered to enable incremental testing

---


## Phases Overview

The implementation plan is organized into 19 phases:

### Phase 0: Project Setup & Infrastructure
**Requirements:** R-ARCH-001, R-DEV-001, R-DEV-003 **Dependencies:** None **Difficulty:** Low **Description:** Create the basic project directory structure.

→ [See full Phase 0 details](implementation_plan_phase00.md)

### Phase 1: Core Data Structures (C++)
**Requirements:** R-MOD-003, R-DOF-001 **Dependencies:** Task 0.2 **Difficulty:** Medium **Description:** Create the C++ Node class representing a point in the structural model.

→ [See full Phase 1 details](implementation_plan_phase01.md)

### Phase 2: Beam Element Foundation (C++)
**Requirements:** R-COORD-001, R-COORD-002, R-COORD-003 **Dependencies:** Task 1.1 **Difficulty:** High **Description:** Create a class to compute local beam coordinate systems.

→ [See full Phase 2 details](implementation_plan_phase02.md)

### Phase 3: Assembly & Solver
**⚠️ UPDATED FOR WARPING DOF SUPPORT ⚠️** The following tasks have been updated to handle both 12-DOF and 14-DOF elements. **Requirements:** R-DOF-005, R-DOF-001, R-DOF-002, R-DOF-007 (warping) **Depe

→ [See full Phase 3 details](implementation_plan_phase03.md)

### Phase 4: Python Front-End & I/O
**Requirements:** R-MOD-001, R-MOD-002, R-ARCH-003 **Dependencies:** Task 3.5 **Difficulty:** Medium **Description:** Create Pythonic wrapper around C++ Model class.

→ [See full Phase 4 details](implementation_plan_phase04.md)

### Phase 5: Loads & Load Cases
**Requirements:** R-LOAD-001, R-LOAD-002, R-LOAD-010 **Dependencies:** Task 4.1 **Difficulty:** Medium **Description:** Create the load case and load combination structures.

→ [See full Phase 5 details](implementation_plan_phase05.md)

### Phase 6: MPC & Rigid Links
**Requirements:** R-MPC-001, R-MPC-002 **Dependencies:** Task 3.2 **Difficulty:** High **Description:** Implement the constraint transformation matrix T.

→ [See full Phase 6 details](implementation_plan_phase06.md)

### Phase 7: Internal Actions & Results
**Requirements:** R-RES-001
**Dependencies:** Task 3.5 (required), Phase 5 Tasks 5.1-5.2 (recommended for full functionality)
**Difficulty:** High
**Description:** Compute element end forces and internal actions (moment, shear, normal force, torsion) along beam elements using **differential equation approach** with analytical closed-form solutions. Supports both Euler-Bernoulli and Timoshenko beam theories with release-specific formulas for all boundary condition combinations.

**Key Features:**
- **Differential Equation Methodology:** Uses analytical solutions to beam equilibrium differential equations (similar to pystructeng) rather than simple shape function interpolation
- **Release-Specific Formulas:** Implements separate analytical formulas for each end release combination (16 for bending, 4 for axial/torsion)
- **Multi-Element Beam Plotting:** Python `Beam` class aggregates results from multiple `BeamElement` objects to provide continuous internal action diagrams across element boundaries
- **Distributed Load Support:** Accounts for linearly varying distributed loads q(x) in internal action computation
- **Beam Theories:** Supports both Euler-Bernoulli (classical) and Timoshenko (includes shear deformation) formulations
- **Warping/Bimoment:** For 14-DOF elements with warping restraint, computes bimoment distribution and warping stresses
- **Visualization:** Matplotlib integration for plotting moment, shear, and force diagrams with extrema marking

**Implementation Phasing:**
- **Phase 7a:** Basic end forces using `f = K*u` (no Phase 5 dependency)
- **Phase 7b:** Enhanced with distributed load support (requires Phase 5)
- **Phase 7c:** Full differential equation approach with all release combinations

**Note:** Phase 7 has a **critical dependency** on Phase 5 (distributed loads) for accurate internal action computation. Without Phase 5, internal actions will be limited to linear interpolation between end forces, which is inaccurate for beams with distributed loads. See Phase 5 documentation for detailed dependency discussion.

→ [See full Phase 7 details](implementation_plan_phase07.md)

### Phase 8: Additional Element Types
**Requirements:** R-MOD-007, R-ELEM-007 **Dependencies:** Task 3.2 **Difficulty:** Medium **Description:** Create spring element connecting two nodes.

→ [See full Phase 8 details](implementation_plan_phase08.md)

### Phase 9: Cargo Modelling
**Requirements:** R-CARGO-001, R-CARGO-002, R-CARGO-003 **Dependencies:** Tasks 8.1, 8.2, 6.2 **Difficulty:** Medium **Description:** Create the Cargo Python-level abstraction.

→ [See full Phase 9 details](implementation_plan_phase09.md)

### Phase 10: Design Codes (Plugin Structure)
**Requirements:** R-CODE-001, R-CODE-002 **Dependencies:** Task 7.2 **Difficulty:** Medium **Description:** Create the pluggable design code module structure.

→ [See full Phase 10 details](implementation_plan_phase10.md)

### Phase 11: Error Handling & Diagnostics
**Requirements:** R-ERR-001, R-ERR-002 **Dependencies:** Task 3.4 **Difficulty:** Medium **Description:** Create structured error reporting.

→ [See full Phase 11 details](implementation_plan_phase11.md)

### Phase 12: LLM Tooling
**Requirements:** R-LLM-001 **Dependencies:** All previous Python tasks **Difficulty:** Medium **Description:** Ensure all Python functions have complete type hints and docstrings.

→ [See full Phase 12 details](implementation_plan_phase12.md)

### Phase 13: Validation & Benchmarks
**Requirements:** R-VAL-001, R-VAL-002 **Dependencies:** Phase 5 complete **Difficulty:** Low **Description:** Create validation benchmark for simply supported beam.

→ [See full Phase 13 details](implementation_plan_phase13.md)

### Phase 14: DevOps & Packaging
**Requirements:** R-DEV-001, R-DEV-002 **Dependencies:** All phases **Difficulty:** Medium **Description:** CI/CD pipeline, documentation, and packaging.

→ [See full Phase 14 details](implementation_plan_phase14.md)

### Phase 15: Nonlinear Springs
**Requirements:** R-ELEM-007 (extended) **Dependencies:** Phase 8 **Difficulty:** High **Description:** Implement tension-only, compression-only, and gap springs with iterative nonlinear solver. Supports static→dynamic load sequencing for proper gap closure behavior.

→ [See full Phase 15 details](implementation_plan_phase15.md)

### Phase 16: Eigenvalue Analysis (Modal/Dynamic)
**Requirements:** R-ASM-006 (extensibility) **Dependencies:** Phase 3, Phase 8 **Difficulty:** High **Description:** Implement eigenvalue analysis for computing natural frequencies and mode shapes. Includes subspace iteration solver, participation factors, and effective modal mass calculations.

→ [See full Phase 16 details](implementation_plan_phase16_eigenvalue.md)

### Phase 17: Web Application Interface
**Requirements:** R-ARCH-003, R-LLM-001, R-LLM-002 **Dependencies:** Phase 12 **Difficulty:** High **Description:** Web-based dashboard with React frontend, Three.js 3D viewer, FastAPI backend, and Claude API integration. Features include collapsible panel layout, realistic beam section rendering, FEM/Results views, natural language commands via chat, and Docker deployment. Both UI and LLM use the same ToolExecutor API.

→ [See full Phase 17 details](implementation_plan_phase17.md)

### Phase 18: WebApp UX Improvements & Model Alignment
**Requirements:** User feedback from Phase 17 **Dependencies:** Phase 17 **Difficulty:** High **Description:** UX improvements based on user testing feedback. Key changes include: Z-up coordinate system, right-click context menus for all element creation/editing, cargo and load combination support, active load case selection, flexible support DOF configuration, model invalidation on changes, searchable settings panel, and aligning frontend types directly with C++ Model class structure.

→ [See full Phase 18 details](implementation_plan_phase18.md)

### Phase 19: Plate Meshing & Plate-Beam Coupling
**Requirements:** R-ELEM-004, R-MESH-001, R-MPC-003 **Dependencies:** Phase 8, Phase 6 **Difficulty:** High **Description:** Comprehensive plate meshing using gmsh with quad preference. Supports general polygons (3+ corners), higher-order elements (MITC8, MITC9), triangular fallback (DKT), mesh size and per-edge division control, plate-beam coupling via rigid links with DOF releases and eccentric offsets, support curves along plate edges, and a unified mesh() function for plates and beams.

→ [See full Phase 19 details](implementation_plan_phase19_plate_meshing.md)

