
# Grillex 2.0 – Requirements Specification (R-ID Style)

## R-ARCH – Vision, Scope & Architecture

**R-ARCH-001**  
The system shall provide a structural analysis & design platform (“Grillex 2.0”) consisting of a C++ FE core, a Python front-end, optional web viewer, and LLM/agent control.

**R-ARCH-002**  
The C++ core shall implement element formulations, global matrix assembly, constraint handling (MPC), and numerical solvers.

**R-ARCH-003**  
The Python front-end shall provide modelling APIs, YAML I/O, orchestration of analyses, and integration with LLM/agent tooling.

**R-ARCH-004**  
C++ functionality shall be exposed to Python via pybind11 (or equivalent), keeping a stable Python API independent of C++ internals.

**R-ARCH-005**  
The architecture shall be extensible to support new element types, design codes, and post-processing routines without breaking existing APIs.

**R-ARCH-006**  
The system shall target offshore / heavy-lift structural problems (beams, plates, grillages, cargo on barges, etc.).

---

## R-MOD – Geometry & Front-End Modelling

**R-MOD-001**  
The Python API shall allow beam creation using end positions `EndAPosition` and `EndBPosition`, each as `[x, y, z]`.

**R-MOD-002**  
The Python API shall optionally allow beams to be defined using explicit node references (e.g. `node_i`, `node_j`).

**R-MOD-003**  
Internally, the model shall maintain explicit Node objects with unique IDs for connectivity, DOF mapping, and BC attachment.

**R-MOD-004**  
A node registry shall reuse existing nodes when a new position matches an existing node within a configurable distance tolerance.

**R-MOD-005**  
If a beam is defined between EndA and EndB, the system shall detect existing nodes lying between those endpoints (considering offsets) and subdivide the beam into multiple finite elements at those nodes.

**R-MOD-006**  
The system shall support plate/shell elements with at least 4 nodes, each assigned coordinates, thickness, and material.

**R-MOD-007**  
The system shall support spring elements between nodes with translational and rotational stiffness components.

**R-MOD-008**  
The system shall support point-mass elements representing a 6×6 mass/inertia matrix at a node.

---

## R-DOF – DOFs, Warping & Constraints

**R-DOF-001**  
Global DOFs shall be node-based and include UX, UY, UZ, RX, RY and RZ (6 DOFs per node).

**R-DOF-002**  
Only nodal DOFs UX, UY, UZ, RX, RY and RZ shall receive global DOF numbers and appear in global matrices.

**R-DOF-003**  
The warping DOF (7th DOF per beam formulation) shall be treated as an element-internal DOF and shall not receive a global DOF number.

**R-DOF-004**  
Element-level warping DOFs shall be condensed or eliminated such that only their effect on nodal DOFs 1–6 appears in the assembled global matrices.

**R-DOF-005**  
The system shall maintain a global DOF map from node/DOF indices to global equation numbers.

**R-DOF-006**  
The system shall support fixed DOFs, prescribed displacements, and general constraints via multi-point constraints (MPC).

---

## R-COORD – Coordinate Systems & Sign Conventions

**R-COORD-001**  
Each beam shall have a local coordinate system with x from EndA to EndB, z vertical (aligned with global Z before rotations), and y forming a right-handed system.

**R-COORD-002**  
The local beam axes shall be constructed by rotating the global system first about the global Z-axis, then about the resulting local Y-axis to align local x from EndA to EndB.

**R-COORD-003**  
The user shall be able to specify an additional rotation about the local x-axis to orient strong/weak axes relative to global directions.

**R-COORD-004**  
At EndB, internal forces and moments acting in the positive local axis directions shall be considered positive; at EndA, forces/moments in the opposite directions shall be considered positive.

**R-COORD-005**  
Axial force N shall be positive in tension.

**R-COORD-006**  
Positive My (bending about local y) shall cause tension in +Z fibres and compression in −Z fibres.

**R-COORD-007**  
Positive Mz (bending about local z) shall cause tension in −Y fibres and compression in +Y fibres.

**R-COORD-008**  
These axis definitions and sign conventions shall be documented explicitly at the beginning of the user documentation.

---

## R-UNITS – Units & Tolerances

**R-UNITS-001**  
The system’s base units shall be meters (m) for length, kilonewtons (kN) for force, and metric tonnes (mT) for mass.

**R-UNITS-002**  
Accelerations shall be expressed in m/s², moments in kNm, distributed loads in kN/m, and mass per length in mT/m.

**R-UNITS-003**  
All YAML input shall assume these base units; no implicit unit conversions shall be applied.

**R-UNITS-004**  
The chosen unit system shall be clearly documented, including at least one worked example (e.g. 1 mT/m → 9.81 kN/m under gravity).

**R-UNITS-005**  
The node-merging tolerance shall be configurable and shall be used consistently when determining whether two node positions are identical.

---

## R-ELEM – Elements (Beams, Plates, Springs, Masses)

**R-ELEM-001**  
Beam elements shall support 3D behaviour including axial, bending about two local axes, and torsion.

**R-ELEM-002**  
Beam elements shall have consistent mass matrices suitable for inertial and dynamic loading.

**R-ELEM-003**  
Beam elements shall support end offsets relative to their connection nodes, and their length shall be computed from offset-adjusted end positions.

**R-ELEM-004**  
Beam stiffness and mass matrices shall correctly incorporate end offsets.

**R-ELEM-005**  
Beam elements shall support end releases (e.g. bending moments, torsion) implemented via DOF transformations or reduced stiffness blocks, not via artificial low stiffness.

**R-ELEM-006**  
Plate/shell elements shall support out-of-plane bending and optionally in-plane membrane behaviour.

**R-ELEM-007**  
Spring elements shall support translational and rotational stiffness between two nodes and optionally include geometric eccentricities.

**R-ELEM-008**  
Point-mass elements shall represent mass and inertia with a 6×6 mass matrix attached to a node.

---

## R-CARGO – Cargo Modelling

**R-CARGO-001**  
Cargo shall be modelled as a Python-level abstraction composed of a point-mass node plus connecting springs and optional rigid links.

**R-CARGO-002**  
A Cargo object shall define a CoG position, mass, inertia (6×6), and connections to structural nodes via springs.

**R-CARGO-003**  
The C++ core shall not define a dedicated “Cargo element”; instead it shall rely on generic point-mass and spring elements and MPC/rigid link infrastructure.

---

## R-LOAD – Loads, Acceleration Fields & Combinations

**R-LOAD-001**  
The system shall support classic loads: nodal forces/moments, line loads on beams, and surface/pressure loads on plates.

**R-LOAD-002**  
Each LoadCase shall be assigned a type: permanent, variable, environmental, or accidental.

**R-LOAD-003**  
Each LoadCase may define a 6-DOF acceleration field at a reference point (`ax, ay, az, arx, ary, arz` plus reference coordinates).

**R-LOAD-004**  
For each node (including offset locations), the system shall compute acceleration from the LoadCase’s acceleration field.

**R-LOAD-005**  
For each element, the inertial load vector shall be computed as `f_inertial_e = - M_e * a_e`, where `a_e` stacks the nodal accelerations and `M_e` is the consistent mass matrix.

**R-LOAD-006**  
The inertial load vectors shall be assembled into the global load vector as element load contributions.

**R-LOAD-007**  
Offsets shall be considered when computing nodal accelerations such that vertical accelerations do not produce spurious lateral forces.

**R-LOAD-008**  
For internal action and moment diagram evaluation, elements shall reconstruct a continuous body load `q(x)` from density, section properties and the acceleration field, e.g. `q(x) = ρA a_z(x)`.

**R-LOAD-009**  
Internal action computation shall reproduce standard closed-form results, including `M_mid = wL²/8` for a simply supported beam with uniform load w.

**R-LOAD-010**  
A LoadCombination shall be defined as a name/ID plus a list of (LoadCase, factor) pairs.

**R-LOAD-011**  
Design code modules shall accept LoadCombinations and use base LoadCase ResultCases with factors to perform code checks.

---

## R-MPC – Constraints, MPC & Rigid Links

**R-MPC-001**  
Constraints between DOFs shall be implemented via a transformation matrix `T` such that `u_full = T * u_reduced`.

**R-MPC-002**  
The transformation matrix `T` shall support master/slave displacement equality and rigid links with eccentricity.

**R-MPC-003**  
For rigid links, the following relation shall hold: `u_S = u_M + θ_M × r_MS` and `θ_S = θ_M`.

**R-MPC-004**  
Global reduction shall be performed via `K_r = Tᵀ K T` and `F_r = Tᵀ F`, solving for `u_r` and then recovering `u_full`.

---

## R-ASM – Assembly & Solvers

**R-ASM-001**  
The system shall assemble a global stiffness matrix K from all elements and constraints.

**R-ASM-002**  
The system shall assemble a global mass matrix M from element mass matrices.

**R-ASM-003**  
The system shall assemble a global load vector F from nodal loads, element loads, and inertial contributions.

**R-ASM-004**  
Linear static analyses shall solve the reduced system `K_r * u_r = F_r` after MPC and boundary condition application.

**R-ASM-005**  
The system shall reconstruct full displacement vectors, including constrained DOFs, after solving.

**R-ASM-006**  
The architecture shall not preclude extension to eigenvalue, modal/dynamic, or nonlinear analyses in the future.

---

## R-RES – Results & Internal Actions

**R-RES-001**  
For each LoadCase (or combination), the system shall provide a ResultCase containing nodal displacements, reactions, and element end forces.

**R-RES-002**  
Beam internal actions (N, V, M, T, and bimoment where applicable) shall be computed using pre-solved differential equations / shape functions rather than simple interpolation.

**R-RES-003**  
For each beam element and each internal action component, the ResultCase shall store the location(s) and value(s) of minimum and maximum along the element.

**R-RES-004**  
The API shall allow querying internal actions at arbitrary positions along an element, e.g. `get_actions(element_id, x)`.

**R-RES-005**  
The system shall support explicit “check locations” on beams where internal actions are extracted for design code checks.

---

## R-CODE – Design Codes & Strength Checks

**R-CODE-001**  
Design code modules (e.g. DNV, NORSOK, Eurocode, AISC/API, ISO) shall be pluggable and separate from the core solver.

**R-CODE-002**  
Each design code module shall define required input parameters, load combination rules, and utilization calculations for relevant failure modes.

**R-CODE-003**  
Design code checks shall be performed at user-defined check locations on beams using internal actions from ResultCases.

**R-CODE-004**  
The system shall store, per check location, the governing load combination, governing check, and resulting utilization.

---

## R-DATA – Data, YAML Input & Result Output

**R-DATA-001**  
Structural models shall be serializable to and from YAML as the primary input format.

**R-DATA-002**  
YAML shall use top-level keys for class names (e.g. Beam, Node, Support, Plate, Cargo, LoadCase), each mapping to a list of object dictionaries.

**R-DATA-003**  
Each class shall define defaults for all properties; serialization shall emit only non-default values.

**R-DATA-004**  
YAML structure and key names shall be LLM-friendly (explicit, descriptive, shallow hierarchies).

**R-DATA-005**  
YAML shall be used for input models only; solver results shall not be written back into the input YAML by default.

**R-DATA-006**  
Result data shall be available in a separate, structured output format (typically JSON) suitable for LLMs, APIs and UIs.

---

## R-VAL – Validation & Benchmarks

**R-VAL-001**  
The system shall include benchmark models with known reference solutions (analytic or trusted FE software) for validation.

**R-VAL-002**  
Benchmarks shall cover at least: simply supported beam with uniform load, cantilever with tip load, beams with offsets and internal nodes, a cargo-on-springs model under acceleration, and a small plate/grillage model.

**R-VAL-003**  
Benchmarks shall include at least one intentionally unconstrained model to test rigid-body mode detection.

**R-VAL-004**  
All benchmarks shall be runnable in an automated test suite and used in CI to detect regressions.

---

## R-ERR – Errors, Warnings & Diagnostics

**R-ERR-001**  
Back-end operations shall distinguish between errors (blocking) and warnings (non-blocking).

**R-ERR-002**  
Errors such as `UNCONSTRAINED_SYSTEM`, `SINGULAR_MATRIX`, and `INVALID_PROPERTY` shall be reported with machine-readable codes and human-readable messages.

**R-ERR-003**  
Warnings about questionable models (e.g. extreme aspect ratios, near singularity, extreme stiffness contrasts) shall be reported with codes, messages, and severity levels.

**R-ERR-004**  
Error and warning reports shall optionally include diagnostic information (e.g. involved DOFs, elements, rigid-body modes).

---

## R-LLM – LLM Tooling & Self-Healing

**R-LLM-001**  
Python functions intended for LLM/tool use shall have complete type hints and docstrings describing inputs, outputs, units, and semantics.

**R-LLM-002**  
The LLM tooling layer (e.g. MCP server) shall expose tools with explicit JSON schemas (or equivalent) for parameters and return values.

**R-LLM-003**  
Tools shall be self-describing such that an LLM can infer how to call them without external documentation.

**R-LLM-004**  
The system shall support a loop where an LLM reads YAML models, modifies them, reruns analyses, and inspects JSON results and diagnostic errors/warnings.

**R-LLM-005**  
The design shall enable LLMs to interpret errors (e.g. unconstrained system) and apply minimal model changes (such as adding supports) to resolve them before re-running analyses.

---

## R-DEV – DevOps, Packaging & Platforms

**R-DEV-001**  
The system shall target the latest stable Python versions (e.g. 3.11/3.12) and use the latest compatible NumPy and SciPy versions, with SciPy used only where necessary.

**R-DEV-002**  
Binary wheels shall be built for Windows (win_amd64) and Linux (manylinux) with toolchains compatible with GLIBCXX_3.4.30.

**R-DEV-003**  
The build system shall be based on CMake and pybind11 for the C++/Python bridge.

**R-DEV-004**  
CI pipelines shall build and test wheels for the supported platforms and run the benchmark validation suite.

---

## R-NFR – Non-Functional & Deployment

**R-NFR-001**  
The system shall be performant for realistic offshore structures with many elements and multiple load cases, using sparse storage and solvers.

**R-NFR-002**  
Given the same YAML input model and design settings, the system shall produce deterministic and reproducible results.

**R-NFR-003**  
The system shall record code versions, material/section library versions, and design code settings used in an analysis.

**R-NFR-004**  
The system shall be deployable on-premise within a corporate network, without requiring public internet access for core functionality.
