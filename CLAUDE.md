# CLAUDE.md - Grillex 2.0 AI Assistant Guide

This document provides comprehensive guidance for AI assistants working with the Grillex codebase.

## Task Execution Workflow

When executing tasks from implementation plan markdown files (`implementation_plan/*.md`):

### 1. Tick Off Completed Tasks

After completing each task or sub-step, update the markdown file to mark items as done:

```markdown
# Before
- [ ] Implement feature X
- [ ] Write tests for feature X

# After
- [x] Implement feature X
- [x] Write tests for feature X
```

### 2. Add Execution Summary

After completing a task, append an **Execution Notes** section below the task:

```markdown
### Execution Notes (Completed YYYY-MM-DD)

**Steps Taken:**
1. First action taken
2. Second action taken
3. ...

**Problems Encountered:**
- **Issue**: Description of the problem
  - **Error**: Exact error message (if applicable)
  - **Root Cause**: Why this happened
  - **Solution**: How it was resolved

- **Issue**: Another problem
  - **Solution**: How it was fixed

**Verification:**
- Test results: X tests passing ✓
- Manual verification steps performed
- Expected output confirmed

**Key Learnings:**
- Important insights for future reference
- Gotchas to remember

**Time Taken:** ~X minutes
```

### 3. Document All Problems and Solutions

Every problem encountered must be documented, even if trivial. This builds institutional knowledge:

- **Build failures**: Include exact error messages and fix
- **Test failures**: Document what failed and why
- **Configuration issues**: Platform-specific problems (especially macOS vs Linux)
- **Dependency issues**: Version conflicts, missing packages

This documentation helps future AI assistants (and humans) avoid the same pitfalls.

---

## Project Overview

**Grillex 2.0** is a structural analysis and design platform for offshore/heavy-lift structures (beams, plates, grillages, cargo on barges). It features:

- **C++ FE Core**: High-performance finite element implementation using Eigen
- **Python Front-End**: Pythonic API for modeling, analysis, and results
- **pybind11 Bindings**: Seamless C++/Python integration
- **YAML I/O**: Human-readable model definition format
- **LLM/Agent Support**: Designed for AI-assisted structural engineering

## Architecture

```
grillex/
├── cpp/                    # C++ finite element core
│   ├── include/grillex/    # C++ header files (.hpp)
│   ├── src/                # C++ implementation files (.cpp)
│   └── bindings/           # pybind11 bindings (bindings.cpp)
├── src/grillex/            # Python package
│   ├── core/               # Core data types and model wrapper
│   ├── io/                 # YAML loader and result writer
│   ├── design_codes/       # Pluggable design code modules
│   └── llm/                # LLM/agent tooling
├── tests/                  # Test suite
│   └── python/             # Python tests (pytest)
├── implementation_plan/    # Phased implementation documentation
└── docs/                   # Sphinx documentation
```

## Key Components

### C++ Core (`cpp/`)

| Component | Header | Description |
|-----------|--------|-------------|
| Node | `node.hpp` | 3D point with up to 7 DOFs (including warping) |
| NodeRegistry | `node_registry.hpp` | Node management with automatic merging |
| Material | `material.hpp` | Material properties (E, G, nu, rho) |
| Section | `section.hpp` | Cross-section properties (A, Iy, Iz, J) |
| LocalAxes | `local_axes.hpp` | Local coordinate system for beams |
| BeamElement | `beam_element.hpp` | 3D beam element (Euler-Bernoulli/Timoshenko, 12/14 DOF) |
| DOFHandler | `dof_handler.hpp` | Global DOF numbering and mapping |
| Assembler | `assembler.hpp` | Global matrix assembly |
| BCHandler | `boundary_condition.hpp` | Boundary condition handling |
| LinearSolver | `solver.hpp` | Linear system solver |
| LoadCase | `load_case.hpp` | Load case management |
| Model | `model.hpp` | Top-level orchestration |
| ConstraintHandler | `constraints.hpp` | MPC and rigid link constraints |

### Python Package (`src/grillex/`)

| Module | Description |
|--------|-------------|
| `core.data_types` | Re-exports C++ types via pybind11 |
| `core.model_wrapper` | High-level `StructuralModel` and `Beam` classes |
| `io.yaml_loader` | YAML model input parser |
| `io.result_writer` | Result export functionality |

## Units Convention

The system uses consistent SI-derived units:

| Quantity | Unit | Symbol |
|----------|------|--------|
| Length | meters | m |
| Force | kilonewtons | kN |
| Mass | metric tonnes | mT |
| Stress | kN/m² | kPa |
| Moment | kN·m | kNm |
| Distributed load | kN/m | kN/m |
| Acceleration | m/s² | m/s² |

**Example**: 1 mT/m self-weight = 9.81 kN/m under standard gravity.

## DOF Convention

Standard 6-DOF per node: `[UX, UY, UZ, RX, RY, RZ]`

Optional 7th DOF for warping: `[UX, UY, UZ, RX, RY, RZ, WARP]`

Beam element DOF ordering (12-DOF):
```
[u_i, v_i, w_i, θx_i, θy_i, θz_i, u_j, v_j, w_j, θx_j, θy_j, θz_j]
```

With warping (14-DOF):
```
[u_i, v_i, w_i, θx_i, θy_i, θz_i, φ'_i, u_j, v_j, w_j, θx_j, θy_j, θz_j, φ'_j]
```

## Sign Conventions

- **Axial force (N)**: Positive in tension
- **Shear force (V)**: Positive in positive local y/z direction
- **Bending moment**: Positive per right-hand rule
- **Local x-axis**: From node_i to node_j (along beam)
- **Local z-axis**: Aligned with global Z before rotations

## Development Workflow

### Building the C++ Extension

**Quick build (from project root):**
```bash
cmake -B build && cmake --build build
```

**Full rebuild (recommended after C++ changes):**
```bash
cd build && cmake .. && make -j4
```

**First-time setup:**
```bash
# Create build directory and configure
mkdir build && cd build
cmake ..

# Build (parallel compilation)
make -j4

# The _grillex_cpp.so module is placed in src/grillex/
```

**macOS-specific notes:**
- Requires CMake 3.20+ (`brew install cmake`)
- Requires Eigen (`brew install eigen`)
- The CMakeLists.txt includes macOS-specific SDK paths for libc++ headers
- If you get `'cstddef' file not found`, the SDK path configuration may need updating

**After modifying C++ code:**
```bash
# Always rebuild before running tests
cd build && cmake .. && make -j4

# Or from project root:
cmake --build build
```

**Common build issues:**
| Issue | Solution |
|-------|----------|
| `ModuleNotFoundError: No module named 'distutils'` | Uses FetchContent for pybind11 v2.13.6 (Python 3.12+ compatible) |
| Eigen version mismatch | CMake accepts any Eigen3 version via `NO_MODULE` |
| `'cstddef' file not found` (macOS) | SDK include paths configured in CMakeLists.txt |

### Running Tests

```bash
# Run all tests with pytest
pytest

# Run with coverage
pytest --cov grillex --cov-report term-missing

# Run specific test file
pytest tests/python/test_phase2_beam_element.py

# Run with verbose output
pytest -v

# Using tox
tox
```

### Installing for Development

```bash
pip install -e .[testing]
```

## Test Organization

Tests are organized by implementation phase:

| Test File | Phase | Coverage |
|-----------|-------|----------|
| `test_basic_imports.py` | - | Import verification |
| `test_phase1_data_structures.py` | 1 | Node, Material, Section |
| `test_phase2_beam_element.py` | 2 | Beam element matrices |
| `test_phase3_*.py` | 3 | Assembly, solver, DOF handling |
| `test_phase4_*.py` | 4 | Python wrapper, YAML, subdivision |
| `test_phase5_*.py` | 5 | Loads, accelerations, combinations |
| `test_phase6_constraints.py` | 6 | MPC, rigid links |
| `test_phase7_*.py` | 7 | Internal actions, end forces |
| `test_beam_factory.py` | - | Beam element factory |
| `test_warping_decoupling.py` | - | Warping DOF handling |

## Implementation Phases

The project follows a phased implementation plan (see `implementation_plan/`):

1. **Phase 0**: Project setup and infrastructure
2. **Phase 1**: Core data structures (Node, Material, Section)
3. **Phase 2**: Beam element foundation (stiffness/mass matrices)
4. **Phase 3**: Assembly and solver
5. **Phase 4**: Python front-end and I/O
6. **Phase 5**: Loads and load cases
7. **Phase 6**: MPC and rigid links
8. **Phase 7**: Internal actions and results (current focus)
9. **Phase 8**: Additional element types
10. **Phase 9**: Cargo modeling
11. **Phase 10-14**: Design codes, error handling, LLM tooling, validation, DevOps

## Beam Formulations

The system supports multiple beam formulations:

| Formulation | DOFs | Use Case |
|-------------|------|----------|
| Euler-Bernoulli | 12 | Slender beams (L/h > 10) |
| Timoshenko | 12 | Deep beams with shear deformation |
| With Warping | 14 | Thin-walled open sections |

Factory function: `create_beam_element(id, node_i, node_j, material, section, config, roll)`

## Code Style Guidelines

### Python
- Use type hints for all public functions
- Follow PEP 8 (max line length: 88, Black-compatible)
- Docstrings: NumPy/Google style
- Required: `numpy>=1.24.0`, `scipy>=1.10.0`, `pyyaml>=6.0`, `pydantic>=2.0`

### C++
- C++17 standard
- Use Eigen for linear algebra
- Header-only where practical
- Doxygen-style comments for public APIs

## Common Tasks

### Creating a Simple Model

```python
from grillex.core import StructuralModel, DOFIndex

# Create model
model = StructuralModel(name="Example")

# Add material and section
model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)

# Create beam
beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

# Apply boundary conditions
model.fix_node_at([0, 0, 0])

# Add load
model.add_point_load([6, 0, 0], DOFIndex.UY, -10.0)

# Analyze
model.analyze()

# Get results
disp = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
```

### Loading from YAML

```python
from grillex.io import load_model_from_yaml

model = load_model_from_yaml("model.yaml")
model.analyze()
```

### YAML Model Format

```yaml
name: "Example Model"

materials:
  - name: Steel
    E: 210000000      # kN/m²
    nu: 0.3
    rho: 7.85e-6      # mT/m³

sections:
  - name: IPE300
    A: 0.00538        # m²
    Iy: 8.36e-5       # m⁴
    Iz: 6.04e-6       # m⁴
    J: 2.01e-7        # m⁴

beams:
  - start: [0, 0, 0]
    end: [6, 0, 0]
    section: IPE300
    material: Steel

boundary_conditions:
  - node: [0, 0, 0]
    type: fixed

load_cases:
  - name: "Dead Load"
    type: Permanent
    loads:
      - node: [6, 0, 0]
        dof: UY
        value: -10.0
```

## Requirements Traceability

All features trace back to requirements in `grillex_requirements.md`:

- **R-ARCH-***: Architecture and vision
- **R-MOD-***: Geometry and modeling
- **R-DOF-***: DOFs, warping, constraints
- **R-COORD-***: Coordinate systems and sign conventions
- **R-UNITS-***: Units and tolerances
- **R-ELEM-***: Element types
- **R-LOAD-***: Loads and combinations
- **R-MPC-***: Constraints and rigid links
- **R-RES-***: Results and internal actions
- **R-CODE-***: Design codes
- **R-DATA-***: YAML I/O
- **R-LLM-***: LLM tooling

## Key Files for Understanding the Codebase

1. `grillex_requirements.md` - Full requirements specification
2. `implementation_plan/implementation_plan_overview.md` - Phase overview
3. `src/grillex/core/data_types.py` - All exposed C++ types
4. `src/grillex/core/model_wrapper.py` - High-level Python API
5. `cpp/include/grillex/beam_element.hpp` - Core beam element interface
6. `cpp/bindings/bindings.cpp` - pybind11 binding definitions

## Debugging Tips

1. **C++ module not found**: Rebuild with `cmake --build build`
2. **Import errors**: Ensure `_grillex_cpp.so` is in `src/grillex/`
3. **Matrix singularity**: Check boundary conditions are sufficient
4. **Wrong results**: Verify units (kN, m, mT) and sign conventions

## Current Development Status

The project is actively implementing **Phase 7: Internal Actions & Results**, which includes:
- Task 7.0: Distributed load query methods (completed)
- Task 7.1: Element end forces (completed)
- Task 7.2: Internal action computation (in progress)

See `implementation_plan/implementation_plan_phase07.md` for details.
