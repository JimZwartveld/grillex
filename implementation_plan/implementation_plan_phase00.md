## Phase 0: Project Setup & Infrastructure

### Task 0.1: Initialize Project Structure
**Requirements:** R-ARCH-001, R-DEV-001, R-DEV-003
**Dependencies:** None
**Difficulty:** Low

**Description:**
Create the basic project directory structure.

**Steps:**
1. Create the following directory structure:
   ```
   grillex2/
   ├── grillex/
   │   ├── __init__.py
   │   ├── core/           # Python front-end core modules
   │   ├── io/             # YAML/JSON I/O
   │   ├── design_codes/   # Pluggable design code modules
   │   └── llm/            # LLM tooling layer
   ├── cpp/
   │   ├── include/        # C++ headers
   │   ├── src/            # C++ source files
   │   └── bindings/       # pybind11 bindings
   ├── tests/
   │   ├── cpp/            # C++ unit tests
   │   ├── python/         # Python unit tests
   │   └── benchmarks/     # Validation benchmarks
   ├── CMakeLists.txt
   ├── pyproject.toml
   ├── setup.py
   └── README.md
   ```

2. Create `pyproject.toml` with:
   - Python version requirement: >=3.11
   - Dependencies: numpy, scipy, pyyaml, pydantic
   - Build system: setuptools with cmake extension

3. Create root `CMakeLists.txt` with:
   - Minimum CMake version 3.20
   - C++17 standard
   - Find pybind11
   - Add subdirectories for cpp source

**Acceptance Criteria:**
- [x] Directory structure exists
- [x] `pip install -e .` runs without error (even if no C++ yet)
- [x] `import grillex` works

### Execution Notes (Completed 2025-12-08)

**Steps Taken:**
1. Created directory structure using `mkdir -p` commands:
   - `src/grillex/{core,io,design_codes,llm}/`
   - `cpp/{include/grillex,src,bindings}/`
   - `tests/{python,cpp,benchmarks}/`
2. Created `__init__.py` files for all Python subpackages
3. Updated `setup.cfg`:
   - Set `python_requires = >=3.11`
   - Added dependencies: numpy>=1.24.0, scipy>=1.10.0, pyyaml>=6.0, pydantic>=2.0
4. Installed package in development mode: `pip install -e .`

**Problems Encountered:**
- **Issue**: Initial confusion with working directory (was in grillex/build instead of grillex/)
  - **Solution**: Used `pwd` to verify location, navigated to correct directory
- **Issue**: New subpackages not immediately importable after creation
  - **Solution**: Reinstalled package with `pip install -e .` to update the editable install

**Verification:**
- All imports successful: `import grillex.{core,io,design_codes,llm}` ✓
- Package installed correctly with all dependencies
- Directory structure matches plan specification

**Time Taken:** ~10 minutes

---

### Task 0.2: Setup C++ Build Infrastructure
**Requirements:** R-DEV-003
**Dependencies:** Task 0.1
**Difficulty:** Medium

**Description:**
Configure CMake for C++ compilation with pybind11.

**Steps:**
1. In `cpp/CMakeLists.txt`:
   - Set up compilation flags for C++17
   - Find or fetch pybind11 (use FetchContent)
   - Create a placeholder shared library target `grillex_core`
   - Create pybind11 module target `_grillex_cpp`

2. Create `cpp/src/placeholder.cpp` with a simple test function

3. Create `cpp/bindings/bindings.cpp` with pybind11 module definition exposing the test function

4. Update `setup.py` or `pyproject.toml` to trigger CMake build

**Acceptance Criteria:**
- [x] `cmake -B build && cmake --build build` succeeds
- [x] Python can import the compiled module
- [x] Test function is callable from Python

### Execution Notes (Completed 2025-12-08)

**Steps Taken:**
1. Created root `CMakeLists.txt`:
   - Set C++17 standard
   - FetchContent for pybind11 v2.13.6
   - Find Eigen3 via system package manager
2. Created `cpp/CMakeLists.txt`:
   - Defined `grillex_core` interface library
   - Created pybind11 module `_grillex_cpp`
   - Set output directory to `src/grillex/`
3. Created placeholder C++ code:
   - `cpp/include/grillex/placeholder.hpp` (header with declarations)
   - `cpp/src/placeholder.cpp` (implementation with test functions)
   - `cpp/bindings/bindings.cpp` (pybind11 module definition)
4. Installed system dependencies:
   - CMake 4.2.0 via Homebrew
   - Eigen 5.0.1 via Homebrew

**Problems Encountered:**
- **Issue 1**: CMake not installed on system
  - **Solution**: Installed via `brew install cmake`

- **Issue 2**: Found system pybind11 with deprecated Python detection (distutils removed in Python 3.12+)
  - **Error**: `ModuleNotFoundError: No module named 'distutils'`
  - **Solution**: Used FetchContent to download compatible pybind11 v2.13.6 instead of system version

- **Issue 3**: Eigen version mismatch (installed 5.0.1, CMake looking for 3.4)
  - **Error**: `version: 5.0.1 - The version found is not compatible with the version requested`
  - **Solution**: Changed `find_package(Eigen3 3.4 REQUIRED)` to `find_package(Eigen3 REQUIRED NO_MODULE)` to accept any version

- **Issue 4**: C++ standard library headers not found during compilation
  - **Error**: `fatal error: 'cstddef' file not found`
  - **Root Cause**: Compiler not finding libc++ headers in macOS SDK
  - **Investigation**:
    - Verified headers exist at `/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/c++/v1/`
    - Tested manual compilation with explicit include path - worked
  - **Solution**: Added macOS-specific configuration to CMakeLists.txt:
    ```cmake
    if(APPLE)
        include_directories(SYSTEM /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/c++/v1)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    endif()
    ```

**Verification:**
- CMake configuration successful
- Build completed: `_grillex_cpp.cpython-313-darwin.so` created in `src/grillex/`
- Python import test: ✓
  ```python
  import grillex._grillex_cpp as cpp
  cpp.get_greeting()  # Returns "Hello from Grillex C++ core!"
  cpp.add_numbers(2, 3)  # Returns 5
  cpp.__version__  # Returns "2.0.0-dev"
  ```

**Key Learnings:**
- macOS Command Line Tools SDK path issues require explicit configuration
- FetchContent provides better control over dependency versions than system packages
- pybind11 2.13+ is required for Python 3.12+ compatibility

**Time Taken:** ~25 minutes (including troubleshooting)

---

### Task 0.3: Setup Testing Infrastructure
**Requirements:** R-VAL-004, R-DEV-004
**Dependencies:** Task 0.1
**Difficulty:** Low

**Description:**
Configure pytest for Python tests and optionally Catch2/GoogleTest for C++.

**Steps:**
1. Add pytest to dev dependencies
2. Create `tests/conftest.py` with common fixtures
3. Create `tests/python/test_placeholder.py` with a simple passing test
4. Configure `pyproject.toml` [tool.pytest.ini_options]

**Acceptance Criteria:**
- [x] `pytest` runs and reports 1 passing test
- [x] Test discovery works correctly

### Execution Notes (Completed 2025-12-08)

**Steps Taken:**
1. Verified existing pytest configuration in `setup.cfg`:
   - Test paths configured: `testpaths = tests`
   - Coverage enabled: `--cov grillex --cov-report term-missing`
2. Created comprehensive test file `tests/python/test_basic_imports.py`:
   - 5 test functions covering imports, C++ module, and placeholder functions
   - Tests for `get_greeting()` and `add_numbers()` functions
   - Version string verification
3. Installed testing dependencies:
   - Command: `pip install -e ".[testing]"`
   - Installed: pytest 9.0.2, pytest-cov 7.0.0, coverage 7.13.0

**Problems Encountered:**
- **Issue**: pytest not initially installed in virtual environment
  - **Solution**: Installed via `pip install -e ".[testing]"` which pulls from setup.cfg extras_require

**Verification:**
- All 7 tests passing:
  - 5 new tests in `test_basic_imports.py`
  - 2 existing tests in `test_skeleton.py`
- Test results:
  ```
  tests/python/test_basic_imports.py::test_grillex_imports PASSED
  tests/python/test_basic_imports.py::test_cpp_module_import PASSED
  tests/python/test_basic_imports.py::test_cpp_greeting PASSED
  tests/python/test_basic_imports.py::test_cpp_add_numbers PASSED
  tests/python/test_basic_imports.py::test_cpp_module_version PASSED
  tests/test_skeleton.py::test_fib PASSED
  tests/test_skeleton.py::test_main PASSED
  ================================ 7 passed in 0.43s ===============================
  ```
- Code coverage: 98% (42 statements, 1 miss)
- Coverage report shows all new modules covered

**Key Learnings:**
- Existing PyScaffold setup already had good pytest configuration
- Testing C++ bindings from Python is straightforward
- Coverage tracking helps ensure all code paths are exercised

**Time Taken:** ~8 minutes

---

