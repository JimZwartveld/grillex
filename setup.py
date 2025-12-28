"""
Setup file for grillex.

This file provides CMake-based build support for the C++ extension module.
The build process:
1. Runs CMake to configure and build the C++ extension
2. The extension is placed in src/grillex/
3. setuptools packages everything into a wheel

For development:
    pip install -e .[testing]

For building wheels:
    pip install build
    python -m build

This file was originally generated with PyScaffold 4.6.
Learn more under: https://pyscaffold.org/
"""

import os
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


def get_ext_suffix():
    """Get the platform-specific extension suffix."""
    import sysconfig
    ext_suffix = sysconfig.get_config_var("EXT_SUFFIX")
    if ext_suffix is None:
        if sys.platform == "win32":
            ext_suffix = ".pyd"
        else:
            ext_suffix = ".so"
    return ext_suffix


def extension_exists():
    """Check if the C++ extension already exists."""
    ext_suffix = get_ext_suffix()
    ext_path = Path(__file__).parent / "src" / "grillex" / ("_grillex_cpp" + ext_suffix)
    return ext_path.exists()


class CMakeExtension(Extension):
    """A custom Extension class that doesn't require sources."""
    def __init__(self, name, sourcedir=""):
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    """Custom build_ext command that runs CMake to build C++ extensions."""

    def run(self):
        """Run the CMake build process."""
        # Check for CMake
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the C++ extension. "
                "Install it with: pip install cmake"
            )

        # Build each extension
        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        """Build a single extension using CMake."""
        import shutil

        # Source directory (project root)
        source_dir = Path(ext.sourcedir) if ext.sourcedir else Path(__file__).parent.resolve()

        # Check if extension already exists in source tree (pre-built by CI)
        ext_suffix = get_ext_suffix()
        ext_filename = "_grillex_cpp" + ext_suffix
        existing_ext = source_dir / "src" / "grillex" / ext_filename

        if existing_ext.exists() and self.inplace:
            # For editable/inplace installs, skip build if extension exists
            print("Extension already exists at " + str(existing_ext) + ", skipping CMake build")
            return

        # Build directory
        build_dir = Path(self.build_temp).resolve()
        build_dir.mkdir(parents=True, exist_ok=True)

        # Output directory where setuptools expects the extension
        extdir = Path(self.get_ext_fullpath(ext.name)).parent.resolve()
        extdir.mkdir(parents=True, exist_ok=True)

        # CMake configuration
        cmake_args = [
            "-DCMAKE_BUILD_TYPE=Release",
            "-DPYTHON_EXECUTABLE=" + sys.executable,
        ]

        # Platform-specific settings
        if sys.platform == "win32":
            # Windows: specify generator and architecture
            cmake_args.extend([
                "-A", "x64",
            ])

        # Build arguments
        build_args = [
            "--config", "Release",
            "-j", str(os.cpu_count() or 1),
        ]

        # Configure
        print("CMake configure: " + str(source_dir) + " -> " + str(build_dir))
        subprocess.check_call(
            ["cmake", str(source_dir)] + cmake_args,
            cwd=build_dir,
        )

        # Build
        print("CMake build: " + str(build_dir))
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args,
            cwd=build_dir,
        )

        # Find and copy the built extension to the output directory
        # Possible locations where CMake might have put the extension
        possible_locations = [
            source_dir / "src" / "grillex" / ext_filename,
            build_dir / "cpp" / ext_filename,
            build_dir / "cpp" / "Release" / ext_filename,
            build_dir / ext_filename,
        ]

        ext_path = None
        for loc in possible_locations:
            if loc.exists():
                ext_path = loc
                break

        if ext_path is None:
            raise RuntimeError(
                "Could not find built extension. Searched in:\n" +
                "\n".join("  - " + str(loc) for loc in possible_locations)
            )

        # Copy to the setuptools output directory
        dest_path = extdir / ext_filename
        print("Copying extension: " + str(ext_path) + " -> " + str(dest_path))
        shutil.copy2(ext_path, dest_path)


# Only declare ext_modules if extension doesn't exist yet
# This avoids issues with setuptools editable wheel builds
if extension_exists():
    # Extension already built, just do a pure Python install
    setup(
        use_scm_version={"version_scheme": "no-guess-dev"},
    )
else:
    # Extension needs to be built
    setup(
        use_scm_version={"version_scheme": "no-guess-dev"},
        ext_modules=[CMakeExtension("grillex._grillex_cpp", sourcedir=str(Path(__file__).parent))],
        cmdclass={
            "build_ext": CMakeBuild,
        },
    )
