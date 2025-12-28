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

from setuptools import setup
from setuptools.command.build_ext import build_ext
from setuptools.command.build_py import build_py


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

        # Build directory
        build_dir = Path(self.build_temp).resolve()
        build_dir.mkdir(parents=True, exist_ok=True)

        # Source directory (project root)
        source_dir = Path(__file__).parent.resolve()

        # CMake configuration
        cmake_args = [
            f"-DCMAKE_BUILD_TYPE=Release",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
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
        print(f"CMake configure: {source_dir} -> {build_dir}")
        subprocess.check_call(
            ["cmake", str(source_dir)] + cmake_args,
            cwd=build_dir,
        )

        # Build
        print(f"CMake build: {build_dir}")
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args,
            cwd=build_dir,
        )

        # The built extension should now be in src/grillex/
        ext_suffix = self._get_extension_suffix()
        ext_path = source_dir / "src" / "grillex" / f"_grillex_cpp{ext_suffix}"

        if not ext_path.exists():
            # Check alternative locations
            for alt_path in [
                build_dir / "cpp" / f"_grillex_cpp{ext_suffix}",
                build_dir / f"_grillex_cpp{ext_suffix}",
            ]:
                if alt_path.exists():
                    import shutil
                    shutil.copy2(alt_path, ext_path)
                    break
            else:
                raise RuntimeError(
                    f"Could not find built extension. "
                    f"Expected at: {ext_path}"
                )

        print(f"Extension built: {ext_path}")

    def _get_extension_suffix(self):
        """Get the platform-specific extension suffix."""
        import sysconfig
        ext_suffix = sysconfig.get_config_var("EXT_SUFFIX")
        if ext_suffix is None:
            if sys.platform == "win32":
                ext_suffix = ".pyd"
            else:
                ext_suffix = ".so"
        return ext_suffix


class CustomBuildPy(build_py):
    """Custom build_py that ensures C++ extension is built first."""

    def run(self):
        # Build C++ extension first
        self.run_command("build_ext")
        # Then run the standard build_py
        super().run()


if __name__ == "__main__":
    try:
        setup(
            use_scm_version={"version_scheme": "no-guess-dev"},
            cmdclass={
                "build_ext": CMakeBuild,
                "build_py": CustomBuildPy,
            },
        )
    except Exception as e:
        print(
            f"\nAn error occurred while building the project:\n{e}\n\n"
            "Please ensure you have the most updated version of setuptools, "
            "setuptools_scm, wheel, and cmake:\n"
            "   pip install -U setuptools setuptools_scm wheel cmake\n\n"
            "Also ensure you have a C++ compiler installed:\n"
            "   - Linux: apt-get install build-essential\n"
            "   - macOS: xcode-select --install\n"
            "   - Windows: Install Visual Studio Build Tools\n",
            file=sys.stderr,
        )
        raise
