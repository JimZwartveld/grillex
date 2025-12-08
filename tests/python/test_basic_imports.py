"""
Test basic imports and C++ module functionality.
"""
import pytest


def test_grillex_imports():
    """Test that all main packages can be imported."""
    import grillex
    import grillex.core
    import grillex.io
    import grillex.design_codes
    import grillex.llm

    assert grillex is not None


def test_cpp_module_import():
    """Test that the C++ module can be imported."""
    import grillex._grillex_cpp as cpp

    assert cpp is not None
    assert hasattr(cpp, 'get_greeting')
    assert hasattr(cpp, 'add_numbers')
    assert hasattr(cpp, '__version__')


def test_cpp_greeting():
    """Test the C++ greeting function."""
    import grillex._grillex_cpp as cpp

    greeting = cpp.get_greeting()
    assert isinstance(greeting, str)
    assert "Grillex" in greeting
    assert "C++" in greeting


def test_cpp_add_numbers():
    """Test the C++ add_numbers function."""
    import grillex._grillex_cpp as cpp

    result = cpp.add_numbers(2, 3)
    assert result == 5

    result = cpp.add_numbers(10, -5)
    assert result == 5

    result = cpp.add_numbers(0, 0)
    assert result == 0


def test_cpp_module_version():
    """Test that the C++ module has a version string."""
    import grillex._grillex_cpp as cpp

    version = cpp.__version__
    assert isinstance(version, str)
    assert len(version) > 0
