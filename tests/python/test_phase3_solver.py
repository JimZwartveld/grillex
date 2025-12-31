"""
Test suite for Phase 3: Linear Solver (Task 3.4)

Tests the LinearSolver class with multiple solver methods and validates:
1. Solver creation and configuration
2. Singularity detection for ill-conditioned systems
3. Error handling and reporting

NOTE: Tests that use BCHandler.apply_to_system have been moved to C++ tests
in tests/cpp/test_boundary_conditions.cpp because pybind11 cannot handle
Eigen::SparseMatrix as input parameters.

Remaining acceptance criteria tested here:
- AC2: Singular systems are detected and reported
"""

import pytest
import numpy as np
from scipy import sparse

from grillex.core import (
    Node, NodeRegistry, Material, Section,
    BeamElement, BeamConfig, BeamFormulation,
    DOFHandler, Assembler, BCHandler, DOFIndex,
    LinearSolver, SolverMethod
)


class TestLinearSolverBasics:
    """Test basic LinearSolver functionality"""

    def test_solver_creation_default(self):
        """Test LinearSolver creation with default method"""
        solver = LinearSolver()
        assert solver.get_method() == SolverMethod.SimplicialLDLT
        assert not solver.is_singular()
        assert solver.get_error_message() == ""

    def test_solver_creation_with_method(self):
        """Test LinearSolver creation with specified method"""
        solver_lu = LinearSolver(SolverMethod.SparseLU)
        assert solver_lu.get_method() == SolverMethod.SparseLU

        solver_ldlt = LinearSolver(SolverMethod.SimplicialLDLT)
        assert solver_ldlt.get_method() == SolverMethod.SimplicialLDLT

        solver_cg = LinearSolver(SolverMethod.ConjugateGradient)
        assert solver_cg.get_method() == SolverMethod.ConjugateGradient

    def test_solver_method_change(self):
        """Test changing solver method"""
        solver = LinearSolver(SolverMethod.SparseLU)
        solver.set_method(SolverMethod.SimplicialLDLT)
        assert solver.get_method() == SolverMethod.SimplicialLDLT

    def test_solver_repr(self):
        """Test __repr__ for solver"""
        solver = LinearSolver(SolverMethod.SimplicialLDLT)
        repr_str = repr(solver)
        assert "LinearSolver" in repr_str
        assert "SimplicialLDLT" in repr_str


class TestSingularityDetection:
    """Test detection of singular systems"""

    def test_singular_system_all_dofs_free(self):
        """Test singularity detection when no BCs are applied (rigid body modes)"""
        # Create simple model without any boundary conditions
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "Test", 0.12, 0.001, 0.0016, 0.0014)

        config = BeamConfig()
        elem = BeamElement(1, node1, node2, mat, sec, config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([elem])

        # No boundary conditions - system is singular (6 rigid body modes)
        F = np.zeros(dof_handler.total_dofs())

        solver = LinearSolver()
        u = solver.solve(K, F)

        # Should detect singularity
        assert solver.is_singular(), "Should detect singular system (no constraints)"
        error_msg = solver.get_error_message().lower()
        assert any(keyword in error_msg for keyword in ['singular', 'rigid', 'positive definite', 'decomposition']), \
            f"Error message should indicate singularity: '{solver.get_error_message()}'"


class TestIterativeSolverSettings:
    """Test iterative solver specific settings"""

    def test_set_solver_parameters(self):
        """Test setting solver parameters"""
        solver = LinearSolver(SolverMethod.ConjugateGradient)
        solver.set_max_iterations(500)
        solver.set_tolerance(1e-8)

        # Parameters are set, we can't query them directly but they'll be used in solve()
        # This test mainly checks that the methods don't throw


class TestAcceptanceCriteria:
    """Test all acceptance criteria for Task 3.4

    NOTE: AC1 (correct displacement) and AC3 (performance) tests have been
    moved to C++ tests in tests/cpp/test_boundary_conditions.cpp because
    pybind11 cannot handle Eigen::SparseMatrix as input parameters.
    """

    def test_ac2_singular_systems_detected(self):
        """AC2: Singular systems are detected and reported"""
        # Create system without boundary conditions
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(1.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "Test", 0.01, 1e-5, 1e-5, 1e-5)

        config = BeamConfig()
        elem = BeamElement(1, node1, node2, mat, sec, config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([elem])

        F = np.zeros(dof_handler.total_dofs())

        solver = LinearSolver()
        u = solver.solve(K, F)

        # Must detect singularity
        assert solver.is_singular(), "Solver should detect singular system"
        error_msg = solver.get_error_message()
        assert len(error_msg) > 0, "Solver should provide error message"
        assert any(keyword in error_msg.lower() for keyword in ['singular', 'rigid', 'decomposition']), \
            f"Error message should mention singularity: '{error_msg}'"


class TestErrorHandling:
    """Test error handling for invalid inputs"""

    def test_dimension_mismatch_k_not_square(self):
        """Test error when K is not square"""
        K_bad = sparse.csr_matrix(np.random.rand(10, 8))  # Not square
        F = np.random.rand(10)

        solver = LinearSolver()
        u = solver.solve(K_bad, F)

        assert solver.is_singular()
        assert "not square" in solver.get_error_message().lower()

    def test_dimension_mismatch_k_f_incompatible(self):
        """Test error when K and F dimensions don't match"""
        K = sparse.csr_matrix(np.eye(10))
        F = np.random.rand(8)  # Wrong size

        solver = LinearSolver()
        u = solver.solve(K, F)

        assert solver.is_singular()
        assert "mismatch" in solver.get_error_message().lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
