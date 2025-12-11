"""
Phase 3: Test Suite for Global Matrix Assembly (Task 3.2)

Tests the Assembler class which assembles global stiffness and mass matrices
from element matrices. Handles both 12-DOF and 14-DOF elements automatically.
"""

import numpy as np
import pytest
from grillex.core import (
    NodeRegistry, Material, Section, BeamElement, BeamConfig,
    DOFHandler, Assembler
)


class TestAssembler:
    """Basic tests for Assembler class functionality"""

    def test_assembler_creation(self):
        """Test creating an Assembler with a DOFHandler"""
        registry = NodeRegistry()
        dof_handler = DOFHandler()

        # Create assembler
        assembler = Assembler(dof_handler)

        # Check that assembler was created
        assert assembler is not None
        assert assembler.get_dof_handler() is dof_handler

    def test_assemble_single_12dof_element_stiffness(self):
        """Test assembling stiffness matrix for a single 12-DOF element"""
        # Create simple beam
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.0053, 8.36e-5, 6.04e-6, 1.2e-5)

        # Create 12-DOF beam (default)
        beam = BeamElement(1, node1, node2, mat, sec)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Assemble
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beam])

        # Should be 12x12 (2 nodes × 6 DOFs)
        assert K_global.shape == (12, 12)

        # Global matrix should match element matrix (no DOF transformations)
        K_elem = beam.global_stiffness_matrix()
        K_global_dense = K_global.todense()

        # Check all entries match
        np.testing.assert_allclose(K_global_dense, K_elem, rtol=1e-10)

    def test_assemble_single_12dof_element_mass(self):
        """Test assembling mass matrix for a single 12-DOF element"""
        # Create simple beam
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.0053, 8.36e-5, 6.04e-6, 1.2e-5)

        # Create 12-DOF beam
        beam = BeamElement(1, node1, node2, mat, sec)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Assemble
        assembler = Assembler(dof_handler)
        M_global = assembler.assemble_mass([beam])

        # Should be 12x12
        assert M_global.shape == (12, 12)

        # Global matrix should match element matrix
        M_elem = beam.global_mass_matrix()
        M_global_dense = M_global.todense()

        # Check all entries match
        np.testing.assert_allclose(M_global_dense, M_elem, rtol=1e-10)

    def test_assemble_single_14dof_element_stiffness(self):
        """Test assembling stiffness matrix for a single 14-DOF element with warping"""
        # Create simple beam with warping
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Enable warping DOF
        node1.enable_warping_dof()
        node2.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "I-Beam", 0.01, 2e-4, 5e-5, 1e-5)
        sec.enable_warping(1e-6, 0.1)  # Enable warping

        # Create 14-DOF beam
        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node1, node2, mat, sec, config)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Assemble
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beam])

        # Should be 14x14 (2 nodes × 7 DOFs)
        assert K_global.shape == (14, 14)

        # Global matrix should match element matrix
        K_elem = beam.global_stiffness_matrix_warping()
        K_global_dense = K_global.todense()

        # Check all entries match
        np.testing.assert_allclose(K_global_dense, K_elem, rtol=1e-10)

    def test_assemble_single_14dof_element_mass(self):
        """Test assembling mass matrix for a single 14-DOF element with warping"""
        # Create simple beam with warping
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Enable warping DOF
        node1.enable_warping_dof()
        node2.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "I-Beam", 0.01, 2e-4, 5e-5, 1e-5)
        sec.enable_warping(1e-6, 0.1)

        # Create 14-DOF beam
        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node1, node2, mat, sec, config)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Assemble
        assembler = Assembler(dof_handler)
        M_global = assembler.assemble_mass([beam])

        # Should be 14x14
        assert M_global.shape == (14, 14)

        # Global matrix should match element matrix
        M_elem = beam.global_mass_matrix_warping()
        M_global_dense = M_global.todense()

        # Check all entries match
        np.testing.assert_allclose(M_global_dense, M_elem, rtol=1e-10)

    def test_assemble_two_12dof_elements(self):
        """Test assembling two 12-DOF elements sharing a node"""
        # Create two-element beam
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(3.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.0053, 8.36e-5, 6.04e-6, 1.2e-5)

        beam1 = BeamElement(1, node1, node2, mat, sec)
        beam2 = BeamElement(2, node2, node3, mat, sec)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Should have 18 DOFs (3 nodes × 6 DOFs)
        assert dof_handler.total_dofs() == 18

        # Assemble
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beam1, beam2])

        # Should be 18x18
        assert K_global.shape == (18, 18)

        # Check that middle node DOFs couple both elements
        # The stiffness at middle node should be sum of contributions
        K_dense = K_global.todense()

        # Middle node DOFs are 6-11
        # Check that middle node has coupling from both elements
        assert K_dense[6, 6] != 0.0  # UX coupling
        assert K_dense[7, 7] != 0.0  # UY coupling
        assert K_dense[8, 8] != 0.0  # UZ coupling

    def test_assemble_mixed_12dof_14dof_elements(self):
        """Test assembling mixed 12-DOF and 14-DOF elements"""
        # Create two elements: one 12-DOF, one 14-DOF
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(3.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Node 2 and 3 have warping, node 1 doesn't
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec1 = Section(1, "IPE300", 0.0053, 8.36e-5, 6.04e-6, 1.2e-5)
        sec2 = Section(2, "I-Beam", 0.01, 2e-4, 5e-5, 1e-5)
        sec2.enable_warping(1e-6, 0.1)

        # First beam: 12-DOF (no warping at node1)
        beam1 = BeamElement(1, node1, node2, mat, sec1)

        # Second beam: 14-DOF (warping at node2 and node3)
        config = BeamConfig()
        config.include_warping = True
        beam2 = BeamElement(2, node2, node3, mat, sec2, config)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Should have 20 DOFs (6 + 7 + 7)
        assert dof_handler.total_dofs() == 20

        # Assemble
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beam1, beam2])

        # Should be 20x20
        assert K_global.shape == (20, 20)

        # Verify assembly succeeded
        assert K_global.nnz > 0  # Has non-zero entries


class TestAssemblerAcceptanceCriteria:
    """Tests matching the Task 3.2 acceptance criteria"""

    def test_assembled_matrix_is_sparse(self):
        """AC: Assembled matrix is sparse"""
        # Create a model with several elements
        registry = NodeRegistry()
        nodes = [registry.get_or_create_node(i * 3.0, 0.0, 0.0) for i in range(5)]

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.0053, 8.36e-5, 6.04e-6, 1.2e-5)

        # Create 4 elements
        beams = [BeamElement(i+1, nodes[i], nodes[i+1], mat, sec) for i in range(4)]

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Should have 30 DOFs (5 nodes × 6 DOFs)
        assert dof_handler.total_dofs() == 30

        # Assemble
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beams[0], beams[1], beams[2], beams[3]])

        # Check that matrix is sparse (not all entries are non-zero)
        # A 30×30 matrix has 900 entries total
        total_entries = 30 * 30
        non_zero_entries = K_global.nnz

        # Sparsity: should have significantly fewer non-zero entries than total
        sparsity = 1.0 - (non_zero_entries / total_entries)

        # For beam elements with 4 elements in a chain, sparsity should be > 40%
        # (More elements would give higher sparsity)
        assert sparsity > 0.40
        assert non_zero_entries < total_entries

    def test_assembled_matrix_is_symmetric(self):
        """AC: Assembled matrix is symmetric"""
        # Create a simple model
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(3.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.0053, 8.36e-5, 6.04e-6, 1.2e-5)

        beam1 = BeamElement(1, node1, node2, mat, sec)
        beam2 = BeamElement(2, node2, node3, mat, sec)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Assemble
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beam1, beam2])

        # Check symmetry
        K_dense = K_global.todense()
        K_transpose = K_dense.T

        np.testing.assert_allclose(K_dense, K_transpose, rtol=1e-10)

    def test_single_12dof_element_matches_element_matrix(self):
        """AC: Single 12-DOF element assembly matches element stiffness matrix"""
        # Create single beam
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.0053, 8.36e-5, 6.04e-6, 1.2e-5)

        beam = BeamElement(1, node1, node2, mat, sec)

        # Get element matrix
        K_elem = beam.global_stiffness_matrix()

        # Number DOFs and assemble
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beam])

        # Global should match element exactly
        K_global_dense = K_global.todense()
        np.testing.assert_allclose(K_global_dense, K_elem, rtol=1e-12)

    def test_single_14dof_element_matches_element_matrix(self):
        """AC: Single 14-DOF element assembly matches element stiffness matrix"""
        # Create single beam with warping
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "I-Beam", 0.01, 2e-4, 5e-5, 1e-5)
        sec.enable_warping(1e-6, 0.1)

        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node1, node2, mat, sec, config)

        # Get element matrix
        K_elem = beam.global_stiffness_matrix_warping()

        # Number DOFs and assemble
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beam])

        # Global should match element exactly
        K_global_dense = K_global.todense()
        np.testing.assert_allclose(K_global_dense, K_elem, rtol=1e-12)

    def test_mixed_12dof_14dof_assembly_works(self):
        """AC: Mixed assembly (12-DOF and 14-DOF elements) works correctly"""
        # Create mixed model
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(3.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node4 = registry.get_or_create_node(9.0, 0.0, 0.0)

        # Enable warping only on nodes 2, 3, 4
        node2.enable_warping_dof()
        node3.enable_warping_dof()
        node4.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec1 = Section(1, "IPE300", 0.0053, 8.36e-5, 6.04e-6, 1.2e-5)
        sec2 = Section(2, "I-Beam", 0.01, 2e-4, 5e-5, 1e-5)
        sec2.enable_warping(1e-6, 0.1)

        # Beam 1: 12-DOF (node1 has no warping)
        beam1 = BeamElement(1, node1, node2, mat, sec1)

        # Beam 2: 14-DOF (both nodes have warping)
        config = BeamConfig()
        config.include_warping = True
        beam2 = BeamElement(2, node2, node3, mat, sec2, config)

        # Beam 3: 14-DOF (both nodes have warping)
        beam3 = BeamElement(3, node3, node4, mat, sec2, config)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Total DOFs: 6 + 7 + 7 + 7 = 27
        assert dof_handler.total_dofs() == 27

        # Assemble both stiffness and mass
        assembler = Assembler(dof_handler)
        K_global = assembler.assemble_stiffness([beam1, beam2, beam3])
        M_global = assembler.assemble_mass([beam1, beam2, beam3])

        # Check correct size
        assert K_global.shape == (27, 27)
        assert M_global.shape == (27, 27)

        # Check symmetry of both
        K_dense = K_global.todense()
        M_dense = M_global.todense()

        np.testing.assert_allclose(K_dense, K_dense.T, rtol=1e-10)
        np.testing.assert_allclose(M_dense, M_dense.T, rtol=1e-10)

        # Check that both matrices have non-zero entries
        assert K_global.nnz > 0
        assert M_global.nnz > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
