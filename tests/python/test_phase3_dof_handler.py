"""
Tests for Phase 3, Task 3.1: DOF Numbering System

This test suite verifies the DOFHandler implementation for global DOF numbering
with support for both standard 6-DOF and 7-DOF (warping) nodes.
"""

import pytest
import numpy as np
from grillex.core import (
    Node, NodeRegistry, Material, Section, BeamElement, BeamFormulation,
    BeamConfig, create_beam_element, DOFHandler
)


class TestDOFHandler:
    """Tests for DOFHandler class"""

    def test_dof_handler_creation(self):
        """Can create empty DOFHandler"""
        dof_handler = DOFHandler()
        assert dof_handler.total_dofs() == 0
        assert dof_handler.has_warping_dofs() == False

    def test_simple_two_node_numbering(self):
        """Simple two-node system gets correct DOF numbering"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Each node has 6 DOFs, so total should be 12
        assert dof_handler.total_dofs() == 12

        # Node 1 DOFs should be numbered 0-5
        assert dof_handler.get_global_dof(node1.id, 0) == 0  # UX
        assert dof_handler.get_global_dof(node1.id, 1) == 1  # UY
        assert dof_handler.get_global_dof(node1.id, 2) == 2  # UZ
        assert dof_handler.get_global_dof(node1.id, 3) == 3  # RX
        assert dof_handler.get_global_dof(node1.id, 4) == 4  # RY
        assert dof_handler.get_global_dof(node1.id, 5) == 5  # RZ

        # Node 2 DOFs should be numbered 6-11
        assert dof_handler.get_global_dof(node2.id, 0) == 6   # UX
        assert dof_handler.get_global_dof(node2.id, 1) == 7   # UY
        assert dof_handler.get_global_dof(node2.id, 2) == 8   # UZ
        assert dof_handler.get_global_dof(node2.id, 3) == 9   # RX
        assert dof_handler.get_global_dof(node2.id, 4) == 10  # RY
        assert dof_handler.get_global_dof(node2.id, 5) == 11  # RZ

    def test_inactive_dof_returns_minus_one(self):
        """Inactive DOFs return -1"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)

        # Deactivate UX DOF - need to assign entire array back
        dof_active = node1.dof_active
        dof_active[0] = False
        node1.dof_active = dof_active

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # UX should not be numbered
        assert dof_handler.get_global_dof(node1.id, 0) == -1

        # Total DOFs should be 5 instead of 6
        assert dof_handler.total_dofs() == 5

    def test_warping_dof_numbering(self):
        """Warping DOF is numbered correctly when enabled"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Enable warping DOF on both nodes
        node1.enable_warping_dof()
        node2.enable_warping_dof()

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Each node has 7 DOFs, so total should be 14
        assert dof_handler.total_dofs() == 14
        assert dof_handler.has_warping_dofs() == True

        # Node 1 warping DOF should be numbered 6
        assert dof_handler.get_global_dof(node1.id, 6) == 6

        # Node 2 warping DOF should be numbered 13
        assert dof_handler.get_global_dof(node2.id, 6) == 13

    def test_mixed_warping_numbering(self):
        """System with mixed warping/non-warping nodes"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(3.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Only node2 has warping
        node2.enable_warping_dof()

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Total: 6 (node1) + 7 (node2) + 6 (node3) = 19
        assert dof_handler.total_dofs() == 19
        assert dof_handler.has_warping_dofs() == True

        # Node 1: DOFs 0-5
        assert dof_handler.get_global_dof(node1.id, 5) == 5

        # Node 2: DOFs 6-12 (warping at 12)
        assert dof_handler.get_global_dof(node2.id, 0) == 6
        assert dof_handler.get_global_dof(node2.id, 6) == 12

        # Node 3: DOFs 13-18
        assert dof_handler.get_global_dof(node3.id, 0) == 13
        assert dof_handler.get_global_dof(node3.id, 5) == 18

        # Node 3 has no warping DOF
        assert dof_handler.get_global_dof(node3.id, 6) == -1

    def test_location_array_12dof_element(self):
        """Location array for standard 12-DOF element"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        steel = Material(1, "Steel", 210e9, 0.3, 7850.0)
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

        elem = BeamElement(1, node1, node2, steel, section)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        loc = dof_handler.get_location_array(elem)

        # Should have 12 entries
        assert len(loc) == 12

        # Node 1 DOFs (0-5)
        assert loc[0:6] == [0, 1, 2, 3, 4, 5]

        # Node 2 DOFs (6-11)
        assert loc[6:12] == [6, 7, 8, 9, 10, 11]

    def test_location_array_14dof_element(self):
        """Location array for 14-DOF warping element"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Enable warping on both nodes
        node1.enable_warping_dof()
        node2.enable_warping_dof()

        steel = Material(1, "Steel", 210e9, 0.3, 7850.0)
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

        # Create element with warping
        config = BeamConfig()
        config.include_warping = True
        elem = create_beam_element(1, node1, node2, steel, section, config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        loc = dof_handler.get_location_array(elem)

        # Should have 14 entries
        assert len(loc) == 14

        # Node 1 DOFs (0-6)
        assert loc[0:7] == [0, 1, 2, 3, 4, 5, 6]

        # Node 2 DOFs (7-13)
        assert loc[7:14] == [7, 8, 9, 10, 11, 12, 13]

    def test_clear_method(self):
        """Clear method resets DOF handler"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node1.enable_warping_dof()

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assert dof_handler.total_dofs() == 7
        assert dof_handler.has_warping_dofs() == True

        dof_handler.clear()

        assert dof_handler.total_dofs() == 0
        assert dof_handler.has_warping_dofs() == False

    def test_renumbering_after_modifications(self):
        """Can renumber DOFs after registry modifications"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assert dof_handler.total_dofs() == 6

        # Add another node
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Renumber
        dof_handler.number_dofs(registry)

        assert dof_handler.total_dofs() == 12


class TestDOFHandlerAcceptanceCriteria:
    """Tests for Task 3.1 acceptance criteria"""

    def test_unique_global_numbers(self):
        """✓ Each active DOF gets a unique global number"""
        registry = NodeRegistry()
        nodes = []
        for i in range(5):
            node = registry.get_or_create_node(float(i), 0.0, 0.0)
            nodes.append(node)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Collect all global DOF numbers
        global_dofs = set()
        for node in nodes:
            for local_dof in range(6):
                g_dof = dof_handler.get_global_dof(node.id, local_dof)
                if g_dof >= 0:
                    assert g_dof not in global_dofs  # Must be unique
                    global_dofs.add(g_dof)

        assert len(global_dofs) == 30  # 5 nodes × 6 DOFs

    def test_location_arrays_correct_mapping(self):
        """✓ Location arrays correctly map element DOFs to global DOFs"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(3.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 0.0, 0.0)

        steel = Material(1, "Steel", 210e9, 0.3, 7850.0)
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

        elem1 = BeamElement(1, node1, node2, steel, section)
        elem2 = BeamElement(2, node2, node3, steel, section)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        loc1 = dof_handler.get_location_array(elem1)
        loc2 = dof_handler.get_location_array(elem2)

        # Element 1: connects nodes 1 and 2
        # Should map to global DOFs 0-5 (node1) and 6-11 (node2)
        assert loc1 == list(range(12))

        # Element 2: connects nodes 2 and 3
        # Should map to global DOFs 6-11 (node2) and 12-17 (node3)
        expected_loc2 = list(range(6, 18))
        assert loc2 == expected_loc2

    def test_inactive_dofs_not_numbered(self):
        """✓ Inactive DOFs are not numbered"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)

        # Deactivate all rotational DOFs - need to assign entire array back
        dof_active = node1.dof_active
        dof_active[3] = False  # RX
        dof_active[4] = False  # RY
        dof_active[5] = False  # RZ
        node1.dof_active = dof_active

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Only 3 DOFs should be numbered (UX, UY, UZ)
        assert dof_handler.total_dofs() == 3

        # Inactive DOFs should return -1
        assert dof_handler.get_global_dof(node1.id, 3) == -1
        assert dof_handler.get_global_dof(node1.id, 4) == -1
        assert dof_handler.get_global_dof(node1.id, 5) == -1

    def test_warping_numbered_only_when_active(self):
        """✓ 7th DOF (warping) numbered only when active on node"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Only node2 has warping enabled
        node2.enable_warping_dof()

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Node 1 should not have warping DOF numbered
        assert dof_handler.get_global_dof(node1.id, 6) == -1

        # Node 2 should have warping DOF numbered
        assert dof_handler.get_global_dof(node2.id, 6) >= 0

    def test_mixed_12dof_14dof_elements(self):
        """✓ Mixed models (some elements 12-DOF, some 14-DOF) work correctly"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(3.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Enable warping only on nodes 2 and 3
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        steel = Material(1, "Steel", 210e9, 0.3, 7850.0)
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

        # Element 1: 12-DOF (node1 doesn't have warping)
        elem1 = BeamElement(1, node1, node2, steel, section)

        # Element 2: 14-DOF (both nodes have warping)
        config = BeamConfig()
        config.include_warping = True
        elem2 = create_beam_element(2, node2, node3, steel, section, config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Total DOFs: 6 (node1) + 7 (node2) + 7 (node3) = 20
        assert dof_handler.total_dofs() == 20

        # Get location arrays
        loc1 = dof_handler.get_location_array(elem1)
        loc2 = dof_handler.get_location_array(elem2)

        # Element 1 is 12-DOF
        assert len(loc1) == 12

        # Element 2 is 14-DOF
        assert len(loc2) == 14

        # Verify no -1 values in location arrays (all DOFs should be active)
        assert all(dof >= 0 for dof in loc1)
        assert all(dof >= 0 for dof in loc2)
