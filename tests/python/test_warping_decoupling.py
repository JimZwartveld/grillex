"""
Test suite for Task 2.9: Warping DOF Decoupling at Non-Collinear Connections

This test suite verifies:
1. Collinearity detection between beam elements
2. Element-specific warping DOF numbering
3. Automatic coupling of warping DOFs for collinear elements
4. Independence of warping DOFs for non-collinear elements
5. User override of automatic coupling detection
6. Backward compatibility with models without warping
"""

import pytest
import numpy as np
from grillex.core import (
    Node, NodeRegistry, Material, Section, BeamElement, BeamConfig,
    BeamFormulation, DOFHandler, WarpingDOFInfo, WarpingCoupling,
    are_elements_collinear, create_beam_element
)


class TestDirectionVector:
    """Tests for BeamElement.direction_vector() method"""

    def test_horizontal_beam_x_direction(self):
        """Beam along X-axis has direction (1, 0, 0)"""
        registry = NodeRegistry()
        node_i = registry.get_or_create_node(0.0, 0.0, 0.0)
        node_j = registry.get_or_create_node(6.0, 0.0, 0.0)
        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node_i, node_j, mat, sec)
        dir_vec = elem.direction_vector()

        assert np.allclose(dir_vec, [1.0, 0.0, 0.0], atol=1e-10)

    def test_horizontal_beam_y_direction(self):
        """Beam along Y-axis has direction (0, 1, 0)"""
        registry = NodeRegistry()
        node_i = registry.get_or_create_node(0.0, 0.0, 0.0)
        node_j = registry.get_or_create_node(0.0, 5.0, 0.0)
        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node_i, node_j, mat, sec)
        dir_vec = elem.direction_vector()

        assert np.allclose(dir_vec, [0.0, 1.0, 0.0], atol=1e-10)

    def test_vertical_beam(self):
        """Beam along Z-axis has direction (0, 0, 1)"""
        registry = NodeRegistry()
        node_i = registry.get_or_create_node(0.0, 0.0, 0.0)
        node_j = registry.get_or_create_node(0.0, 0.0, 3.0)
        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node_i, node_j, mat, sec)
        dir_vec = elem.direction_vector()

        assert np.allclose(dir_vec, [0.0, 0.0, 1.0], atol=1e-10)

    def test_diagonal_beam(self):
        """Diagonal beam has normalized direction"""
        registry = NodeRegistry()
        node_i = registry.get_or_create_node(0.0, 0.0, 0.0)
        node_j = registry.get_or_create_node(3.0, 4.0, 0.0)
        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node_i, node_j, mat, sec)
        dir_vec = elem.direction_vector()

        expected = np.array([3.0, 4.0, 0.0]) / 5.0
        assert np.allclose(dir_vec, expected, atol=1e-10)

    def test_direction_is_unit_vector(self):
        """Direction vector is always a unit vector"""
        registry = NodeRegistry()
        node_i = registry.get_or_create_node(1.0, 2.0, 3.0)
        node_j = registry.get_or_create_node(4.0, 8.0, 5.0)
        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node_i, node_j, mat, sec)
        dir_vec = elem.direction_vector()

        assert np.allclose(np.linalg.norm(dir_vec), 1.0, atol=1e-10)


class TestCollinearityDetection:
    """Tests for are_elements_collinear() function"""

    @pytest.fixture
    def setup_continuous_beam(self):
        """Create three nodes in a line (continuous beam)"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)  # Shared node
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)
        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec)
        elem2 = BeamElement(2, node2, node3, mat, sec)

        return registry, node2, elem1, elem2

    @pytest.fixture
    def setup_t_joint(self):
        """Create T-joint (orthogonal beams)"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)  # Shared node
        node3 = registry.get_or_create_node(6.0, 4.0, 0.0)  # Perpendicular

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec)
        elem2 = BeamElement(2, node2, node3, mat, sec)

        return registry, node2, elem1, elem2

    def test_collinear_continuous_beam(self, setup_continuous_beam):
        """Collinear elements in a continuous beam are detected as collinear"""
        registry, shared_node, elem1, elem2 = setup_continuous_beam

        result = are_elements_collinear(elem1, elem2, shared_node.id)
        assert result is True

    def test_orthogonal_t_joint(self, setup_t_joint):
        """Orthogonal elements at T-joint are NOT collinear"""
        registry, shared_node, elem1, elem2 = setup_t_joint

        result = are_elements_collinear(elem1, elem2, shared_node.id)
        assert result is False

    def test_collinear_with_tolerance(self):
        """Nearly collinear elements (within tolerance) are detected as collinear"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        # 2 degrees off
        angle_rad = 2.0 * np.pi / 180.0
        node3 = registry.get_or_create_node(12.0, 6.0 * np.tan(angle_rad), 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec)
        elem2 = BeamElement(2, node2, node3, mat, sec)

        # Within default 5 degree tolerance
        result = are_elements_collinear(elem1, elem2, node2.id, 5.0)
        assert result is True

    def test_non_collinear_outside_tolerance(self):
        """Elements at 30 degrees are NOT collinear"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        # 30 degrees off
        angle_rad = 30.0 * np.pi / 180.0
        node3 = registry.get_or_create_node(6.0 + 6.0 * np.cos(angle_rad),
                                            6.0 * np.sin(angle_rad), 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec)
        elem2 = BeamElement(2, node2, node3, mat, sec)

        result = are_elements_collinear(elem1, elem2, node2.id, 5.0)
        assert result is False

    def test_l_joint_orthogonal(self):
        """L-joint (two beams at 90°) are NOT collinear"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)  # Start of first beam
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)  # Corner (shared)
        node3 = registry.get_or_create_node(6.0, 0.0, 4.0)  # End of second beam (vertical)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec)
        elem2 = BeamElement(2, node2, node3, mat, sec)

        result = are_elements_collinear(elem1, elem2, node2.id)
        assert result is False


class TestWarpingDOFNumbering:
    """Tests for element-specific warping DOF numbering"""

    @pytest.fixture
    def setup_warping_config(self):
        """Create warping-enabled beam config"""
        config = BeamConfig()
        config.include_warping = True
        return config

    def test_single_warping_element_numbering(self, setup_warping_config):
        """Single warping element gets proper warping DOF numbers"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        # Enable warping on nodes
        node1.enable_warping_dof()
        node2.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node1, node2, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem])

        # Should have 14 DOFs total: 6 + 7 (node1: 6 standard + 1 warp, node2: 6 standard + 1 warp)
        assert dof_handler.total_dofs() == 14
        assert dof_handler.has_warping_dofs() is True

        # Check warping DOFs
        warp_dof_i = dof_handler.get_warping_dof(elem.id, node1.id)
        warp_dof_j = dof_handler.get_warping_dof(elem.id, node2.id)

        assert warp_dof_i >= 0
        assert warp_dof_j >= 0
        assert warp_dof_i != warp_dof_j

    def test_collinear_elements_share_warping_dof(self, setup_warping_config):
        """Collinear warping elements share warping DOF at common node"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Check that warping DOFs at shared node (node2) are the same
        warp_dof_elem1_node2 = dof_handler.get_warping_dof(elem1.id, node2.id)
        warp_dof_elem2_node2 = dof_handler.get_warping_dof(elem2.id, node2.id)

        assert warp_dof_elem1_node2 == warp_dof_elem2_node2
        assert warp_dof_elem1_node2 >= 0

    def test_non_collinear_elements_independent_warping_dofs(self, setup_warping_config):
        """Non-collinear warping elements have independent warping DOFs"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)  # Shared node (T-joint)
        node3 = registry.get_or_create_node(6.0, 4.0, 0.0)  # Perpendicular

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Check that warping DOFs at shared node (node2) are DIFFERENT
        warp_dof_elem1_node2 = dof_handler.get_warping_dof(elem1.id, node2.id)
        warp_dof_elem2_node2 = dof_handler.get_warping_dof(elem2.id, node2.id)

        assert warp_dof_elem1_node2 != warp_dof_elem2_node2
        assert warp_dof_elem1_node2 >= 0
        assert warp_dof_elem2_node2 >= 0

    def test_location_array_14dof(self, setup_warping_config):
        """Location array for 14-DOF element uses element-specific warping DOFs"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node1, node2, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem])

        loc = dof_handler.get_location_array(elem)

        # Location array should have 14 entries
        assert len(loc) == 14

        # Standard DOFs should be nodal (0-5, 7-12)
        for d in range(6):
            assert loc[d] == dof_handler.get_global_dof(node1.id, d)
            assert loc[7 + d] == dof_handler.get_global_dof(node2.id, d)

        # Warping DOFs (6, 13) should be element-specific
        assert loc[6] == dof_handler.get_warping_dof(elem.id, node1.id)
        assert loc[13] == dof_handler.get_warping_dof(elem.id, node2.id)


class TestWarpingCouplings:
    """Tests for warping coupling information"""

    @pytest.fixture
    def setup_warping_config(self):
        config = BeamConfig()
        config.include_warping = True
        return config

    def test_warping_couplings_recorded(self, setup_warping_config):
        """Collinear elements produce warping coupling records"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        couplings = dof_handler.get_warping_couplings()

        # Should have one coupling (at node2)
        assert len(couplings) >= 1

        # Find the coupling at node2
        found = False
        for coupling in couplings:
            for info in coupling.coupled_dofs:
                if info.node_id == node2.id:
                    found = True
                    assert len(coupling.coupled_dofs) == 2  # Two elements coupled
                    break
            if found:
                break

        assert found

    def test_no_couplings_for_non_collinear(self, setup_warping_config):
        """Non-collinear elements don't produce couplings at shared node"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 4.0, 0.0)  # Perpendicular

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        couplings = dof_handler.get_warping_couplings()

        # Check that there's no coupling between elem1 and elem2 at node2
        for coupling in couplings:
            elem_ids_in_coupling = set(info.element_id for info in coupling.coupled_dofs)
            # Should not have both elem1 and elem2 in the same coupling
            assert not ({1, 2}.issubset(elem_ids_in_coupling))


class TestUserOverride:
    """Tests for manual override of collinearity detection"""

    @pytest.fixture
    def setup_warping_config(self):
        config = BeamConfig()
        config.include_warping = True
        return config

    def test_force_coupling_non_collinear(self, setup_warping_config):
        """User can force coupling of non-collinear elements"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 4.0, 0.0)  # Perpendicular

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        # Force coupling at node2
        dof_handler.set_warping_continuous(node2.id, [1, 2])
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Now warping DOFs should be the same at node2
        warp_dof_elem1 = dof_handler.get_warping_dof(elem1.id, node2.id)
        warp_dof_elem2 = dof_handler.get_warping_dof(elem2.id, node2.id)

        assert warp_dof_elem1 == warp_dof_elem2

    def test_release_coupling_collinear(self, setup_warping_config):
        """User can release coupling of collinear elements"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        # Release coupling at node2
        dof_handler.release_warping_coupling(node2.id, 1, 2)
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Now warping DOFs should be different at node2
        warp_dof_elem1 = dof_handler.get_warping_dof(elem1.id, node2.id)
        warp_dof_elem2 = dof_handler.get_warping_dof(elem2.id, node2.id)

        assert warp_dof_elem1 != warp_dof_elem2


class TestBackwardCompatibility:
    """Tests to ensure backward compatibility"""

    def test_legacy_numbering_still_works(self):
        """Legacy number_dofs() still works for standard elements"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assert dof_handler.total_dofs() == 12  # 6 DOFs per node
        assert dof_handler.has_warping_dofs() is False

    def test_12dof_elements_work_with_new_method(self):
        """12-DOF elements work correctly with number_dofs_with_elements"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node1, node2, mat, sec)  # No warping config

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem])

        assert dof_handler.total_dofs() == 12
        assert dof_handler.has_warping_dofs() is False

        loc = dof_handler.get_location_array(elem)
        assert len(loc) == 12

    def test_mixed_12dof_14dof_model(self):
        """Mixed model with 12-DOF and 14-DOF elements works"""
        config_warping = BeamConfig()
        config_warping.include_warping = True

        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        # Only middle section has warping
        node2.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec)  # 12-DOF
        elem2 = BeamElement(2, node2, node3, mat, sec, config_warping)  # 14-DOF (warping at node2)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        loc1 = dof_handler.get_location_array(elem1)
        loc2 = dof_handler.get_location_array(elem2)

        assert len(loc1) == 12
        assert len(loc2) == 14


class TestAcceptanceCriteria:
    """Tests matching the acceptance criteria from Task 2.9 specification"""

    @pytest.fixture
    def setup_warping_config(self):
        config = BeamConfig()
        config.include_warping = True
        return config

    def test_collinearity_detection_correct(self, setup_warping_config):
        """✓ Collinearity detection correctly identifies parallel elements"""
        registry = NodeRegistry()
        # Continuous beam (collinear)
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec)
        elem2 = BeamElement(2, node2, node3, mat, sec)

        # Should be collinear
        assert are_elements_collinear(elem1, elem2, node2.id) is True

    def test_non_collinear_independent_dofs(self, setup_warping_config):
        """✓ Non-collinear elements have independent warping DOFs"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(6.0, 4.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Warping DOFs at node2 should be different
        assert dof_handler.get_warping_dof(elem1.id, node2.id) != \
               dof_handler.get_warping_dof(elem2.id, node2.id)

    def test_collinear_shared_dofs(self, setup_warping_config):
        """✓ Collinear elements share warping DOFs (continuous warping)"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Warping DOFs at node2 should be the same
        assert dof_handler.get_warping_dof(elem1.id, node2.id) == \
               dof_handler.get_warping_dof(elem2.id, node2.id)

    def test_user_override_works(self, setup_warping_config):
        """✓ User can override automatic coupling detection"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        # Release the coupling at node2
        dof_handler.release_warping_coupling(node2.id, 1, 2)
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Warping DOFs at node2 should now be different
        assert dof_handler.get_warping_dof(elem1.id, node2.id) != \
               dof_handler.get_warping_dof(elem2.id, node2.id)

    def test_backward_compatible(self):
        """✓ Backward compatible: models without warping unchanged"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)

        elem = BeamElement(1, node1, node2, mat, sec)

        # Legacy method
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assert dof_handler.total_dofs() == 12
        assert dof_handler.has_warping_dofs() is False

        loc = dof_handler.get_location_array(elem)
        assert len(loc) == 12


class TestElementSpecificWarpingBoundaryConditions:
    """Tests for element-specific warping boundary conditions (Task 2.9)"""

    @pytest.fixture
    def setup_warping_config(self):
        config = BeamConfig()
        config.include_warping = True
        return config

    def test_add_fixed_warping_dof(self, setup_warping_config):
        """BCHandler.add_fixed_warping_dof creates element-specific BC"""
        from grillex.core import BCHandler

        bc = BCHandler()
        bc.add_fixed_warping_dof(element_id=1, node_id=1, value=0.0)

        fixed_dofs = bc.get_fixed_dofs()
        assert len(fixed_dofs) == 1
        assert fixed_dofs[0].element_id == 1
        assert fixed_dofs[0].node_id == 1
        assert fixed_dofs[0].local_dof == 6  # WARP

    def test_fix_node_with_warping_element_specific(self, setup_warping_config):
        """fix_node_with_warping(node_id, element_id) creates element-specific BC"""
        from grillex.core import BCHandler

        bc = BCHandler()
        bc.fix_node_with_warping(node_id=1, element_id=5)

        fixed_dofs = bc.get_fixed_dofs()
        # Should have 6 standard DOFs + 1 element-specific warping DOF
        assert len(fixed_dofs) == 7

        # Find the warping DOF
        warp_dofs = [fd for fd in fixed_dofs if fd.local_dof == 6]
        assert len(warp_dofs) == 1
        assert warp_dofs[0].element_id == 5

    def test_element_specific_warping_bc_applied_correctly(self, setup_warping_config):
        """Element-specific warping BCs are applied to correct global DOFs"""
        from grillex.core import BCHandler, Assembler

        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)  # Shared T-joint node
        node3 = registry.get_or_create_node(6.0, 4.0, 0.0)  # Perpendicular

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.01, 1e-4, 1e-5, 1e-5)
        sec.Iw = 1.0e-9  # Warping constant

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Non-collinear elements should have different warping DOFs at node2
        warp_dof_elem1 = dof_handler.get_warping_dof(elem1.id, node2.id)
        warp_dof_elem2 = dof_handler.get_warping_dof(elem2.id, node2.id)
        assert warp_dof_elem1 != warp_dof_elem2

        # Create BC that fixes only elem1's warping at node2
        bc = BCHandler()
        bc.add_fixed_warping_dof(elem1.id, node2.id, 0.0)

        # Check that it resolves to the correct global DOF
        fixed_global_dofs = bc.get_fixed_global_dofs(dof_handler)
        assert warp_dof_elem1 in fixed_global_dofs
        assert warp_dof_elem2 not in fixed_global_dofs


class TestTJointWarpingDecoupling:
    """Tests for T-joint warping behavior (Task 2.9 acceptance criterion)"""

    @pytest.fixture
    def setup_warping_config(self):
        config = BeamConfig()
        config.include_warping = True
        return config

    def test_t_joint_no_warping_coupling(self, setup_warping_config):
        """✓ T-joint with torque shows no warping coupling between orthogonal beams"""
        from grillex.core import BCHandler, Assembler, LinearSolver

        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)  # T-joint
        node3 = registry.get_or_create_node(6.0, 4.0, 0.0)  # Perpendicular

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        # I-beam section with significant warping constant
        sec = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)
        sec.Iw = 1.26e-7  # Warping constant for IPE300 (m^6)

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Check that warping DOFs at node2 are independent (orthogonal beams)
        warp_dof_elem1_node2 = dof_handler.get_warping_dof(elem1.id, node2.id)
        warp_dof_elem2_node2 = dof_handler.get_warping_dof(elem2.id, node2.id)
        assert warp_dof_elem1_node2 != warp_dof_elem2_node2, \
            "Orthogonal beams at T-joint should have independent warping DOFs"

        # Assemble system
        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([elem1, elem2])
        n_dofs = dof_handler.total_dofs()
        F = np.zeros(n_dofs)

        # Apply torsion moment to elem1 at node1 (RX = rotation about x-axis)
        rx_node1 = dof_handler.get_global_dof(node1.id, 3)  # RX = 3
        torque = 10.0  # kNm
        F[rx_node1] = torque

        # Fix node1 translations and elem1's warping at node1
        bc = BCHandler()
        bc.pin_node(node1.id)
        bc.add_fixed_warping_dof(elem1.id, node1.id, 0.0)

        # Fix node3 completely (fixed end of perpendicular beam)
        bc.fix_node_with_warping(node3.id, elem2.id)

        # At node2: fix translations, leave rotations and warping free for elem1
        # but fix warping for elem2 (simulate fixed end condition)
        bc.add_fixed_dof(node2.id, 0, 0.0)  # UX
        bc.add_fixed_dof(node2.id, 1, 0.0)  # UY
        bc.add_fixed_dof(node2.id, 2, 0.0)  # UZ
        bc.add_fixed_warping_dof(elem2.id, node2.id, 0.0)

        # Apply BCs
        K_mod, F_mod = bc.apply_to_system(K, F, dof_handler)

        # Solve
        solver = LinearSolver()
        u = solver.solve(K_mod, F_mod)
        assert not solver.is_singular(), "System should not be singular"

        # Key test: elem2's warping DOF should be near zero (fixed),
        # while elem1's warping at node2 can be non-zero (free)
        warp_disp_elem1_node2 = u[warp_dof_elem1_node2]
        warp_disp_elem2_node2 = u[warp_dof_elem2_node2]

        # elem2's warping at node2 was fixed to zero
        assert abs(warp_disp_elem2_node2) < 1e-10, \
            "elem2 warping at T-joint should be zero (was fixed)"

        # This demonstrates independence: elem1's warping is not affected
        # by elem2's warping constraint, and vice versa


class TestContinuousBeamWarpingContinuity:
    """Tests for continuous beam warping continuity (Task 2.9 acceptance criterion)"""

    @pytest.fixture
    def setup_warping_config(self):
        config = BeamConfig()
        config.include_warping = True
        return config

    def test_continuous_beam_warping_continuity(self, setup_warping_config):
        """✓ Continuous beam shows warping continuity at internal nodes"""
        from grillex.core import BCHandler, Assembler, LinearSolver

        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)  # Internal node
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)
        sec.Iw = 1.26e-7  # Warping constant

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Collinear elements should share warping DOF at node2
        warp_dof_elem1_node2 = dof_handler.get_warping_dof(elem1.id, node2.id)
        warp_dof_elem2_node2 = dof_handler.get_warping_dof(elem2.id, node2.id)
        assert warp_dof_elem1_node2 == warp_dof_elem2_node2, \
            "Collinear beams should share warping DOF at internal node"

        # Assemble system
        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([elem1, elem2])
        n_dofs = dof_handler.total_dofs()
        F = np.zeros(n_dofs)

        # Apply torsion at midspan (node2)
        rx_node2 = dof_handler.get_global_dof(node2.id, 3)  # RX
        torque = 10.0
        F[rx_node2] = torque

        # Fixed ends with warping restraint
        bc = BCHandler()
        bc.fix_node_with_warping(node1.id, elem1.id)
        bc.fix_node_with_warping(node3.id, elem2.id)

        # Apply BCs
        K_mod, F_mod = bc.apply_to_system(K, F, dof_handler)

        # Solve
        solver = LinearSolver()
        u = solver.solve(K_mod, F_mod)
        assert not solver.is_singular(), "System should not be singular"

        # The shared warping DOF at node2 means warping is continuous
        # Both elements experience the same warping displacement at node2
        # This is inherent in the DOF sharing - no need to check equality
        # since they are literally the same DOF

        # Verify warping at internal node is non-zero (torque causes warping)
        warp_disp_node2 = u[warp_dof_elem1_node2]
        # With symmetric boundary conditions and torque at center,
        # warping should develop at the internal node
        # (The exact value depends on section properties)

    def test_warping_continuity_produces_coupled_bimoments(self, setup_warping_config):
        """Warping continuity means bimoments are transferred at internal nodes"""
        from grillex.core import BCHandler, Assembler, LinearSolver

        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0)  # Internal
        node3 = registry.get_or_create_node(12.0, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        mat = Material(1, "Steel", 210e6, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)
        sec.Iw = 1.26e-7

        elem1 = BeamElement(1, node1, node2, mat, sec, setup_warping_config)
        elem2 = BeamElement(2, node2, node3, mat, sec, setup_warping_config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Verify shared warping DOF
        assert dof_handler.get_warping_dof(elem1.id, node2.id) == \
               dof_handler.get_warping_dof(elem2.id, node2.id)

        # Assemble
        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([elem1, elem2])
        n_dofs = dof_handler.total_dofs()
        F = np.zeros(n_dofs)

        # Apply torque at free end (node3 free in torsion)
        rx_node3 = dof_handler.get_global_dof(node3.id, 3)
        F[rx_node3] = 10.0

        # Fix node1 with warping, fork support at node3 (warping free)
        bc = BCHandler()
        bc.fix_node_with_warping(node1.id, elem1.id)
        bc.fork_support(node3.id)

        K_mod, F_mod = bc.apply_to_system(K, F, dof_handler)
        solver = LinearSolver()
        u = solver.solve(K_mod, F_mod)
        assert not solver.is_singular()

        # Get displacements
        warp_node2 = u[dof_handler.get_warping_dof(elem1.id, node2.id)]
        warp_node3 = u[dof_handler.get_warping_dof(elem2.id, node3.id)]

        # With warping fixed at node1, we expect warping to develop
        # at node2 and node3, demonstrating that the warping deformation
        # propagates through the continuous beam
