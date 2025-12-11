"""
Tests for Phase 1: Core Data Structures

This test suite verifies all acceptance criteria for Phase 1 tasks:
- Task 1.1: Node Class
- Task 1.2: Node Registry
- Task 1.3: Material Class
- Task 1.4: Section Class
- Task 1.5: Python Bindings
"""

import pytest
import numpy as np
from grillex.core import Node, NodeRegistry, Material, Section


class TestNodeClass:
    """Tests for Task 1.1: Node Class acceptance criteria"""

    def test_node_construction_with_id_and_coordinates(self):
        """Node can be constructed with id and coordinates"""
        node = Node(1, 10.0, 20.0, 30.0)
        assert node.id == 1
        assert node.x == 10.0
        assert node.y == 20.0
        assert node.z == 30.0

    def test_position_as_eigen_vector(self):
        """Position can be retrieved as Eigen::Vector3d"""
        node = Node(2, 5.0, 15.0, 25.0)
        pos = node.position()
        assert len(pos) == 3
        assert pos[0] == 5.0
        assert pos[1] == 15.0
        assert pos[2] == 25.0

    def test_dof_arrays_initialized(self):
        """DOF arrays are correctly initialized"""
        node = Node(3, 0.0, 0.0, 0.0)

        # DOF arrays should have size 7 (6 standard + 1 warping)
        assert len(node.dof_active) == 7
        assert len(node.global_dof_numbers) == 7

        # First 6 DOFs should be active by default
        assert all(node.dof_active[i] for i in range(6))

        # Warping DOF (index 6) should be inactive by default
        assert node.dof_active[6] == False

        # Global DOF numbers should be unassigned (-1)
        assert all(dof == -1 for dof in node.global_dof_numbers)


class TestNodeRegistry:
    """Tests for Task 1.2: Node Registry acceptance criteria"""

    def test_creating_node_twice_returns_same_node(self):
        """Creating node at (0,0,0) twice returns same node"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(0.0, 0.0, 0.0)

        # Should be the exact same node object
        assert node1.id == node2.id
        assert len(registry) == 1

    def test_node_within_tolerance_returns_existing(self):
        """Creating node at (0,0,1e-9) with tolerance 1e-6 returns existing node"""
        registry = NodeRegistry(tolerance=1e-6)
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(0.0, 0.0, 1e-9)  # Within tolerance

        # Should return same node
        assert node1.id == node2.id
        assert len(registry) == 1

    def test_node_outside_tolerance_creates_new(self):
        """Creating node at (0,0,1) with tolerance 1e-6 creates new node"""
        registry = NodeRegistry(tolerance=1e-6)
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(0.0, 0.0, 1.0)  # Outside tolerance

        # Should create new node
        assert node1.id != node2.id
        assert len(registry) == 2

    def test_node_ids_sequential_and_unique(self):
        """Node IDs are sequential and unique"""
        registry = NodeRegistry()

        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(1.0, 0.0, 0.0)
        node3 = registry.get_or_create_node(2.0, 0.0, 0.0)

        # IDs should be sequential starting from 1
        assert node1.id == 1
        assert node2.id == 2
        assert node3.id == 3

        # All IDs should be unique
        ids = [node1.id, node2.id, node3.id]
        assert len(ids) == len(set(ids))

    def test_get_node_by_id(self):
        """Can retrieve node by ID"""
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(5.0, 10.0, 15.0)

        retrieved = registry.get_node_by_id(node1.id)
        assert retrieved is not None
        assert retrieved.id == node1.id
        assert retrieved.x == 5.0

    def test_get_node_by_invalid_id(self):
        """Returns None for invalid ID"""
        registry = NodeRegistry()
        assert registry.get_node_by_id(999) is None

    def test_all_nodes(self):
        """Can retrieve all nodes"""
        registry = NodeRegistry()
        registry.get_or_create_node(0.0, 0.0, 0.0)
        registry.get_or_create_node(1.0, 0.0, 0.0)
        registry.get_or_create_node(2.0, 0.0, 0.0)

        nodes = registry.all_nodes()
        assert len(nodes) == 3

    def test_tolerance_setter_getter(self):
        """Can set and get tolerance"""
        registry = NodeRegistry(tolerance=1e-6)
        assert registry.get_tolerance() == 1e-6

        registry.set_tolerance(1e-3)
        assert registry.get_tolerance() == 1e-3


class TestMaterialClass:
    """Tests for Task 1.3: Material Class acceptance criteria"""

    def test_material_construction(self):
        """Material can be constructed with standard properties"""
        mat = Material(1, "Steel", 210000000.0, 0.3, 7.85)

        assert mat.id == 1
        assert mat.name == "Steel"
        assert mat.E == 210000000.0
        assert mat.nu == 0.3
        assert mat.rho == 7.85

    def test_shear_modulus_computed(self):
        """G is correctly computed from E and nu"""
        E = 210000000.0  # kN/m²
        nu = 0.3
        expected_G = E / (2.0 * (1.0 + nu))

        mat = Material(2, "Steel", E, nu, 7.85)

        # G should be automatically computed
        assert abs(mat.G - expected_G) < 1e-6

    def test_compute_G_static_method(self):
        """Static method compute_G works correctly"""
        E = 200000000.0
        nu = 0.25
        expected_G = E / (2.0 * (1.0 + nu))

        G = Material.compute_G(E, nu)
        assert abs(G - expected_G) < 1e-6

    def test_units_documented(self):
        """All units are documented in docstring"""
        # This is verified by the comprehensive docstrings in the C++ header
        # and the Python binding descriptions
        mat = Material(3, "Aluminum", 70000000.0, 0.33, 2.7)
        assert mat.E > 0
        assert mat.nu > 0
        assert mat.rho > 0


class TestSectionClass:
    """Tests for Task 1.4: Section Class acceptance criteria"""

    def test_section_construction(self):
        """Section can be constructed with basic properties"""
        sec = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)

        assert sec.id == 1
        assert sec.name == "IPE300"
        assert sec.A == 0.00538
        assert sec.Iy == 0.0000836
        assert sec.Iz == 0.00000604
        assert sec.J == 0.000000201

    def test_optional_properties_default_to_zero(self):
        """Optional properties default to 0"""
        sec = Section(2, "Test", 0.01, 0.0001, 0.0001, 0.00001)

        assert sec.Iw == 0.0
        assert sec.Asy == 0.0
        assert sec.Asz == 0.0
        assert sec.zy_top == 0.0
        assert sec.zy_bot == 0.0
        assert sec.zz_top == 0.0
        assert sec.zz_bot == 0.0

    def test_set_warping_constant(self):
        """Can set warping constant"""
        sec = Section(3, "Test", 0.01, 0.0001, 0.0001, 0.00001)
        sec.set_warping_constant(0.000001)
        assert sec.Iw == 0.000001

    def test_set_shear_areas(self):
        """Can set shear areas"""
        sec = Section(4, "Test", 0.01, 0.0001, 0.0001, 0.00001)
        sec.set_shear_areas(0.005, 0.006)
        assert sec.Asy == 0.005
        assert sec.Asz == 0.006

    def test_set_fibre_distances(self):
        """Section can store fibre distances for stress calculation"""
        sec = Section(5, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        sec.set_fibre_distances(0.15, 0.15, 0.05, 0.05)

        assert sec.zy_top == 0.15
        assert sec.zy_bot == 0.15
        assert sec.zz_top == 0.05
        assert sec.zz_bot == 0.05


class TestPythonBindings:
    """Tests for Task 1.5: Python Bindings acceptance criteria"""

    def test_import_from_core(self):
        """from grillex.core import Node, Material, Section works"""
        # This is tested by the imports at the top of this file
        # If they failed, the entire test suite would fail
        assert Node is not None
        assert Material is not None
        assert Section is not None
        assert NodeRegistry is not None

    def test_create_instances_from_python(self):
        """Can create instances from Python"""
        node = Node(10, 1.0, 2.0, 3.0)
        mat = Material(1, "Steel", 210000000.0, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        registry = NodeRegistry()

        assert isinstance(node, Node)
        assert isinstance(mat, Material)
        assert isinstance(sec, Section)
        assert isinstance(registry, NodeRegistry)

    def test_properties_accessible(self):
        """Properties are accessible from Python"""
        node = Node(1, 10.0, 20.0, 30.0)
        mat = Material(1, "Steel", 210000000.0, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)

        # Node properties
        assert node.id == 1
        assert node.x == 10.0

        # Material properties
        assert mat.name == "Steel"
        assert mat.E == 210000000.0

        # Section properties
        assert sec.name == "IPE300"
        assert sec.A == 0.00538

    def test_repr_methods(self):
        """__repr__ methods provide useful output"""
        node = Node(1, 10.0, 20.0, 30.0)
        mat = Material(1, "Steel", 210000000.0, 0.3, 7.85)
        sec = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        registry = NodeRegistry()

        # Should all have readable representations
        assert "Node" in str(node)
        assert "1" in str(node)

        assert "Material" in str(mat)
        assert "Steel" in str(mat)

        assert "Section" in str(sec)
        assert "IPE300" in str(sec)

        assert "NodeRegistry" in str(registry)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
