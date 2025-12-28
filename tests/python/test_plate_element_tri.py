"""Tests for PlateElementTri (DKT triangular plate element).

This module tests the 3-node Discrete Kirchhoff Triangle plate element.
"""

import pytest
import numpy as np
from grillex.core import Node, Material, PlateElementTri


@pytest.fixture
def steel():
    """Create standard steel material for testing."""
    return Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-3)


@pytest.fixture
def nodes_equilateral():
    """Create nodes for an equilateral triangle (side length 1m)."""
    h = np.sqrt(3) / 2  # Height of equilateral triangle
    n1 = Node(1, 0.0, 0.0, 0.0)
    n2 = Node(2, 1.0, 0.0, 0.0)
    n3 = Node(3, 0.5, h, 0.0)
    return n1, n2, n3


@pytest.fixture
def nodes_right_triangle():
    """Create nodes for a right triangle (1m x 1m)."""
    n1 = Node(1, 0.0, 0.0, 0.0)
    n2 = Node(2, 1.0, 0.0, 0.0)
    n3 = Node(3, 0.0, 1.0, 0.0)
    return n1, n2, n3


@pytest.fixture
def nodes_inclined():
    """Create nodes for a triangle inclined in space."""
    n1 = Node(1, 0.0, 0.0, 0.0)
    n2 = Node(2, 1.0, 0.0, 0.5)  # Z offset
    n3 = Node(3, 0.5, 1.0, 0.25)
    return n1, n2, n3


class TestPlateElementTriBasic:
    """Basic tests for PlateElementTri construction and properties."""

    def test_create_element(self, nodes_equilateral, steel):
        """Test basic element creation."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        assert elem.id == 1
        assert elem.thickness == 0.01
        assert elem.num_dofs() == 18
        assert elem.has_warping() == False

    def test_nodes_stored_correctly(self, nodes_equilateral, steel):
        """Test that nodes are stored correctly."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        assert len(elem.nodes) == 3
        assert elem.nodes[0].id == 1
        assert elem.nodes[1].id == 2
        assert elem.nodes[2].id == 3

    def test_material_reference(self, nodes_equilateral, steel):
        """Test that material is stored correctly."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        assert elem.material.E == 210e6
        assert elem.material.nu == 0.3

    def test_area_equilateral(self, nodes_equilateral, steel):
        """Test area calculation for equilateral triangle."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        expected_area = np.sqrt(3) / 4  # For unit side length
        np.testing.assert_almost_equal(elem.area(), expected_area, decimal=10)

    def test_area_right_triangle(self, nodes_right_triangle, steel):
        """Test area calculation for right triangle."""
        n1, n2, n3 = nodes_right_triangle
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        expected_area = 0.5  # 1/2 * base * height
        np.testing.assert_almost_equal(elem.area(), expected_area, decimal=10)

    def test_centroid(self, nodes_right_triangle, steel):
        """Test centroid calculation."""
        n1, n2, n3 = nodes_right_triangle
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        centroid = elem.centroid()
        # Centroid is average of vertices
        expected = np.array([1/3, 1/3, 0.0])
        np.testing.assert_array_almost_equal(centroid, expected, decimal=10)


class TestPlateElementTriLocalAxes:
    """Tests for local coordinate system computation."""

    def test_local_axes_flat_triangle(self, nodes_right_triangle, steel):
        """Test local axes for flat triangle in XY plane."""
        n1, n2, n3 = nodes_right_triangle
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)

        # x_axis should be from n1 to n2 (1, 0, 0)
        np.testing.assert_array_almost_equal(elem.x_axis, [1, 0, 0], decimal=10)

        # z_axis should be normal to plate (0, 0, 1)
        np.testing.assert_array_almost_equal(elem.z_axis, [0, 0, 1], decimal=10)

        # y_axis completes right-hand system
        np.testing.assert_array_almost_equal(elem.y_axis, [0, 1, 0], decimal=10)

    def test_local_axes_inclined(self, nodes_inclined, steel):
        """Test local axes for inclined triangle."""
        n1, n2, n3 = nodes_inclined
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)

        # Verify orthonormality
        assert abs(np.linalg.norm(elem.x_axis) - 1.0) < 1e-10
        assert abs(np.linalg.norm(elem.y_axis) - 1.0) < 1e-10
        assert abs(np.linalg.norm(elem.z_axis) - 1.0) < 1e-10

        assert abs(np.dot(elem.x_axis, elem.y_axis)) < 1e-10
        assert abs(np.dot(elem.y_axis, elem.z_axis)) < 1e-10
        assert abs(np.dot(elem.z_axis, elem.x_axis)) < 1e-10

    def test_transformation(self, nodes_right_triangle, steel):
        """Test coordinate transformations."""
        n1, n2, n3 = nodes_right_triangle
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)

        # Test round-trip transformation
        global_vec = np.array([1.0, 2.0, 3.0])
        local_vec = elem.to_local(global_vec)
        back_to_global = elem.to_global(local_vec)
        np.testing.assert_array_almost_equal(global_vec, back_to_global, decimal=10)


class TestPlateElementTriStiffnessMatrix:
    """Tests for stiffness matrix computation."""

    def test_stiffness_matrix_shape(self, nodes_equilateral, steel):
        """Test that stiffness matrix has correct shape (18x18)."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        K = elem.global_stiffness_matrix()
        assert K.shape == (18, 18)

    def test_stiffness_matrix_symmetric(self, nodes_equilateral, steel):
        """Test that stiffness matrix is symmetric."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        K = elem.global_stiffness_matrix()
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)

    def test_stiffness_matrix_positive_semidefinite(self, nodes_equilateral, steel):
        """Test that stiffness matrix is positive semi-definite."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        K = elem.global_stiffness_matrix()
        eigenvalues = np.linalg.eigvalsh(K)
        # Should have 6 zero eigenvalues (rigid body modes) and 12 positive
        # Allow small negative values due to numerical precision
        assert np.all(eigenvalues >= -1e-8)

    def test_stiffness_scales_with_thickness_cubed(self, nodes_equilateral, steel):
        """Test that bending stiffness scales with t³."""
        n1, n2, n3 = nodes_equilateral

        t1 = 0.01
        t2 = 0.02

        elem1 = PlateElementTri(1, n1, n2, n3, t1, steel)
        elem2 = PlateElementTri(2, n1, n2, n3, t2, steel)

        K1 = elem1.global_stiffness_matrix()
        K2 = elem2.global_stiffness_matrix()

        # Find a non-zero bending stiffness entry
        # DOF 2 (w) is bending related
        ratio = K2[2, 2] / K1[2, 2]
        expected_ratio = (t2 / t1) ** 3
        np.testing.assert_almost_equal(ratio, expected_ratio, decimal=3)

    def test_stiffness_scales_with_E(self, nodes_equilateral):
        """Test that stiffness scales with Young's modulus."""
        n1, n2, n3 = nodes_equilateral

        mat1 = Material(1, "Mat1", E=210e6, nu=0.3, rho=7.85e-3)
        mat2 = Material(2, "Mat2", E=420e6, nu=0.3, rho=7.85e-3)

        elem1 = PlateElementTri(1, n1, n2, n3, 0.01, mat1)
        elem2 = PlateElementTri(2, n1, n2, n3, 0.01, mat2)

        K1 = elem1.global_stiffness_matrix()
        K2 = elem2.global_stiffness_matrix()

        # Stiffness should double when E doubles
        np.testing.assert_allclose(K2, 2 * K1, rtol=1e-10)


class TestPlateElementTriMassMatrix:
    """Tests for mass matrix computation."""

    def test_mass_matrix_shape(self, nodes_equilateral, steel):
        """Test that mass matrix has correct shape (18x18)."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        M = elem.global_mass_matrix()
        assert M.shape == (18, 18)

    def test_mass_matrix_symmetric(self, nodes_equilateral, steel):
        """Test that mass matrix is symmetric."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        M = elem.global_mass_matrix()
        np.testing.assert_allclose(M, M.T, rtol=1e-10)

    def test_mass_matrix_positive_semidefinite(self, nodes_equilateral, steel):
        """Test that mass matrix is positive semi-definite."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        M = elem.global_mass_matrix()
        eigenvalues = np.linalg.eigvalsh(M)
        assert np.all(eigenvalues >= -1e-10)

    def test_total_mass_translational(self, nodes_equilateral, steel):
        """Test that total translational mass equals rho * area * thickness."""
        n1, n2, n3 = nodes_equilateral
        t = 0.01
        elem = PlateElementTri(1, n1, n2, n3, t, steel)
        M = elem.global_mass_matrix()

        # Sum translational masses in z-direction (DOFs 2, 8, 14)
        total_mass_z = M[2, 2] + M[8, 8] + M[14, 14]
        expected_mass = steel.rho * elem.area() * t
        np.testing.assert_almost_equal(total_mass_z, expected_mass, decimal=6)


class TestPlateElementTriDifferentGeometries:
    """Tests with different triangle geometries."""

    def test_thin_triangle(self, steel):
        """Test with a thin, elongated triangle."""
        n1 = Node(1, 0.0, 0.0, 0.0)
        n2 = Node(2, 5.0, 0.0, 0.0)
        n3 = Node(3, 2.5, 0.5, 0.0)

        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        K = elem.global_stiffness_matrix()
        M = elem.global_mass_matrix()

        assert K.shape == (18, 18)
        assert M.shape == (18, 18)
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)
        np.testing.assert_allclose(M, M.T, rtol=1e-10)

    def test_vertical_triangle(self, steel):
        """Test with a triangle in the XZ plane."""
        n1 = Node(1, 0.0, 0.0, 0.0)
        n2 = Node(2, 1.0, 0.0, 0.0)
        n3 = Node(3, 0.5, 0.0, 1.0)

        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        K = elem.global_stiffness_matrix()

        assert K.shape == (18, 18)
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)

    def test_arbitrary_orientation(self, steel):
        """Test with arbitrary 3D orientation."""
        n1 = Node(1, 1.0, 2.0, 3.0)
        n2 = Node(2, 4.0, 2.5, 3.5)
        n3 = Node(3, 2.5, 4.0, 4.0)

        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        K = elem.global_stiffness_matrix()
        M = elem.global_mass_matrix()

        assert K.shape == (18, 18)
        assert M.shape == (18, 18)
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)
        np.testing.assert_allclose(M, M.T, rtol=1e-10)


class TestPlateElementTriRepr:
    """Tests for string representation."""

    def test_repr(self, nodes_equilateral, steel):
        """Test __repr__ method."""
        n1, n2, n3 = nodes_equilateral
        elem = PlateElementTri(1, n1, n2, n3, 0.01, steel)
        repr_str = repr(elem)
        assert "PlateElementTri" in repr_str
        assert "id=1" in repr_str
        assert "thickness=" in repr_str
        assert "area=" in repr_str
