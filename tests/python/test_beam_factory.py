"""
Tests for Task 2.8: Unified Beam Element Factory

This test suite verifies the factory implementation for creating beam elements
with different configurations (Euler-Bernoulli, Timoshenko, with/without warping).
"""

import pytest
import numpy as np
from grillex.core import (
    Node, NodeRegistry, Material, Section, BeamElement, BeamFormulation,
    BeamConfig, BeamElementBase, create_beam_element
)


class TestBeamConfig:
    """Tests for BeamConfig struct"""

    def test_default_config(self):
        """Default config is Euler-Bernoulli without warping"""
        config = BeamConfig()
        assert config.formulation == BeamFormulation.EulerBernoulli
        assert config.include_warping == False
        assert config.include_shear_deformation == False

    def test_config_get_formulation(self):
        """get_formulation respects include_shear_deformation flag"""
        config = BeamConfig()
        config.formulation = BeamFormulation.EulerBernoulli
        config.include_shear_deformation = True

        # Should return Timoshenko due to include_shear_deformation
        assert config.get_formulation() == BeamFormulation.Timoshenko

    def test_config_explicit_timoshenko(self):
        """Explicit Timoshenko formulation works"""
        config = BeamConfig()
        config.formulation = BeamFormulation.Timoshenko
        assert config.get_formulation() == BeamFormulation.Timoshenko

    def test_config_with_warping(self):
        """Config with warping enabled"""
        config = BeamConfig()
        config.include_warping = True
        assert config.include_warping == True


class TestBeamElementFactory:
    """Tests for create_beam_element factory function"""

    @pytest.fixture
    def setup_nodes_and_properties(self):
        """Create common test objects"""
        node1 = Node(1, 0.0, 0.0, 0.0)
        node2 = Node(2, 6.0, 0.0, 0.0)

        steel = Material(1, "Steel", 210e9, 0.3, 7850.0)
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

        return node1, node2, steel, section

    def test_factory_creates_euler_bernoulli_element(self, setup_nodes_and_properties):
        """Factory creates Euler-Bernoulli element with default config"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()  # Default: Euler-Bernoulli, no warping
        elem = create_beam_element(1, node1, node2, steel, section, config)

        assert elem is not None
        assert elem.num_dofs() == 12
        assert elem.get_formulation() == BeamFormulation.EulerBernoulli
        assert elem.has_warping() == False

    def test_factory_creates_timoshenko_element(self, setup_nodes_and_properties):
        """Factory creates Timoshenko element"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        config.formulation = BeamFormulation.Timoshenko
        elem = create_beam_element(1, node1, node2, steel, section, config)

        assert elem is not None
        assert elem.num_dofs() == 12
        assert elem.get_formulation() == BeamFormulation.Timoshenko
        assert elem.has_warping() == False

    def test_factory_creates_warping_element(self, setup_nodes_and_properties):
        """Factory creates element with warping DOF"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        config.include_warping = True
        elem = create_beam_element(1, node1, node2, steel, section, config)

        assert elem is not None
        assert elem.num_dofs() == 14
        assert elem.has_warping() == True

    def test_factory_creates_timoshenko_with_warping(self, setup_nodes_and_properties):
        """Factory creates Timoshenko element with warping"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        config.formulation = BeamFormulation.Timoshenko
        config.include_warping = True
        elem = create_beam_element(1, node1, node2, steel, section, config)

        assert elem is not None
        assert elem.num_dofs() == 14
        assert elem.get_formulation() == BeamFormulation.Timoshenko
        assert elem.has_warping() == True

    def test_factory_with_include_shear_deformation(self, setup_nodes_and_properties):
        """Factory respects include_shear_deformation alias"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        config.include_shear_deformation = True  # Alias for Timoshenko
        elem = create_beam_element(1, node1, node2, steel, section, config)

        assert elem is not None
        assert elem.get_formulation() == BeamFormulation.Timoshenko


class TestPolymorphicInterface:
    """Tests for polymorphic interface through BeamElementBase"""

    @pytest.fixture
    def setup_nodes_and_properties(self):
        """Create common test objects"""
        node1 = Node(1, 0.0, 0.0, 0.0)
        node2 = Node(2, 6.0, 0.0, 0.0)

        steel = Material(1, "Steel", 210e9, 0.3, 7850.0)
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

        return node1, node2, steel, section

    def test_polymorphic_stiffness_matrix_12dof(self, setup_nodes_and_properties):
        """Polymorphic stiffness matrix returns 12x12 for standard element"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        elem = create_beam_element(1, node1, node2, steel, section, config)

        K = elem.compute_local_stiffness()
        assert K.shape == (12, 12)

    def test_polymorphic_stiffness_matrix_14dof(self, setup_nodes_and_properties):
        """Polymorphic stiffness matrix returns 14x14 for warping element"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        config.include_warping = True
        elem = create_beam_element(1, node1, node2, steel, section, config)

        K = elem.compute_local_stiffness()
        assert K.shape == (14, 14)

    def test_polymorphic_mass_matrix_12dof(self, setup_nodes_and_properties):
        """Polymorphic mass matrix returns 12x12 for standard element"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        elem = create_beam_element(1, node1, node2, steel, section, config)

        M = elem.compute_local_mass()
        assert M.shape == (12, 12)

    def test_polymorphic_mass_matrix_14dof(self, setup_nodes_and_properties):
        """Polymorphic mass matrix returns 14x14 for warping element"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        config.include_warping = True
        elem = create_beam_element(1, node1, node2, steel, section, config)

        M = elem.compute_local_mass()
        assert M.shape == (14, 14)

    def test_polymorphic_transformation_matrix_12dof(self, setup_nodes_and_properties):
        """Polymorphic transformation matrix returns 12x12 for standard element"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        elem = create_beam_element(1, node1, node2, steel, section, config)

        T = elem.compute_transformation()
        assert T.shape == (12, 12)

    def test_polymorphic_transformation_matrix_14dof(self, setup_nodes_and_properties):
        """Polymorphic transformation matrix returns 14x14 for warping element"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        config.include_warping = True
        elem = create_beam_element(1, node1, node2, steel, section, config)

        T = elem.compute_transformation()
        assert T.shape == (14, 14)


class TestBackwardCompatibility:
    """Tests to ensure existing code continues to work"""

    @pytest.fixture
    def setup_nodes_and_properties(self):
        """Create common test objects"""
        node1 = Node(1, 0.0, 0.0, 0.0)
        node2 = Node(2, 6.0, 0.0, 0.0)

        steel = Material(1, "Steel", 210e9, 0.3, 7850.0)
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

        return node1, node2, steel, section

    def test_old_constructor_still_works(self, setup_nodes_and_properties):
        """Old BeamElement constructor without config still works"""
        node1, node2, steel, section = setup_nodes_and_properties

        elem = BeamElement(1, node1, node2, steel, section)

        assert elem is not None
        assert elem.id == 1
        assert elem.length > 0

    def test_old_methods_still_work(self, setup_nodes_and_properties):
        """Old methods (without config) still work"""
        node1, node2, steel, section = setup_nodes_and_properties

        elem = BeamElement(1, node1, node2, steel, section)

        K = elem.local_stiffness_matrix()
        assert K.shape == (12, 12)

        M = elem.local_mass_matrix()
        assert M.shape == (12, 12)

        T = elem.transformation_matrix()
        assert T.shape == (12, 12)

    def test_new_constructor_with_config(self, setup_nodes_and_properties):
        """New BeamElement constructor with config works"""
        node1, node2, steel, section = setup_nodes_and_properties

        config = BeamConfig()
        config.formulation = BeamFormulation.Timoshenko

        elem = BeamElement(1, node1, node2, steel, section, config)

        assert elem is not None
        assert elem.config.formulation == BeamFormulation.Timoshenko
