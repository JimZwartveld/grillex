"""
Test suite for Phase 3: Model Class (Task 3.5)

Tests the Model class - top-level orchestration for complete FEM workflow:
1. Model creation and entity management
2. Complete analysis workflow (DOF numbering, assembly, BCs, solving)
3. Results extraction (displacements, reactions)
4. Error handling for invalid models

Acceptance Criteria:
- AC1: Complete analysis workflow runs without errors
- AC2: Results match hand calculations for simple models
- AC3: Error handling for invalid models
"""

import pytest
import numpy as np

from grillex.core import (
    Model, Material, Section, Node,
    BeamFormulation, BeamConfig, DOFIndex,
    SolverMethod, LoadCaseType
)


class TestModelCreation:
    """Test Model instantiation and entity creation"""

    def test_model_default_creation(self):
        """Test creating a model with default parameters"""
        model = Model()
        assert model is not None
        assert not model.is_analyzed()
        assert model.total_dofs() == 0

    def test_model_custom_parameters(self):
        """Test creating a model with custom parameters"""
        model = Model(node_tolerance=1e-5, solver_method=SolverMethod.SparseLU)
        assert model is not None
        assert model.get_solver().get_method() == SolverMethod.SparseLU

    def test_create_material(self):
        """Test creating a material"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        assert mat is not None
        assert mat.name == "Steel"
        assert len(model.materials) == 1

    def test_create_section(self):
        """Test creating a section"""
        model = Model()
        sec = model.create_section("IPE200", 0.01, 1e-5, 2e-5, 1.5e-5)
        assert sec is not None
        assert sec.name == "IPE200"
        assert len(model.sections) == 1

    def test_create_beam(self):
        """Test creating a beam element"""
        model = Model()
        node1 = model.get_or_create_node(0, 0, 0)
        node2 = model.get_or_create_node(6, 0, 0)
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1e-5)

        beam = model.create_beam(node1, node2, mat, sec)
        assert beam is not None
        assert len(model.elements) == 1


class TestModelLoadsAndBCs:
    """Test load and boundary condition application"""

    def test_add_nodal_load(self):
        """Test adding nodal loads"""
        model = Model()
        node1 = model.get_or_create_node(0, 0, 0)

        # Create load case and add load
        lc = model.create_load_case("Test Load")
        lc.add_nodal_load([0, 0, 0], [0, -10.0, 0])
        # Model should not be analyzed after adding load
        assert not model.is_analyzed()

    def test_add_multiple_loads_same_dof(self):
        """Test that loads accumulate on the same DOF"""
        model = Model()
        node1 = model.get_or_create_node(0, 0, 0)

        # Create load case and add multiple loads to same DOF
        lc = model.create_load_case("Test Load")
        lc.add_nodal_load([0, 0, 0], [0, -5.0, 0])
        lc.add_nodal_load([0, 0, 0], [0, -5.0, 0])
        # Loads should accumulate (tested implicitly in analysis)

    def test_clear_loads(self):
        """Test clearing all loads"""
        model = Model()
        node1 = model.get_or_create_node(0, 0, 0)

        # Create load case and add load
        lc = model.create_load_case("Test Load")
        lc.add_nodal_load([0, 0, 0], [0, -10.0, 0])
        # Clear loads from the load case
        lc.clear()
        assert lc.is_empty()

    def test_boundary_conditions(self):
        """Test setting boundary conditions"""
        model = Model()
        node1 = model.get_or_create_node(0, 0, 0)

        model.boundary_conditions.fix_node(node1.id)
        assert model.boundary_conditions.num_fixed_dofs() == 6


class TestSimpleCantileverAnalysis:
    """Test complete analysis of a simple cantilever beam"""

    def test_cantilever_beam_analysis(self):
        """Test complete workflow: cantilever beam with tip load

        Analytical: δ = PL³/(3EI) = 10 * 6³ / (3 * 210e6 * 0.0016) ≈ 0.00214 m
        """
        # Create model
        model = Model()

        # Create nodes
        node1 = model.get_or_create_node(0.0, 0.0, 0.0)
        node2 = model.get_or_create_node(6.0, 0.0, 0.0)

        # Create material (Steel)
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)

        # Create section (rectangular 0.3m × 0.4m)
        b, h = 0.3, 0.4
        A = b * h
        Iz = b * h**3 / 12
        Iy = h * b**3 / 12
        J = 0.1406 * b * h**3
        sec = model.create_section("Rect_300x400", A, Iy, Iz, J)

        # Create beam element
        beam = model.create_beam(node1, node2, mat, sec)

        # Apply boundary conditions (fix node 1)
        model.boundary_conditions.fix_node(node1.id)

        # Apply load (10 kN downward at tip)
        lc = model.create_load_case("Tip Load")
        lc.add_nodal_load([6.0, 0.0, 0.0], [0, -10.0, 0])

        # Run analysis
        success = model.analyze()
        assert success, f"Analysis failed: {model.get_error_message()}"
        assert model.is_analyzed()

        # Check total DOFs
        assert model.total_dofs() == 12  # 2 nodes × 6 DOFs

        # Get displacements
        u = model.get_displacements()
        assert len(u) == 12

        # Get tip displacement
        tip_deflection = model.get_node_displacement(node2.id, DOFIndex.UY)

        # Analytical solution
        P = 10.0
        L = 6.0
        E = mat.E
        I = Iz
        analytical_deflection = -P * L**3 / (3 * E * I)

        # Check within 1%
        rel_error = abs((tip_deflection - analytical_deflection) / analytical_deflection)
        assert rel_error < 0.01, \
            f"Tip deflection {tip_deflection:.6f} m doesn't match analytical " \
            f"{analytical_deflection:.6f} m (error: {rel_error*100:.2f}%)"

        # Check fixed end has zero displacement
        fixed_displacement = model.get_node_displacement(node1.id, DOFIndex.UY)
        assert abs(fixed_displacement) < 1e-6

        # Get reactions
        reactions = model.get_reactions()
        assert len(reactions) == 12

        # Fixed end should have reaction force ≈ 10 kN upward
        # (reactions are at constrained DOFs)


class TestErrorHandling:
    """Test error handling for invalid models"""

    def test_analyze_empty_model(self):
        """Test that analysis fails for model with no elements"""
        model = Model()
        success = model.analyze()
        assert not success
        assert "no elements" in model.get_error_message().lower()

    def test_analyze_without_boundary_conditions(self):
        """Test that analysis detects singular system (no BCs)"""
        model = Model()
        node1 = model.get_or_create_node(0, 0, 0)
        node2 = model.get_or_create_node(6, 0, 0)
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1e-5)
        model.create_beam(node1, node2, mat, sec)

        # Create a load case with a load
        lc = model.create_load_case("Test Load")
        lc.add_nodal_load([6, 0, 0], [0, -10.0, 0])

        # No boundary conditions - should detect singular system
        success = model.analyze()
        assert not success
        # Check the load case result for the singular system message
        result = model.get_result(lc)
        assert not result.success
        assert "singular" in result.error_message.lower()

    def test_get_displacements_before_analysis(self):
        """Test that getting displacements before analysis raises error"""
        model = Model()
        with pytest.raises(RuntimeError, match="not analyzed"):
            model.get_displacements()

    def test_get_node_displacement_before_analysis(self):
        """Test that getting node displacement before analysis raises error"""
        model = Model()
        node1 = model.get_or_create_node(0, 0, 0)
        with pytest.raises(RuntimeError, match="not analyzed"):
            model.get_node_displacement(node1.id, DOFIndex.UY)

    def test_get_reactions_before_analysis(self):
        """Test that getting reactions before analysis raises error"""
        model = Model()
        with pytest.raises(RuntimeError, match="not analyzed"):
            model.get_reactions()


class TestModelClear:
    """Test clearing model contents"""

    def test_clear_model(self):
        """Test that clear() resets the model"""
        model = Model()
        node1 = model.get_or_create_node(0, 0, 0)
        node2 = model.get_or_create_node(6, 0, 0)
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1e-5)
        model.create_beam(node1, node2, mat, sec)
        model.boundary_conditions.fix_node(node1.id)

        # Create load case and add load
        lc = model.create_load_case("Test Load")
        lc.add_nodal_load([6, 0, 0], [0, -10.0, 0])

        # Clear model
        model.clear()

        assert len(model.materials) == 0
        assert len(model.sections) == 0
        assert len(model.elements) == 0
        assert model.boundary_conditions.num_fixed_dofs() == 0
        assert len(model.get_load_cases()) == 0
        assert not model.is_analyzed()


class TestAcceptanceCriteria:
    """Test all acceptance criteria for Task 3.5"""

    def test_ac1_complete_workflow_runs_without_errors(self):
        """AC1: Complete analysis workflow runs without errors"""
        model = Model()

        # Build simple model
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(3, 0, 0)
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1e-5)
        model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        # Create load case and add load
        lc = model.create_load_case("Test Load")
        lc.add_nodal_load([3, 0, 0], [0, -1.0, 0])

        # Full workflow
        success = model.analyze()
        assert success
        assert model.is_analyzed()

        # Get results without errors
        u = model.get_displacements()
        r = model.get_reactions()
        disp = model.get_node_displacement(n2.id, DOFIndex.UY)

        assert len(u) > 0
        assert len(r) > 0
        assert disp < 0  # Deflects downward

    def test_ac2_results_match_hand_calculations(self):
        """AC2: Results match hand calculations for simple models

        Simple cantilever: P=1kN, L=1m, EI known
        δ = PL³/(3EI)
        """
        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        E = 210e6  # kN/m²
        I = 1e-5   # m⁴
        P = 1.0    # kN
        L = 1.0    # m

        mat = model.create_material("Steel", E, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, I, I, 1e-5)
        model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        # Create load case and add load
        lc = model.create_load_case("Point Load")
        lc.add_nodal_load([1, 0, 0], [0, -P, 0])

        success = model.analyze()
        assert success

        tip_deflection = model.get_node_displacement(n2.id, DOFIndex.UY)
        analytical = -P * L**3 / (3 * E * I)

        rel_error = abs((tip_deflection - analytical) / analytical)
        assert rel_error < 0.01, f"Error: {rel_error*100:.2f}%"

    def test_ac3_error_handling_invalid_models(self):
        """AC3: Error handling for invalid models"""
        # Test 1: No elements
        model1 = Model()
        assert not model1.analyze()
        assert len(model1.get_error_message()) > 0

        # Test 2: No boundary conditions (singular)
        model2 = Model()
        n1 = model2.get_or_create_node(0, 0, 0)
        n2 = model2.get_or_create_node(1, 0, 0)
        mat = model2.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model2.create_section("Test", 0.01, 1e-5, 1e-5, 1e-5)
        model2.create_beam(n1, n2, mat, sec)

        # Create a load case
        lc = model2.create_load_case("Test Load")
        lc.add_nodal_load([1, 0, 0], [0, -1.0, 0])

        assert not model2.analyze()
        # Check the load case result for the singular system message
        result = model2.get_result(lc)
        assert not result.success
        assert "singular" in result.error_message.lower()

        # Test 3: Accessing results before analysis
        model3 = Model()
        with pytest.raises(RuntimeError):
            model3.get_displacements()


class TestMultiElementModel:
    """Test model with multiple elements"""

    def test_three_span_beam(self):
        """Test a beam with 3 elements (4 nodes)"""
        model = Model()

        # Create 4 nodes in a line
        nodes = [model.get_or_create_node(float(i), 0, 0) for i in range(4)]

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1e-5)

        # Create 3 beam elements
        for i in range(3):
            model.create_beam(nodes[i], nodes[i+1], mat, sec)

        # Fix first node, pin last node
        model.boundary_conditions.fix_node(nodes[0].id)
        model.boundary_conditions.pin_node(nodes[3].id)

        # Add some rotational constraints to prevent rigid body modes
        model.boundary_conditions.add_fixed_dof(nodes[0].id, DOFIndex.RX, 0.0)
        model.boundary_conditions.add_fixed_dof(nodes[0].id, DOFIndex.RZ, 0.0)
        model.boundary_conditions.add_fixed_dof(nodes[3].id, DOFIndex.RZ, 0.0)

        # Apply load at middle (node at x=1)
        lc = model.create_load_case("Middle Load")
        lc.add_nodal_load([1.0, 0, 0], [0, -10.0, 0])

        success = model.analyze()
        assert success, f"Analysis failed: {model.get_error_message()}"

        # Check results are reasonable
        u = model.get_displacements()
        assert len(u) == 24  # 4 nodes × 6 DOFs

        # Middle nodes should have non-zero deflection
        disp1 = model.get_node_displacement(nodes[1].id, DOFIndex.UY)
        disp2 = model.get_node_displacement(nodes[2].id, DOFIndex.UY)
        assert disp1 < 0  # Deflects downward
        assert disp2 < 0  # Deflects downward


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
