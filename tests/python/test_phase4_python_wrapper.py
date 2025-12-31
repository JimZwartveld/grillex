"""
Test suite for Phase 4: Python Model Wrapper (Task 4.1)

Tests the Python wrapper layer that provides a more Pythonic API:
1. Beam class for multi-element beam representation
2. StructuralModel wrapper with convenience methods
3. Coordinate-based beam creation
4. Simplified load and BC application
5. Material/section library management

Acceptance Criteria (Task 4.1):
- AC1: Beams can be created with coordinate lists
- AC2: Model can be analyzed from Python
- AC3: Results are accessible from Python
"""

import pytest
import numpy as np

from grillex.core import (
    StructuralModel, Beam,
    DOFIndex, LoadCaseType
)


class TestBeamClass:
    """Test the Beam abstraction class"""

    def test_beam_creation(self):
        """Test creating a Beam object"""
        from grillex._grillex_cpp import Material, Section

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam = Beam([0, 0, 0], [6, 0, 0], sec, mat, beam_id=1)

        assert beam.beam_id == 1
        assert np.allclose(beam.start_pos, [0, 0, 0])
        assert np.allclose(beam.end_pos, [6, 0, 0])
        assert beam.length == pytest.approx(6.0)
        assert len(beam.elements) == 0

    def test_beam_length_calculation(self):
        """Test beam length calculation for various orientations"""
        from grillex._grillex_cpp import Material, Section

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Horizontal beam
        beam1 = Beam([0, 0, 0], [6, 0, 0], sec, mat)
        assert beam1.length == pytest.approx(6.0)

        # Vertical beam
        beam2 = Beam([0, 0, 0], [0, 0, 4], sec, mat)
        assert beam2.length == pytest.approx(4.0)

        # Diagonal beam
        beam3 = Beam([0, 0, 0], [3, 4, 0], sec, mat)
        assert beam3.length == pytest.approx(5.0)

    def test_beam_direction_vector(self):
        """Test beam direction vector computation"""
        from grillex._grillex_cpp import Material, Section

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam = Beam([0, 0, 0], [6, 0, 0], sec, mat)
        direction = beam.get_direction()

        assert np.allclose(direction, [1, 0, 0])
        assert np.linalg.norm(direction) == pytest.approx(1.0)

    def test_beam_midpoint(self):
        """Test beam midpoint calculation"""
        from grillex._grillex_cpp import Material, Section

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam = Beam([0, 0, 0], [6, 0, 0], sec, mat)
        midpoint = beam.get_midpoint()

        assert np.allclose(midpoint, [3, 0, 0])

    def test_beam_repr(self):
        """Test beam string representation"""
        from grillex._grillex_cpp import Material, Section

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam = Beam([0, 0, 0], [6, 0, 0], sec, mat, beam_id=1)
        repr_str = repr(beam)

        assert "Beam" in repr_str
        assert "id=1" in repr_str
        assert "length=6.000m" in repr_str


class TestStructuralModelCreation:
    """Test StructuralModel instantiation and basic properties"""

    def test_model_creation(self):
        """Test creating a StructuralModel"""
        model = StructuralModel(name="Test Model")

        assert model.name == "Test Model"
        assert model.num_nodes() == 0
        assert model.num_elements() == 0
        assert model.num_beams() == 0
        assert not model.is_analyzed()

    def test_model_repr(self):
        """Test model string representation"""
        model = StructuralModel(name="My Bridge")
        repr_str = repr(model)

        assert "StructuralModel" in repr_str
        assert "name='My Bridge'" in repr_str
        assert "analyzed=False" in repr_str


class TestMaterialAndSectionLibrary:
    """Test material and section library management"""

    def test_add_material(self):
        """Test adding a material to the library"""
        model = StructuralModel()
        mat = model.add_material("Steel", 210e6, 0.3, 7.85e-6)

        assert mat is not None
        assert mat.name == "Steel"
        assert model.get_material("Steel") is mat

    def test_add_duplicate_material(self):
        """Test that adding duplicate material returns existing one"""
        model = StructuralModel()
        mat1 = model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        mat2 = model.add_material("Steel", 210e6, 0.3, 7.85e-6)

        assert mat1 is mat2

    def test_add_section(self):
        """Test adding a section to the library"""
        model = StructuralModel()
        sec = model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        assert sec is not None
        assert sec.name == "IPE300"
        assert model.get_section("IPE300") is sec

    def test_add_duplicate_section(self):
        """Test that adding duplicate section returns existing one"""
        model = StructuralModel()
        sec1 = model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        sec2 = model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        assert sec1 is sec2

    def test_get_nonexistent_material(self):
        """Test getting a material that doesn't exist"""
        model = StructuralModel()
        mat = model.get_material("NonExistent")

        assert mat is None

    def test_get_nonexistent_section(self):
        """Test getting a section that doesn't exist"""
        model = StructuralModel()
        sec = model.get_section("NonExistent")

        assert sec is None


class TestBeamCreation:
    """Test beam creation via coordinates (Acceptance Criterion 1)"""

    def test_add_beam_by_coords(self):
        """Test adding a beam using coordinates - AC1"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam = model.add_beam_by_coords(
            [0, 0, 0], [6, 0, 0],
            section_name="IPE300",
            material_name="Steel"
        )

        assert beam is not None
        assert model.num_beams() == 1
        assert model.num_elements() == 1
        assert model.num_nodes() == 2

    def test_add_multiple_beams(self):
        """Test adding multiple beams"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam1 = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        beam2 = model.add_beam_by_coords([6, 0, 0], [12, 0, 0], "IPE300", "Steel")

        assert model.num_beams() == 2
        assert model.num_elements() == 2
        assert model.num_nodes() == 3  # Shared node at [6,0,0]

    def test_add_beam_nonexistent_material(self):
        """Test error when material doesn't exist"""
        model = StructuralModel()

        with pytest.raises(ValueError, match="Material.*not found"):
            model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

    def test_add_beam_nonexistent_section(self):
        """Test error when section doesn't exist"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)

        with pytest.raises(ValueError, match="Section.*not found"):
            model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")


class TestBoundaryConditions:
    """Test boundary condition application via coordinates"""

    def test_fix_node_at(self):
        """Test fixing a node at coordinates"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        model.fix_node_at([0, 0, 0])
        assert model.boundary_conditions.num_fixed_dofs() == 6

    def test_pin_node_at(self):
        """Test pinning a node at coordinates"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        model.pin_node_at([0, 0, 0])
        assert model.boundary_conditions.num_fixed_dofs() == 3

    def test_fix_specific_dof(self):
        """Test fixing a specific DOF at coordinates"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        model.fix_dof_at([0, 0, 0], DOFIndex.UY)
        assert model.boundary_conditions.num_fixed_dofs() == 1

    def test_fix_node_at_nonexistent(self):
        """Test error when trying to fix nonexistent node"""
        model = StructuralModel()

        with pytest.raises(ValueError, match="No node found"):
            model.fix_node_at([100, 100, 100])


class TestLoadApplication:
    """Test load application via coordinates"""

    def test_create_load_case(self):
        """Test creating a load case"""
        model = StructuralModel()
        lc = model.create_load_case("Dead Load", LoadCaseType.Permanent)

        assert lc is not None
        assert lc.name == "Dead Load"
        assert lc.type == LoadCaseType.Permanent

    def test_get_default_load_case(self):
        """Test getting default load case"""
        model = StructuralModel()
        lc = model.get_default_load_case()

        assert lc is not None

    def test_add_point_load(self):
        """Test adding a point load at coordinates"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        # Should not raise an error

    def test_add_point_load_no_force_or_moment(self):
        """Test error when neither force nor moment is provided"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        with pytest.raises(ValueError, match="At least one of 'force' or 'moment'"):
            model.add_point_load([6, 0, 0])


class TestAnalysisWorkflow:
    """Test complete analysis workflow (Acceptance Criterion 2)"""

    def test_simple_cantilever_analysis(self):
        """Test complete cantilever analysis workflow - AC2"""
        # Create model
        model = StructuralModel(name="Cantilever Test")

        # Add materials and sections
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        b, h = 0.3, 0.4
        A = b * h
        Iz = b * h**3 / 12
        Iy = h * b**3 / 12
        J = 0.1406 * b * h**3
        model.add_section("Rect_300x400", A, Iy, Iz, J)

        # Add beam
        beam = model.add_beam_by_coords(
            [0, 0, 0], [6, 0, 0],
            "Rect_300x400", "Steel"
        )

        # Apply boundary conditions
        model.fix_node_at([0, 0, 0])

        # Apply load
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])

        # Analyze
        success = model.analyze()
        assert success
        assert model.is_analyzed()

        # Check DOF count
        assert model.total_dofs() == 12  # 2 nodes × 6 DOFs

    def test_multi_beam_analysis(self):
        """Test analysis of multi-beam structure - AC2"""
        model = StructuralModel(name="Two-span beam")

        # Setup
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Add beams
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        model.add_beam_by_coords([6, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Boundary conditions
        model.fix_node_at([0, 0, 0])
        model.pin_node_at([12, 0, 0])

        # Loads
        model.add_point_load([6, 0, 0], force=[0, -20.0, 0])

        # Analyze
        success = model.analyze()
        assert success


class TestResultsAccess:
    """Test results access (Acceptance Criterion 3)"""

    def test_get_displacement_at(self):
        """Test getting displacement at coordinates - AC3"""
        # Setup simple cantilever
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])

        # Analyze
        model.analyze()

        # Get displacement
        disp = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
        assert disp < 0  # Downward displacement
        assert abs(disp) > 0  # Non-zero

    def test_get_all_displacements(self):
        """Test getting global displacement vector - AC3"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])

        model.analyze()

        # Get all displacements
        u = model.get_all_displacements()
        assert isinstance(u, np.ndarray)
        assert len(u) == 12  # 2 nodes × 6 DOFs

    def test_get_reactions(self):
        """Test getting reaction forces - AC3"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])

        model.analyze()

        # Get reactions
        R = model.get_reactions()
        assert isinstance(R, np.ndarray)
        assert len(R) == 12

    def test_access_cpp_model(self):
        """Test accessing underlying C++ model"""
        model = StructuralModel()
        cpp_model = model.cpp_model

        assert cpp_model is not None
        # Should be able to use it like the C++ model
        assert hasattr(cpp_model, 'analyze')


class TestAcceptanceCriteria:
    """Explicit tests for all acceptance criteria"""

    def test_ac1_beams_created_with_coordinates(self):
        """AC1: Beams can be created with coordinate lists"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam with coordinate lists (not node references)
        beam = model.add_beam_by_coords(
            start_pos=[0.0, 0.0, 0.0],
            end_pos=[6.0, 0.0, 0.0],
            section_name="IPE300",
            material_name="Steel"
        )

        assert beam is not None
        assert beam.length == pytest.approx(6.0)
        assert np.allclose(beam.start_pos, [0, 0, 0])
        assert np.allclose(beam.end_pos, [6, 0, 0])

    def test_ac2_model_analyzed_from_python(self):
        """AC2: Model can be analyzed from Python"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])

        # Model can be analyzed from Python
        success = model.analyze()
        assert success
        assert model.is_analyzed()

    def test_ac3_results_accessible_from_python(self):
        """AC3: Results are accessible from Python"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        # Results are accessible from Python
        # Via coordinates (Pythonic)
        disp = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
        assert isinstance(disp, float)

        # Via global vector
        u = model.get_all_displacements()
        assert isinstance(u, np.ndarray)

        # Reactions
        R = model.get_reactions()
        assert isinstance(R, np.ndarray)
