"""
Test suite for Phase 4: Beam Subdivision at Internal Nodes (Task 4.4)

Tests the automatic beam subdivision functionality:
1. Detection of internal nodes along beam lines
2. Splitting beams at internal nodes
3. Property propagation to sub-beams

Acceptance Criteria (Task 4.4):
- AC1: Beam with one internal node becomes two elements
- AC2: Beam with multiple internal nodes becomes multiple elements
- AC3: Section/material properties propagate to sub-beams

Requirements: R-MOD-005
"""

import pytest
import numpy as np

from grillex.core import (
    StructuralModel, Beam,
    DOFIndex
)


class TestPointOnLineSegment:
    """Test the _point_on_line_segment helper method"""

    def test_point_on_horizontal_line(self):
        """Test point detection on horizontal line segment"""
        model = StructuralModel()

        start = np.array([0.0, 0.0, 0.0])
        end = np.array([10.0, 0.0, 0.0])

        # Point at midpoint
        point = np.array([5.0, 0.0, 0.0])
        is_on, t = model._point_on_line_segment(point, start, end)
        assert is_on is True
        assert t == pytest.approx(0.5)

        # Point at 25%
        point = np.array([2.5, 0.0, 0.0])
        is_on, t = model._point_on_line_segment(point, start, end)
        assert is_on is True
        assert t == pytest.approx(0.25)

    def test_point_at_endpoints_not_detected(self):
        """Test that points at endpoints are not considered internal"""
        model = StructuralModel()

        start = np.array([0.0, 0.0, 0.0])
        end = np.array([10.0, 0.0, 0.0])

        # Point at start
        is_on, _ = model._point_on_line_segment(start, start, end)
        assert is_on is False

        # Point at end
        is_on, _ = model._point_on_line_segment(end, start, end)
        assert is_on is False

    def test_point_off_line_not_detected(self):
        """Test that points not on the line are not detected"""
        model = StructuralModel()

        start = np.array([0.0, 0.0, 0.0])
        end = np.array([10.0, 0.0, 0.0])

        # Point above line
        point = np.array([5.0, 1.0, 0.0])
        is_on, _ = model._point_on_line_segment(point, start, end)
        assert is_on is False

        # Point beside line
        point = np.array([5.0, 0.0, 1.0])
        is_on, _ = model._point_on_line_segment(point, start, end)
        assert is_on is False

    def test_point_outside_segment_not_detected(self):
        """Test that points outside the segment are not detected"""
        model = StructuralModel()

        start = np.array([0.0, 0.0, 0.0])
        end = np.array([10.0, 0.0, 0.0])

        # Point before start
        point = np.array([-5.0, 0.0, 0.0])
        is_on, _ = model._point_on_line_segment(point, start, end)
        assert is_on is False

        # Point after end
        point = np.array([15.0, 0.0, 0.0])
        is_on, _ = model._point_on_line_segment(point, start, end)
        assert is_on is False

    def test_diagonal_line_segment(self):
        """Test point detection on diagonal line segment"""
        model = StructuralModel()

        start = np.array([0.0, 0.0, 0.0])
        end = np.array([6.0, 8.0, 0.0])  # 3-4-5 triangle scaled by 2

        # Point at midpoint
        point = np.array([3.0, 4.0, 0.0])
        is_on, t = model._point_on_line_segment(point, start, end)
        assert is_on is True
        assert t == pytest.approx(0.5)

    def test_3d_line_segment(self):
        """Test point detection on 3D line segment"""
        model = StructuralModel()

        start = np.array([0.0, 0.0, 0.0])
        end = np.array([4.0, 4.0, 4.0])

        # Point at midpoint
        point = np.array([2.0, 2.0, 2.0])
        is_on, t = model._point_on_line_segment(point, start, end)
        assert is_on is True
        assert t == pytest.approx(0.5)


class TestFindInternalNodes:
    """Test the _find_internal_nodes method"""

    def test_no_internal_nodes(self):
        """Test beam with no internal nodes"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        internal_nodes = model._find_internal_nodes(beam)
        assert len(internal_nodes) == 0

    def test_one_internal_node(self):
        """Test beam with one internal node"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam first
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create internal node at midpoint
        model.get_or_create_node(6, 0, 0)

        internal_nodes = model._find_internal_nodes(beam)
        assert len(internal_nodes) == 1
        assert internal_nodes[0][0] == pytest.approx(6.0)  # Distance from start

    def test_multiple_internal_nodes(self):
        """Test beam with multiple internal nodes"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam first
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create internal nodes at 4m and 8m (out of order)
        model.get_or_create_node(8, 0, 0)
        model.get_or_create_node(4, 0, 0)

        internal_nodes = model._find_internal_nodes(beam)
        assert len(internal_nodes) == 2

        # Should be sorted by distance
        assert internal_nodes[0][0] == pytest.approx(4.0)
        assert internal_nodes[1][0] == pytest.approx(8.0)

    def test_node_not_on_line_ignored(self):
        """Test that nodes not on the beam line are ignored"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam along X axis
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create node NOT on beam line
        model.get_or_create_node(6, 1, 0)  # 1m off to the side

        internal_nodes = model._find_internal_nodes(beam)
        assert len(internal_nodes) == 0


class TestSplitBeamAtNodes:
    """Test the _split_beam_at_nodes method"""

    def test_split_beam_at_one_node(self):
        """Test splitting beam at one internal node"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create internal node
        internal_node = model.get_or_create_node(6, 0, 0)

        # Get internal nodes
        internal_nodes = model._find_internal_nodes(beam)

        # Split the beam
        new_beams = model._split_beam_at_nodes(beam, internal_nodes)

        assert len(new_beams) == 2

        # Check lengths
        assert new_beams[0].length == pytest.approx(6.0)
        assert new_beams[1].length == pytest.approx(6.0)

        # Check each beam has an element
        assert len(new_beams[0].elements) == 1
        assert len(new_beams[1].elements) == 1

    def test_split_beam_at_multiple_nodes(self):
        """Test splitting beam at multiple internal nodes"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create internal nodes at 4m and 8m
        model.get_or_create_node(4, 0, 0)
        model.get_or_create_node(8, 0, 0)

        # Get internal nodes
        internal_nodes = model._find_internal_nodes(beam)

        # Split the beam
        new_beams = model._split_beam_at_nodes(beam, internal_nodes)

        assert len(new_beams) == 3

        # Check lengths (each should be 4m)
        for beam in new_beams:
            assert beam.length == pytest.approx(4.0)


class TestSubdivideBeams:
    """Test the public subdivide_beams method"""

    def test_subdivide_single_beam_one_internal_node(self):
        """Test subdividing a single beam with one internal node"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create internal node
        model.get_or_create_node(6, 0, 0)

        # Verify initial state
        assert len(model.beams) == 1
        assert model.num_elements() == 1

        # Subdivide
        n_split = model.subdivide_beams()

        assert n_split == 1
        assert len(model.beams) == 2
        # Old element removed, 2 new elements created
        assert model.num_elements() == 2

    def test_subdivide_single_beam_multiple_internal_nodes(self):
        """Test subdividing a single beam with multiple internal nodes"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create internal nodes
        model.get_or_create_node(3, 0, 0)
        model.get_or_create_node(6, 0, 0)
        model.get_or_create_node(9, 0, 0)

        # Verify initial state
        assert len(model.beams) == 1

        # Subdivide
        n_split = model.subdivide_beams()

        assert n_split == 1
        assert len(model.beams) == 4  # 4 segments

    def test_subdivide_no_internal_nodes(self):
        """Test subdivide_beams when no internal nodes exist"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Verify initial state
        assert len(model.beams) == 1

        # Subdivide - should do nothing
        n_split = model.subdivide_beams()

        assert n_split == 0
        assert len(model.beams) == 1

    def test_subdivide_multiple_beams(self):
        """Test subdividing multiple beams"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create two beams
        model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")
        model.add_beam_by_coords([0, 0, 0], [0, 12, 0], "IPE300", "Steel")

        # Create internal nodes for both beams
        model.get_or_create_node(6, 0, 0)   # On first beam
        model.get_or_create_node(0, 6, 0)   # On second beam

        # Verify initial state
        assert len(model.beams) == 2

        # Subdivide
        n_split = model.subdivide_beams()

        assert n_split == 2  # Both beams were split
        assert len(model.beams) == 4  # Each becomes 2


class TestPropertyPropagation:
    """Test that material and section properties propagate to sub-beams"""

    def test_material_propagates(self):
        """Test that material properties propagate to sub-beams"""
        model = StructuralModel()
        mat = model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create internal node
        model.get_or_create_node(6, 0, 0)

        # Subdivide
        model.subdivide_beams()

        # Check material on both sub-beams
        for beam in model.beams:
            assert beam.material.name == "Steel"
            assert beam.material.E == pytest.approx(210e6)

    def test_section_propagates(self):
        """Test that section properties propagate to sub-beams"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create internal node
        model.get_or_create_node(6, 0, 0)

        # Subdivide
        model.subdivide_beams()

        # Check section on both sub-beams
        for beam in model.beams:
            assert beam.section.name == "IPE300"
            assert beam.section.A == pytest.approx(0.01)


class TestSubdivisionWithAnalysis:
    """Test that beam subdivision works correctly with analysis"""

    def test_subdivided_cantilever_analysis(self):
        """Test analysis of a subdivided cantilever beam"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        # Create internal node at midpoint
        model.get_or_create_node(3, 0, 0)

        # Subdivide
        n_split = model.subdivide_beams()
        assert n_split == 1

        # Apply boundary conditions
        model.fix_node_at([0, 0, 0])
        # Also fix twist to prevent rigid body rotation
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ, 0.0)

        # Apply load at tip
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])

        # Analyze
        success = model.analyze()
        assert success is True

        # Check results - tip should deflect
        tip_disp = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
        assert tip_disp < 0  # Should deflect downward

        # Midpoint should also deflect but less than tip
        mid_disp = model.get_displacement_at([3, 0, 0], DOFIndex.UY)
        assert mid_disp < 0
        assert abs(mid_disp) < abs(tip_disp)


class TestAcceptanceCriteria:
    """Test the acceptance criteria from Task 4.4"""

    def test_ac1_one_internal_node_becomes_two_elements(self):
        """AC1: Beam with one internal node becomes two elements"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create one internal node
        model.get_or_create_node(6, 0, 0)

        # Initial: 1 beam, 1 element
        assert len(model.beams) == 1
        assert model.num_elements() == 1

        # Subdivide
        model.subdivide_beams()

        # After: 2 beams, 2 elements
        assert len(model.beams) == 2
        assert model.num_elements() == 2

    def test_ac2_multiple_internal_nodes_becomes_multiple_elements(self):
        """AC2: Beam with multiple internal nodes becomes multiple elements"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create three internal nodes
        model.get_or_create_node(3, 0, 0)
        model.get_or_create_node(6, 0, 0)
        model.get_or_create_node(9, 0, 0)

        # Initial: 1 beam
        assert len(model.beams) == 1

        # Subdivide
        model.subdivide_beams()

        # After: 4 beams (4 segments)
        assert len(model.beams) == 4
        assert model.num_elements() == 4

    def test_ac3_properties_propagate_to_sub_beams(self):
        """AC3: Section/material properties propagate to sub-beams"""
        model = StructuralModel()
        model.add_material("CustomSteel", 205e6, 0.29, 7.8e-6)
        model.add_section("HEB200", 0.0078, 5.7e-5, 2.0e-5, 1.2e-7)

        # Create beam
        model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "HEB200", "CustomSteel")

        # Create internal node
        model.get_or_create_node(6, 0, 0)

        # Subdivide
        model.subdivide_beams()

        # Verify all sub-beams have correct material
        for beam in model.beams:
            assert beam.material.name == "CustomSteel"
            assert beam.material.E == pytest.approx(205e6)
            assert beam.material.nu == pytest.approx(0.29)
            assert beam.material.rho == pytest.approx(7.8e-6)

        # Verify all sub-beams have correct section
        for beam in model.beams:
            assert beam.section.name == "HEB200"
            assert beam.section.A == pytest.approx(0.0078)
            assert beam.section.Iy == pytest.approx(5.7e-5)
            assert beam.section.Iz == pytest.approx(2.0e-5)
            assert beam.section.J == pytest.approx(1.2e-7)


class TestEdgeCases:
    """Test edge cases and boundary conditions"""

    def test_near_endpoint_tolerance(self):
        """Test that nodes very close to endpoints are not considered internal"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create node very close to start (within tolerance)
        model.get_or_create_node(1e-9, 0, 0)

        internal_nodes = model._find_internal_nodes(beam)
        assert len(internal_nodes) == 0

    def test_near_line_tolerance(self):
        """Test that nodes very close to line but not on it are not detected"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam along X axis
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create node slightly above line (more than tolerance)
        model.get_or_create_node(6, 1e-3, 0)  # 1mm above

        internal_nodes = model._find_internal_nodes(beam)
        assert len(internal_nodes) == 0

    def test_very_close_to_line(self):
        """Test that nodes very close to line (within tolerance) are detected"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam along X axis
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Create node very slightly above line (within tolerance)
        model.get_or_create_node(6, 1e-9, 0)

        internal_nodes = model._find_internal_nodes(beam)
        assert len(internal_nodes) == 1

    def test_empty_beams_list(self):
        """Test subdivide_beams with no beams in model"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Don't create any beams
        n_split = model.subdivide_beams()

        assert n_split == 0
        assert len(model.beams) == 0
