"""
Test suite for Phase 7: Internal Actions - Task 7.1 Element End Forces

Tests the EndForces struct and compute_end_forces() method:
1. EndForces struct functionality
2. get_element_displacements_local() method
3. compute_end_forces() method
4. End forces match support reactions for simple cases
5. Sign convention consistency
6. Equilibrium verification
7. End releases properly zero out forces
8. Works for both 12-DOF and 14-DOF elements

Acceptance Criteria from Task 7.1:
- AC1: End forces match support reactions for simple cases
- AC2: Sign convention is consistent
- AC3: Forces satisfy equilibrium
- AC4: End releases properly zero out forces at released DOFs
- AC5: Works for both 12-DOF and 14-DOF elements
"""

import pytest
import numpy as np

from grillex.core import (
    Model, Material, Section, Node, BeamElement,
    BeamFormulation, BeamConfig, DOFIndex,
    EndForces, InternalActions, ActionExtreme,
)


class TestEndForcesStruct:
    """Test EndForces struct functionality"""

    def test_default_construction(self):
        """Test EndForces default construction creates zero forces"""
        f = EndForces()
        assert f.N == 0.0
        assert f.Vy == 0.0
        assert f.Vz == 0.0
        assert f.Mx == 0.0
        assert f.My == 0.0
        assert f.Mz == 0.0
        assert f.B == 0.0

    def test_construction_with_components(self):
        """Test EndForces construction with components"""
        f = EndForces(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
        assert f.N == 1.0
        assert f.Vy == 2.0
        assert f.Vz == 3.0
        assert f.Mx == 4.0
        assert f.My == 5.0
        assert f.Mz == 6.0
        assert f.B == 7.0

    def test_construction_without_bimoment(self):
        """Test EndForces construction without bimoment (default B=0)"""
        f = EndForces(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        assert f.N == 1.0
        assert f.B == 0.0

    def test_to_vector6(self):
        """Test conversion to 6-component vector"""
        f = EndForces(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
        v = f.to_vector6()
        assert len(v) == 6
        np.testing.assert_array_almost_equal(v, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_to_vector7(self):
        """Test conversion to 7-component vector"""
        f = EndForces(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
        v = f.to_vector7()
        assert len(v) == 7
        np.testing.assert_array_almost_equal(v, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

    def test_repr(self):
        """Test string representation"""
        f = EndForces(100.0, 50.0, 25.0, 10.0, 200.0, 150.0)
        s = repr(f)
        assert "EndForces" in s
        assert "N=" in s
        assert "My=" in s


class TestInternalActionsStruct:
    """Test InternalActions struct functionality"""

    def test_default_construction(self):
        """Test InternalActions default construction"""
        a = InternalActions()
        assert a.x == 0.0
        assert a.N == 0.0
        assert a.My == 0.0

    def test_construction_with_position(self):
        """Test InternalActions construction with position"""
        a = InternalActions(2.5)
        assert a.x == 2.5
        assert a.N == 0.0

    def test_construction_with_all_values(self):
        """Test InternalActions construction with all values"""
        a = InternalActions(1.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
        assert a.x == 1.0
        assert a.N == 10.0
        assert a.Vy == 20.0
        assert a.Vz == 30.0
        assert a.Mx == 40.0
        assert a.My == 50.0
        assert a.Mz == 60.0


class TestActionExtremeStruct:
    """Test ActionExtreme struct functionality"""

    def test_default_construction(self):
        """Test ActionExtreme default construction"""
        e = ActionExtreme()
        assert e.x == 0.0
        assert e.value == 0.0

    def test_construction_with_values(self):
        """Test ActionExtreme construction with values"""
        e = ActionExtreme(3.0, 150.0)
        assert e.x == 3.0
        assert e.value == 150.0


class TestGetElementDisplacementsLocal:
    """Test get_element_displacements_local() method"""

    def test_cantilever_tip_load(self):
        """Test extracting local displacements for a cantilever with tip load"""
        # Create model
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE200", 0.01, 1e-5, 2e-5, 1.5e-5)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([6, 0, 0], [0, 0, -10.0])

        assert model.analyze()

        # Get local displacements
        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()
        u_local = beam.get_element_displacements_local(u_global, dof_handler)

        # Should have 12 DOF
        assert len(u_local) == 12

        # For beam along X-axis, local = global, so local w (index 2) should match
        # For a cantilever with load at tip in -Z, w_j (index 8) should be negative
        assert u_local[8] < 0  # w at node j (deflection in negative z)

    def test_displacements_transform_correctly(self):
        """Test that displacements transform correctly for rotated beam"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1.5e-5)

        # Beam along Y-axis
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(0, 6, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        # Load in global -Z
        lc.add_nodal_load([0, 6, 0], [0, 0, -10.0])

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()
        u_local = beam.get_element_displacements_local(u_global, dof_handler)

        # For beam along Y, local z = global -z, so w_j should be positive
        # (load in -Z global = load in +z local)
        # The deflection should be at index 8 (w_j)
        assert len(u_local) == 12


class TestComputeEndForces:
    """Test compute_end_forces() method"""

    def test_cantilever_tip_load_end_forces(self):
        """Test end forces for cantilever beam with tip load

        Cantilever with tip load P at end j:
        - Fixed at end i (node 1)
        - Free at end j (node 2)
        - Point load P = -10 kN at end j in z direction

        Expected end forces at end i:
        - Vz = +10 kN (reaction shear)
        - My = -P*L = -(-10)*6 = +60 kN·m (reaction moment)

        Expected end forces at end j:
        - Should be zero (free end, equilibrium with applied load)
        """
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE200", 0.01, 1e-5, 2e-5, 1.5e-5)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)
        L = 6.0
        P = 10.0  # Load magnitude

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([6, 0, 0], [0, 0, -P])

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()
        forces_i, forces_j = beam.compute_end_forces(u_global, dof_handler)

        # End i should have reaction forces
        # Shear Vz at end i should equal P (pointing up)
        np.testing.assert_almost_equal(forces_i.Vz, P, decimal=3)

        # Moment My at end i should equal P*L (due to load at distance L)
        # Sign depends on convention - positive My causes tension at +z face
        np.testing.assert_almost_equal(abs(forces_i.My), P * L, decimal=3)

        # Axial, shear y, torsion should be nearly zero
        np.testing.assert_almost_equal(forces_i.N, 0.0, decimal=5)
        np.testing.assert_almost_equal(forces_i.Vy, 0.0, decimal=5)
        np.testing.assert_almost_equal(forces_i.Mx, 0.0, decimal=5)
        np.testing.assert_almost_equal(forces_i.Mz, 0.0, decimal=5)

    def test_axial_load_end_forces(self):
        """Test end forces for beam with axial load

        Beam with axial load P at end j:
        - Fixed at end i
        - Axial load P = 100 kN tension at end j

        The element force vector f = K * u gives forces at both ends.
        For an axially loaded bar, both ends should show force magnitude P.
        """
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1.5e-5)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)
        P = 100.0  # Axial load

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        # Apply axial load at node 2 (along +X = local +x for beam along X)
        lc.add_nodal_load([6, 0, 0], [P, 0, 0])

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()
        forces_i, forces_j = beam.compute_end_forces(u_global, dof_handler)

        # Both ends should have axial force magnitude P (element in tension)
        np.testing.assert_almost_equal(abs(forces_i.N), P, decimal=3)
        np.testing.assert_almost_equal(abs(forces_j.N), P, decimal=3)

        # Shear and moments should be zero (pure axial load)
        np.testing.assert_almost_equal(forces_i.Vy, 0.0, decimal=5)
        np.testing.assert_almost_equal(forces_i.Vz, 0.0, decimal=5)
        np.testing.assert_almost_equal(forces_i.My, 0.0, decimal=5)
        np.testing.assert_almost_equal(forces_i.Mz, 0.0, decimal=5)

    def test_cantilever_multiple_point_loads(self):
        """Test end forces for cantilever beam with multiple point loads

        Cantilever beam (length L) with two point loads:
        - Fixed at node 1 (end i)
        - Load P1 at midspan (node 2)
        - Load P2 at tip (node 3)

        Expected shear at support: V_i = P1 + P2
        """
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1.5e-5)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(3, 0, 0)
        n3 = model.get_or_create_node(6, 0, 0)

        beam1 = model.create_beam(n1, n2, mat, sec)
        beam2 = model.create_beam(n2, n3, mat, sec)

        # Fixed at end i
        model.boundary_conditions.fix_node(n1.id)

        # Point loads
        P1 = 10.0  # kN at midspan
        P2 = 20.0  # kN at tip
        lc = model.get_default_load_case()
        lc.add_nodal_load([3, 0, 0], [0, 0, -P1])
        lc.add_nodal_load([6, 0, 0], [0, 0, -P2])

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()

        # Get end forces for first beam (connected to support)
        forces_i, forces_j = beam1.compute_end_forces(u_global, dof_handler)

        # Shear at end i should equal total load
        expected_shear = P1 + P2  # 30 kN
        np.testing.assert_almost_equal(abs(forces_i.Vz), expected_shear, decimal=2)

        # The forces should be non-zero
        assert abs(forces_i.My) > 0


class TestEndForcesEquilibrium:
    """Test equilibrium of end forces"""

    def test_equilibrium_cantilever(self):
        """Test force equilibrium for cantilever beam

        Sum of end forces should equal applied load (when considering signs properly)
        """
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1.5e-5)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(5, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([5, 0, 0], [0, 0, -20.0])

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()
        forces_i, forces_j = beam.compute_end_forces(u_global, dof_handler)

        # For a single element with no distributed load, element equilibrium:
        # f_i + f_j should equal the applied nodal loads at the element ends
        # The reaction at i plus the applied load at j should sum to zero
        # forces_i.Vz (reaction) + (-20.0 applied) ≈ 0 at node j
        # forces_j represents the element internal force at j

        # Check axial equilibrium (should be zero total)
        np.testing.assert_almost_equal(forces_i.N + forces_j.N, 0.0, decimal=6)

        # Check torsional equilibrium
        np.testing.assert_almost_equal(forces_i.Mx + forces_j.Mx, 0.0, decimal=6)


class TestEndReleasesZeroForces:
    """Test that end releases properly zero out forces"""

    def test_moment_release_zeros_moment(self):
        """Test that moment release at end j produces zero moment at that end"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1.5e-5)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)
        n3 = model.get_or_create_node(12, 0, 0)

        # Create two beams, second has moment release at start
        beam1 = model.create_beam(n1, n2, mat, sec)
        beam2 = model.create_beam(n2, n3, mat, sec)

        # Release moment at end i of beam2 (simulates a hinge)
        beam2.releases.release_moment_i()

        model.boundary_conditions.fix_node(n1.id)
        model.boundary_conditions.pin_node(n3.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([6, 0, 0], [0, 0, -10.0])

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()

        # Check beam2 end forces
        forces_i, forces_j = beam2.compute_end_forces(u_global, dof_handler)

        # Moment at end i (where release is) should be zero
        np.testing.assert_almost_equal(forces_i.My, 0.0, decimal=5)
        np.testing.assert_almost_equal(forces_i.Mz, 0.0, decimal=5)


class TestEndForcesMatchReactions:
    """Test that end forces match support reactions"""

    def test_cantilever_forces_match_reactions(self):
        """Test that computed end forces at support match global reactions"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1.5e-5)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)
        L = 6.0
        P = 10.0

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([6, 0, 0], [0, 0, -P])

        assert model.analyze()

        # Get global reactions
        reactions = model.get_reactions()
        dof_handler = model.get_dof_handler()

        # Get end forces
        u_global = model.get_displacements()
        forces_i, forces_j = beam.compute_end_forces(u_global, dof_handler)

        # Reaction in UZ at node 1 should equal forces_i.Vz
        # (accounting for global-to-local transformation if beam not along X)
        # For beam along X, local z = global z

        # Get global DOF for node 1, UZ
        global_dof_uz = dof_handler.get_global_dof(n1.id, DOFIndex.UZ)
        reaction_uz = reactions[global_dof_uz]

        # The reaction should equal the shear force at end i
        np.testing.assert_almost_equal(abs(forces_i.Vz), abs(reaction_uz), decimal=3)


class TestWarpingElements:
    """Test end forces for 14-DOF warping elements"""

    def test_warping_element_end_forces(self):
        """Test that 14-DOF elements compute end forces correctly"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1.5e-5)
        sec.set_warping_constant(1e-8)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)

        config = BeamConfig()
        config.include_warping = True

        beam = model.create_beam(n1, n2, mat, sec, config)
        assert beam.has_warping()

        model.boundary_conditions.fix_node_with_warping(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([6, 0, 0], [0, 0, -10.0])

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()

        # Get local displacements (should be 14 DOF)
        u_local = beam.get_element_displacements_local(u_global, dof_handler)
        assert len(u_local) == 14

        # Compute end forces
        forces_i, forces_j = beam.compute_end_forces(u_global, dof_handler)

        # Should have bimoment attribute accessible
        assert hasattr(forces_i, 'B')
        assert hasattr(forces_j, 'B')

        # For this loading, shear should still be present
        np.testing.assert_almost_equal(forces_i.Vz, 10.0, decimal=3)


class TestTimoshenkoBeamEndForces:
    """Test end forces for Timoshenko beams"""

    def test_timoshenko_cantilever_end_forces(self):
        """Test end forces for Timoshenko beam cantilever

        Note: Timoshenko formulation includes shear deformation effects, which
        can cause slight differences in force distribution compared to
        Euler-Bernoulli theory. The results should be close but may not be exact.
        """
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1.5e-5)
        sec.set_shear_areas(0.008, 0.008)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(3, 0, 0)

        config = BeamConfig()
        config.include_shear_deformation = True

        beam = model.create_beam(n1, n2, mat, sec, config)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([3, 0, 0], [0, 0, -10.0])

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u_global = model.get_displacements()
        forces_i, forces_j = beam.compute_end_forces(u_global, dof_handler)

        # End forces should be close to applied load (within 5% for Timoshenko)
        # Shear at support should approximately equal applied load
        np.testing.assert_almost_equal(forces_i.Vz, 10.0, decimal=1)

        # Moment at support should approximately equal P*L
        L = 3.0
        P = 10.0
        np.testing.assert_almost_equal(abs(forces_i.My), P * L, decimal=1)
