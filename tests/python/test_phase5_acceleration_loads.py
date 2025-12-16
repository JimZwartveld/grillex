"""
Tests for Phase 5 Task 5.3: Acceleration Field Loads

Tests cover:
- Gravity loads (az = -9.81 m/s²)
- Rotational acceleration effects
- Combination with other load types
- Verification against analytical solutions

Acceptance Criteria:
- Gravity load produces correct weight forces
- Rotational acceleration produces centrifugal effects
- Results match: 1 mT/m beam with gravity → 9.81 kN/m equivalent load
"""

import pytest
import numpy as np
from grillex.core import (
    StructuralModel,
    DOFIndex,
    LoadCaseType,
)


class TestGravityLoads:
    """Test gravity loading via acceleration field."""

    def test_cantilever_gravity_basic(self):
        """Test cantilever beam under gravity - basic check."""
        model = StructuralModel()

        # Create material: Steel with density 7.85 mT/m³ (7850 kg/m³)
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)

        # Create section: A = 0.01 m² (100 cm²)
        model.add_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        # Create beam: 10m long along X
        beam = model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "Test", "Steel")

        # Fix start
        model.fix_node_at([0, 0, 0])

        # Create load case with gravity
        lc = model.create_load_case("Gravity", LoadCaseType.Permanent)

        # Set gravity acceleration: g = 9.81 m/s² downward (negative Z)
        accel = np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0])
        ref_point = np.array([0.0, 0.0, 0.0])
        lc.set_acceleration_field(accel, ref_point)

        # Analyze
        success = model.analyze()
        assert success, "Analysis failed"

        # Set active load case and get tip displacement
        model.set_active_load_case(lc)
        disp_z = model.get_displacement_at([10, 0, 0], DOFIndex.UZ)

        # The beam should deflect downward (negative Z)
        assert disp_z < 0, "Tip should deflect downward under gravity"

    def test_cantilever_gravity_reactions(self):
        """Test that gravity reactions match beam weight."""
        model = StructuralModel()

        # Create material with density 1.0 mT/m³ for easy calculation
        model.add_material("Test", 210e6, 0.3, 1.0)

        # Create section: A = 0.01 m² → mass per length = 0.01 * 1.0 = 0.01 mT/m
        model.add_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        # Create beam: 10m long
        beam = model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "Test", "Test")

        # Fix start fully
        model.fix_node_at([0, 0, 0])

        # Create load case with gravity
        lc = model.create_load_case("Gravity", LoadCaseType.Permanent)
        accel = np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0])
        lc.set_acceleration_field(accel, np.array([0.0, 0.0, 0.0]))

        # Analyze
        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        reactions = model.get_reactions_at([0, 0, 0])

        # Total beam weight:
        # mass = rho * A * L = 1.0 * 0.01 * 10 = 0.1 mT
        # Weight = 0.1 * 9.81 = 0.981 kN
        rho = 1.0
        A = 0.01
        L = 10.0
        g = 9.81
        total_weight = rho * A * L * g

        # Reaction should equal weight (positive reaction to resist negative load)
        reaction_z = reactions[DOFIndex.UZ]
        assert abs(reaction_z - total_weight) < 0.01 * total_weight, \
            f"Reaction {reaction_z} should equal weight {total_weight}"

    def test_simply_supported_gravity(self):
        """Test simply supported beam under gravity - reactions at both ends."""
        model = StructuralModel()

        # Material with density that gives nice numbers
        model.add_material("Test", 210e6, 0.3, 0.001)

        # A = 0.1 m²
        model.add_section("Test", 0.1, 1e-3, 1e-3, 1e-4)

        # 6m beam
        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Test")

        # Pin-roller supports with torsional restraint
        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)  # Torsional restraint
        model.fix_dof_at([6, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([6, 0, 0], DOFIndex.UZ, 0.0)

        # Gravity
        lc = model.create_load_case("Gravity", LoadCaseType.Permanent)
        lc.set_acceleration_field(
            np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0])
        )

        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        r1 = model.get_reactions_at([0, 0, 0])
        r2 = model.get_reactions_at([6, 0, 0])

        # Total weight
        rho = 0.001
        A = 0.1
        L = 6.0
        total_weight = rho * A * L * 9.81

        # For simply supported beam, each support takes half the load
        assert abs(r1[DOFIndex.UZ] - total_weight / 2) < 0.01 * total_weight, \
            f"R1 = {r1[DOFIndex.UZ]}, expected {total_weight / 2}"
        assert abs(r2[DOFIndex.UZ] - total_weight / 2) < 0.01 * total_weight, \
            f"R2 = {r2[DOFIndex.UZ]}, expected {total_weight / 2}"

    def test_gravity_equivalent_to_distributed_load(self):
        """Test that gravity produces equivalent results to distributed load."""
        # Create two identical models
        model_grav = StructuralModel()
        model_dist = StructuralModel()

        # Same material and section
        rho = 0.01  # mT/m³
        A = 0.1     # m²

        model_grav.add_material("Steel", 210e6, 0.3, rho)
        model_grav.add_section("Test", A, 1e-3, 1e-3, 1e-4)

        model_dist.add_material("Steel", 210e6, 0.3, rho)
        model_dist.add_section("Test", A, 1e-3, 1e-3, 1e-4)

        # Same geometry
        L = 6.0
        beam_g = model_grav.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel")
        beam_d = model_dist.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel")

        # Same BCs: cantilever
        model_grav.fix_node_at([0, 0, 0])
        model_dist.fix_node_at([0, 0, 0])

        # Gravity model: use acceleration field
        lc_g = model_grav.create_load_case("Gravity", LoadCaseType.Permanent)
        g = 9.81
        lc_g.set_acceleration_field(
            np.array([0.0, 0.0, -g, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0])
        )

        # Distributed load model: equivalent distributed load
        # w = rho * A * g (weight per unit length)
        w = rho * A * g  # kN/m (downward)
        lc_d = model_dist.create_load_case("Distributed", LoadCaseType.Permanent)
        model_dist.add_line_load(beam_d, (0.0, 0.0, -w), load_case=lc_d)

        # Analyze both
        assert model_grav.analyze(), "Gravity model failed"
        assert model_dist.analyze(), "Distributed model failed"

        model_grav.set_active_load_case(lc_g)
        model_dist.set_active_load_case(lc_d)

        deflection_g = model_grav.get_displacement_at([L, 0, 0], DOFIndex.UZ)
        deflection_d = model_dist.get_displacement_at([L, 0, 0], DOFIndex.UZ)

        # Tip deflections should be very close
        # (consistent mass vs work-equivalent loads may differ slightly)
        # Allow 5% tolerance due to consistent vs lumped mass differences
        rel_diff = abs(deflection_g - deflection_d) / max(abs(deflection_d), 1e-10)
        assert rel_diff < 0.05, \
            f"Gravity deflection {deflection_g} differs from distributed {deflection_d} by {rel_diff*100:.1f}%"


class TestRotationalAcceleration:
    """Test rotational acceleration effects."""

    def test_angular_acceleration_produces_forces(self):
        """Test that angular acceleration produces tangential forces."""
        model = StructuralModel()

        model.add_material("Steel", 210e6, 0.3, 0.01)
        model.add_section("Test", 0.1, 1e-3, 1e-3, 1e-4)

        # Beam at radial distance from rotation axis
        beam = model.add_beam_by_coords([5, 0, 0], [5, 6, 0], "Test", "Steel")

        # Fix both ends (to see effect in forces)
        model.fix_node_at([5, 0, 0])
        model.fix_node_at([5, 6, 0])

        # Apply angular acceleration about Z axis
        lc = model.create_load_case("Rotation", LoadCaseType.Variable)
        lc.set_acceleration_field(
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),  # α_z = 1 rad/s²
            np.array([0.0, 0.0, 0.0])  # About origin
        )

        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        r1 = model.get_reactions_at([5, 0, 0])
        r2 = model.get_reactions_at([5, 6, 0])

        # There should be non-zero reactions from the angular acceleration
        has_reactions = any(abs(r1[dof]) > 1e-10 for dof in r1) or \
                       any(abs(r2[dof]) > 1e-10 for dof in r2)
        assert has_reactions, "Angular acceleration should produce non-zero forces"


class TestAccelerationWithOtherLoads:
    """Test acceleration combined with other load types."""

    def test_gravity_plus_nodal_load(self):
        """Test gravity combined with a nodal load."""
        model = StructuralModel()

        rho = 0.01
        A = 0.1
        model.add_material("Steel", 210e6, 0.3, rho)
        model.add_section("Test", A, 1e-3, 1e-3, 1e-4)

        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel")

        model.fix_node_at([0, 0, 0])

        # Combined gravity + point load
        lc = model.create_load_case("Combined", LoadCaseType.Permanent)
        lc.set_acceleration_field(
            np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0])
        )

        # Add point load at tip using LoadCase API directly
        tip_node = model.find_node_at([L, 0, 0])
        P = -10.0  # kN point load at tip (downward)
        lc.add_nodal_load(tip_node.id, DOFIndex.UZ, P)

        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        reactions = model.get_reactions_at([0, 0, 0])

        # Total reaction should be weight + point load
        weight = rho * A * L * 9.81
        expected_reaction = weight + abs(P)

        # Allow small tolerance for numerical differences
        reaction_z = reactions[DOFIndex.UZ]
        assert abs(reaction_z - expected_reaction) < 0.05 * expected_reaction, \
            f"Reaction {reaction_z} should equal {expected_reaction}"

    def test_gravity_plus_line_load(self):
        """Test gravity combined with a distributed line load."""
        model = StructuralModel()

        rho = 0.01
        A = 0.1
        model.add_material("Steel", 210e6, 0.3, rho)
        model.add_section("Test", A, 1e-3, 1e-3, 1e-4)

        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel")

        model.fix_node_at([0, 0, 0])

        # Gravity + additional distributed load
        lc = model.create_load_case("Combined", LoadCaseType.Permanent)
        lc.set_acceleration_field(
            np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0])
        )

        w_extra = -5.0  # kN/m additional distributed load (downward)
        model.add_line_load(beam, (0.0, 0.0, w_extra), load_case=lc)

        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        reactions = model.get_reactions_at([0, 0, 0])

        # Total reaction should be gravity + line load
        weight = rho * A * L * 9.81
        line_load_total = abs(w_extra) * L
        expected_reaction = weight + line_load_total

        # Allow small tolerance for numerical differences
        reaction_z = reactions[DOFIndex.UZ]
        assert abs(reaction_z - expected_reaction) < 0.05 * expected_reaction, \
            f"Reaction {reaction_z} should equal {expected_reaction}"


class TestMassMatrix:
    """Test mass matrix computation directly."""

    def test_mass_matrix_symmetry(self):
        """Test that mass matrix is symmetric."""
        model = StructuralModel()

        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        elem = beam.elements[0]

        M = elem.global_mass_matrix()

        # Check symmetry
        assert np.allclose(M, M.T), "Mass matrix should be symmetric"

    def test_mass_matrix_positive_diagonal(self):
        """Test that mass matrix diagonal elements are positive."""
        model = StructuralModel()

        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        elem = beam.elements[0]

        M = elem.local_mass_matrix()

        # All diagonal elements should be positive (or zero for some DOFs)
        for i in range(12):
            assert M[i, i] >= 0, f"Mass diagonal M[{i},{i}] = {M[i, i]} should be non-negative"

    def test_mass_matrix_total_mass(self):
        """Test that mass matrix encodes correct total mass."""
        model = StructuralModel()

        # Simple material/section for easy calculation
        rho = 1.0  # mT/m³
        A = 0.1    # m²
        L = 6.0    # m
        model.add_material("Test", 210e6, 0.3, rho)
        model.add_section("Test", A, 1e-4, 1e-4, 1e-5)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Test")
        elem = beam.elements[0]

        M = elem.local_mass_matrix()

        # For consistent mass matrix, sum of translational diagonal entries
        # (scaled by shape functions) should relate to total mass
        # Total mass = rho * A * L = 1.0 * 0.1 * 6.0 = 0.6 mT
        expected_mass = rho * A * L

        # The translational mass contribution from diagonal entries
        # M[0,0] + M[6,6] represents axial DOFs
        # For consistent mass: m_ii = rho*A*L * (140/420) = m/3
        # So M[0,0] + M[6,6] = 2 * m/3 = 2m/3 ... wait, let me check the formula
        # Actually for consistent mass matrix, M[0,0] = M[6,6] = (rho*A*L) * (140/420) = m/3
        # So sum = 2m/3
        axial_sum = M[0, 0] + M[6, 6]
        # 140/420 * 2 = 280/420 = 2/3
        expected_axial_sum = expected_mass * 280 / 420

        assert abs(axial_sum - expected_axial_sum) < 0.01 * expected_axial_sum, \
            f"Axial mass sum {axial_sum} should equal {expected_axial_sum}"


class TestAccelerationAcceptanceCriteria:
    """Tests for acceptance criteria from implementation plan."""

    def test_ac1_gravity_produces_weight_forces(self):
        """AC1: Gravity load (az = -9.81) produces correct weight forces."""
        model = StructuralModel()

        # Use density that gives easy-to-verify numbers
        # 1 mT/m³ with A = 0.001 m² gives 0.001 mT/m = 1 kg/m
        model.add_material("Test", 210e6, 0.3, 1.0)
        model.add_section("Test", 0.001, 1e-6, 1e-6, 1e-7)

        L = 1.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Test")

        model.fix_node_at([0, 0, 0])

        lc = model.create_load_case("Gravity", LoadCaseType.Permanent)
        lc.set_acceleration_field(
            np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0])
        )

        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        reactions = model.get_reactions_at([0, 0, 0])

        # Total mass = 1.0 * 0.001 * 1.0 = 0.001 mT = 1 kg
        # Weight = 0.001 * 9.81 = 0.00981 kN = 9.81 N
        expected_weight = 1.0 * 0.001 * 1.0 * 9.81

        assert abs(reactions[DOFIndex.UZ] - expected_weight) < 1e-5, \
            f"Reaction {reactions[DOFIndex.UZ]} kN should equal weight {expected_weight} kN"

    def test_ac2_rotational_acceleration_produces_forces(self):
        """AC2: Rotational acceleration produces centrifugal effects."""
        model = StructuralModel()

        model.add_material("Test", 210e6, 0.3, 1.0)
        model.add_section("Test", 0.01, 1e-5, 1e-5, 1e-6)

        # Beam at radial distance from rotation axis
        beam = model.add_beam_by_coords([5, 0, 0], [5, 2, 0], "Test", "Test")

        model.fix_node_at([5, 0, 0])
        model.fix_node_at([5, 2, 0])

        # Apply angular acceleration about Z
        lc = model.create_load_case("Rotation", LoadCaseType.Variable)
        lc.set_acceleration_field(
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.0]),
            np.array([0.0, 0.0, 0.0])
        )

        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        r1 = model.get_reactions_at([5, 0, 0])
        r2 = model.get_reactions_at([5, 2, 0])

        # There should be non-zero reactions from the angular acceleration
        has_reactions = any(abs(r1[dof]) > 1e-10 for dof in r1) or \
                       any(abs(r2[dof]) > 1e-10 for dof in r2)
        assert has_reactions, "Angular acceleration should produce non-zero forces"

    def test_ac3_mass_per_meter_with_gravity(self):
        """AC3: 1 mT/m beam with gravity → 9.81 kN/m equivalent load."""
        model = StructuralModel()

        # Set up beam with exactly 1 mT/m mass per unit length
        # mass/length = rho * A = 1 mT/m
        # Use rho = 10 mT/m³ and A = 0.1 m² → rho * A = 1 mT/m
        rho = 10.0
        A = 0.1
        model.add_material("Heavy", 210e6, 0.3, rho)
        model.add_section("Large", A, 1e-3, 1e-3, 1e-4)

        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Large", "Heavy")

        model.fix_node_at([0, 0, 0])

        lc = model.create_load_case("Gravity", LoadCaseType.Permanent)
        lc.set_acceleration_field(
            np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0])
        )

        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        reactions = model.get_reactions_at([0, 0, 0])

        # With 1 mT/m and g = 9.81 m/s², the equivalent load is 9.81 kN/m
        # Total load on 6m beam = 9.81 * 6 = 58.86 kN
        expected_total_load = 9.81 * L

        # Allow small tolerance for numerical differences
        reaction_z = reactions[DOFIndex.UZ]
        rel_error = abs(reaction_z - expected_total_load) / expected_total_load
        assert rel_error < 0.01, \
            f"Reaction {reaction_z} kN should equal {expected_total_load} kN (error: {rel_error*100:.2f}%)"


class TestNoAcceleration:
    """Test behavior when no acceleration is applied."""

    def test_zero_acceleration_no_effect(self):
        """Test that zero acceleration produces no inertial forces."""
        model = StructuralModel()

        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        model.fix_node_at([0, 0, 0])

        # Load case with no loads (no acceleration set)
        lc = model.create_load_case("Empty", LoadCaseType.Permanent)

        success = model.analyze()
        assert success, "Analysis failed"

        model.set_active_load_case(lc)
        disp_ux = model.get_displacement_at([6, 0, 0], DOFIndex.UX)
        disp_uy = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
        disp_uz = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)

        # All displacements should be zero
        assert abs(disp_ux) < 1e-15, "No loads should produce zero displacement"
        assert abs(disp_uy) < 1e-15, "No loads should produce zero displacement"
        assert abs(disp_uz) < 1e-15, "No loads should produce zero displacement"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
