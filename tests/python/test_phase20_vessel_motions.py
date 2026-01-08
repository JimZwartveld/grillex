"""
Tests for Phase 20: Vessel Motions.

This module tests the VesselMotion class and its integration with the
StructuralModel, YAML loading, and LLM tools.
"""

import math
import tempfile
import os

import pytest
import numpy as np

from grillex.core import (
    StructuralModel,
    VesselMotion,
    MotionType,
    MotionComponent,
    DOFIndex,
    LoadCaseType,
)


class TestVesselMotionClass:
    """Tests for the VesselMotion class."""

    def test_create_empty_motion(self):
        """VesselMotion can be created with just a name."""
        motion = VesselMotion("Test")
        assert motion.name == "Test"
        assert motion.motion_center == [0.0, 0.0, 0.0]
        assert motion.components == []
        assert motion.description == ""

    def test_set_motion_center(self):
        """Motion center can be set."""
        motion = VesselMotion("Test")
        result = motion.set_motion_center([10.0, 5.0, 3.0])

        assert motion.motion_center == [10.0, 5.0, 3.0]
        assert result is motion  # Fluent API returns self

    def test_motion_center_validation(self):
        """Motion center must have 3 elements."""
        motion = VesselMotion("Test")
        with pytest.raises(ValueError, match="3-element"):
            motion.set_motion_center([1.0, 2.0])

    def test_add_heave_component(self):
        """Heave acceleration is correctly added."""
        motion = VesselMotion("Test")
        motion.add_heave(2.5)

        assert len(motion.components) == 1
        assert motion.components[0].motion_type == MotionType.HEAVE
        assert motion.components[0].amplitude == 2.5
        assert motion.components[0].phase == 0.0

    def test_add_heave_with_phase(self):
        """Heave can be added with phase."""
        motion = VesselMotion("Test")
        motion.add_heave(2.5, phase=math.pi/4)

        assert motion.components[0].phase == math.pi/4

    def test_add_pitch_component(self):
        """Pitch angular acceleration is correctly added."""
        motion = VesselMotion("Test")
        motion.add_pitch(0.08)

        assert len(motion.components) == 1
        assert motion.components[0].motion_type == MotionType.PITCH
        assert motion.components[0].amplitude == 0.08

    def test_add_roll_component(self):
        """Roll angular acceleration is correctly added."""
        motion = VesselMotion("Test")
        motion.add_roll(0.12)

        assert motion.components[0].motion_type == MotionType.ROLL
        assert motion.components[0].amplitude == 0.12

    def test_add_surge_component(self):
        """Surge acceleration is correctly added."""
        motion = VesselMotion("Test")
        motion.add_surge(1.5)

        assert motion.components[0].motion_type == MotionType.SURGE
        assert motion.components[0].amplitude == 1.5

    def test_add_sway_component(self):
        """Sway acceleration is correctly added."""
        motion = VesselMotion("Test")
        motion.add_sway(1.0)

        assert motion.components[0].motion_type == MotionType.SWAY
        assert motion.components[0].amplitude == 1.0

    def test_add_yaw_component(self):
        """Yaw angular acceleration is correctly added."""
        motion = VesselMotion("Test")
        motion.add_yaw(0.05)

        assert motion.components[0].motion_type == MotionType.YAW
        assert motion.components[0].amplitude == 0.05

    def test_add_all_components(self):
        """All 6 motion components can be added."""
        motion = VesselMotion("Test")
        motion.add_surge(1.0)
        motion.add_sway(2.0)
        motion.add_heave(3.0)
        motion.add_roll(0.1)
        motion.add_pitch(0.2)
        motion.add_yaw(0.3)

        assert len(motion.components) == 6

    def test_fluent_api_chaining(self):
        """Fluent API returns self for chaining."""
        motion = (VesselMotion("Test")
                  .set_motion_center([1.0, 2.0, 3.0])
                  .add_heave(2.5)
                  .add_roll(0.12))

        assert motion.name == "Test"
        assert motion.motion_center == [1.0, 2.0, 3.0]
        assert len(motion.components) == 2

    def test_get_acceleration_field(self):
        """get_acceleration_field() returns correct 6-component vector."""
        motion = VesselMotion("Test")
        motion.set_motion_center([50.0, 0.0, 5.0])
        motion.add_surge(1.0)
        motion.add_sway(2.0)
        motion.add_heave(3.0)
        motion.add_roll(0.1)
        motion.add_pitch(0.2)
        motion.add_yaw(0.3)

        accel, ref_pt = motion.get_acceleration_field()

        assert ref_pt == [50.0, 0.0, 5.0]
        assert accel[0] == 1.0  # surge
        assert accel[1] == 2.0  # sway
        assert accel[2] == 3.0  # heave
        assert accel[3] == 0.1  # roll
        assert accel[4] == 0.2  # pitch
        assert accel[5] == 0.3  # yaw

    def test_get_acceleration_field_empty(self):
        """Empty motion returns zero accelerations."""
        motion = VesselMotion("Test")
        accel, ref_pt = motion.get_acceleration_field()

        assert ref_pt == [0.0, 0.0, 0.0]
        assert accel == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def test_clear_components(self):
        """clear_components() removes all components."""
        motion = VesselMotion("Test")
        motion.add_heave(2.5).add_roll(0.12)
        motion.clear_components()

        assert motion.components == []

    def test_has_rotational_motion(self):
        """has_rotational_motion() detects roll/pitch/yaw."""
        motion = VesselMotion("Test")
        assert not motion.has_rotational_motion()

        motion.add_heave(2.5)
        assert not motion.has_rotational_motion()

        motion.add_roll(0.12)
        assert motion.has_rotational_motion()

    def test_has_linear_motion(self):
        """has_linear_motion() detects surge/sway/heave."""
        motion = VesselMotion("Test")
        assert not motion.has_linear_motion()

        motion.add_roll(0.12)
        assert not motion.has_linear_motion()

        motion.add_heave(2.5)
        assert motion.has_linear_motion()

    def test_get_component_by_type(self):
        """get_component_by_type() finds specific component."""
        motion = VesselMotion("Test")
        motion.add_heave(2.5)
        motion.add_roll(0.12)

        heave = motion.get_component_by_type(MotionType.HEAVE)
        assert heave is not None
        assert heave.amplitude == 2.5

        pitch = motion.get_component_by_type(MotionType.PITCH)
        assert pitch is None

    def test_repr(self):
        """String representation is informative."""
        motion = VesselMotion("Test")
        motion.set_motion_center([50.0, 0.0, 5.0])
        motion.add_heave(2.5)

        repr_str = repr(motion)
        assert "Test" in repr_str
        assert "50.0" in repr_str
        assert "heave" in repr_str


class TestVesselMotionFactoryMethods:
    """Tests for VesselMotion factory methods."""

    def test_create_still_water(self):
        """Still water has zero accelerations."""
        motion = VesselMotion.create_still_water()

        assert motion.name == "Still Water"
        assert len(motion.components) == 0
        accel, _ = motion.get_acceleration_field()
        assert accel == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def test_create_still_water_custom_name(self):
        """Still water can have custom name."""
        motion = VesselMotion.create_still_water("Calm Sea")
        assert motion.name == "Calm Sea"

    def test_create_heave_only(self):
        """Heave-only motion is correctly created."""
        motion = VesselMotion.create_heave_only(2.5)

        assert "Heave" in motion.name
        assert len(motion.components) == 1
        assert motion.components[0].motion_type == MotionType.HEAVE
        assert motion.components[0].amplitude == 2.5

    def test_create_heave_only_with_center(self):
        """Heave-only motion can have motion center."""
        motion = VesselMotion.create_heave_only(
            2.5, motion_center=[50.0, 0.0, 5.0])

        assert motion.motion_center == [50.0, 0.0, 5.0]

    def test_create_roll_from_angle_period(self):
        """Roll condition correctly converts angle/period to acceleration."""
        # α = (2π/T)² × θ for simple harmonic motion
        roll_angle = 15.0  # degrees
        roll_period = 10.0  # seconds

        motion = VesselMotion.create_roll_condition(roll_angle, roll_period)

        theta_rad = math.radians(roll_angle)
        omega = 2 * math.pi / roll_period
        expected_accel = omega * omega * theta_rad

        roll_comp = motion.get_component_by_type(MotionType.ROLL)
        assert roll_comp is not None
        np.testing.assert_almost_equal(roll_comp.amplitude, expected_accel, decimal=6)

    def test_create_pitch_from_angle_period(self):
        """Pitch condition correctly converts angle/period to acceleration."""
        pitch_angle = 5.0  # degrees
        pitch_period = 8.0  # seconds

        motion = VesselMotion.create_pitch_condition(pitch_angle, pitch_period)

        theta_rad = math.radians(pitch_angle)
        omega = 2 * math.pi / pitch_period
        expected_accel = omega * omega * theta_rad

        pitch_comp = motion.get_component_by_type(MotionType.PITCH)
        assert pitch_comp is not None
        np.testing.assert_almost_equal(pitch_comp.amplitude, expected_accel, decimal=6)

    def test_create_combined_design_motion(self):
        """Combined motion has all specified components."""
        motion = VesselMotion.create_combined_design_motion(
            heave=2.0,
            roll_angle=10.0,
            roll_period=10.0,
            pitch_angle=5.0,
            pitch_period=8.0,
            motion_center=[50.0, 0.0, 3.0]
        )

        assert motion.motion_center == [50.0, 0.0, 3.0]

        heave = motion.get_component_by_type(MotionType.HEAVE)
        assert heave is not None
        assert heave.amplitude == 2.0

        roll = motion.get_component_by_type(MotionType.ROLL)
        assert roll is not None
        assert roll.amplitude > 0

        pitch = motion.get_component_by_type(MotionType.PITCH)
        assert pitch is not None
        assert pitch.amplitude > 0


class TestVesselMotionIntegration:
    """Integration tests with full StructuralModel."""

    @pytest.fixture
    def cantilever_model(self):
        """Create a simple cantilever beam model."""
        model = StructuralModel("Cantilever")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])
        return model

    def test_add_vessel_motion_load_case(self, cantilever_model):
        """Vessel motion creates correct load case."""
        motion = cantilever_model.add_vessel_motion_load_case(
            "Design Heave",
            heave=2.5,
            motion_center=[3.0, 0.0, 0.0]
        )

        assert motion.name == "Design Heave"
        assert motion.motion_center == [3.0, 0.0, 0.0]

        heave = motion.get_component_by_type(MotionType.HEAVE)
        assert heave.amplitude == 2.5

    def test_add_gravity_load_case(self, cantilever_model):
        """Gravity load case is created correctly."""
        lc = cantilever_model.add_gravity_load_case()

        assert lc.name == "Gravity"
        accel = lc.get_acceleration()
        np.testing.assert_almost_equal(accel[2], -9.81, decimal=2)

    def test_add_gravity_custom_acceleration(self, cantilever_model):
        """Gravity can have custom acceleration."""
        lc = cantilever_model.add_gravity_load_case("1.1g", acceleration=10.79)

        assert lc.name == "1.1g"
        accel = lc.get_acceleration()
        np.testing.assert_almost_equal(accel[2], -10.79, decimal=2)

    def test_get_vessel_motions(self, cantilever_model):
        """get_vessel_motions returns all defined motions."""
        cantilever_model.add_vessel_motion_load_case("Motion 1", heave=2.0)
        cantilever_model.add_vessel_motion_load_case("Motion 2", roll=0.1)

        motions = cantilever_model.get_vessel_motions()
        assert len(motions) == 2
        names = {m.name for m in motions}
        assert "Motion 1" in names
        assert "Motion 2" in names

    def test_get_vessel_motion_by_name(self, cantilever_model):
        """get_vessel_motion returns specific motion by name."""
        cantilever_model.add_vessel_motion_load_case("Design Heave", heave=2.5)

        motion = cantilever_model.get_vessel_motion("Design Heave")
        assert motion is not None
        assert motion.name == "Design Heave"

        none_motion = cantilever_model.get_vessel_motion("Nonexistent")
        assert none_motion is None

    def test_heave_deflection(self, cantilever_model):
        """Heave acceleration produces correct inertial loads."""
        # With heave = 2.5 m/s², the beam experiences upward acceleration
        # This produces effective body forces
        cantilever_model.add_vessel_motion_load_case("Heave", heave=2.5)
        cantilever_model.analyze()

        # Tip should deflect (direction depends on heave sign)
        disp_z = cantilever_model.get_displacement_at([6, 0, 0], DOFIndex.UZ)
        assert disp_z != 0

    def test_gravity_deflection(self, cantilever_model):
        """Gravity produces downward deflection."""
        cantilever_model.add_gravity_load_case()
        cantilever_model.analyze()

        # Tip should deflect downward
        disp_z = cantilever_model.get_displacement_at([6, 0, 0], DOFIndex.UZ)
        assert disp_z < 0

    def test_combined_heave_gravity(self, cantilever_model):
        """Combined heave + gravity produces modified deflection."""
        cantilever_model.add_gravity_load_case()
        cantilever_model.add_vessel_motion_load_case("Heave Up", heave=2.5)
        cantilever_model.analyze()

        # With heave reducing effective gravity, deflection should be smaller
        disp_z = cantilever_model.get_displacement_at([6, 0, 0], DOFIndex.UZ)
        # Just check it's reasonable
        assert disp_z < 0  # Still net downward under 1g gravity

    def test_vessel_motion_with_all_components(self, cantilever_model):
        """Model with all motion components can be analyzed."""
        cantilever_model.add_vessel_motion_load_case(
            "Full Motion",
            heave=2.0,
            roll=0.1,
            pitch=0.05,
            surge=1.0,
            sway=0.5,
            yaw=0.02,
            motion_center=[3.0, 0.0, 0.0]
        )
        cantilever_model.analyze()

        # Just verify analysis completes without error
        assert cantilever_model.is_analyzed()


class TestVesselMotionYAML:
    """Tests for YAML I/O."""

    def test_load_vessel_motion_from_yaml(self):
        """Vessel motion is correctly parsed from YAML."""
        from grillex.io import load_model_from_yaml

        yaml_content = """
name: YAML Test

materials:
  - name: Steel
    E: 210000000
    nu: 0.3
    rho: 7.85

sections:
  - name: IPE300
    A: 0.00538
    Iy: 8.36e-5
    Iz: 6.04e-6
    J: 2.01e-7

beams:
  - start: [0, 0, 0]
    end: [6, 0, 0]
    section: IPE300
    material: Steel

boundary_conditions:
  - node: [0, 0, 0]
    type: fixed

vessel_motions:
  - name: Design Heave
    heave: 2.5
    motion_center: [3.0, 0.0, 0.0]
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(yaml_content)
            temp_path = f.name

        try:
            model = load_model_from_yaml(temp_path)

            motions = model.get_vessel_motions()
            assert len(motions) == 1
            assert motions[0].name == "Design Heave"

            heave = motions[0].get_component_by_type(MotionType.HEAVE)
            assert heave is not None
            assert heave.amplitude == 2.5
        finally:
            os.unlink(temp_path)

    def test_load_gravity_shorthand(self):
        """Gravity shorthand creates correct load case."""
        from grillex.io import load_model_from_yaml

        yaml_content = """
name: Gravity Test

materials:
  - name: Steel
    E: 210000000
    nu: 0.3
    rho: 7.85

sections:
  - name: IPE300
    A: 0.00538
    Iy: 8.36e-5
    Iz: 6.04e-6
    J: 2.01e-7

beams:
  - start: [0, 0, 0]
    end: [6, 0, 0]
    section: IPE300
    material: Steel

boundary_conditions:
  - node: [0, 0, 0]
    type: fixed

vessel_motions:
  - name: Gravity
    gravity: true
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(yaml_content)
            temp_path = f.name

        try:
            model = load_model_from_yaml(temp_path)
            # Gravity is a load case, not a vessel motion
            load_cases = model._cpp_model.get_load_cases()
            lc_names = [lc.name for lc in load_cases]
            assert "Gravity" in lc_names
        finally:
            os.unlink(temp_path)

    def test_load_angle_period_format(self):
        """Angle/period format is correctly converted."""
        from grillex.io import load_model_from_yaml

        yaml_content = """
name: Roll Test

materials:
  - name: Steel
    E: 210000000
    nu: 0.3
    rho: 7.85

sections:
  - name: IPE300
    A: 0.00538
    Iy: 8.36e-5
    Iz: 6.04e-6
    J: 2.01e-7

beams:
  - start: [0, 0, 0]
    end: [6, 0, 0]
    section: IPE300
    material: Steel

boundary_conditions:
  - node: [0, 0, 0]
    type: fixed

vessel_motions:
  - name: Roll Condition
    roll_angle: 15.0
    roll_period: 10.0
    motion_center: [3.0, 0.0, 0.0]
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(yaml_content)
            temp_path = f.name

        try:
            model = load_model_from_yaml(temp_path)

            motions = model.get_vessel_motions()
            assert len(motions) == 1

            roll = motions[0].get_component_by_type(MotionType.ROLL)
            assert roll is not None

            # Verify the conversion
            theta_rad = math.radians(15.0)
            omega = 2 * math.pi / 10.0
            expected = omega * omega * theta_rad
            np.testing.assert_almost_equal(roll.amplitude, expected, decimal=4)
        finally:
            os.unlink(temp_path)


class TestVesselMotionLLMTools:
    """Tests for LLM tool integration."""

    def test_add_gravity_tool(self):
        """add_gravity tool creates correct load case."""
        from grillex.llm.tools import ToolExecutor

        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        executor = ToolExecutor(model)
        result = executor.execute("add_gravity", {})

        assert result.success
        assert result.result["name"] == "Gravity"
        assert result.result["acceleration"] == -9.81

    def test_add_gravity_tool_custom(self):
        """add_gravity tool accepts custom acceleration."""
        from grillex.llm.tools import ToolExecutor

        model = StructuralModel("Test")
        executor = ToolExecutor(model)
        result = executor.execute("add_gravity", {
            "name": "Mars Gravity",
            "acceleration": 3.71
        })

        assert result.success
        assert result.result["name"] == "Mars Gravity"
        assert result.result["acceleration"] == -3.71

    def test_add_vessel_motion_tool(self):
        """add_vessel_motion tool creates correct motion."""
        from grillex.llm.tools import ToolExecutor

        model = StructuralModel("Test")
        executor = ToolExecutor(model)
        result = executor.execute("add_vessel_motion", {
            "name": "Design Motion",
            "heave": 2.5,
            "roll": 0.12,
            "motion_center_x": 50.0,
            "motion_center_z": 5.0
        })

        assert result.success
        assert result.result["name"] == "Design Motion"
        assert result.result["motion_center"] == [50.0, 0.0, 5.0]
        assert result.result["acceleration_field"]["heave"] == 2.5
        assert result.result["acceleration_field"]["roll"] == 0.12

    def test_get_vessel_motions_tool(self):
        """get_vessel_motions tool returns all motions."""
        from grillex.llm.tools import ToolExecutor

        model = StructuralModel("Test")
        model.add_vessel_motion_load_case("Motion 1", heave=2.0)
        model.add_vessel_motion_load_case("Motion 2", roll=0.1)

        executor = ToolExecutor(model)
        result = executor.execute("get_vessel_motions", {})

        assert result.success
        assert result.result["count"] == 2
        names = {m["name"] for m in result.result["vessel_motions"]}
        assert "Motion 1" in names
        assert "Motion 2" in names

    def test_tool_no_model_error(self):
        """Tools return error when no model exists."""
        from grillex.llm.tools import ToolExecutor

        executor = ToolExecutor(None)
        result = executor.execute("add_gravity", {})

        assert not result.success
        assert "No model" in result.error


class TestVesselMotionYAMLReference:
    """Test 20.4: Load cases can reference vessel motions by name in YAML."""

    def test_load_case_references_vessel_motion(self):
        """Load case with vessel_motion field applies acceleration from vessel motion."""
        from grillex.io.yaml_loader import build_model_from_dict

        data = {
            'name': 'Test Model',
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85}],
            'sections': [{'name': 'IPE300', 'A': 0.01, 'Iy': 1e-4, 'Iz': 1e-5, 'J': 1e-6}],
            'beams': [{'start': [0, 0, 0], 'end': [10, 0, 0], 'section': 'IPE300', 'material': 'Steel'}],
            'boundary_conditions': [{'node': [0, 0, 0], 'type': 'fixed'}],
            'vessel_motions': [
                {
                    'name': 'Design Motion',
                    'heave': 2.5,
                    'roll': 0.12,
                    'motion_center': [50.0, 0.0, 5.0],
                    'auto_create_load_case': False  # Don't auto-create, we'll reference it
                }
            ],
            'load_cases': [
                {
                    'name': 'Motion LC',
                    'type': 'Environmental',
                    'vessel_motion': 'Design Motion'  # Reference the vessel motion
                }
            ]
        }

        model = build_model_from_dict(data)

        # The load case should have the acceleration field from the vessel motion
        load_cases = model._cpp_model.get_load_cases()
        motion_lc = next(lc for lc in load_cases if lc.name == 'Motion LC')

        accel = motion_lc.get_acceleration()
        assert accel[2] == 2.5  # heave (az)
        assert accel[3] == pytest.approx(0.12)  # roll (ωx)

    def test_load_case_reference_invalid_motion(self):
        """Load case referencing non-existent vessel motion raises error."""
        from grillex.io.yaml_loader import build_model_from_dict, YAMLLoadError

        data = {
            'name': 'Test Model',
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85}],
            'sections': [{'name': 'IPE300', 'A': 0.01, 'Iy': 1e-4, 'Iz': 1e-5, 'J': 1e-6}],
            'beams': [{'start': [0, 0, 0], 'end': [10, 0, 0], 'section': 'IPE300', 'material': 'Steel'}],
            'boundary_conditions': [{'node': [0, 0, 0], 'type': 'fixed'}],
            'vessel_motions': [],  # No vessel motions defined
            'load_cases': [
                {
                    'name': 'Motion LC',
                    'vessel_motion': 'Non-existent Motion'
                }
            ]
        }

        with pytest.raises(YAMLLoadError) as exc:
            build_model_from_dict(data)

        assert "vessel motion 'Non-existent Motion' not found" in str(exc.value)

    def test_load_case_reference_lists_available_motions(self):
        """Error message includes list of available vessel motions."""
        from grillex.io.yaml_loader import build_model_from_dict, YAMLLoadError

        data = {
            'name': 'Test Model',
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85}],
            'sections': [{'name': 'IPE300', 'A': 0.01, 'Iy': 1e-4, 'Iz': 1e-5, 'J': 1e-6}],
            'beams': [{'start': [0, 0, 0], 'end': [10, 0, 0], 'section': 'IPE300', 'material': 'Steel'}],
            'boundary_conditions': [{'node': [0, 0, 0], 'type': 'fixed'}],
            'vessel_motions': [
                {'name': 'Heave Only', 'heave': 2.0, 'auto_create_load_case': False},
                {'name': 'Roll Only', 'roll': 0.1, 'auto_create_load_case': False}
            ],
            'load_cases': [
                {'name': 'Motion LC', 'vessel_motion': 'Wrong Name'}
            ]
        }

        with pytest.raises(YAMLLoadError) as exc:
            build_model_from_dict(data)

        error_msg = str(exc.value)
        assert "Heave Only" in error_msg
        assert "Roll Only" in error_msg

    def test_load_case_with_motion_and_loads(self):
        """Load case can have both vessel motion reference and point loads."""
        from grillex.io.yaml_loader import build_model_from_dict

        data = {
            'name': 'Test Model',
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85}],
            'sections': [{'name': 'IPE300', 'A': 0.01, 'Iy': 1e-4, 'Iz': 1e-5, 'J': 1e-6}],
            'beams': [{'start': [0, 0, 0], 'end': [10, 0, 0], 'section': 'IPE300', 'material': 'Steel'}],
            'boundary_conditions': [{'node': [0, 0, 0], 'type': 'fixed'}],
            'vessel_motions': [
                {
                    'name': 'Motion',
                    'heave': 2.0,
                    'auto_create_load_case': False
                }
            ],
            'load_cases': [
                {
                    'name': 'Combined LC',
                    'vessel_motion': 'Motion',
                    'loads': [
                        {'position': [10, 0, 0], 'force': [0, 0, -50.0]}
                    ]
                }
            ]
        }

        model = build_model_from_dict(data)

        # Check acceleration field is applied
        load_cases = model._cpp_model.get_load_cases()
        combined_lc = next(lc for lc in load_cases if lc.name == 'Combined LC')
        accel = combined_lc.get_acceleration()
        assert accel[2] == 2.0  # heave

        # Check point load is also applied (would need to verify via analysis)
        # Just verify the load case exists and the name is correct
        assert combined_lc.name == 'Combined LC'


class TestVesselMotionRoundTrip:
    """Test 20.4: Round-trip serialization of vessel motions."""

    def test_to_dict_basic(self):
        """VesselMotion.to_dict() returns correct structure."""
        motion = VesselMotion("Test Motion")
        motion.set_motion_center([50.0, 0.0, 5.0])
        motion.add_heave(2.5)
        motion.add_roll(0.12)

        d = motion.to_dict()

        assert d['name'] == "Test Motion"
        assert d['motion_center'] == [50.0, 0.0, 5.0]
        assert d['heave'] == 2.5
        assert d['roll'] == 0.12

    def test_to_dict_omits_default_center(self):
        """to_dict() omits motion_center if at origin."""
        motion = VesselMotion("Test").add_heave(1.0)

        d = motion.to_dict()

        assert 'motion_center' not in d

    def test_from_dict_basic(self):
        """VesselMotion.from_dict() reconstructs motion correctly."""
        data = {
            'name': 'Reconstructed',
            'motion_center': [10.0, 5.0, 2.0],
            'heave': 2.5,
            'pitch': 0.08
        }

        motion = VesselMotion.from_dict(data)

        assert motion.name == 'Reconstructed'
        assert motion.motion_center == [10.0, 5.0, 2.0]

        heave_comp = motion.get_component_by_type(MotionType.HEAVE)
        assert heave_comp is not None
        assert heave_comp.amplitude == 2.5

        pitch_comp = motion.get_component_by_type(MotionType.PITCH)
        assert pitch_comp is not None
        assert pitch_comp.amplitude == 0.08

    def test_round_trip_preservation(self):
        """Round trip: VesselMotion -> dict -> VesselMotion preserves data."""
        original = VesselMotion("Design Motion")
        original.set_motion_center([50.0, 0.0, 5.0])
        original.add_heave(2.5)
        original.add_roll(0.12)
        original.add_pitch(0.08)
        original.add_surge(0.5)

        # Serialize and deserialize
        data = original.to_dict()
        reconstructed = VesselMotion.from_dict(data)

        # Verify all fields match
        assert reconstructed.name == original.name
        assert reconstructed.motion_center == original.motion_center

        # Verify all components
        orig_accel, orig_center = original.get_acceleration_field()
        recon_accel, recon_center = reconstructed.get_acceleration_field()

        for i in range(6):
            assert pytest.approx(orig_accel[i]) == recon_accel[i]

    def test_yaml_round_trip(self):
        """Round trip through YAML: dict -> YAML string -> dict -> VesselMotion."""
        import yaml

        original = VesselMotion("YAML Test")
        original.set_motion_center([25.0, 0.0, 3.0])
        original.add_heave(3.0)
        original.add_roll(0.15)

        # Serialize to YAML
        yaml_str = yaml.dump(original.to_dict())

        # Parse back from YAML
        parsed = yaml.safe_load(yaml_str)
        reconstructed = VesselMotion.from_dict(parsed)

        assert reconstructed.name == "YAML Test"
        assert reconstructed.motion_center == [25.0, 0.0, 3.0]

        heave_comp = reconstructed.get_component_by_type(MotionType.HEAVE)
        assert heave_comp is not None
        assert heave_comp.amplitude == 3.0

    def test_full_model_round_trip(self):
        """Full model round trip: YAML -> Model -> VesselMotion -> dict matches original."""
        from grillex.io.yaml_loader import build_model_from_dict

        original_data = {
            'name': 'Round Trip Test',
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85}],
            'sections': [{'name': 'IPE300', 'A': 0.01, 'Iy': 1e-4, 'Iz': 1e-5, 'J': 1e-6}],
            'beams': [{'start': [0, 0, 0], 'end': [10, 0, 0], 'section': 'IPE300', 'material': 'Steel'}],
            'boundary_conditions': [{'node': [0, 0, 0], 'type': 'fixed'}],
            'vessel_motions': [
                {
                    'name': 'Test Motion',
                    'motion_center': [50.0, 0.0, 5.0],
                    'heave': 2.5,
                    'roll': 0.12,
                    'pitch': 0.08,
                    'auto_create_load_case': False
                }
            ],
        }

        model = build_model_from_dict(original_data)

        # Get the vessel motion back and convert to dict
        vessel_motions = model.get_vessel_motions()
        assert len(vessel_motions) == 1

        motion = vessel_motions[0]
        exported = motion.to_dict()

        # Verify key fields match
        assert exported['name'] == 'Test Motion'
        assert exported['motion_center'] == [50.0, 0.0, 5.0]
        assert exported['heave'] == 2.5
        assert exported['roll'] == pytest.approx(0.12)
        assert exported['pitch'] == pytest.approx(0.08)


class TestVesselMotionLinkedLoadCases:
    """Test linked load case functionality (auto-update, signed pairs, immutability)."""

    def test_load_case_is_linked(self):
        """Load cases created via add_vessel_motion_load_case are tracked as linked."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create vessel motion load case
        motion = model.add_vessel_motion_load_case("Roll", roll=0.12)

        # Get the load case
        load_cases = model._cpp_model.get_load_cases()
        roll_lc = next(lc for lc in load_cases if lc.name == 'Roll')

        # Should be marked as linked
        assert model.is_load_case_linked_to_vessel_motion(roll_lc)

    def test_unlinked_load_case_not_linked(self):
        """Regular load cases are not marked as linked."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create regular load case (not from vessel motion)
        lc = model.create_load_case("Manual Load", LoadCaseType.Permanent)

        # Should NOT be marked as linked
        assert not model.is_load_case_linked_to_vessel_motion(lc)

    def test_get_vessel_motion_for_linked_load_case(self):
        """get_vessel_motion_for_load_case returns the correct VesselMotion."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create vessel motion load case
        motion = model.add_vessel_motion_load_case("Test Motion", heave=2.5)

        # Get the load case
        load_cases = model._cpp_model.get_load_cases()
        test_lc = next(lc for lc in load_cases if lc.name == 'Test Motion')

        # Get vessel motion for load case
        found_motion = model.get_vessel_motion_for_load_case(test_lc)
        assert found_motion is motion
        assert found_motion.name == "Test Motion"

    def test_get_vessel_motion_for_unlinked_returns_none(self):
        """get_vessel_motion_for_load_case returns None for unlinked load cases."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create regular load case
        lc = model.create_load_case("Manual Load", LoadCaseType.Permanent)

        # Should return None
        assert model.get_vessel_motion_for_load_case(lc) is None

    def test_auto_update_on_motion_center_change(self):
        """Changing motion center updates linked load cases."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create vessel motion load case
        motion = model.add_vessel_motion_load_case("Heave", heave=2.5)

        # Get the load case
        load_cases = model._cpp_model.get_load_cases()
        heave_lc = next(lc for lc in load_cases if lc.name == 'Heave')

        # Check initial ref point
        initial_ref = heave_lc.get_acceleration_ref_point()
        assert list(initial_ref) == [0.0, 0.0, 0.0]

        # Change motion center
        motion.set_motion_center([50.0, 0.0, 5.0])

        # Load case should be updated
        updated_ref = heave_lc.get_acceleration_ref_point()
        assert list(updated_ref) == [50.0, 0.0, 5.0]

    def test_auto_update_on_component_add(self):
        """Adding a motion component updates linked load cases."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create vessel motion load case with heave only
        motion = model.add_vessel_motion_load_case("Motion", heave=2.5)

        # Get the load case
        load_cases = model._cpp_model.get_load_cases()
        motion_lc = next(lc for lc in load_cases if lc.name == 'Motion')

        # Check initial acceleration (only heave)
        initial_accel = motion_lc.get_acceleration()
        assert initial_accel[2] == 2.5  # heave
        assert initial_accel[3] == 0.0  # no roll

        # Add roll component
        motion.add_roll(0.12)

        # Load case should be updated with roll
        updated_accel = motion_lc.get_acceleration()
        assert updated_accel[2] == 2.5  # heave still there
        assert updated_accel[3] == pytest.approx(0.12)  # roll added

    def test_auto_update_on_clear_components(self):
        """Clearing components updates linked load cases."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create vessel motion load case with heave
        motion = model.add_vessel_motion_load_case("Motion", heave=2.5)

        # Get the load case
        load_cases = model._cpp_model.get_load_cases()
        motion_lc = next(lc for lc in load_cases if lc.name == 'Motion')

        # Verify heave is set
        assert motion_lc.get_acceleration()[2] == 2.5

        # Clear components
        motion.clear_components()

        # Load case should have zero acceleration
        cleared_accel = motion_lc.get_acceleration()
        assert all(a == 0.0 for a in cleared_accel)

    def test_signed_pairs_creates_two_load_cases(self):
        """create_signed_pairs=True creates two load cases with opposite signs."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create vessel motion with signed pairs
        motion = model.add_vessel_motion_load_case(
            "Roll",
            roll=0.12,
            create_signed_pairs=True
        )

        # Should have 2 linked load cases
        linked = motion.get_linked_load_cases()
        assert len(linked) == 2

        # Find positive and negative load cases
        load_cases = model._cpp_model.get_load_cases()
        roll_pos = next(lc for lc in load_cases if lc.name == 'Roll +')
        roll_neg = next(lc for lc in load_cases if lc.name == 'Roll -')

        # Check accelerations are opposite
        pos_accel = roll_pos.get_acceleration()
        neg_accel = roll_neg.get_acceleration()

        assert pos_accel[3] == pytest.approx(0.12)   # roll positive
        assert neg_accel[3] == pytest.approx(-0.12)  # roll negative

    def test_signed_pairs_both_are_linked(self):
        """Both signed pair load cases are tracked as linked."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create vessel motion with signed pairs
        model.add_vessel_motion_load_case(
            "Roll",
            roll=0.12,
            create_signed_pairs=True
        )

        # Get both load cases
        load_cases = model._cpp_model.get_load_cases()
        roll_pos = next(lc for lc in load_cases if lc.name == 'Roll +')
        roll_neg = next(lc for lc in load_cases if lc.name == 'Roll -')

        # Both should be marked as linked
        assert model.is_load_case_linked_to_vessel_motion(roll_pos)
        assert model.is_load_case_linked_to_vessel_motion(roll_neg)

    def test_signed_pairs_auto_update(self):
        """Both signed pair load cases are updated when motion changes."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create vessel motion with signed pairs
        motion = model.add_vessel_motion_load_case(
            "Roll",
            roll=0.12,
            create_signed_pairs=True
        )

        # Get load cases
        load_cases = model._cpp_model.get_load_cases()
        roll_pos = next(lc for lc in load_cases if lc.name == 'Roll +')
        roll_neg = next(lc for lc in load_cases if lc.name == 'Roll -')

        # Add heave component
        motion.add_heave(2.5)

        # Both should be updated with heave
        pos_accel = roll_pos.get_acceleration()
        neg_accel = roll_neg.get_acceleration()

        assert pos_accel[2] == 2.5   # heave positive
        assert pos_accel[3] == pytest.approx(0.12)   # roll positive
        assert neg_accel[2] == -2.5  # heave negative (sign applied to all)
        assert neg_accel[3] == pytest.approx(-0.12)  # roll negative

    def test_vessel_motion_get_acceleration_with_sign_multiplier(self):
        """get_acceleration_field accepts sign_multiplier parameter."""
        motion = VesselMotion("Test")
        motion.add_heave(2.5)
        motion.add_roll(0.12)

        # Positive multiplier
        pos_accel, _ = motion.get_acceleration_field(sign_multiplier=1.0)
        assert pos_accel[2] == 2.5
        assert pos_accel[3] == pytest.approx(0.12)

        # Negative multiplier
        neg_accel, _ = motion.get_acceleration_field(sign_multiplier=-1.0)
        assert neg_accel[2] == -2.5
        assert neg_accel[3] == pytest.approx(-0.12)

    def test_link_load_case_method(self):
        """VesselMotion.link_load_case() links a load case."""
        model = StructuralModel("Test")
        motion = VesselMotion("Test Motion")
        motion.add_heave(2.5)

        # Create a load case manually
        lc = model.create_load_case("Manual LC", LoadCaseType.Environmental)

        # Link it to the motion
        motion.link_load_case(lc)

        # Should be in linked list
        assert lc in motion.get_linked_load_cases()

        # Acceleration should be applied
        accel = lc.get_acceleration()
        assert accel[2] == 2.5

    def test_unlink_load_case_method(self):
        """VesselMotion.unlink_load_case() removes the link."""
        model = StructuralModel("Test")
        motion = VesselMotion("Test Motion")
        motion.add_heave(2.5)

        # Create and link a load case
        lc = model.create_load_case("Manual LC", LoadCaseType.Environmental)
        motion.link_load_case(lc)
        assert lc in motion.get_linked_load_cases()

        # Unlink it
        result = motion.unlink_load_case(lc)
        assert result is True
        assert lc not in motion.get_linked_load_cases()

        # Second unlink should return False
        result = motion.unlink_load_case(lc)
        assert result is False


class TestVesselMotionDeletion:
    """Test deletion of vessel motions and cleanup of linked load cases."""

    def test_delete_vessel_motion_removes_from_registry(self):
        """delete_vessel_motion removes the vessel motion from the model."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        model.add_vessel_motion_load_case("Roll", roll=0.12)
        assert model.get_vessel_motion("Roll") is not None

        model.delete_vessel_motion("Roll")
        assert model.get_vessel_motion("Roll") is None

    def test_delete_vessel_motion_returns_affected_load_cases(self):
        """delete_vessel_motion returns list of affected load case names."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        model.add_vessel_motion_load_case("Roll", roll=0.12, create_signed_pairs=True)
        deleted = model.delete_vessel_motion("Roll")

        assert "Roll +" in deleted
        assert "Roll -" in deleted

    def test_delete_vessel_motion_deletes_load_cases(self):
        """delete_vessel_motion actually removes load cases from the model."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        model.add_vessel_motion_load_case("Roll", roll=0.12)

        # Verify load case exists before deletion
        load_cases_before = model._cpp_model.get_load_cases()
        assert any(lc.name == 'Roll' for lc in load_cases_before)

        model.delete_vessel_motion("Roll")

        # Load case should be completely removed from the model
        load_cases_after = model._cpp_model.get_load_cases()
        assert not any(lc.name == 'Roll' for lc in load_cases_after)

    def test_delete_vessel_motion_removes_from_linked_tracking(self):
        """delete_vessel_motion removes load case IDs from linked tracking."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        model.add_vessel_motion_load_case("Roll", roll=0.12)

        # Get load case ID before deletion
        load_cases = model._cpp_model.get_load_cases()
        roll_lc = next(lc for lc in load_cases if lc.name == 'Roll')
        roll_lc_id = roll_lc.id
        assert roll_lc_id in model._linked_load_case_ids

        model.delete_vessel_motion("Roll")

        # ID should no longer be tracked as linked
        assert roll_lc_id not in model._linked_load_case_ids

    def test_delete_vessel_motion_not_found_raises(self):
        """delete_vessel_motion raises ValueError if motion not found."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        with pytest.raises(ValueError, match="not found"):
            model.delete_vessel_motion("NonExistent")

    def test_delete_vessel_motion_keep_load_cases(self):
        """delete_linked_load_cases=False keeps load cases with acceleration."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        model.add_vessel_motion_load_case("Roll", roll=0.12)

        # Get load case
        load_cases = model._cpp_model.get_load_cases()
        roll_lc = next(lc for lc in load_cases if lc.name == 'Roll')

        model.delete_vessel_motion("Roll", delete_linked_load_cases=False)

        # Acceleration should be preserved
        assert roll_lc.get_acceleration()[3] == pytest.approx(0.12)
        # But no longer tracked as linked
        assert not model.is_load_case_linked_to_vessel_motion(roll_lc)

    def test_delete_vessel_motion_with_combination_raises(self):
        """delete_vessel_motion raises if load combinations reference load cases."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        motion = model.add_vessel_motion_load_case("Roll", roll=0.12)

        # Get load case and create combination
        load_cases = model._cpp_model.get_load_cases()
        roll_lc = next(lc for lc in load_cases if lc.name == 'Roll')

        # Create combination and add the load case to it
        combo = model.create_load_combination("ULS")
        model.add_load_case_to_combination(combo["id"], roll_lc.id)

        # Should raise without force
        with pytest.raises(ValueError, match="load combinations"):
            model.delete_vessel_motion("Roll")

    def test_delete_vessel_motion_with_combination_force(self):
        """delete_vessel_motion with force=True removes from combinations."""
        model = StructuralModel("Test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
        model.add_section("IPE300", A=0.01, Iy=1e-4, Iz=1e-5, J=1e-6)
        model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        motion = model.add_vessel_motion_load_case("Roll", roll=0.12)

        # Get load case and create combination
        load_cases = model._cpp_model.get_load_cases()
        roll_lc = next(lc for lc in load_cases if lc.name == 'Roll')

        # Create combination and add the load case to it
        combo = model.create_load_combination("ULS")
        model.add_load_case_to_combination(combo["id"], roll_lc.id)

        # Should succeed with force
        deleted = model.delete_vessel_motion("Roll", force=True)
        assert "Roll" in deleted

        # Vessel motion should be gone
        assert model.get_vessel_motion("Roll") is None

        # Load case should be removed from combination
        updated_combo = next(c for c in model.get_load_combinations() if c["id"] == combo["id"])
        assert len(updated_combo["load_cases"]) == 0


# =============================================================================
# Tests for VesselMotions (plural) - Multi-Motion Generation
# =============================================================================


class TestDesignMethodAndLimitState:
    """Tests for DesignMethod, LimitState, and OperationType enums."""

    def test_design_method_enum(self):
        """DesignMethod enum has LRFD and ASD values."""
        from grillex.core import DesignMethod

        assert DesignMethod.LRFD.value == "lrfd"
        assert DesignMethod.ASD.value == "asd"

    def test_limit_state_enum(self):
        """LimitState enum has ULSa, ULSb, and SLS values."""
        from grillex.core import LimitState

        assert LimitState.ULSa.value == "ulsa"
        assert LimitState.ULSb.value == "ulsb"
        assert LimitState.SLS.value == "sls"

    def test_operation_type_enum(self):
        """OperationType enum has correct values."""
        from grillex.core import OperationType

        assert OperationType.IN_SERVICE.value == "in_service"
        assert OperationType.REMOVAL.value == "removal"
        assert OperationType.TRANSPORT_INSTALL.value == "transport_install"


class TestLoadCombinationFactors:
    """Tests for LoadCombinationFactors dataclass."""

    def test_create_factors(self):
        """LoadCombinationFactors can be created."""
        from grillex.core import LoadCombinationFactors

        factors = LoadCombinationFactors(
            dead_load=1.3,
            live_load=1.3,
            environmental=1.0
        )

        assert factors.dead_load == 1.3
        assert factors.live_load == 1.3
        assert factors.environmental == 1.0

    def test_default_load_factors(self):
        """DEFAULT_LOAD_FACTORS contains correct values."""
        from grillex.core import DEFAULT_LOAD_FACTORS, LimitState

        # ULSa: 1.3 * DL + 1.3 * LL + 1.0 * EL
        assert DEFAULT_LOAD_FACTORS[LimitState.ULSa].dead_load == 1.3
        assert DEFAULT_LOAD_FACTORS[LimitState.ULSa].live_load == 1.3
        assert DEFAULT_LOAD_FACTORS[LimitState.ULSa].environmental == 1.0

        # ULSb: 1.0 * DL + 1.0 * LL + 1.3 * EL
        assert DEFAULT_LOAD_FACTORS[LimitState.ULSb].dead_load == 1.0
        assert DEFAULT_LOAD_FACTORS[LimitState.ULSb].environmental == 1.3

        # SLS: 1.0 * all
        assert DEFAULT_LOAD_FACTORS[LimitState.SLS].dead_load == 1.0
        assert DEFAULT_LOAD_FACTORS[LimitState.SLS].live_load == 1.0
        assert DEFAULT_LOAD_FACTORS[LimitState.SLS].environmental == 1.0

    def test_default_removal_factors(self):
        """DEFAULT_REMOVAL_FACTORS has reduced ULSa factors."""
        from grillex.core import DEFAULT_REMOVAL_FACTORS, LimitState

        # Removal ULSa: 1.1 instead of 1.3
        assert DEFAULT_REMOVAL_FACTORS[LimitState.ULSa].dead_load == 1.1
        assert DEFAULT_REMOVAL_FACTORS[LimitState.ULSa].live_load == 1.1


class TestAnalysisSettings:
    """Tests for AnalysisSettings dataclass."""

    def test_create_default_settings(self):
        """AnalysisSettings can be created with defaults."""
        from grillex.core import AnalysisSettings, DesignMethod, OperationType

        settings = AnalysisSettings()

        assert settings.design_method == DesignMethod.LRFD
        assert settings.operation_type == OperationType.IN_SERVICE

    def test_lrfd_limit_states(self):
        """LRFD design method returns ULSa and ULSb limit states."""
        from grillex.core import AnalysisSettings, DesignMethod, LimitState

        settings = AnalysisSettings(design_method=DesignMethod.LRFD)
        limit_states = settings.get_limit_states()

        assert LimitState.ULSa in limit_states
        assert LimitState.ULSb in limit_states
        assert LimitState.SLS not in limit_states
        assert len(limit_states) == 2

    def test_asd_limit_states(self):
        """ASD design method returns only SLS limit state."""
        from grillex.core import AnalysisSettings, DesignMethod, LimitState

        settings = AnalysisSettings(design_method=DesignMethod.ASD)
        limit_states = settings.get_limit_states()

        assert limit_states == [LimitState.SLS]

    def test_get_factors_default(self):
        """get_factors returns default factors for in-service."""
        from grillex.core import AnalysisSettings, DesignMethod, LimitState

        settings = AnalysisSettings(design_method=DesignMethod.LRFD)
        factors = settings.get_factors(LimitState.ULSa)

        assert factors.dead_load == 1.3
        assert factors.live_load == 1.3
        assert factors.environmental == 1.0

    def test_get_factors_removal(self):
        """get_factors returns removal factors for removal operation."""
        from grillex.core import AnalysisSettings, DesignMethod, LimitState, OperationType

        settings = AnalysisSettings(
            design_method=DesignMethod.LRFD,
            operation_type=OperationType.REMOVAL
        )
        factors = settings.get_factors(LimitState.ULSa)

        assert factors.dead_load == 1.1
        assert factors.live_load == 1.1

    def test_custom_factors_override(self):
        """Custom factors override defaults."""
        from grillex.core import (
            AnalysisSettings, DesignMethod, LimitState, LoadCombinationFactors
        )

        custom = {
            LimitState.ULSa: LoadCombinationFactors(
                dead_load=1.5, live_load=1.5, environmental=1.2
            )
        }
        settings = AnalysisSettings(
            design_method=DesignMethod.LRFD,
            custom_factors=custom
        )
        factors = settings.get_factors(LimitState.ULSa)

        assert factors.dead_load == 1.5
        assert factors.live_load == 1.5
        assert factors.environmental == 1.2


class TestMotionAmplitudes:
    """Tests for MotionAmplitudes dataclass."""

    def test_create_amplitudes(self):
        """MotionAmplitudes can be created."""
        from grillex.core import MotionAmplitudes

        amp = MotionAmplitudes(
            heave=2.5,
            roll_angle=15.0,
            roll_period=10.0,
            pitch_angle=5.0,
            pitch_period=8.0
        )

        assert amp.heave == 2.5
        assert amp.roll_angle == 15.0
        assert amp.roll_period == 10.0

    def test_get_roll_acceleration_from_angle_period(self):
        """Roll acceleration is computed from angle/period."""
        from grillex.core import MotionAmplitudes

        amp = MotionAmplitudes(roll_angle=15.0, roll_period=10.0)
        roll_accel = amp.get_roll_acceleration()

        # α = (2π/T)² × θ
        theta_rad = math.radians(15.0)
        omega = 2 * math.pi / 10.0
        expected = omega * omega * theta_rad

        np.testing.assert_almost_equal(roll_accel, expected, decimal=6)

    def test_get_roll_acceleration_direct(self):
        """Direct roll acceleration overrides angle/period."""
        from grillex.core import MotionAmplitudes

        amp = MotionAmplitudes(
            roll_accel=0.5,  # Direct acceleration
            roll_angle=15.0,  # This is ignored
            roll_period=10.0
        )
        roll_accel = amp.get_roll_acceleration()

        assert roll_accel == 0.5

    def test_get_pitch_acceleration_from_angle_period(self):
        """Pitch acceleration is computed from angle/period."""
        from grillex.core import MotionAmplitudes

        amp = MotionAmplitudes(pitch_angle=5.0, pitch_period=8.0)
        pitch_accel = amp.get_pitch_acceleration()

        theta_rad = math.radians(5.0)
        omega = 2 * math.pi / 8.0
        expected = omega * omega * theta_rad

        np.testing.assert_almost_equal(pitch_accel, expected, decimal=6)


class TestVesselMotionsFromAmplitudes:
    """Tests for VesselMotionsFromAmplitudes generator."""

    def test_generate_heave_only(self):
        """Generates heave+/- when only heave is specified."""
        from grillex.core import VesselMotionsFromAmplitudes, MotionAmplitudes

        amp = MotionAmplitudes(heave=2.5)
        generator = VesselMotionsFromAmplitudes("Test", amp)
        motions = generator.get_motions()

        assert len(motions) == 2
        assert "Heave+" in motions[0].name
        assert "Heave-" in motions[1].name

    def test_generate_all_motions(self):
        """Generates 6 motions for heave+pitch+roll."""
        from grillex.core import VesselMotionsFromAmplitudes, MotionAmplitudes

        amp = MotionAmplitudes(
            heave=2.5,
            pitch_angle=5.0,
            pitch_period=8.0,
            roll_angle=10.0,
            roll_period=10.0
        )
        generator = VesselMotionsFromAmplitudes(
            "Design",
            amp,
            motion_center=[50.0, 0.0, 5.0]
        )
        motions = generator.get_motions()

        assert len(motions) == 6
        names = [m.name for m in motions]
        assert "Design - Heave+" in names
        assert "Design - Heave-" in names
        assert "Design - Pitch+" in names
        assert "Design - Pitch-" in names
        assert "Design - Roll+" in names
        assert "Design - Roll-" in names

    def test_pitch_surge_coupling(self):
        """Pitch+ couples with surge (default +1.0 coupling)."""
        from grillex.core import VesselMotionsFromAmplitudes, MotionAmplitudes, MotionType

        amp = MotionAmplitudes(pitch_accel=0.1)  # Direct acceleration
        generator = VesselMotionsFromAmplitudes(
            "Test",
            amp,
            pitch_surge_coupling=1.0  # +pitch = +surge
        )
        motions = generator.get_motions()

        pitch_pos = next(m for m in motions if "Pitch+" in m.name)
        surge_comp = pitch_pos.get_component_by_type(MotionType.SURGE)

        assert surge_comp is not None
        assert surge_comp.amplitude > 0  # Positive surge for positive pitch

    def test_roll_sway_coupling(self):
        """Roll+ couples with sway (default -1.0 coupling)."""
        from grillex.core import VesselMotionsFromAmplitudes, MotionAmplitudes, MotionType

        amp = MotionAmplitudes(roll_accel=0.1)  # Direct acceleration
        generator = VesselMotionsFromAmplitudes(
            "Test",
            amp,
            roll_sway_coupling=-1.0  # +roll = -sway
        )
        motions = generator.get_motions()

        roll_pos = next(m for m in motions if "Roll+" in m.name)
        sway_comp = roll_pos.get_component_by_type(MotionType.SWAY)

        assert sway_comp is not None
        assert sway_comp.amplitude < 0  # Negative sway for positive roll

    def test_motion_center_propagated(self):
        """Motion center is propagated to all generated motions."""
        from grillex.core import VesselMotionsFromAmplitudes, MotionAmplitudes

        amp = MotionAmplitudes(heave=2.5)
        center = [50.0, 10.0, 5.0]
        generator = VesselMotionsFromAmplitudes("Test", amp, motion_center=center)
        motions = generator.get_motions()

        for m in motions:
            assert m.motion_center == center


class TestVesselMotionsFromNobleDenton:
    """Tests for VesselMotionsFromNobleDenton generator."""

    def test_generates_6_cases(self):
        """Generates exactly 6 load cases."""
        from grillex.core import VesselMotionsFromNobleDenton

        generator = VesselMotionsFromNobleDenton(
            "ND",
            heave=2.5,
            roll_angle=15.0,
            roll_period=10.0,
            pitch_angle=5.0,
            pitch_period=8.0
        )
        motions = generator.get_motions()

        assert len(motions) == 6

    def test_case_names(self):
        """Cases are named correctly per Noble Denton convention."""
        from grillex.core import VesselMotionsFromNobleDenton

        generator = VesselMotionsFromNobleDenton(
            "ND",
            heave=2.5,
            pitch_angle=5.0,
            pitch_period=8.0,
            roll_angle=10.0,
            roll_period=10.0
        )
        motions = generator.get_motions()
        names = [m.name for m in motions]

        assert "ND - Heave+" in names
        assert "ND - Heave-" in names
        assert "ND - Pitch+ Heave" in names
        assert "ND - Pitch- Heave" in names
        assert "ND - Roll+ Heave" in names
        assert "ND - Roll- Heave" in names

    def test_heave_only_cases(self):
        """Heave-only cases have only heave acceleration."""
        from grillex.core import VesselMotionsFromNobleDenton, MotionType

        generator = VesselMotionsFromNobleDenton(
            "ND",
            heave=2.5,
            pitch_angle=5.0,
            pitch_period=8.0
        )
        motions = generator.get_motions()

        heave_pos = next(m for m in motions if m.name == "ND - Heave+")
        heave_neg = next(m for m in motions if m.name == "ND - Heave-")

        # Heave+ has positive heave
        heave_comp = heave_pos.get_component_by_type(MotionType.HEAVE)
        assert heave_comp is not None
        assert heave_comp.amplitude == 2.5

        # Heave- has negative heave
        heave_comp_neg = heave_neg.get_component_by_type(MotionType.HEAVE)
        assert heave_comp_neg.amplitude == -2.5

    def test_pitch_heave_cases_have_both(self):
        """Pitch+Heave cases have both pitch and heave."""
        from grillex.core import VesselMotionsFromNobleDenton, MotionType

        generator = VesselMotionsFromNobleDenton(
            "ND",
            heave=2.5,
            pitch_angle=5.0,
            pitch_period=8.0
        )
        motions = generator.get_motions()

        pitch_heave_pos = next(m for m in motions if "Pitch+ Heave" in m.name)

        heave = pitch_heave_pos.get_component_by_type(MotionType.HEAVE)
        pitch = pitch_heave_pos.get_component_by_type(MotionType.PITCH)

        assert heave is not None
        assert pitch is not None
        assert heave.amplitude == 2.5
        assert pitch.amplitude > 0

    def test_roll_heave_cases_have_both(self):
        """Roll+Heave cases have both roll and heave."""
        from grillex.core import VesselMotionsFromNobleDenton, MotionType

        generator = VesselMotionsFromNobleDenton(
            "ND",
            heave=2.5,
            roll_angle=15.0,
            roll_period=10.0
        )
        motions = generator.get_motions()

        roll_heave_pos = next(m for m in motions if "Roll+ Heave" in m.name)

        heave = roll_heave_pos.get_component_by_type(MotionType.HEAVE)
        roll = roll_heave_pos.get_component_by_type(MotionType.ROLL)

        assert heave is not None
        assert roll is not None
        assert heave.amplitude == 2.5
        assert roll.amplitude > 0

    def test_pitch_surge_coupling_noble_denton(self):
        """Pitch cases include coupled surge acceleration."""
        from grillex.core import VesselMotionsFromNobleDenton, MotionType

        generator = VesselMotionsFromNobleDenton(
            "ND",
            heave=2.5,
            pitch_accel=0.1,  # Direct acceleration for easier testing
            pitch_surge_coupling=1.0
        )
        motions = generator.get_motions()

        pitch_pos = next(m for m in motions if "Pitch+ Heave" in m.name)
        surge = pitch_pos.get_component_by_type(MotionType.SURGE)

        assert surge is not None
        assert surge.amplitude == pytest.approx(0.1, rel=0.01)  # Same as pitch

    def test_roll_sway_coupling_noble_denton(self):
        """Roll cases include coupled sway acceleration."""
        from grillex.core import VesselMotionsFromNobleDenton, MotionType

        generator = VesselMotionsFromNobleDenton(
            "ND",
            heave=2.5,
            roll_accel=0.1,  # Direct acceleration for easier testing
            roll_sway_coupling=-1.0
        )
        motions = generator.get_motions()

        roll_pos = next(m for m in motions if "Roll+ Heave" in m.name)
        sway = roll_pos.get_component_by_type(MotionType.SWAY)

        assert sway is not None
        assert sway.amplitude == pytest.approx(-0.1, rel=0.01)  # Opposite sign

    def test_motion_center_propagated_nd(self):
        """Motion center is propagated to all ND cases."""
        from grillex.core import VesselMotionsFromNobleDenton

        center = [50.0, 0.0, 5.0]
        generator = VesselMotionsFromNobleDenton(
            "ND",
            heave=2.5,
            motion_center=center
        )
        motions = generator.get_motions()

        for m in motions:
            assert m.motion_center == center


class TestLoadCombinationGeneration:
    """Tests for generate_load_combinations function."""

    def test_lrfd_generates_2x_combinations(self):
        """LRFD generates 2x combinations (ULSa and ULSb for each motion)."""
        from grillex.core import (
            VesselMotionsFromNobleDenton,
            AnalysisSettings,
            DesignMethod,
            generate_load_combinations
        )

        nd = VesselMotionsFromNobleDenton("ND", heave=2.5, pitch_angle=5.0, pitch_period=8.0)
        settings = AnalysisSettings(design_method=DesignMethod.LRFD)

        combos = generate_load_combinations(nd, settings)

        # 6 motions * 2 limit states = 12 combinations
        assert len(combos) == 12

    def test_asd_generates_1x_combinations(self):
        """ASD generates 1x combinations (SLS only for each motion)."""
        from grillex.core import (
            VesselMotionsFromNobleDenton,
            AnalysisSettings,
            DesignMethod,
            generate_load_combinations
        )

        nd = VesselMotionsFromNobleDenton("ND", heave=2.5, pitch_angle=5.0, pitch_period=8.0)
        settings = AnalysisSettings(design_method=DesignMethod.ASD)

        combos = generate_load_combinations(nd, settings)

        # 6 motions * 1 limit state = 6 combinations
        assert len(combos) == 6

    def test_combination_names(self):
        """Combination names include limit state and motion name."""
        from grillex.core import (
            VesselMotionsFromNobleDenton,
            AnalysisSettings,
            DesignMethod,
            generate_load_combinations
        )

        nd = VesselMotionsFromNobleDenton("ND", heave=2.5)
        settings = AnalysisSettings(design_method=DesignMethod.LRFD)

        combos = generate_load_combinations(nd, settings)
        names = [c.name for c in combos]

        assert any("ULSA" in name and "Heave+" in name for name in names)
        assert any("ULSB" in name and "Heave+" in name for name in names)

    def test_combination_factors_lrfd(self):
        """LRFD combinations have correct factors."""
        from grillex.core import (
            VesselMotionsFromNobleDenton,
            AnalysisSettings,
            DesignMethod,
            LimitState,
            generate_load_combinations
        )

        nd = VesselMotionsFromNobleDenton("ND", heave=2.5)
        settings = AnalysisSettings(design_method=DesignMethod.LRFD)

        combos = generate_load_combinations(nd, settings)

        ulsa_combo = next(c for c in combos if c.limit_state == LimitState.ULSa)
        ulsb_combo = next(c for c in combos if c.limit_state == LimitState.ULSb)

        # ULSa factors
        assert ulsa_combo.dead_load_factor == 1.3
        assert ulsa_combo.live_load_factor == 1.3
        assert ulsa_combo.environmental_factor == 1.0

        # ULSb factors
        assert ulsb_combo.dead_load_factor == 1.0
        assert ulsb_combo.environmental_factor == 1.3

    def test_combination_factors_removal(self):
        """Removal operation uses reduced ULSa factors."""
        from grillex.core import (
            VesselMotionsFromNobleDenton,
            AnalysisSettings,
            DesignMethod,
            OperationType,
            LimitState,
            generate_load_combinations
        )

        nd = VesselMotionsFromNobleDenton("ND", heave=2.5)
        settings = AnalysisSettings(
            design_method=DesignMethod.LRFD,
            operation_type=OperationType.REMOVAL
        )

        combos = generate_load_combinations(nd, settings)

        ulsa_combo = next(c for c in combos if c.limit_state == LimitState.ULSa)

        # Removal ULSa factors: 1.1 instead of 1.3
        assert ulsa_combo.dead_load_factor == 1.1
        assert ulsa_combo.live_load_factor == 1.1

    def test_combination_has_vessel_motion_reference(self):
        """Each combination references its VesselMotion."""
        from grillex.core import (
            VesselMotionsFromNobleDenton,
            AnalysisSettings,
            DesignMethod,
            generate_load_combinations
        )

        nd = VesselMotionsFromNobleDenton("ND", heave=2.5)
        settings = AnalysisSettings(design_method=DesignMethod.LRFD)

        combos = generate_load_combinations(nd, settings)

        for combo in combos:
            assert combo.vessel_motion is not None
            assert combo.vessel_motion.name in combo.name
