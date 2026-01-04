"""
YAML input parser for structural models.

This module provides functions to load structural models from YAML files,
making it easy to define models in a human-readable format.

YAML Schema:
    name: "Model Name"  # Optional, defaults to filename

    materials:
      - name: Steel
        E: 210000000      # Young's modulus [kN/m²]
        nu: 0.3           # Poisson's ratio
        rho: 7.85e-6      # Density [mT/m³]

    sections:
      - name: IPE300
        A: 0.00538        # Area [m²]
        Iy: 8.36e-5       # Moment of inertia y [m⁴]
        Iz: 6.04e-6       # Moment of inertia z [m⁴]
        J: 2.01e-7        # Torsional constant [m⁴]

    beams:
      - start: [0, 0, 0]  # Start coordinates [m]
        end: [6, 0, 0]    # End coordinates [m]
        section: IPE300   # Section name (must exist)
        material: Steel   # Material name (must exist)

    boundary_conditions:
      - node: [0, 0, 0]   # Node coordinates
        type: fixed       # fixed, pinned, or custom
      - node: [6, 0, 0]
        type: custom
        dofs: [UY]        # List of DOF names to fix

    load_cases:
      - name: "Dead Load"
        type: Permanent   # Permanent, Variable, Environmental, Accidental
        loads:
          - node: [6, 0, 0]
            dof: UY
            value: -10.0  # [kN] or [kN·m]

      - name: "Design Motion LC"
        type: Environmental
        vessel_motion: "Design Heave + Roll"  # Reference to vessel motion

    vessel_motions:
      - name: "Design Heave + Roll"
        motion_center: [50.0, 0.0, 5.0]  # [m] reference point for rotations
        heave: 2.5       # [m/s²] vertical acceleration
        roll: 0.12       # [rad/s²] roll angular acceleration
        pitch: 0.08      # [rad/s²] pitch angular acceleration
        # Alternative: angle/period format (converted to rad/s²)
        # roll_angle: 15.0   # [degrees]
        # roll_period: 10.0  # [seconds]

      - name: "Gravity"
        gravity: true     # Shorthand for -9.81 m/s² in Z

Usage:
    from grillex.io import load_model_from_yaml

    model = load_model_from_yaml("model.yaml")
    model.analyze()

    disp = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
"""

from pathlib import Path
from typing import Any, Dict, Optional
import yaml
import numpy as np

from grillex.core import StructuralModel, DOFIndex, LoadCaseType
from grillex.core.vessel_motion import VesselMotion


class YAMLLoadError(Exception):
    """Exception raised for errors during YAML model loading."""
    pass


def load_model_from_yaml(file_path: str) -> StructuralModel:
    """Load a structural model from a YAML file.

    Args:
        file_path: Path to YAML file

    Returns:
        StructuralModel instance

    Raises:
        YAMLLoadError: If YAML is invalid or model cannot be built
        FileNotFoundError: If file doesn't exist
    """
    path = Path(file_path)

    if not path.exists():
        raise FileNotFoundError(f"YAML file not found: {file_path}")

    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise YAMLLoadError(f"Invalid YAML syntax: {e}")

    if data is None:
        raise YAMLLoadError("YAML file is empty")

    if not isinstance(data, dict):
        raise YAMLLoadError("YAML root must be a dictionary")

    # Use filename as default model name
    default_name = path.stem

    return build_model_from_dict(data, default_name)


def build_model_from_dict(data: dict[str, Any], default_name: str = "Unnamed") -> StructuralModel:
    """Build a structural model from a dictionary (parsed YAML).

    Args:
        data: Dictionary with model data
        default_name: Default model name if not specified

    Returns:
        StructuralModel instance

    Raises:
        YAMLLoadError: If data is invalid or model cannot be built
    """
    # Create model
    model_name = data.get('name', default_name)
    model = StructuralModel(name=model_name)

    # Load materials
    try:
        _load_materials(model, data.get('materials', []))
    except Exception as e:
        raise YAMLLoadError(f"Error loading materials: {e}")

    # Load sections
    try:
        _load_sections(model, data.get('sections', []))
    except Exception as e:
        raise YAMLLoadError(f"Error loading sections: {e}")

    # Load beams
    try:
        _load_beams(model, data.get('beams', []))
    except Exception as e:
        raise YAMLLoadError(f"Error loading beams: {e}")

    # Load boundary conditions
    try:
        _load_boundary_conditions(model, data.get('boundary_conditions', []))
    except Exception as e:
        raise YAMLLoadError(f"Error loading boundary conditions: {e}")

    # Load vessel motions first (so load cases can reference them)
    vessel_motion_defs: Dict[str, VesselMotion] = {}
    try:
        vessel_motion_defs = _load_vessel_motions(model, data.get('vessel_motions', []))
    except Exception as e:
        raise YAMLLoadError(f"Error loading vessel motions: {e}")

    # Load load cases (pass vessel motion definitions for references)
    try:
        _load_load_cases(model, data.get('load_cases', []), vessel_motion_defs)
    except Exception as e:
        raise YAMLLoadError(f"Error loading load cases: {e}")

    return model


def _load_materials(model: StructuralModel, materials_data: list[dict]) -> None:
    """Load materials into the model.

    Args:
        model: StructuralModel instance
        materials_data: List of material dictionaries

    Raises:
        ValueError: If material data is invalid
    """
    if not isinstance(materials_data, list):
        raise ValueError("'materials' must be a list")

    for i, mat_data in enumerate(materials_data):
        if not isinstance(mat_data, dict):
            raise ValueError(f"Material {i} must be a dictionary")

        # Required fields
        required = ['name', 'E', 'nu', 'rho']
        for field in required:
            if field not in mat_data:
                raise ValueError(f"Material {i} missing required field: {field}")

        try:
            model.add_material(
                name=mat_data['name'],
                E=float(mat_data['E']),
                nu=float(mat_data['nu']),
                rho=float(mat_data['rho'])
            )
        except Exception as e:
            raise ValueError(f"Error creating material '{mat_data.get('name', i)}': {e}")


def _load_sections(model: StructuralModel, sections_data: list[dict]) -> None:
    """Load sections into the model.

    Args:
        model: StructuralModel instance
        sections_data: List of section dictionaries

    Raises:
        ValueError: If section data is invalid
    """
    if not isinstance(sections_data, list):
        raise ValueError("'sections' must be a list")

    for i, sec_data in enumerate(sections_data):
        if not isinstance(sec_data, dict):
            raise ValueError(f"Section {i} must be a dictionary")

        # Required fields
        required = ['name', 'A', 'Iy', 'Iz', 'J']
        for field in required:
            if field not in sec_data:
                raise ValueError(f"Section {i} missing required field: {field}")

        try:
            model.add_section(
                name=sec_data['name'],
                A=float(sec_data['A']),
                Iy=float(sec_data['Iy']),
                Iz=float(sec_data['Iz']),
                J=float(sec_data['J'])
            )
        except Exception as e:
            raise ValueError(f"Error creating section '{sec_data.get('name', i)}': {e}")


def _load_beams(model: StructuralModel, beams_data: list[dict]) -> None:
    """Load beams into the model.

    Args:
        model: StructuralModel instance
        beams_data: List of beam dictionaries

    Raises:
        ValueError: If beam data is invalid
    """
    if not isinstance(beams_data, list):
        raise ValueError("'beams' must be a list")

    for i, beam_data in enumerate(beams_data):
        if not isinstance(beam_data, dict):
            raise ValueError(f"Beam {i} must be a dictionary")

        # Required fields
        required = ['start', 'end', 'section', 'material']
        for field in required:
            if field not in beam_data:
                raise ValueError(f"Beam {i} missing required field: {field}")

        # Validate coordinates
        start = beam_data['start']
        end = beam_data['end']

        if not isinstance(start, list) or len(start) != 3:
            raise ValueError(f"Beam {i}: 'start' must be a list of 3 coordinates")
        if not isinstance(end, list) or len(end) != 3:
            raise ValueError(f"Beam {i}: 'end' must be a list of 3 coordinates")

        try:
            model.add_beam_by_coords(
                start_pos=[float(x) for x in start],
                end_pos=[float(x) for x in end],
                section_name=beam_data['section'],
                material_name=beam_data['material']
            )
        except Exception as e:
            raise ValueError(f"Error creating beam {i}: {e}")


def _load_boundary_conditions(model: StructuralModel, bc_data: list[dict]) -> None:
    """Load boundary conditions into the model.

    Args:
        model: StructuralModel instance
        bc_data: List of boundary condition dictionaries

    Raises:
        ValueError: If BC data is invalid
    """
    if not isinstance(bc_data, list):
        raise ValueError("'boundary_conditions' must be a list")

    for i, bc in enumerate(bc_data):
        if not isinstance(bc, dict):
            raise ValueError(f"Boundary condition {i} must be a dictionary")

        if 'node' not in bc:
            raise ValueError(f"Boundary condition {i} missing 'node' field")
        if 'type' not in bc:
            raise ValueError(f"Boundary condition {i} missing 'type' field")

        node = bc['node']
        if not isinstance(node, list) or len(node) != 3:
            raise ValueError(f"Boundary condition {i}: 'node' must be a list of 3 coordinates")

        node_pos = [float(x) for x in node]
        bc_type = bc['type'].lower()

        try:
            if bc_type == 'fixed':
                model.fix_node_at(node_pos)
            elif bc_type == 'pinned':
                model.pin_node_at(node_pos)
            elif bc_type == 'custom':
                # Custom BCs with specific DOFs
                if 'dofs' not in bc:
                    raise ValueError(f"Boundary condition {i}: 'custom' type requires 'dofs' field")

                dof_names = bc['dofs']
                if not isinstance(dof_names, list):
                    raise ValueError(f"Boundary condition {i}: 'dofs' must be a list")

                for dof_name in dof_names:
                    dof = _parse_dof(dof_name)
                    value = bc.get('value', 0.0)
                    model.fix_dof_at(node_pos, dof, value)
            else:
                raise ValueError(f"Boundary condition {i}: unknown type '{bc_type}'. Use 'fixed', 'pinned', or 'custom'")
        except Exception as e:
            raise ValueError(f"Error applying boundary condition {i}: {e}")


def _load_load_cases(
    model: StructuralModel,
    load_cases_data: list[dict],
    vessel_motion_defs: Optional[Dict[str, VesselMotion]] = None
) -> None:
    """Load load cases into the model.

    Args:
        model: StructuralModel instance
        load_cases_data: List of load case dictionaries
        vessel_motion_defs: Dictionary of pre-defined vessel motions that
            load cases can reference by name

    Raises:
        ValueError: If load case data is invalid
    """
    if vessel_motion_defs is None:
        vessel_motion_defs = {}

    if not isinstance(load_cases_data, list):
        raise ValueError("'load_cases' must be a list")

    for i, lc_data in enumerate(load_cases_data):
        if not isinstance(lc_data, dict):
            raise ValueError(f"Load case {i} must be a dictionary")

        if 'name' not in lc_data:
            raise ValueError(f"Load case {i} missing 'name' field")

        # Parse load case type
        lc_type_str = lc_data.get('type', 'Variable')
        lc_type = _parse_load_case_type(lc_type_str)

        # Create load case
        lc_name = lc_data['name']
        load_case = model.create_load_case(lc_name, lc_type)

        # Check for vessel motion reference
        if 'vessel_motion' in lc_data:
            vm_name = lc_data['vessel_motion']
            if vm_name not in vessel_motion_defs:
                available = list(vessel_motion_defs.keys())
                raise ValueError(
                    f"Load case '{lc_name}': vessel motion '{vm_name}' not found. "
                    f"Available vessel motions: {available}"
                )

            # Get the vessel motion and apply its accelerations to this load case
            vessel_motion = vessel_motion_defs[vm_name]
            accel_6d, motion_center = vessel_motion.get_acceleration_field()
            load_case.set_acceleration_field(
                np.array(accel_6d),
                np.array(motion_center)
            )

        # Load nodal loads
        loads = lc_data.get('loads', [])
        if not isinstance(loads, list):
            raise ValueError(f"Load case '{lc_name}': 'loads' must be a list")

        for j, load in enumerate(loads):
            if not isinstance(load, dict):
                raise ValueError(f"Load case '{lc_name}', load {j}: must be a dictionary")

            # Support both old format (node/dof/value) and new format (position/force/moment)
            if 'position' in load:
                # New format: position + force/moment vectors
                position = load['position']
                if not isinstance(position, list) or len(position) != 3:
                    raise ValueError(f"Load case '{lc_name}', load {j}: 'position' must be a list of 3 coordinates")

                try:
                    pos = [float(x) for x in position]
                    force = None
                    moment = None

                    if 'force' in load:
                        force_data = load['force']
                        if not isinstance(force_data, list) or len(force_data) != 3:
                            raise ValueError(f"Load case '{lc_name}', load {j}: 'force' must be a list of 3 values")
                        force = [float(x) for x in force_data]

                    if 'moment' in load:
                        moment_data = load['moment']
                        if not isinstance(moment_data, list) or len(moment_data) != 3:
                            raise ValueError(f"Load case '{lc_name}', load {j}: 'moment' must be a list of 3 values")
                        moment = [float(x) for x in moment_data]

                    if force is None and moment is None:
                        raise ValueError(f"Load case '{lc_name}', load {j}: must specify 'force' and/or 'moment'")

                    model.add_point_load(pos, force=force, moment=moment, load_case=load_case)
                except Exception as e:
                    raise ValueError(f"Load case '{lc_name}', load {j}: {e}")

            elif 'node' in load:
                # Legacy format: node/dof/value - convert to new format
                if 'dof' not in load:
                    raise ValueError(f"Load case '{lc_name}', load {j}: missing 'dof' field")
                if 'value' not in load:
                    raise ValueError(f"Load case '{lc_name}', load {j}: missing 'value' field")

                node = load['node']
                if not isinstance(node, list) or len(node) != 3:
                    raise ValueError(f"Load case '{lc_name}', load {j}: 'node' must be a list of 3 coordinates")

                try:
                    node_pos = [float(x) for x in node]
                    dof = _parse_dof(load['dof'])
                    value = float(load['value'])

                    # Convert DOF-based load to force/moment vectors
                    force = [0.0, 0.0, 0.0]
                    moment = [0.0, 0.0, 0.0]

                    if dof == DOFIndex.UX:
                        force[0] = value
                    elif dof == DOFIndex.UY:
                        force[1] = value
                    elif dof == DOFIndex.UZ:
                        force[2] = value
                    elif dof == DOFIndex.RX:
                        moment[0] = value
                    elif dof == DOFIndex.RY:
                        moment[1] = value
                    elif dof == DOFIndex.RZ:
                        moment[2] = value
                    else:
                        raise ValueError(f"Unsupported DOF for point load: {dof}")

                    model.add_point_load(node_pos, force=force, moment=moment, load_case=load_case)
                except Exception as e:
                    raise ValueError(f"Load case '{lc_name}', load {j}: {e}")
            else:
                raise ValueError(f"Load case '{lc_name}', load {j}: missing 'position' or 'node' field")


def _parse_dof(dof_str: str) -> DOFIndex:
    """Parse a DOF string to DOFIndex enum.

    Args:
        dof_str: DOF name (UX, UY, UZ, RX, RY, RZ, WARP)

    Returns:
        DOFIndex enum value

    Raises:
        ValueError: If DOF name is invalid
    """
    dof_map = {
        'UX': DOFIndex.UX,
        'UY': DOFIndex.UY,
        'UZ': DOFIndex.UZ,
        'RX': DOFIndex.RX,
        'RY': DOFIndex.RY,
        'RZ': DOFIndex.RZ,
        'WARP': DOFIndex.WARP
    }

    dof_upper = dof_str.upper()
    if dof_upper not in dof_map:
        valid = ', '.join(dof_map.keys())
        raise ValueError(f"Invalid DOF '{dof_str}'. Valid values: {valid}")

    return dof_map[dof_upper]


def _parse_load_case_type(type_str: str) -> LoadCaseType:
    """Parse a load case type string to LoadCaseType enum.

    Args:
        type_str: Load case type name

    Returns:
        LoadCaseType enum value

    Raises:
        ValueError: If type name is invalid
    """
    type_map = {
        'PERMANENT': LoadCaseType.Permanent,
        'VARIABLE': LoadCaseType.Variable,
        'ENVIRONMENTAL': LoadCaseType.Environmental,
        'ACCIDENTAL': LoadCaseType.Accidental
    }

    type_upper = type_str.upper()
    if type_upper not in type_map:
        valid = ', '.join(type_map.keys())
        raise ValueError(f"Invalid load case type '{type_str}'. Valid values: {valid}")

    return type_map[type_upper]


def _load_vessel_motions(model: StructuralModel, vessel_motions_data: list[dict]) -> dict:
    """Load vessel motions into the model.

    Vessel motions define 6-DOF accelerations for offshore structures on floating
    vessels. Each vessel motion can optionally create a load case with the
    appropriate acceleration field applied.

    Args:
        model: StructuralModel instance
        vessel_motions_data: List of vessel motion dictionaries

    Returns:
        Dictionary mapping vessel motion names to VesselMotion objects

    Raises:
        ValueError: If vessel motion data is invalid
    """
    import math
    from grillex.core.vessel_motion import VesselMotion

    if not isinstance(vessel_motions_data, list):
        raise ValueError("'vessel_motions' must be a list")

    vessel_motion_defs = {}

    for i, vm_data in enumerate(vessel_motions_data):
        if not isinstance(vm_data, dict):
            raise ValueError(f"Vessel motion {i} must be a dictionary")

        if 'name' not in vm_data:
            raise ValueError(f"Vessel motion {i} missing 'name' field")

        name = vm_data['name']

        # Check for gravity shorthand
        if vm_data.get('gravity', False):
            model.add_gravity_load_case(
                name=name,
                acceleration=vm_data.get('acceleration', 9.81),
                load_type=LoadCaseType.Permanent
            )
            continue

        # Parse motion center
        motion_center = None
        if 'motion_center' in vm_data:
            mc = vm_data['motion_center']
            if not isinstance(mc, list) or len(mc) != 3:
                raise ValueError(
                    f"Vessel motion '{name}': 'motion_center' must be a list of 3 coordinates"
                )
            motion_center = [float(x) for x in mc]

        # Parse load type
        lc_type_str = vm_data.get('load_type', 'Environmental')
        lc_type = _parse_load_case_type(lc_type_str)

        # Parse accelerations - either direct values or computed from angle/period

        # Linear accelerations (direct values in m/s²)
        surge = float(vm_data.get('surge', 0.0))
        sway = float(vm_data.get('sway', 0.0))
        heave = float(vm_data.get('heave', 0.0))

        # Angular accelerations - can be direct (rad/s²) or computed from angle/period
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # Roll: direct value or angle/period
        if 'roll' in vm_data:
            roll = float(vm_data['roll'])
        elif 'roll_angle' in vm_data and 'roll_period' in vm_data:
            angle_deg = float(vm_data['roll_angle'])
            period = float(vm_data['roll_period'])
            angle_rad = math.radians(angle_deg)
            omega = 2 * math.pi / period
            roll = omega * omega * angle_rad

        # Pitch: direct value or angle/period
        if 'pitch' in vm_data:
            pitch = float(vm_data['pitch'])
        elif 'pitch_angle' in vm_data and 'pitch_period' in vm_data:
            angle_deg = float(vm_data['pitch_angle'])
            period = float(vm_data['pitch_period'])
            angle_rad = math.radians(angle_deg)
            omega = 2 * math.pi / period
            pitch = omega * omega * angle_rad

        # Yaw: direct value or angle/period
        if 'yaw' in vm_data:
            yaw = float(vm_data['yaw'])
        elif 'yaw_angle' in vm_data and 'yaw_period' in vm_data:
            angle_deg = float(vm_data['yaw_angle'])
            period = float(vm_data['yaw_period'])
            angle_rad = math.radians(angle_deg)
            omega = 2 * math.pi / period
            yaw = omega * omega * angle_rad

        # Check if we should auto-create a load case (default: True)
        auto_create_load_case = vm_data.get('auto_create_load_case', True)

        if auto_create_load_case:
            # Create the vessel motion load case
            motion = model.add_vessel_motion_load_case(
                name=name,
                heave=heave,
                pitch=pitch,
                roll=roll,
                surge=surge,
                sway=sway,
                yaw=yaw,
                motion_center=motion_center,
                load_type=lc_type
            )
            vessel_motion_defs[name] = motion
        else:
            # Just create the VesselMotion object without creating a load case
            motion = VesselMotion(name)
            if motion_center is not None:
                motion.set_motion_center(motion_center)
            if surge != 0.0:
                motion.add_surge(surge)
            if sway != 0.0:
                motion.add_sway(sway)
            if heave != 0.0:
                motion.add_heave(heave)
            if roll != 0.0:
                motion.add_roll(roll)
            if pitch != 0.0:
                motion.add_pitch(pitch)
            if yaw != 0.0:
                motion.add_yaw(yaw)
            # Register the motion on the model so it can be queried later
            model.register_vessel_motion(motion)
            vessel_motion_defs[name] = motion

    return vessel_motion_defs
