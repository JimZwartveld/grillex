=============================
Loads and Boundary Conditions
=============================

This guide covers how to apply loads and boundary conditions in Grillex.

Boundary Conditions
===================

Boundary conditions constrain the movement of nodes in your model.
They must be applied to prevent rigid body motion.

Fixed Support
-------------

A fixed support restrains all 6 DOFs (translations and rotations):

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>>
    >>> model = StructuralModel(name="Fixed Support Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>>
    >>> beam = model.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE200", "Steel")
    >>>
    >>> # Fix all DOFs at the left end
    >>> model.fix_node_at([0, 0, 0])

Pinned Support
--------------

A pinned support restrains translations but allows rotations:

.. doctest::

    >>> # Pin support (translations fixed, rotations free)
    >>> model.pin_node_at([5, 0, 0])

Individual DOF Constraints
--------------------------

You can also fix individual DOFs:

.. doctest::

    >>> model2 = StructuralModel(name="Individual DOF Example")
    >>> _ = model2.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model2.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>> beam = model2.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE200", "Steel")
    >>>
    >>> # Fix only vertical displacement
    >>> model2.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
    >>>
    >>> # Fix horizontal displacement
    >>> model2.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)

Prescribed Displacements
------------------------

Apply non-zero prescribed displacements:

.. doctest::

    >>> # Apply a prescribed settlement of 5mm downward
    >>> model2.fix_dof_at([5, 0, 0], DOFIndex.UZ, -0.005)

Point Loads
===========

Point loads are concentrated forces or moments applied at nodes.

Applying Point Forces
---------------------

.. doctest::

    >>> model3 = StructuralModel(name="Point Load Example")
    >>> _ = model3.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model3.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> beam = model3.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model3.fix_node_at([0, 0, 0])
    >>>
    >>> # Apply 10 kN downward force at free end
    >>> model3.add_point_load([6, 0, 0], force=[0, 0, -10.0])
    >>>
    >>> # Apply 5 kN horizontal force
    >>> model3.add_point_load([6, 0, 0], force=[5.0, 0, 0])

Applying Point Moments
----------------------

.. doctest::

    >>> # Apply a 20 kNm moment about Y-axis (causing rotation about Y)
    >>> model3.add_point_load([6, 0, 0], moment=[0, 20.0, 0])

DOF Index Reference
-------------------

.. list-table:: DOF Indices
   :header-rows: 1
   :widths: 20 30 50

   * - DOFIndex
     - Value
     - Description
   * - UX
     - 0
     - Translation along X (axial for X-aligned beams)
   * - UY
     - 1
     - Translation along Y
   * - UZ
     - 2
     - Translation along Z (vertical for horizontal beams)
   * - RX
     - 3
     - Rotation about X (torsion for X-aligned beams)
   * - RY
     - 4
     - Rotation about Y
   * - RZ
     - 5
     - Rotation about Z

Line Loads
==========

Line loads are distributed loads applied along beam elements.

Uniform Distributed Load
------------------------

.. doctest::

    >>> model4 = StructuralModel(name="Line Load Example")
    >>> _ = model4.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model4.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> beam = model4.add_beam_by_coords([0, 0, 0], [8, 0, 0], "IPE300", "Steel")
    >>> model4.fix_node_at([0, 0, 0])
    >>> model4.fix_node_at([8, 0, 0])
    >>>
    >>> # Apply 5 kN/m uniform downward load
    >>> model4.add_line_load(beam, [0, 0, -5.0])

Trapezoidal Loads
-----------------

Apply linearly varying loads:

.. doctest::

    >>> # Apply load varying from -3 kN/m at start to -7 kN/m at end
    >>> model4.add_line_load_to_beam(beam, DOFIndex.UZ, -3.0, -7.0)  # doctest: +SKIP

Self-Weight
===========

Apply gravity loads using acceleration:

.. doctest::

    >>> model5 = StructuralModel(name="Self-Weight Example")
    >>> _ = model5.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model5.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> beam = model5.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model5.fix_node_at([0, 0, 0])
    >>> model5.fix_node_at([6, 0, 0])
    >>>
    >>> # Apply standard gravity (9.81 m/s² downward)
    >>> # This creates inertial forces based on element mass
    >>> model5.add_acceleration(0, 0, -9.81)  # doctest: +SKIP

The self-weight load is automatically computed as:

.. math::

    w = \rho \cdot A \cdot g

For steel IPE300: w = 7.85×10⁻³ × 0.00538 × 9.81 ≈ 0.414 kN/m

Vessel Motions
==============

For offshore structures on floating vessels, Grillex provides vessel motion
support to apply 6-DOF accelerations from vessel motions.

Adding Gravity Load Case
------------------------

The simplest use case is adding a gravity load:

.. doctest::

    >>> model = StructuralModel(name="Gravity Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>> beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>> model.fix_node_at([6, 0, 0])
    >>>
    >>> # Add gravity load case (applies -9.81 m/s² in Z)
    >>> motion = model.add_gravity_load_case("Self-Weight")

Vessel Motion Load Case
-----------------------

For offshore structures, apply vessel motion accelerations:

.. doctest::

    >>> model2 = StructuralModel(name="Vessel Motion Example")
    >>> _ = model2.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model2.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>> beam = model2.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model2.fix_node_at([0, 0, 0])
    >>> model2.fix_node_at([6, 0, 0])
    >>>
    >>> # Add vessel motion load case with heave and roll
    >>> motion = model2.add_vessel_motion_load_case(
    ...     name="Design Heave + Roll",
    ...     heave=2.5,       # m/s² vertical acceleration
    ...     roll=0.12,       # rad/s² roll angular acceleration
    ...     motion_center=[50.0, 0.0, 5.0]  # Midship, waterline
    ... )

Motion Components
~~~~~~~~~~~~~~~~~

The six vessel motion DOFs are:

.. list-table:: Vessel Motion Components
   :header-rows: 1
   :widths: 15 20 65

   * - Motion
     - Units
     - Description
   * - Surge
     - m/s²
     - Longitudinal translation (X), positive forward
   * - Sway
     - m/s²
     - Transverse translation (Y), positive to port
   * - Heave
     - m/s²
     - Vertical translation (Z), positive upward
   * - Roll
     - rad/s²
     - Rotation about X-axis, positive is starboard down
   * - Pitch
     - rad/s²
     - Rotation about Y-axis, positive is bow down
   * - Yaw
     - rad/s²
     - Rotation about Z-axis, positive is bow to port

Motion Center
~~~~~~~~~~~~~

The motion center is the reference point for rotational motions. For a barge,
this is typically at the waterline amidships. Structures located away from
the motion center experience additional tangential accelerations due to rotation.

For a point at distance r from the motion center, the tangential acceleration
due to angular acceleration α is:

.. math::

    a_{tangential} = r \cdot \alpha

Using VesselMotion Class
------------------------

For more control, use the VesselMotion class directly:

.. doctest::

    >>> from grillex.core import VesselMotion, MotionType
    >>>
    >>> # Create vessel motion with fluent API
    >>> motion = VesselMotion("Design Motion")
    >>> _ = motion.set_motion_center([50.0, 0.0, 5.0])
    >>> _ = motion.add_heave(2.5)
    >>> _ = motion.add_roll(0.12)
    >>> _ = motion.add_pitch(0.08)

Factory Methods
~~~~~~~~~~~~~~~

Convenience factory methods for common scenarios:

.. doctest::

    >>> # Still water (no motions)
    >>> still = VesselMotion.create_still_water()
    >>> len(still.components)
    0
    >>>
    >>> # Heave-only condition
    >>> heave = VesselMotion.create_heave_only(2.5)
    >>> heave.get_component_by_type(MotionType.HEAVE).amplitude
    2.5
    >>>
    >>> # Roll from angle and period
    >>> # α = (2π/T)² × θ where θ is in radians
    >>> roll = VesselMotion.create_roll_condition(
    ...     roll_angle=15.0,  # degrees
    ...     roll_period=10.0   # seconds
    ... )
    >>> roll_accel = roll.get_component_by_type(MotionType.ROLL).amplitude
    >>> 0.1 < roll_accel < 0.2  # ~0.103 rad/s²
    True

Linked Load Cases and Auto-Update
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Load cases created via ``add_vessel_motion_load_case()`` are automatically linked
to the VesselMotion object. When the vessel motion is modified, all linked load
cases are automatically updated:

.. doctest::

    >>> model3 = StructuralModel(name="Linked LC Example")
    >>> _ = model3.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model3.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>> _ = model3.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model3.fix_node_at([0, 0, 0])
    >>>
    >>> # Create a vessel motion load case
    >>> motion = model3.add_vessel_motion_load_case("Heave", heave=2.5)
    >>>
    >>> # Get the linked load case
    >>> lc = model3._cpp_model.get_load_cases()[0]
    >>> model3.is_load_case_linked_to_vessel_motion(lc)
    True
    >>>
    >>> # Modify the vessel motion - linked load case updates automatically
    >>> _ = motion.add_roll(0.12)
    >>> float(lc.get_acceleration()[3])  # Roll component now present
    0.12

Signed Motion Pairs
~~~~~~~~~~~~~~~~~~~

For design scenarios that require both positive and negative motion directions,
use ``create_signed_pairs=True`` to automatically create two load cases:

.. doctest::

    >>> model4 = StructuralModel(name="Signed Pairs Example")
    >>> _ = model4.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model4.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>> _ = model4.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model4.fix_node_at([0, 0, 0])
    >>>
    >>> # Create vessel motion with signed pairs
    >>> motion = model4.add_vessel_motion_load_case(
    ...     "Roll",
    ...     roll=0.12,
    ...     create_signed_pairs=True  # Creates "Roll +" and "Roll -"
    ... )
    >>>
    >>> # Two linked load cases are created
    >>> len(motion.get_linked_load_cases())
    2

This creates load cases named "Roll +" and "Roll -" with opposite acceleration
signs. Both load cases are linked and automatically update when the VesselMotion
is modified.

Vessel Motion Generators
------------------------

For offshore design workflows, Grillex provides generator classes that create
multiple load cases and load combinations automatically based on industry standards.

Noble Denton Generator
~~~~~~~~~~~~~~~~~~~~~~

The ``VesselMotionsFromNobleDenton`` generator creates 6 standalone load cases
per Noble Denton guidelines for heavy lift and transportation operations:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> from grillex.core.vessel_motion import (
    ...     VesselMotionsFromNobleDenton,
    ...     AnalysisSettings,
    ...     DesignMethod,
    ... )
    >>>
    >>> # Create Noble Denton generator
    >>> nd_generator = VesselMotionsFromNobleDenton(
    ...     name="Transport",
    ...     heave=2.5,           # m/s² heave acceleration
    ...     roll_angle=15.0,     # degrees roll amplitude
    ...     roll_period=10.0,    # seconds roll period
    ...     pitch_angle=5.0,     # degrees pitch amplitude
    ...     pitch_period=8.0,    # seconds pitch period
    ...     motion_center=[50.0, 0.0, 5.0]  # Rotation center
    ... )
    >>>
    >>> # Generate the 6 load cases
    >>> motions = nd_generator.get_motions()
    >>> [m.name for m in motions]  # doctest: +NORMALIZE_WHITESPACE
    ['Transport - Heave+', 'Transport - Heave-',
     'Transport - Pitch+', 'Transport - Pitch-',
     'Transport - Roll+', 'Transport - Roll-']

**Key characteristics:**

- **6 standalone load cases**: Heave±, Pitch±, Roll± (each motion type is separate)
- **No surge/sway coupling**: Rotations are defined at the motion center, so coupling
  is handled implicitly through the reference point
- **Angular acceleration from angle/period**: α = (2π/T)² × θ for simple harmonic motion

Noble Denton Load Combinations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Noble Denton combinations pair heave with roll or pitch (8 combinations per limit state):

.. doctest::

    >>> # Get motion combinations (for load combination generation)
    >>> combos = nd_generator.get_motion_combinations()
    >>> len(combos)  # 8 pairings: Heave± × Roll±, Heave± × Pitch±
    8

The 8 combinations are:

- Heave+ & Roll+, Heave+ & Roll-, Heave- & Roll+, Heave- & Roll-
- Heave+ & Pitch+, Heave+ & Pitch-, Heave- & Pitch+, Heave- & Pitch-

Amplitudes Generator
~~~~~~~~~~~~~~~~~~~~

The ``VesselMotionsFromAmplitudes`` generator creates load cases from direct
acceleration amplitudes with surge/sway coupling rules:

.. doctest::

    >>> from grillex.core.vessel_motion import (
    ...     VesselMotionsFromAmplitudes,
    ...     MotionAmplitudes,
    ... )
    >>>
    >>> # Define motion amplitudes
    >>> amplitudes = MotionAmplitudes(
    ...     heave=2.5,        # m/s²
    ...     roll_accel=0.12,  # rad/s²
    ...     pitch_accel=0.08, # rad/s²
    ...     yaw_accel=0.05,   # rad/s²
    ... )
    >>>
    >>> # Create generator with coupling rules
    >>> amp_generator = VesselMotionsFromAmplitudes(
    ...     name="Design Motion",
    ...     amplitudes=amplitudes,
    ...     motion_center=[50.0, 0.0, 5.0],
    ... )
    >>>
    >>> # Get generated load cases (±pairs for each motion)
    >>> motions = amp_generator.get_motions()
    >>> len(motions)  # 8 load cases: Heave±, Pitch±, Roll±, Yaw±
    8

**Key characteristics:**

- **8 load cases**: ± pairs for Heave, Pitch, Roll, Yaw
- **Coupling rules applied**: +pitch → +surge, +roll → -sway
- **16 combinations per limit state**: All 2⁴ permutations of Heave± × Pitch± × Roll± × Yaw±

Load Combination Generation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Generate design load combinations with appropriate factors. Grillex supports
two design methods: **LRFD** and **ASD**.

**LRFD (Load and Resistance Factor Design):**

.. doctest::

    >>> from grillex.core.vessel_motion import generate_load_combinations
    >>>
    >>> # Configure LRFD settings (default)
    >>> settings = AnalysisSettings(design_method=DesignMethod.LRFD)
    >>>
    >>> # Generate combinations for Noble Denton
    >>> combinations = generate_load_combinations(nd_generator, settings)
    >>> len(combinations)  # 8 combos × 2 limit states (ULSa, ULSb) = 16
    16

.. list-table:: LRFD Load Combination Factors
   :header-rows: 1
   :widths: 20 20 20 20 20

   * - Limit State
     - Dead Load
     - Live Load
     - Environmental
     - Use Case
   * - ULS-a
     - 1.3
     - 1.3
     - 0.7
     - Permanent load dominant
   * - ULS-b
     - 1.0
     - 1.0
     - 1.3
     - Environmental load dominant

**ASD (Allowable Stress Design):**

ASD uses unfactored service loads with a single SLS (Serviceability Limit State):

.. doctest::

    >>> # Configure ASD settings
    >>> asd_settings = AnalysisSettings(design_method=DesignMethod.ASD)
    >>>
    >>> # Generate combinations - only SLS
    >>> asd_combinations = generate_load_combinations(nd_generator, asd_settings)
    >>> len(asd_combinations)  # 8 combos × 1 limit state (SLS) = 8
    8
    >>> asd_combinations[0].name
    'SLS - Heave+ Roll+'

.. list-table:: ASD Load Combination Factors
   :header-rows: 1
   :widths: 25 25 25 25

   * - Limit State
     - Dead Load
     - Live Load
     - Environmental
   * - SLS
     - 1.0
     - 1.0
     - 1.0

**Design Method Comparison:**

.. list-table:: Design Method Comparison
   :header-rows: 1
   :widths: 25 25 25 25

   * - Design Method
     - Limit States
     - Noble Denton Combos
     - Amplitudes Combos
   * - LRFD
     - ULS-a, ULS-b
     - 8 × 2 = 16
     - 16 × 2 = 32
   * - ASD
     - SLS only
     - 8 × 1 = 8
     - 16 × 1 = 16

Integrating with StructuralModel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add a generator to a model to create load cases and combinations automatically:

.. doctest::

    >>> model = StructuralModel(name="Offshore Module")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>> _ = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>> model.fix_node_at([6, 0, 0])
    >>>
    >>> # Add Noble Denton generator (creates load cases and combinations)
    >>> nd = VesselMotionsFromNobleDenton(
    ...     "Transport", heave=2.5,
    ...     roll_angle=15.0, roll_period=10.0,
    ...     pitch_angle=5.0, pitch_period=8.0,
    ... )
    >>> settings = AnalysisSettings(design_method=DesignMethod.LRFD)
    >>> motions = model.add_vessel_motions_generator(
    ...     generator=nd,
    ...     analysis_settings=settings,
    ...     create_load_cases=True,
    ...     generate_combinations=True,
    ... )
    >>>
    >>> # Query generated combinations
    >>> combos = model.get_generated_load_combinations()
    >>> len(combos)  # 16 combinations
    16
    >>> combos[0].name  # Example combination name
    'ULSA - Heave+ Roll+'

Load Cases
==========

Grillex supports multiple load cases and combinations:

Creating Load Cases
-------------------

.. doctest::

    >>> from grillex.core import LoadCaseType
    >>>
    >>> model6 = StructuralModel(name="Load Case Example")
    >>> _ = model6.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model6.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> beam = model6.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model6.fix_node_at([0, 0, 0])
    >>> model6.pin_node_at([6, 0, 0])

Load Case Types
---------------

.. list-table:: Load Case Types
   :header-rows: 1
   :widths: 30 70

   * - Type
     - Description
   * - Permanent
     - Dead loads, self-weight (static connections active)
   * - Variable
     - Live loads, imposed loads (dynamic connections active)
   * - Environmental
     - Wind, wave, current loads (dynamic connections active)
   * - Accidental
     - Impact, explosion loads

Common Loading Patterns
=======================

Cantilever with Point Load
--------------------------

.. doctest::

    >>> model = StructuralModel(name="Cantilever")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> beam = model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "IPE300", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>> model.add_point_load([4, 0, 0], force=[0, 0, -20.0])
    >>>
    >>> _ = model.analyze()
    >>> disp = model.get_displacement_at([4, 0, 0], DOFIndex.UZ)
    >>> disp < 0  # Deflects downward
    True

Simply Supported with UDL
-------------------------

.. doctest::

    >>> model = StructuralModel(name="Simply Supported")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE400", A=0.00845, Iy=2.31e-4, Iz=1.32e-5, J=5.1e-7)
    >>>
    >>> # Create two beam segments to have a node at midspan
    >>> beam1 = model.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE400", "Steel")
    >>> beam2 = model.add_beam_by_coords([5, 0, 0], [10, 0, 0], "IPE400", "Steel")
    >>>
    >>> # Pin at left, roller at right
    >>> model.fix_node_at([0, 0, 0])  # Fixed for simplicity
    >>> model.pin_node_at([10, 0, 0])
    >>>
    >>> # 8 kN/m uniform load
    >>> model.add_line_load(beam1, [0, 0, -8.0])
    >>> model.add_line_load(beam2, [0, 0, -8.0])
    >>>
    >>> _ = model.analyze()
    >>> # Maximum deflection at midspan
    >>> mid_disp = model.get_displacement_at([5, 0, 0], DOFIndex.UZ)
    >>> mid_disp < 0
    True

Managing Loads and Boundary Conditions
======================================

Grillex provides full CRUD (Create, Read, Update, Delete) operations for loads
and boundary conditions.

Querying Boundary Conditions
----------------------------

Get all boundary conditions in the model:

.. doctest::

    >>> model = StructuralModel(name="Query Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>> _ = model.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE200", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>>
    >>> bcs = model.get_boundary_conditions()
    >>> len(bcs)  # 6 DOFs fixed
    6

Updating Boundary Conditions
----------------------------

Update the prescribed value of an existing boundary condition:

.. doctest::

    >>> model.fix_dof_at([5, 0, 0], DOFIndex.UZ, 0.0)
    >>> # Later, apply a settlement
    >>> _ = model.update_boundary_condition([5, 0, 0], DOFIndex.UZ, -0.005)

Removing Boundary Conditions
----------------------------

Remove specific or all boundary conditions at a node:

.. doctest::

    >>> # Remove a specific DOF constraint
    >>> _ = model.remove_boundary_condition([0, 0, 0], DOFIndex.RZ)
    >>>
    >>> # Remove all constraints at a node
    >>> _ = model.remove_boundary_condition([0, 0, 0])

Managing Point Loads
--------------------

Query, update, and delete point loads:

.. doctest::

    >>> model2 = StructuralModel(name="Point Load Mgmt")
    >>> _ = model2.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model2.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>> _ = model2.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE200", "Steel")
    >>> model2.fix_node_at([0, 0, 0])
    >>>
    >>> # Add point loads
    >>> model2.add_point_load([5, 0, 0], force=[0, 0, -10])
    >>> model2.add_point_load([2.5, 0, 0], force=[0, 0, -5])
    >>>
    >>> # Query point loads
    >>> loads = model2.get_point_loads()
    >>> len(loads)
    2
    >>>
    >>> # Update a point load
    >>> _ = model2.update_point_load(0, force=[0, 0, -15])
    >>>
    >>> # Delete a point load
    >>> _ = model2.delete_point_load(1)

Managing Line Loads
-------------------

Query, update, and delete line loads:

.. doctest::

    >>> model3 = StructuralModel(name="Line Load Mgmt")
    >>> _ = model3.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model3.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>> beam = model3.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE200", "Steel")
    >>> model3.fix_node_at([0, 0, 0])
    >>>
    >>> # Add line load
    >>> model3.add_line_load(beam, [0, 0, -5])
    >>>
    >>> # Query line loads
    >>> loads = model3.get_beam_line_loads()
    >>> len(loads)
    1
    >>>
    >>> # Update line load intensity
    >>> _ = model3.update_line_load(0, w_start=[0, 0, -10])
    >>>
    >>> # Delete line load
    >>> _ = model3.delete_line_load(0)

Managing Load Cases
-------------------

Create, query, and delete load cases:

.. doctest::

    >>> model4 = StructuralModel(name="Load Case Mgmt")
    >>> _ = model4.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model4.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>> _ = model4.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE200", "Steel")
    >>>
    >>> # Create load cases
    >>> lc1 = model4.create_load_case("Dead Load", LoadCaseType.Permanent)
    >>> lc2 = model4.create_load_case("Live Load", LoadCaseType.Variable)
    >>>
    >>> # Query load cases
    >>> all_lcs = model4.get_load_cases()
    >>> lc = model4.get_load_case("Dead Load")
    >>>
    >>> # Delete a load case
    >>> _ = model4.delete_load_case(lc2)

See Also
========

- :doc:`beam_basics` for beam element properties
- :doc:`analysis_workflow` for running the analysis
- :doc:`results_and_postprocessing` for extracting results
