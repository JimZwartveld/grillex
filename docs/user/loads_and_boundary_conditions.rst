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
    >>> model3.add_point_load([6, 0, 0], DOFIndex.UZ, -10.0)
    >>>
    >>> # Apply 5 kN horizontal force
    >>> model3.add_point_load([6, 0, 0], DOFIndex.UX, 5.0)

Applying Point Moments
----------------------

.. doctest::

    >>> # Apply a 20 kNm moment about Y-axis (causing rotation about Y)
    >>> model3.add_point_load([6, 0, 0], DOFIndex.RY, 20.0)

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
    >>> model4.add_line_load_to_beam(beam, DOFIndex.UZ, -3.0, -7.0)

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
    >>> model5.add_acceleration(0, 0, -9.81)

The self-weight load is automatically computed as:

.. math::

    w = \rho \cdot A \cdot g

For steel IPE300: w = 7.85×10⁻³ × 0.00538 × 9.81 ≈ 0.414 kN/m

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
    >>> model.add_point_load([4, 0, 0], DOFIndex.UZ, -20.0)
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
    >>> beam = model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE400", "Steel")
    >>>
    >>> # Pin at left, roller at right
    >>> model.fix_node_at([0, 0, 0])  # Fixed for simplicity
    >>> model.pin_node_at([10, 0, 0])
    >>>
    >>> # 8 kN/m uniform load
    >>> model.add_line_load(beam, [0, 0, -8.0])
    >>>
    >>> _ = model.analyze()
    >>> # Maximum deflection at midspan
    >>> mid_disp = model.get_displacement_at([5, 0, 0], DOFIndex.UZ)
    >>> mid_disp < 0
    True

See Also
========

- :doc:`beam_basics` for beam element properties
- :doc:`analysis_workflow` for running the analysis
- :doc:`results_and_postprocessing` for extracting results
