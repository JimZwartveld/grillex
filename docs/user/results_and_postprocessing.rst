==========================
Results and Postprocessing
==========================

This guide covers how to extract and interpret analysis results in Grillex.

Displacement Results
====================

After running an analysis, you can extract nodal displacements.

Point Displacements
-------------------

Get displacement at a specific location:

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>>
    >>> # Create and analyze a simple cantilever
    >>> model = StructuralModel(name="Results Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>> beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>> model.add_point_load([6, 0, 0], force=[0, 0, -10.0])
    >>> _ = model.analyze()
    >>>
    >>> # Get vertical displacement at free end
    >>> uz = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)
    >>> uz < 0  # Negative means downward
    True
    >>>
    >>> # Get rotation about Y-axis
    >>> ry = model.get_displacement_at([6, 0, 0], DOFIndex.RY)

All DOF Displacements
---------------------

Get all displacements at a node:

.. doctest::

    >>> # Get all 6 DOFs at the free end
    >>> all_disp = model.get_displacements_at([6, 0, 0])  # doctest: +SKIP
    >>> len(all_disp) == 6  # doctest: +SKIP
    True

Reaction Forces
===============

Support reactions are automatically computed after analysis.

Getting Reactions
-----------------

.. doctest::

    >>> # Get all reaction forces at the fixed support
    >>> reactions = model.get_reactions_at([0, 0, 0])
    >>> len(reactions) == 6
    True

For a cantilever with vertical tip load, expect:

- Vertical reaction Rz = +P (upward to balance load)
- Moment reaction My = -P×L (to balance moment from load)

.. doctest::

    >>> # Vertical reaction should balance the applied load
    >>> import numpy as np
    >>> rz = reactions[2]  # Reaction in Z direction
    >>> bool(np.isclose(rz, 10.0, rtol=0.01))  # Should equal +10 kN
    True

Internal Forces
===============

Internal forces (axial, shear, moment) can be extracted along beam elements.

End Forces
----------

Get forces at element ends:

.. doctest::

    >>> # Internal actions are available via the beam's underlying elements
    >>> # For simple cases, reactions give the forces at supports

Internal Action Diagrams
------------------------

Grillex can compute internal actions at any point along a beam:

.. code-block:: python

    # Get internal actions at normalized position along beam
    # Position 0.0 = start, 0.5 = middle, 1.0 = end
    actions = beam.get_internal_actions_at(0.5)  # At midspan

Internal Action Components
--------------------------

The internal actions include:

.. list-table:: Internal Action Components
   :header-rows: 1
   :widths: 20 30 50

   * - Component
     - Symbol
     - Description
   * - N
     - Axial Force
     - Force along beam axis (+ = tension)
   * - Vy
     - Shear Y
     - Shear force in local y direction
   * - Vz
     - Shear Z
     - Shear force in local z direction
   * - Mx
     - Torsion
     - Torque about beam axis
   * - My
     - Bending Y
     - Bending moment about local y-axis
   * - Mz
     - Bending Z
     - Bending moment about local z-axis

Result Verification
===================

Analytical Comparison
---------------------

For a cantilever beam with point load P at the tip:

.. doctest::

    >>> # Analytical deflection: delta = P * L^3 / (3 * E * I)
    >>> P = 10.0  # kN
    >>> L = 6.0   # m
    >>> E = 210e6  # kN/m^2
    >>> I = 8.36e-5  # m^4 (Iy for IPE300)
    >>>
    >>> delta_analytical = P * L**3 / (3 * E * I)
    >>>
    >>> # Compare with FEM result
    >>> delta_fem = abs(model.get_displacement_at([6, 0, 0], DOFIndex.UZ))
    >>>
    >>> # Should match within 1%
    >>> abs(delta_fem - delta_analytical) / delta_analytical < 0.01
    True

Result Units
============

All results use the same unit system as inputs:

.. list-table:: Result Units
   :header-rows: 1
   :widths: 30 30 40

   * - Result Type
     - Unit
     - Notes
   * - Displacement
     - meters (m)
     - All translations
   * - Rotation
     - radians (rad)
     - All rotations
   * - Force/Reaction
     - kilonewtons (kN)
     - Point and distributed
   * - Moment
     - kN·m
     - Bending and torsion
   * - Stress
     - kN/m² (kPa)
     - If computed

Exporting Results
=================

Results can be exported to JSON for further processing:

.. code-block:: python

    from grillex.io import export_results_to_json

    export_results_to_json(model, "results.json")

The JSON output includes:
- Nodal displacements
- Support reactions
- Internal forces at check locations

Multi-Load Case Results
=======================

When analyzing multiple load cases:

.. code-block:: python

    # Results can be retrieved per load case
    model.analyze()

    # Get results for specific load case
    disp = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)

Summary Statistics
==================

For design purposes, you may want envelope results:

.. code-block:: python

    # Maximum displacement in the model
    max_disp = max(abs(model.get_displacement_at(pos, DOFIndex.UZ))
                   for pos in model.get_node_positions())

    # Maximum moment (from internal actions)
    # This would use beam.find_component_extrema()

See Also
========

- :doc:`analysis_workflow` for running the analysis
- :doc:`beam_basics` for element formulations
