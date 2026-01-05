==================
Coordinate Systems
==================

Understanding coordinate systems is essential for correctly modeling
structures in Grillex. This guide explains both global and local
coordinate systems used in the software.

Global Coordinate System
========================

Grillex uses a right-handed Cartesian coordinate system for the global
reference frame:

- **X-axis**: Typically horizontal (longitudinal direction)
- **Y-axis**: Typically horizontal (transverse direction)
- **Z-axis**: Vertical (positive upward)

.. note::

    The global Z-axis is typically aligned with the direction opposite
    to gravity for structural analysis.

Local Coordinate System
=======================

Each beam element has its own local coordinate system, which is essential
for understanding internal forces and applying loads.

Local Axis Definitions
----------------------

For a beam element connecting node i to node j:

- **Local x-axis**: Along the beam axis, from node i to node j
- **Local y-axis**: Perpendicular to x, typically horizontal (principal axis)
- **Local z-axis**: Perpendicular to both x and y (right-hand rule)

Default Orientation
-------------------

For beams without roll angle, the local axes are determined as follows:

**Horizontal beams** (beam axis not parallel to global Z):

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> import numpy as np
    >>>
    >>> model = StructuralModel(name="Coordinate Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)
    >>>
    >>> # Horizontal beam along global X
    >>> beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
    >>>
    >>> # For this beam:
    >>> # - Local x points along global X (from start to end)
    >>> # - Local z points along global Z (up)
    >>> # - Local y = z × x (right-hand rule)

For a horizontal beam along global X:
- Local x = [1, 0, 0] (along beam)
- Local z = [0, 0, 1] (global Z, upward)
- Local y = [0, 1, 0] (global Y)

**Vertical beams** (beam axis parallel to global Z):

For vertical beams, a special convention is used to avoid ambiguity:
- Local z aligns with global X
- Local y aligns with global Y

Roll Angle
----------

The roll angle (ψ) rotates the local y and z axes about the local x-axis:

.. doctest::

    >>> import math
    >>>
    >>> # Beam with 45° roll angle
    >>> beam_rolled = model.add_beam_by_coords(
    ...     [0, 0, 0], [6, 0, 0], "Test", "Steel",
    ...     roll_angle=math.pi/4  # 45 degrees in radians
    ... )

The transformation is:

.. math::

    \begin{bmatrix} y' \\ z' \end{bmatrix} =
    \begin{bmatrix} \cos\psi & \sin\psi \\ -\sin\psi & \cos\psi \end{bmatrix}
    \begin{bmatrix} y \\ z \end{bmatrix}

Where y' and z' are the rotated local axes.

Coordinate Transformation
=========================

When loads are applied in global coordinates, they are transformed to
local coordinates for element calculations.

Force Transformation
--------------------

Global forces are transformed to local forces using the rotation matrix R:

.. math::

    \mathbf{F}_{local} = \mathbf{R}^T \mathbf{F}_{global}

Where R is the 3×3 rotation matrix formed by the local axis unit vectors.

Stiffness Transformation
------------------------

The element stiffness matrix is transformed from local to global coordinates:

.. math::

    \mathbf{K}_{global} = \mathbf{T}^T \mathbf{K}_{local} \mathbf{T}

Where T is the full transformation matrix (12×12 for standard elements,
14×14 for warping elements).

Practical Examples
==================

Example 1: Horizontal Beam
--------------------------

.. doctest::

    >>> model = StructuralModel(name="Horizontal Beam")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> # Beam along X-axis
    >>> beam = model.add_beam_by_coords([0, 0, 0], [10, 0, 0], "IPE300", "Steel")
    >>>
    >>> # For vertical loading (gravity), use UZ (local z = global Z for this beam)
    >>> model.fix_node_at([0, 0, 0])
    >>> model.add_point_load([10, 0, 0], force=[0, 0, -10.0])  # Force in -Z direction
    >>> _ = model.analyze()
    >>>
    >>> # Vertical displacement is negative (downward)
    >>> disp = model.get_displacement_at([10, 0, 0], 2)  # UZ
    >>> disp < 0
    True

Example 2: Inclined Beam
------------------------

.. doctest::

    >>> model2 = StructuralModel(name="Inclined Beam")
    >>> _ = model2.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model2.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>>
    >>> # Beam from origin to (3, 0, 4) - 3-4-5 triangle
    >>> beam = model2.add_beam_by_coords([0, 0, 0], [3, 0, 4], "IPE200", "Steel")
    >>> beam.length  # doctest: +ELLIPSIS
    5.0...

Example 3: Vertical Column
--------------------------

.. doctest::

    >>> model3 = StructuralModel(name="Vertical Column")
    >>> _ = model3.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model3.add_section("HEB200", A=0.00781, Iy=5.70e-5, Iz=2.00e-5, J=5.9e-7)
    >>>
    >>> # Vertical beam (column)
    >>> column = model3.add_beam_by_coords([0, 0, 0], [0, 0, 5], "HEB200", "Steel")
    >>>
    >>> # For a vertical beam:
    >>> # - Local x points upward (along global Z)
    >>> # - Local z points along global X (by convention)
    >>> # - Local y points along global Y

Summary
=======

.. list-table:: Coordinate System Quick Reference
   :header-rows: 1
   :widths: 30 70

   * - Beam Orientation
     - Local Axis Convention
   * - Horizontal (X-direction)
     - x→X, z→Z(up), y→Y
   * - Horizontal (Y-direction)
     - x→Y, z→Z(up), y→-X
   * - Vertical (Z-direction)
     - x→Z, z→X, y→Y
   * - With roll angle ψ
     - y and z rotated by ψ about x

See Also
========

- :doc:`beam_basics` for beam element fundamentals
- :doc:`loads_and_boundary_conditions` for applying loads in correct coordinates
