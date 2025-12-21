===========
Beam Basics
===========

Grillex provides several beam element formulations for different structural
analysis needs. This guide covers the fundamentals of beam elements.

Beam Element Types
==================

Grillex supports three beam formulations:

.. list-table:: Beam Formulations
   :header-rows: 1
   :widths: 25 15 60

   * - Formulation
     - DOFs
     - Use Case
   * - Euler-Bernoulli
     - 12
     - Slender beams (L/h > 10), neglects shear deformation
   * - Timoshenko
     - 12
     - Deep beams (L/h < 10), includes shear deformation
   * - Warping (14-DOF)
     - 14
     - Thin-walled open sections, includes warping effects

Degree of Freedom Convention
============================

Each node in a beam element has 6 standard degrees of freedom:

.. list-table:: Standard DOFs per Node
   :header-rows: 1
   :widths: 20 30 50

   * - Index
     - DOF
     - Description
   * - 0 (UX)
     - Translation X
     - Displacement along local x-axis (axial)
   * - 1 (UY)
     - Translation Y
     - Displacement along local y-axis
   * - 2 (UZ)
     - Translation Z
     - Displacement along local z-axis
   * - 3 (RX)
     - Rotation X
     - Rotation about local x-axis (torsion)
   * - 4 (RY)
     - Rotation Y
     - Rotation about local y-axis
   * - 5 (RZ)
     - Rotation Z
     - Rotation about local z-axis

For 14-DOF warping elements, an additional warping DOF (φ') is included.

Creating Beams
==============

Basic Beam Creation
-------------------

Create beams using coordinate points:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>>
    >>> model = StructuralModel(name="Beam Creation Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("HEA200", A=0.00538, Iy=3.69e-5, Iz=1.34e-5, J=2.1e-7)
    >>>
    >>> # Create a horizontal beam
    >>> beam1 = model.add_beam_by_coords(
    ...     start=[0, 0, 0],
    ...     end=[5, 0, 0],
    ...     section="HEA200",
    ...     material="Steel"
    ... )
    >>>
    >>> # Check beam properties
    >>> beam1.length  # doctest: +ELLIPSIS
    5.0...

Beams with Roll Angle
---------------------

The roll angle rotates the beam's local y and z axes about the local x-axis:

.. doctest::

    >>> import math
    >>> beam2 = model.add_beam_by_coords(
    ...     start=[0, 0, 0],
    ...     end=[0, 5, 0],
    ...     section="HEA200",
    ...     material="Steel",
    ...     roll_angle=math.pi/4  # 45 degrees
    ... )

Beam Subdivision
----------------

For more accurate results, beams can be subdivided into multiple elements:

.. doctest::

    >>> beam3 = model.add_beam_by_coords(
    ...     start=[0, 0, 0],
    ...     end=[10, 0, 0],
    ...     section="HEA200",
    ...     material="Steel",
    ...     num_elements=4  # Creates 4 elements with 3 internal nodes
    ... )

Materials and Sections
======================

Material Properties
-------------------

Materials define the elastic properties of the beam:

.. doctest::

    >>> model2 = StructuralModel(name="Material Example")
    >>>
    >>> # Add steel with all properties
    >>> _ = model2.add_material(
    ...     name="S355",
    ...     E=210e6,      # Young's modulus in kN/m²
    ...     nu=0.3,       # Poisson's ratio
    ...     rho=7.85e-3   # Density in mT/m³
    ... )

The shear modulus G is automatically computed as:

.. math::

    G = \frac{E}{2(1 + \nu)}

Section Properties
------------------

Sections define the geometric properties:

.. doctest::

    >>> _ = model2.add_section(
    ...     name="IPE300",
    ...     A=0.00538,     # Cross-sectional area in m²
    ...     Iy=8.36e-5,    # Moment of inertia about y-axis in m⁴
    ...     Iz=6.04e-6,    # Moment of inertia about z-axis in m⁴
    ...     J=2.01e-7      # Torsional constant in m⁴
    ... )

For warping analysis, include the warping constant:

.. doctest::

    >>> _ = model2.add_section(
    ...     name="IPE300_warping",
    ...     A=0.00538,
    ...     Iy=8.36e-5,
    ...     Iz=6.04e-6,
    ...     J=2.01e-7,
    ...     Iw=1.26e-7    # Warping constant in m⁶
    ... )

End Releases
============

End releases allow specific DOFs to be released at beam ends, creating
hinges or rollers:

Moment Releases (Hinges)
------------------------

A simply supported beam has moment releases at both ends:

.. doctest::

    >>> model3 = StructuralModel(name="Simply Supported Beam")
    >>> _ = model3.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model3.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>>
    >>> # Create beam - note: end releases are configured via beam properties
    >>> beam = model3.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE200", "Steel")

Sign Conventions
================

Internal Forces
---------------

Grillex uses the following sign conventions for internal forces:

- **Axial force (N)**: Positive in tension
- **Shear force (Vy, Vz)**: Positive in positive local y/z direction
- **Bending moment (My, Mz)**: Positive per right-hand rule
- **Torsion (Mx)**: Positive per right-hand rule about x-axis

Displacements and Rotations
---------------------------

- **Translations**: Positive in positive axis direction
- **Rotations**: Positive per right-hand rule (counter-clockwise when
  looking along positive axis direction)

See Also
========

- :doc:`coordinate_systems` for detailed coordinate system explanation
- :doc:`loads_and_boundary_conditions` for applying loads to beams
