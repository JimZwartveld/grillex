Plate Meshing
=============

Grillex supports plate elements for modeling deck plates, bulkheads,
and other planar structures. This guide covers the plate meshing API.

Overview
--------

Plate meshing in Grillex follows a two-step process:

1. **Define plate geometry** - Specify corners, thickness, and material
2. **Generate mesh** - Call ``mesh()`` to create finite elements

The meshing uses Gmsh for mesh generation, supporting both quadrilateral
and triangular elements with optional higher-order formulations.

Basic Plate Example
-------------------

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Plate Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

    # Add a 4m x 2m horizontal plate
    >>> plate = model.add_plate(
    ...     corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
    ...     thickness=0.02,
    ...     material="Steel",
    ...     mesh_size=1.0
    ... )
    >>> plate.name
    'Plate_1'

Mesh Control
------------

You can control the mesh density using ``mesh_size`` or by specifying
the number of elements along each edge:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Mesh Control Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> plate = model.add_plate(
    ...     corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
    ...     thickness=0.02,
    ...     material="Steel",
    ...     mesh_size=0.5
    ... )

    # Set specific edge divisions (overrides mesh_size for that edge)
    >>> model.set_edge_divisions(plate, edge_index=0, n_elements=8)  # Bottom edge
    >>> model.set_edge_divisions(plate, edge_index=2, n_elements=8)  # Top edge

Edge indices are 0-based and follow the corner order:

- Edge 0: from corner[0] to corner[1]
- Edge 1: from corner[1] to corner[2]
- Edge 2: from corner[2] to corner[3]
- Edge 3: from corner[3] to corner[0]

Element Types
-------------

Grillex supports multiple plate element formulations:

+----------+-------+----------------------------------+
| Type     | Nodes | Description                      |
+==========+=======+==================================+
| MITC4    | 4     | Mixed Interpolation, 4-node quad |
+----------+-------+----------------------------------+
| MITC8    | 8     | Serendipity 8-node quad          |
+----------+-------+----------------------------------+
| MITC9    | 9     | Lagrangian 9-node quad           |
+----------+-------+----------------------------------+
| DKT      | 3     | Discrete Kirchhoff Triangle      |
+----------+-------+----------------------------------+

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Element Type Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

    # Use 8-node serendipity elements
    >>> plate = model.add_plate(
    ...     corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
    ...     thickness=0.015,
    ...     material="Steel",
    ...     element_type="MITC8"
    ... )
    >>> plate.element_type
    'MITC8'

Boundary Conditions
-------------------

Add supports along plate edges using ``add_support_curve()``:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Support Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> plate = model.add_plate(
    ...     corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
    ...     thickness=0.02,
    ...     material="Steel"
    ... )

    # Simply support edge 0 (restrain vertical displacement)
    >>> _ = model.add_support_curve(plate, edge_index=0, uz=True)

    # Fix edge 2 completely
    >>> _ = model.add_support_curve(plate, edge_index=2, ux=True, uy=True, uz=True)

Available restraint options:

- ``ux``, ``uy``, ``uz``: Translation restraints
- ``rotation_about_edge``: Rotation about the edge direction

Plate-Beam Coupling
-------------------

Plates can be coupled to beams using rigid links. This is useful for
modeling stiffened plates or deck structures:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Coupling Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.98e-8)

    # Create plate
    >>> plate = model.add_plate(
    ...     corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
    ...     thickness=0.012,
    ...     material="Steel"
    ... )

    # Create stiffener beam along plate edge
    >>> beam = model.add_beam_by_coords([0, 0, -0.1], [4, 0, -0.1], "IPE200", "Steel")

    # Couple plate edge 0 to beam with offset
    >>> _ = model.couple_plate_to_beam(
    ...     plate, edge_index=0, beam=beam,
    ...     offset=[0, 0, -0.1]  # Plate is 100mm above beam centroid
    ... )

Optional moment releases can be specified:

.. code-block:: python

    # Allow rotation about the edge (hinged connection)
    model.couple_plate_to_beam(
        plate, edge_index=0, beam=beam,
        releases={"R_EDGE": True}
    )

Generating the Mesh
-------------------

After defining plates and their constraints, call ``mesh()`` to generate
the finite element mesh:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Mesh Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> plate = model.add_plate(
    ...     corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
    ...     thickness=0.02,
    ...     material="Steel",
    ...     mesh_size=1.0
    ... )
    >>> _ = model.add_support_curve(plate, edge_index=0, uz=True)
    >>> _ = model.add_support_curve(plate, edge_index=3, uz=True)

    # Generate mesh
    >>> stats = model.mesh()
    >>> stats.n_plate_elements >= 4
    True

The returned ``MeshStatistics`` object contains:

- ``n_plate_nodes``: Number of nodes created
- ``n_plate_elements``: Total plate elements
- ``n_quad_elements``: Quadrilateral elements
- ``n_tri_elements``: Triangular elements
- ``n_support_dofs``: DOFs restrained by support curves
- ``n_rigid_links``: Rigid links from plate-beam coupling

Querying Results
----------------

After analysis, you can query plate element results:

**Displacements** at any point within an element:

.. code-block:: python

    # Get displacement at element center (xi=0, eta=0)
    element = model.get_plate_elements()[0]
    disp = model.get_plate_displacement(element, xi=0.0, eta=0.0)
    print(f"Vertical displacement: {disp['UZ']:.4f} m")

**Bending moments** per unit width:

.. code-block:: python

    moments = model.get_plate_moments(element, xi=0.0, eta=0.0)
    print(f"Mx = {moments['Mx']:.2f} kN.m/m")
    print(f"My = {moments['My']:.2f} kN.m/m")
    print(f"Mxy = {moments['Mxy']:.2f} kN.m/m")

**Stresses** at top, middle, or bottom surface:

.. code-block:: python

    stress_top = model.get_plate_stress(element, surface="top", xi=0.0, eta=0.0)
    stress_bot = model.get_plate_stress(element, surface="bottom", xi=0.0, eta=0.0)

    print(f"Top surface stress: sigma_x = {stress_top['sigma_x']:.1f} kN/m^2")

Natural coordinates (xi, eta) range from -1 to +1 for quadrilateral elements
and use area coordinates for triangles.

Units
-----

All plate-related quantities use consistent units:

+-------------------+--------+
| Quantity          | Unit   |
+===================+========+
| Coordinates       | m      |
+-------------------+--------+
| Thickness         | m      |
+-------------------+--------+
| Mesh size         | m      |
+-------------------+--------+
| Displacement      | m      |
+-------------------+--------+
| Rotation          | rad    |
+-------------------+--------+
| Moments (Mx, My)  | kN.m/m |
+-------------------+--------+
| Stress            | kN/m^2 |
+-------------------+--------+

Troubleshooting
---------------

**Singular matrix errors:**
  Plate elements only have bending stiffness (UZ, RX, RY). In-plane DOFs
  (UX, UY, RZ) must be restrained. Use ``add_support_curve()`` or fix
  at least one node's in-plane DOFs.

**Mesh generation fails:**
  - Ensure all corner points are coplanar
  - Check that the polygon is convex or simple
  - Try a coarser ``mesh_size``
  - Verify gmsh is installed: ``pip install gmsh``

**Results seem wrong:**
  - Check boundary conditions are correctly applied
  - Verify material properties (E should be in kN/m^2)
  - Ensure mesh is sufficiently refined for the load pattern

Beam to Plate Conversion
------------------------

Grillex can convert beam elements into plate representations. This is useful
for:

- Converting simplified beam models to detailed plate models
- Modeling wide flanges or deck plates from beam representations
- Creating plate representations of I-beam webs for stress analysis

Basic Conversion
~~~~~~~~~~~~~~~~

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Beam to Plate")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("Flat", A=0.06, Iy=1e-4, Iz=1e-4, J=1e-4)

    # Create a beam
    >>> beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Flat", "Steel")

    # Convert to a horizontal plate
    >>> plate = model.convert_beam_to_plate(
    ...     beam=beam,
    ...     width=0.5,
    ...     thickness=0.02,
    ...     orientation="horizontal"
    ... )
    >>> plate.n_corners
    4
    >>> round(plate.get_area(), 1)
    3.0

Orientation Options
~~~~~~~~~~~~~~~~~~~

The ``orientation`` parameter controls how the plate is oriented relative
to the beam axis:

- ``"horizontal"``: Plate normal points up (global +Z). For horizontal beams,
  the plate lies in the XY plane.
- ``"vertical"``: Plate normal is perpendicular to beam axis and global Z.
  Creates a vertical web-like plate.
- ``"top"``: Like horizontal, but offset up by width/2.
- ``"bottom"``: Like horizontal, but offset down by width/2.

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Orientations")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("Flat", A=0.06, Iy=1e-4, Iz=1e-4, J=1e-4)
    >>> beam = model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")

    # Vertical plate (like a web)
    >>> web = model.convert_beam_to_plate(beam, width=0.3, thickness=0.01,
    ...     orientation="vertical")

    # Top flange (offset up)
    >>> top_flange = model.convert_beam_to_plate(beam, width=0.15, thickness=0.02,
    ...     orientation="top")

Batch Conversion
~~~~~~~~~~~~~~~~

Convert multiple beams at once using ``convert_beams_to_plates()``:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Batch Conversion")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("Flat", A=0.06, Iy=1e-4, Iz=1e-4, J=1e-4)

    # Create multiple beams
    >>> _ = model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")
    >>> _ = model.add_beam_by_coords([0, 2, 0], [4, 2, 0], "Flat", "Steel")

    # Convert all beams with fixed width
    >>> plates = model.convert_beams_to_plates(width=0.5, thickness=0.02)
    >>> len(plates)
    2

    # Or use a function to compute width from section properties
    >>> plates = model.convert_beams_to_plates(
    ...     width_function=lambda b: b.section.A / 0.02,
    ...     thickness=0.02
    ... )

Automatic Thickness
~~~~~~~~~~~~~~~~~~~

If ``thickness`` is not specified, it's estimated from the beam's section
area divided by the plate width:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> model = StructuralModel(name="Auto Thickness")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("Flat", A=0.06, Iy=1e-4, Iz=1e-4, J=1e-4)
    >>> beam = model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")

    # thickness = A / width = 0.06 / 0.3 = 0.2
    >>> plate = model.convert_beam_to_plate(beam, width=0.3)
    >>> plate.thickness
    0.2
