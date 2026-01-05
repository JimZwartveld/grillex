===============
Getting Started
===============

This guide will help you get started with Grillex 2.0 for structural analysis.

Installation
============

Install Grillex using pip:

.. code-block:: bash

    pip install grillex

For development installation:

.. code-block:: bash

    git clone https://github.com/JimZwartveld/grillex.git
    cd grillex
    pip install -e .[testing]

Quick Start Example
===================

Here's a simple example of analyzing a cantilever beam:

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>>
    >>> # Create a new structural model
    >>> model = StructuralModel(name="Cantilever Example")
    >>>
    >>> # Add a steel material (E in kN/m², nu, rho in mT/m³)
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>>
    >>> # Add an IPE300 section (A in m², I in m⁴)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> # Create a 6m beam along the X-axis
    >>> beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>>
    >>> # Fix the left end (cantilever support)
    >>> model.fix_node_at([0, 0, 0])
    >>>
    >>> # Apply a 10 kN downward point load at the free end
    >>> model.add_point_load([6, 0, 0], force=[0, 0, -10.0])
    >>>
    >>> # Run the analysis
    >>> _ = model.analyze()
    >>>
    >>> # Get the vertical displacement at the free end (should be negative)
    >>> disp = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)
    >>> disp < 0  # Beam deflects downward
    True

Understanding the Result
------------------------

For a cantilever beam with a point load at the free end, the analytical
deflection is:

.. math::

    \delta = \frac{PL^3}{3EI}

Where:
- P = 10 kN (applied load)
- L = 6 m (beam length)
- E = 210×10⁶ kN/m² (Young's modulus)
- I = 8.36×10⁻⁵ m⁴ (moment of inertia about y-axis for bending about z)

Units Convention
================

Grillex uses a consistent SI-derived unit system:

.. list-table:: Standard Units
   :header-rows: 1
   :widths: 30 30 20

   * - Quantity
     - Unit
     - Symbol
   * - Length
     - meters
     - m
   * - Force
     - kilonewtons
     - kN
   * - Mass
     - metric tonnes
     - mT
   * - Stress/Modulus
     - kN/m² (kPa)
     - kN/m²
   * - Moment
     - kN·m
     - kNm
   * - Distributed load
     - kN/m
     - kN/m
   * - Acceleration
     - m/s²
     - m/s²

.. note::

    Always use these units when defining your model. For example:

    - Steel's Young's modulus: E = 210×10⁶ kN/m² (not 210 GPa)
    - Steel density: ρ = 7.85×10⁻³ mT/m³ (not 7850 kg/m³)

Next Steps
==========

- Learn about :doc:`beam_basics` to understand beam element formulations
- Understand :doc:`coordinate_systems` for proper model orientation
- See :doc:`loads_and_boundary_conditions` for applying loads
- Review :doc:`analysis_workflow` for the complete analysis process
