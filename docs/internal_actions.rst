=====================================
Internal Actions Along Beam Elements
=====================================

This chapter describes the analytical computation of internal actions (axial force, shear force, bending moment, and torsion) and kinematic quantities (displacements and rotations) along beam elements. The approach uses closed-form solutions derived from differential equations, which provides exact results for polynomial distributed loads and properly accounts for element end releases.

Overview
========

Internal actions and displacements at any position along a beam element are computed using analytical formulas derived from the governing differential equations of beam theory. This approach offers several advantages over simple linear interpolation between end values:

- **Accuracy**: Exact for polynomial distributed loads (uniform, triangular, trapezoidal)
- **Release handling**: Proper boundary conditions for hinges, rollers, and other releases
- **Beam theory support**: Both Euler-Bernoulli (slender beams) and Timoshenko (short, deep beams) theories
- **Extrema detection**: Analytical identification of maximum/minimum values

The following sections present the governing differential equations and their analytical solutions for each component.

Notation and Sign Conventions
=============================

Coordinate System
-----------------

Each beam element uses a local coordinate system:

- :math:`x`: Axial direction along the beam, from node :math:`i` (start) to node :math:`j` (end)
- :math:`y`: Transverse direction (strong axis bending plane)
- :math:`z`: Transverse direction (weak axis bending plane)

The element length is denoted :math:`L`, with positions measured as :math:`x \in [0, L]`.

Kinematic Variables
-------------------

- :math:`u`: Axial displacement
- :math:`w`: Transverse displacement (deflection)
- :math:`\phi`: Rotation (slope for Euler-Bernoulli, bending rotation for Timoshenko)
- :math:`\theta`: Twist rotation (torsion)

Subscripts :math:`1` and :math:`2` denote values at the start (:math:`x=0`) and end (:math:`x=L`) of the element, respectively.

Loading
-------

Distributed loads are represented as trapezoidal (linearly varying) functions:

.. math::

   q(x) = q_1 + \frac{q_2 - q_1}{L} x

where :math:`q_1` is the load intensity at :math:`x=0` and :math:`q_2` is the load intensity at :math:`x=L`. Uniform loads are the special case where :math:`q_1 = q_2`.

Material and Section Properties
-------------------------------

- :math:`E`: Modulus of elasticity
- :math:`G`: Shear modulus
- :math:`A`: Cross-sectional area
- :math:`I`: Second moment of area (moment of inertia)
- :math:`J`: Torsional constant
- :math:`k`: Shear correction factor (Timoshenko theory)
- :math:`EA`: Axial stiffness
- :math:`EI`: Bending stiffness
- :math:`GJ`: Torsional stiffness
- :math:`kAG`: Shear stiffness (Timoshenko theory)

Sign Conventions
----------------

Internal actions follow standard structural engineering conventions:

- **Axial force** :math:`N`: Positive in tension
- **Shear force** :math:`V`: Positive according to right-hand rule about local axes
- **Bending moment** :math:`M`: Positive according to right-hand rule about local axes
- **Torsion** :math:`M_x`: Positive according to right-hand rule about local :math:`x`-axis


Axial Behavior
==============

Governing Differential Equation
-------------------------------

The axial equilibrium of a beam element with distributed axial load :math:`q_x(x)` is governed by:

.. math::

   \frac{dN}{dx} + q_x = 0

The constitutive relation for linear elastic material is:

.. math::

   N = EA \frac{du}{dx}

Combining these yields the second-order differential equation for axial displacement:

.. math::

   EA \frac{d^2 u}{dx^2} + q_x = 0

Boundary Conditions
-------------------

The axial degree of freedom at each end can be either fixed (displacement prescribed) or free (force prescribed). This gives four possible combinations:

.. list-table:: Axial Release Combinations
   :header-rows: 1
   :widths: 30 70

   * - Combination
     - Description
   * - Fixed-Fixed
     - Both ends restrained axially
   * - Fixed-Free
     - Start fixed, end free (cantilever-type)
   * - Free-Fixed
     - Start free, end fixed
   * - Free-Free
     - Both ends free (rigid body motion)

Axial Displacement Solutions
----------------------------

**Fixed-Fixed** (both ends restrained):

.. math::

   u(x) = \frac{EA L u_1 - \frac{L q_1 x^2}{2} + x \left( 6EA(-u_1 + u_2) + L^2(2q_1 + q_2) + x^2(q_1 - q_2) \right) / 6}{EA L}

**Fixed-Free** (start fixed, end free):

.. math::

   u(x) = \frac{EA L u_1 + \frac{Lx(L(q_1 + q_2) - q_1 x)}{2} + \frac{x^3(q_1 - q_2)}{6}}{EA L}

**Free-Fixed** (start free, end fixed):

.. math::

   u(x) = \frac{L(6EA u_2 + 2L^2 q_1 + L^2 q_2 - 3q_1 x^2) + x^3(q_1 - q_2)}{6 EA L}

Axial Force Solutions
---------------------

**Fixed-Fixed** (both ends restrained):

.. math::

   N(x) = \frac{6EA(-u_1 + u_2) + L(2Lq_1 + Lq_2 - 6q_1 x) + 3x^2(q_1 - q_2)}{6L}

**Fixed-Free** (start fixed, end free):

.. math::

   N(x) = \frac{L \bigl( L(q_1 + q_2) - 2q_1 x \bigr) + x^2(q_1 - q_2)}{2L}

**Free-Fixed** (start free, end fixed):

.. math::

   N(x) = \frac{x \bigl( -2Lq_1 + x(q_1 - q_2) \bigr)}{2L}

**Free-Free** (both ends free):

.. math::

   N(x) = 0

The free-free case represents rigid body motion with no internal axial force.


Torsion
=======

Governing Differential Equation
-------------------------------

For uniform (St. Venant) torsion without distributed torque, the equilibrium equation is:

.. math::

   \frac{dM_x}{dx} = 0

The constitutive relation is:

.. math::

   M_x = GJ \frac{d\theta}{dx}

where :math:`\theta` is the twist rotation.

Boundary Conditions
-------------------

Similar to axial behavior, torsion has four combinations based on whether twist rotation is fixed or free at each end.

Twist Rotation Solutions
------------------------

**Fixed-Fixed** (both ends restrained against twist):

.. math::

   \theta(x) = \frac{L \theta_1 - x(\theta_1 - \theta_2)}{L}

This is a linear interpolation between the end values.

**Fixed-Free** (start fixed, end free):

.. math::

   \theta(x) = \theta_1

The twist angle is constant along the element, equal to the fixed end value.

**Free-Fixed** (start free, end fixed):

.. math::

   \theta(x) = \theta_2

The twist angle is constant along the element, equal to the fixed end value.

Torsion Moment Solutions
------------------------

**Fixed-Fixed** (both ends restrained against twist):

.. math::

   M_x(x) = -\frac{GJ(\theta_1 - \theta_2)}{L}

The torsion is constant along the element when both ends are restrained.

**Fixed-Free** and **Free-Fixed**:

.. math::

   M_x(x) = 0

When one end is free to twist, the torsion moment is zero throughout the element (assuming no distributed torque).


Bending: Euler-Bernoulli Theory
===============================

Euler-Bernoulli beam theory is applicable to slender beams where shear deformation is negligible (typically :math:`L/h > 20` where :math:`h` is the section depth).

Governing Differential Equations
--------------------------------

The equilibrium equations for bending in the :math:`x`-:math:`y` plane are:

.. math::

   \frac{dV_y}{dx} + q_y = 0

.. math::

   \frac{dM_z}{dx} - V_y = 0

The Euler-Bernoulli constitutive relation assumes plane sections remain plane and perpendicular to the neutral axis:

.. math::

   M_z = EI_z \frac{d^2 w}{dx^2}

where :math:`w` is the transverse deflection and :math:`\phi = dw/dx` is the slope.

Combining these equations yields the fourth-order differential equation:

.. math::

   EI_z \frac{d^4 w}{dx^4} = q_y

Boundary Conditions
-------------------

For bending, each end has two degrees of freedom: transverse displacement :math:`w` and rotation :math:`\phi`. Each can be either fixed or free, giving :math:`2^4 = 16` possible combinations.

The boundary condition naming convention is :math:`w_1`-:math:`\phi_1`-:math:`w_2`-:math:`\phi_2`, where each term indicates whether that degree of freedom is fixed or free.

.. list-table:: Common Bending Release Combinations
   :header-rows: 1
   :widths: 35 65

   * - Combination
     - Structural Condition
   * - Fixed-Fixed-Fixed-Fixed
     - Fully fixed at both ends
   * - Fixed-Fixed-Free-Free
     - Cantilever (fixed at start, free at end)
   * - Free-Free-Fixed-Fixed
     - Cantilever (free at start, fixed at end)
   * - Fixed-Free-Fixed-Free
     - Simply supported (pinned-pinned)
   * - Fixed-Fixed-Fixed-Free
     - Propped cantilever (fixed at start, pinned at end)
   * - Fixed-Free-Fixed-Fixed
     - Propped cantilever (pinned at start, fixed at end)

Deflection Solutions (Euler-Bernoulli)
--------------------------------------

**Fixed-Fixed-Fixed-Fixed** (fully fixed both ends):

.. math::

   w(x) = \frac{1}{120 EI L^3} \Bigl[ 120 EI L^3 (\phi_1 x + w_1) + 5L^3 q_1 x^4 + L^2 x^5(-q_1 + q_2) \\
   + Lx^2 \bigl( -120 EI L (2\phi_1 + \phi_2) - 360 EI (w_1 - w_2) + L^4(3q_1 + 2q_2) \bigr) \\
   + x^3 \bigl( 120 EI L (\phi_1 + \phi_2) + 240 EI (w_1 - w_2) - L^4(7q_1 + 3q_2) \bigr) \Bigr]

**Fixed-Fixed-Free-Fixed**:

.. math::

   w(x) = \frac{1}{240 EI L} \Bigl[ 240 EI L (\phi_1 x + w_1) + 10 L x^3 (-2L(q_1 + q_2) + q_1 x) \\
   + x^2 \bigl( -120 EI \phi_1 + 120 EI \phi_2 + 15 L^3 q_1 + 25 L^3 q_2 + 2x^3(-q_1 + q_2) \bigr) \Bigr]

**Fixed-Fixed-Fixed-Free** (propped cantilever):

.. math::

   w(x) = \frac{1}{240 EI L^3} \Bigl[ 240 EI L^3 (\phi_1 x + w_1) + 10 L^3 q_1 x^4 + 2L^2 x^5 (-q_1 + q_2) \\
   + Lx^2 \bigl( -360 EI L \phi_1 - 360 EI w_1 + 360 EI w_2 + 8L^4 q_1 + 7L^4 q_2 \bigr) \\
   + x^3 \bigl( 120 EI L \phi_1 - 120 EI(-w_1 + w_2) - L^4(16q_1 + 9q_2) \bigr) \Bigr]

**Free-Fixed-Fixed-Fixed**:

.. math::

   w(x) = \frac{1}{240 EI L} \Bigl[ 120 EI L (-L\phi_1 - L\phi_2 + 2\phi_1 x + 2w_2) \\
   + L(7L^4 q_1 + 3L^4 q_2 + 10 q_1 x^4) \\
   + x^2 \bigl( -120 EI \phi_1 + 120 EI \phi_2 - 15 L^3 q_1 - 5L^3 q_2 + 2x^3(-q_1 + q_2) \bigr) \Bigr]

**Fixed-Free-Fixed-Fixed** (propped cantilever):

.. math::

   w(x) = \frac{1}{240 EI L^3} \Bigl[ 240 EI L^3 w_1 + 10 L^3 q_1 x^4 \\
   + L^2 x \bigl( -120 EI L \phi_2 + 360 EI(-w_1 + w_2) + L^4(3q_1 + 2q_2) + 2x^4(-q_1 + q_2) \bigr) \\
   - x^3 \bigl( -120 EI L \phi_2 + 120 EI(-w_1 + w_2) + L^4(11q_1 + 4q_2) \bigr) \Bigr]

**Fixed-Fixed-Free-Free** (cantilever: fixed at start, free at end):

.. math::

   w(x) = \frac{EI L (\phi_1 x + w_1) + \frac{Lx^2(L^2(2q_1 + 4q_2) - 2Lx(q_1 + q_2) + q_1 x^2)}{24} + \frac{x^5(-q_1 + q_2)}{120}}{EI L}

**Free-Free-Fixed-Fixed** (cantilever: free at start, fixed at end):

.. math::

   w(x) = \frac{120 EI L (-L\phi_2 + w_2) + L(11L^4 q_1 + 4L^4 q_2 + 5q_1 x^4 - 5x(-24 EI \phi_2 + 3L^3 q_1 + L^3 q_2)) - x^5(q_1 - q_2)}{120 EI L}

**Fixed-Free-Fixed-Free** (simply supported, pinned-pinned):

.. math::

   w(x) = \frac{360 EI L w_1 + 5Lx^3(-2L(2q_1 + q_2) + 3q_1 x) + x(-360 EI w_1 + 360 EI w_2 + 8L^4 q_1 + 7L^4 q_2 + 3x^4(-q_1 + q_2))}{360 EI L}

Rotation Solutions (Euler-Bernoulli)
------------------------------------

**Fixed-Fixed-Fixed-Fixed** (fully fixed both ends):

.. math::

   \phi(x) = \frac{1}{120 EI L^3} \Bigl[ 120 EI L^3 \phi_1 + 20 L^3 q_1 x^3 + 5L^2 x^4(-q_1 + q_2) \\
   - 2Lx \bigl( 120 EI L(2\phi_1 + \phi_2) + 360 EI(w_1 - w_2) - L^4(3q_1 + 2q_2) \bigr) \\
   + 3x^2 \bigl( 120 EI L(\phi_1 + \phi_2) + 240 EI(w_1 - w_2) - L^4(7q_1 + 3q_2) \bigr) \Bigr]

**Fixed-Fixed-Free-Fixed**:

.. math::

   \phi(x) = \frac{EI L \phi_1 + \frac{Lx^2(-3L(q_1 + q_2) + 2q_1 x)}{12} + \frac{x(-24 EI \phi_1 + 24 EI \phi_2 + 3L^3 q_1 + 5L^3 q_2 + x^3(-q_1 + q_2))}{24}}{EI L}

**Fixed-Fixed-Fixed-Free** (propped cantilever):

.. math::

   \phi(x) = \frac{1}{240 EI L^3} \Bigl[ 240 EI L^3 \phi_1 + 40 L^3 q_1 x^3 + 10 L^2 x^4(-q_1 + q_2) \\
   + 2Lx \bigl( -360 EI L \phi_1 - 360 EI w_1 + 360 EI w_2 + 8L^4 q_1 + 7L^4 q_2 \bigr) \\
   + 3x^2 \bigl( 120 EI L \phi_1 - 120 EI(-w_1 + w_2) - L^4(16q_1 + 9q_2) \bigr) \Bigr]

**Free-Fixed-Fixed-Fixed**:

.. math::

   \phi(x) = \frac{EI L \phi_1 + \frac{Lq_1 x^3}{6} + \frac{x(-24 EI \phi_1 + 24 EI \phi_2 - 3L^3 q_1 - L^3 q_2 + x^3(-q_1 + q_2))}{24}}{EI L}

**Fixed-Free-Fixed-Fixed** (propped cantilever):

.. math::

   \phi(x) = \frac{1}{240 EI L^3} \Bigl[ -120 EI L^3 \phi_2 + 360 EI L^2(-w_1 + w_2) \\
   + L^3(3L^3 q_1 + 2L^3 q_2 + 40 q_1 x^3) + 10 L^2 x^4(-q_1 + q_2) \\
   + 3x^2 \bigl( 120 EI L \phi_2 - 120 EI(-w_1 + w_2) - L^4(11q_1 + 4q_2) \bigr) \Bigr]

**Fixed-Fixed-Free-Free** (cantilever: fixed at start, free at end):

.. math::

   \phi(x) = \frac{EI L \phi_1 + \frac{Lx(L^2(2q_1 + 4q_2) - 3Lx(q_1 + q_2) + 2q_1 x^2)}{12} + \frac{x^4(-q_1 + q_2)}{24}}{EI L}

**Free-Free-Fixed-Fixed** (cantilever: free at start, fixed at end):

.. math::

   \phi(x) = \frac{L(24 EI \phi_2 - 3L^3 q_1 - L^3 q_2 + 4q_1 x^3) - x^4(q_1 - q_2)}{24 EI L}

**Fixed-Free-Fixed-Free** (simply supported, pinned-pinned):

.. math::

   \phi(x) = \frac{-360 EI w_1 + 360 EI w_2 + 8L^4 q_1 + 7L^4 q_2 + 30Lx^2(-L(2q_1 + q_2) + 2q_1 x) + 15x^4(-q_1 + q_2)}{360 EI L}

Shear Force Solutions (Euler-Bernoulli)
---------------------------------------

**Fixed-Fixed-Fixed-Fixed** (fully fixed both ends):

.. math::

   V_y(x) = \frac{120 EI L (\phi_1 + \phi_2) + 240 EI (w_1 - w_2) + L^3 (-7Lq_1 - 3Lq_2 + 20 q_1 x) - 10 L^2 x^2 (q_1 - q_2)}{20 L^3}

**Fixed-Fixed-Free-Fixed** (cantilever-like, end displacement free):

.. math::

   V_y(x) = -\frac{L \bigl( L(q_1 + q_2) - 2q_1 x \bigr) + x^2(q_1 - q_2)}{2L}

**Fixed-Fixed-Fixed-Free** (end rotation free, hinge at end):

.. math::

   V_y(x) = \frac{120 EI L \phi_1 + 120 EI (w_1 - w_2) - L^3 (16Lq_1 + 9Lq_2 - 40 q_1 x) - 20 L^2 x^2 (q_1 - q_2)}{40 L^3}

**Free-Fixed-Fixed-Fixed** (start displacement free):

.. math::

   V_y(x) = \frac{x (2Lq_1 - x(q_1 - q_2))}{2L}

**Fixed-Free-Fixed-Fixed** (start rotation free, hinge at start):

.. math::

   V_y(x) = \frac{120 EI L \phi_2 + 120 EI (w_1 - w_2) - L^3 (11Lq_1 + 4Lq_2 - 40 q_1 x) - 20 L^2 x^2 (q_1 - q_2)}{40 L^3}

**Fixed-Fixed-Free-Free** (cantilever: fixed at start, free at end):

.. math::

   V_y(x) = -\frac{L \bigl( L(q_1 + q_2) - 2q_1 x \bigr) + x^2(q_1 - q_2)}{2L}

**Free-Free-Fixed-Fixed** (cantilever: free at start, fixed at end):

.. math::

   V_y(x) = \frac{x (2Lq_1 - x(q_1 - q_2))}{2L}

**Fixed-Free-Fixed-Free** (simply supported, pinned-pinned):

.. math::

   V_y(x) = \frac{-L \bigl( -L(2q_1 + q_2) + 6q_1 x \bigr) + 3x^2(q_1 - q_2)}{6L}

Bending Moment Solutions (Euler-Bernoulli)
------------------------------------------

**Fixed-Fixed-Fixed-Fixed** (fully fixed both ends):

.. math::

   M_z(x) = \frac{1}{60 L^3} \Bigl[ -120 EI L^2 (2\phi_1 + \phi_2) + 360 EI L (-w_1 + w_2) + L^3 (3L^2 q_1 + 2L^2 q_2 + 30 q_1 x^2) \\
   + 10 L^2 x^3 (-q_1 + q_2) + 3x \bigl( 120 EI L (\phi_1 + \phi_2) + 240 EI (w_1 - w_2) - L^4 (7q_1 + 3q_2) \bigr) \Bigr]

**Fixed-Fixed-Free-Fixed**:

.. math::

   M_z(x) = \frac{-24 EI \phi_1 + 24 EI \phi_2 + 3L^3 q_1 + 5L^3 q_2 - 12Lx \bigl( L(q_1 + q_2) - q_1 x \bigr) - 4x^3(q_1 - q_2)}{24L}

**Fixed-Fixed-Fixed-Free** (propped cantilever):

.. math::

   M_z(x) = \frac{1}{120 L^3} \Bigl[ 60 L^3 q_1 x^2 + 20 L^2 x^3 (-q_1 + q_2) + L \bigl( -360 EI L \phi_1 - 360 EI w_1 + 360 EI w_2 + 8L^4 q_1 + 7L^4 q_2 \bigr) \\
   - 3x \bigl( -120 EI L \phi_1 + 120 EI (-w_1 + w_2) + L^4 (16 q_1 + 9q_2) \bigr) \Bigr]

**Free-Fixed-Fixed-Fixed**:

.. math::

   M_z(x) = \frac{-24 EI \phi_1 + 24 EI \phi_2 - 3L^3 q_1 - L^3 q_2 + 12Lq_1 x^2 - 4x^3(q_1 - q_2)}{24L}

**Fixed-Free-Fixed-Fixed** (propped cantilever):

.. math::

   M_z(x) = \frac{x \bigl( 360 EI L \phi_2 - 360 EI (-w_1 + w_2) - 3L^4 (11q_1 + 4q_2) + 60 L^3 q_1 x + 20 L^2 x^2 (-q_1 + q_2) \bigr)}{120 L^3}

**Fixed-Fixed-Free-Free** (cantilever: fixed at start, free at end):

.. math::

   M_z(x) = \frac{L \bigl( L^2(q_1 + 2q_2) - 3Lx(q_1 + q_2) + 3q_1 x^2 \bigr) - x^3(q_1 - q_2)}{6L}

**Free-Free-Fixed-Fixed** (cantilever: free at start, fixed at end):

.. math::

   M_z(x) = \frac{x^2 (3Lq_1 - x(q_1 - q_2))}{6L}

**Fixed-Free-Fixed-Free** (simply supported, pinned-pinned):

.. math::

   M_z(x) = \frac{x \bigl( -L \bigl( L(2q_1 + q_2) - 3q_1 x \bigr) + x^2(-q_1 + q_2) \bigr)}{6L}


Bending: Timoshenko Theory
==========================

Timoshenko beam theory includes the effect of shear deformation, which becomes significant for short, deep beams (typically :math:`L/h < 20`). The theory introduces an independent rotation variable :math:`\phi` representing the rotation of the cross-section due to bending alone.

Governing Differential Equations
--------------------------------

The equilibrium equations remain the same as Euler-Bernoulli:

.. math::

   \frac{dV_y}{dx} + q_y = 0

.. math::

   \frac{dM_z}{dx} - V_y = 0

However, the constitutive relations differ:

.. math::

   M_z = EI_z \frac{d\phi}{dx}

.. math::

   V_y = kAG \left( \frac{dw}{dx} - \phi \right)

The first equation relates moment to the bending rotation :math:`\phi`, while the second includes shear deformation through the term :math:`dw/dx - \phi`.

The key parameter distinguishing Timoshenko from Euler-Bernoulli behavior is the ratio:

.. math::

   \Phi = \frac{12 EI}{L^2 \cdot kAG}

When :math:`\Phi \to 0` (very slender beam or infinite shear stiffness), Timoshenko theory reduces to Euler-Bernoulli.

Deflection Solutions (Timoshenko)
---------------------------------

**Fixed-Fixed-Fixed-Fixed** (fully fixed both ends):

.. math::

   w(x) = \frac{1}{120 EI L \cdot kAG (12EI + L^2 kAG)} \Bigl[ 120 EI L \cdot kAG (12EI + L^2 kAG)(\phi_1 x + w_1) \\
   - 60 EI L q_1 x^2 (12EI + L^2 kAG) + 20 EI x^3 (12EI + L^2 kAG)(q_1 - q_2) \\
   + 6EI x (80 EI L^2 q_1 + 40 EI L^2 q_2 - 120 EI L \cdot kAG (\phi_1 + \phi_2) - 240 EI \cdot kAG(w_1 - w_2) + 7L^4 kAG \cdot q_1 + 3L^4 kAG \cdot q_2) \\
   + 5L \cdot kAG \cdot q_1 x^4 (12EI + L^2 kAG) + kAG \cdot x^5 (12EI + L^2 kAG)(-q_1 + q_2) \\
   + kAG \cdot x^2 \bigl( -720 EI^2 \phi_1 + 720 EI^2 \phi_2 + 30 EI L^3 q_1 + 30 EI L^3 q_2 - 240 EI L^2 kAG \cdot \phi_1 \\
   - 120 EI L^2 kAG \cdot \phi_2 - 360 EI L \cdot kAG(w_1 - w_2) + 3L^5 kAG \cdot q_1 + 2L^5 kAG \cdot q_2 + (\ldots) \bigr) \Bigr]

**Fixed-Fixed-Free-Fixed**:

.. math::

   w(x) = \frac{EI L \cdot kAG (\phi_1 x + w_1) + \frac{EI L x (L(q_1 + q_2) - q_1 x)}{2} + \frac{EI x^3 (q_1 - q_2)}{6} + \frac{L \cdot kAG \cdot x^3 (-2L(q_1 + q_2) + q_1 x)}{24} + (\ldots)}{EI L \cdot kAG}

**Fixed-Fixed-Free-Free** (cantilever: fixed at start, free at end):

.. math::

   w(x) = \frac{EI L \cdot kAG (\phi_1 x + w_1) + \frac{EI L x (L(q_1 + q_2) - q_1 x)}{2} + \frac{EI x^3 (q_1 - q_2)}{6} + \frac{L \cdot kAG \cdot x^2 (L^2(2q_1 + 4q_2) - 2Lx(q_1 + q_2) + q_1 x^2)}{24} + \frac{kAG \cdot x^5 (-q_1 + q_2)}{120}}{EI L \cdot kAG}

**Fixed-Free-Fixed-Free** (simply supported, pinned-pinned):

.. math::

   w(x) = \frac{EI L \cdot kAG \cdot w_1 + \frac{EI L x (L(2q_1 + q_2) - 3q_1 x)}{6} + \frac{EI x^3 (q_1 - q_2)}{6} + \frac{L \cdot kAG \cdot x^3 (-2L(2q_1 + q_2) + 3q_1 x)}{72} + \frac{kAG \cdot x (-360 EI w_1 + 360 EI w_2 + 8L^4 q_1 + 7L^4 q_2 + (\ldots))}{360}}{EI L \cdot kAG}

Rotation Solutions (Timoshenko)
-------------------------------

**Fixed-Fixed-Fixed-Fixed** (fully fixed both ends):

.. math::

   \phi(x) = \frac{1}{EI L (12EI + L^2 kAG)} \Bigl[ EI L \phi_1 (12EI + L^2 kAG) + \frac{L q_1 x^3 (12EI + L^2 kAG)}{6} - \frac{x^4 (12EI + L^2 kAG)(q_1 - q_2)}{24} \\
   + x \bigl( -1440 EI^2 \phi_1 + 1440 EI^2 \phi_2 + 60 EI L^3 q_1 + 60 EI L^3 q_2 - 480 EI L^2 kAG \cdot \phi_1 - 240 EI L^2 kAG \cdot \phi_2 \\
   - 720 EI L \cdot kAG (w_1 - w_2) + 6L^5 kAG \cdot q_1 + 4L^5 kAG \cdot q_2 + (\ldots) \bigr) \Bigr]

**Fixed-Fixed-Free-Free** (cantilever: fixed at start, free at end):

.. math::

   \phi(x) = \frac{EI L \phi_1 + \frac{Lx(L^2(2q_1 + 4q_2) - 3Lx(q_1 + q_2) + 2q_1 x^2)}{12} + \frac{x^4(-q_1 + q_2)}{24}}{EI L}

**Fixed-Free-Fixed-Free** (simply supported, pinned-pinned):

.. math::

   \phi(x) = \frac{-360 EI w_1 + 360 EI w_2 + 8L^4 q_1 + 7L^4 q_2 + 30Lx^2(-L(2q_1 + q_2) + 2q_1 x) + 15x^4(-q_1 + q_2)}{360 EI L}

Shear Force Solutions (Timoshenko)
----------------------------------

**Fixed-Fixed-Fixed-Fixed** (fully fixed both ends):

.. math::

   V_y(x) = \frac{1}{20L(12EI + L^2 kAG)} \Bigl[ -80 EI L^2 q_1 - 40 EI L^2 q_2 + 120 EI L \cdot kAG (\phi_1 + \phi_2) \\
   + 240 EI \cdot kAG (w_1 - w_2) - 7L^4 kAG \cdot q_1 - 3L^4 kAG \cdot q_2 \\
   + 20Lq_1 x (12EI + L^2 kAG) + 10x^2 (12EI + L^2 kAG)(-q_1 + q_2) \Bigr]

**Fixed-Fixed-Free-Fixed**:

.. math::

   V_y(x) = \frac{L \bigl( -L(q_1 + q_2) + 2q_1 x \bigr) + x^2(-q_1 + q_2)}{2L}

**Fixed-Fixed-Fixed-Free** (propped cantilever):

.. math::

   V_y(x) = \frac{1}{L(3EI + L^2 kAG)} \Bigl[ -EI L^2 q_1 - \frac{EI L^2 q_2}{2} + 3EI L \cdot kAG \cdot \phi_1 + 3EI \cdot kAG (w_1 - w_2) \\
   - \frac{2L^4 kAG \cdot q_1}{5} - \frac{9L^4 kAG \cdot q_2}{40} + Lq_1 x (3EI + L^2 kAG) + \frac{x^2 (3EI + L^2 kAG)(-q_1 + q_2)}{2} \Bigr]

**Free-Fixed-Fixed-Fixed**:

.. math::

   V_y(x) = \frac{x (2Lq_1 + x(-q_1 + q_2))}{2L}

**Fixed-Free-Fixed-Fixed** (propped cantilever):

.. math::

   V_y(x) = \frac{1}{L(3EI + L^2 kAG)} \Bigl[ -EI L^2 q_1 - \frac{EI L^2 q_2}{2} + 3EI L \cdot kAG \cdot \phi_2 + 3EI \cdot kAG (w_1 - w_2) \\
   - \frac{11L^4 kAG \cdot q_1}{40} - \frac{L^4 kAG \cdot q_2}{10} + Lq_1 x (3EI + L^2 kAG) + \frac{x^2 (3EI + L^2 kAG)(-q_1 + q_2)}{2} \Bigr]

**Fixed-Fixed-Free-Free** (cantilever: fixed at start, free at end):

.. math::

   V_y(x) = \frac{L \bigl( -L(q_1 + q_2) + 2q_1 x \bigr) + x^2(-q_1 + q_2)}{2L}

**Free-Free-Fixed-Fixed** (cantilever: free at start, fixed at end):

.. math::

   V_y(x) = \frac{x (2Lq_1 + x(-q_1 + q_2))}{2L}

**Fixed-Free-Fixed-Free** (simply supported, pinned-pinned):

.. math::

   V_y(x) = \frac{L \bigl( -L(2q_1 + q_2) + 6q_1 x \bigr) + 3x^2(-q_1 + q_2)}{6L}

Bending Moment Solutions (Timoshenko)
-------------------------------------

**Fixed-Fixed-Fixed-Fixed** (fully fixed both ends):

.. math::

   M_z(x) = \frac{1}{L(12EI + L^2 kAG)} \Bigl[ -12EI^2 \phi_1 + 12EI^2 \phi_2 + \frac{EI L^3 q_1}{2} + \frac{EI L^3 q_2}{2} \\
   - 4EI L^2 kAG \cdot \phi_1 - 2EI L^2 kAG \cdot \phi_2 - 6EI L \cdot kAG (w_1 - w_2) \\
   + \frac{L^5 kAG \cdot q_1}{20} + \frac{L^5 kAG \cdot q_2}{30} + \frac{Lq_1 x^2 (12EI + L^2 kAG)}{2} - \frac{x^3 (12EI + L^2 kAG)(q_1 - q_2)}{6} \\
   - \frac{x}{20} \bigl( 80EI L^2 q_1 + 40EI L^2 q_2 - 120EI L \cdot kAG (\phi_1 + \phi_2) - 240EI \cdot kAG (w_1 - w_2) + 7L^4 kAG \cdot q_1 + 3L^4 kAG \cdot q_2 \bigr) \Bigr]

**Fixed-Fixed-Free-Fixed**:

.. math::

   M_z(x) = \frac{-24EI \phi_1 + 24EI \phi_2 + 3L^3 q_1 + 5L^3 q_2 - 12Lx \bigl( L(q_1 + q_2) - q_1 x \bigr) - 4x^3(q_1 - q_2)}{24L}

**Fixed-Fixed-Fixed-Free** (propped cantilever):

.. math::

   M_z(x) = \frac{1}{120L(3EI + L^2 kAG)} \Bigl[ L \cdot kAG \bigl( -360EI L \phi_1 - 360EI (w_1 - w_2) + 8L^4 q_1 + 7L^4 q_2 \bigr) \\
   + 60Lq_1 x^2 (3EI + L^2 kAG) - 20x^3 (3EI + L^2 kAG)(q_1 - q_2) \\
   - 3x \bigl( 40EI L^2 q_1 + 20EI L^2 q_2 - 120EI L \cdot kAG \cdot \phi_1 - 120EI \cdot kAG (w_1 - w_2) + 16L^4 kAG \cdot q_1 + 9L^4 kAG \cdot q_2 \bigr) \Bigr]

**Free-Fixed-Fixed-Fixed**:

.. math::

   M_z(x) = \frac{-24EI \phi_1 + 24EI \phi_2 - 3L^3 q_1 - L^3 q_2 + 12Lq_1 x^2 - 4x^3(q_1 - q_2)}{24L}

**Fixed-Free-Fixed-Fixed** (propped cantilever):

.. math::

   M_z(x) = \frac{x}{120L(3EI + L^2 kAG)} \Bigl[ -120EI L^2 q_1 - 60EI L^2 q_2 + 360EI L \cdot kAG \cdot \phi_2 + 360EI \cdot kAG (w_1 - w_2) \\
   - 33L^4 kAG \cdot q_1 - 12L^4 kAG \cdot q_2 + 60Lq_1 x (3EI + L^2 kAG) - 20x^2 (3EI + L^2 kAG)(q_1 - q_2) \Bigr]

**Fixed-Fixed-Free-Free** (cantilever: fixed at start, free at end):

.. math::

   M_z(x) = \frac{L \bigl( L^2(q_1 + 2q_2) - 3Lx(q_1 + q_2) + 3q_1 x^2 \bigr) - x^3(q_1 - q_2)}{6L}

**Free-Free-Fixed-Fixed** (cantilever: free at start, fixed at end):

.. math::

   M_z(x) = \frac{x^2 (3Lq_1 - x(q_1 - q_2))}{6L}

**Fixed-Free-Fixed-Free** (simply supported, pinned-pinned):

.. math::

   M_z(x) = \frac{x \bigl( -L \bigl( L(2q_1 + q_2) - 3q_1 x \bigr) + x^2(-q_1 + q_2) \bigr)}{6L}


Biaxial Bending
===============

The formulas presented above apply to bending in the :math:`x`-:math:`y` plane with moment :math:`M_z` and shear :math:`V_y`. For bending in the :math:`x`-:math:`z` plane, the same formulas apply with the following substitutions:

.. list-table:: Variable Mapping for Biaxial Bending
   :header-rows: 1
   :widths: 50 50

   * - :math:`x`-:math:`y` plane
     - :math:`x`-:math:`z` plane
   * - :math:`w` (deflection in :math:`y`)
     - :math:`v` (deflection in :math:`z`)
   * - :math:`\phi` (rotation about :math:`z`)
     - :math:`\psi` (rotation about :math:`y`)
   * - :math:`V_y` (shear in :math:`y`)
     - :math:`V_z` (shear in :math:`z`)
   * - :math:`M_z` (moment about :math:`z`)
     - :math:`M_y` (moment about :math:`y`)
   * - :math:`q_y` (load in :math:`y`)
     - :math:`q_z` (load in :math:`z`)
   * - :math:`EI_z` (stiffness about :math:`z`)
     - :math:`EI_y` (stiffness about :math:`y`)


Finding Moment Extrema
======================

For beams with distributed loads, the maximum moment typically occurs at an interior point where the shear force is zero. The location of extrema can be found analytically.

For a trapezoidal load, the shear force varies linearly or quadratically with :math:`x`. Setting :math:`V(x) = 0` and solving for :math:`x` yields the location(s) of moment extrema.

For the simply supported beam (Fixed-Free-Fixed-Free) with uniform load :math:`q = q_1 = q_2`:

.. math::

   V_y(x) = q \left( \frac{L}{2} - x \right)

Setting :math:`V_y = 0` gives :math:`x = L/2`. The maximum moment at midspan is:

.. math::

   M_{z,\max} = \frac{qL^2}{8}

For more complex release combinations or non-uniform loads, the extrema positions must be computed from the specific shear force formula for that configuration.


Verification Examples
=====================

The following classical results can be used to verify the implementation:

Simply Supported Beam with Uniform Load
---------------------------------------

- Configuration: Fixed-Free-Fixed-Free (pinned-pinned)
- Load: :math:`q_1 = q_2 = q`
- Maximum moment: :math:`M_{\max} = qL^2/8` at :math:`x = L/2`
- End moments: :math:`M(0) = M(L) = 0`
- End shears: :math:`V(0) = qL/2`, :math:`V(L) = -qL/2`
- Midspan deflection: :math:`w(L/2) = 5qL^4/(384EI)`

Cantilever with Uniform Load
----------------------------

- Configuration: Fixed-Fixed-Free-Free (fixed at start, free at end)
- Load: :math:`q_1 = q_2 = q`
- Maximum moment: :math:`M_{\max} = qL^2/2` at :math:`x = 0` (support)
- Free end: :math:`M(L) = 0`, :math:`V(L) = 0`
- Tip deflection: :math:`w(L) = qL^4/(8EI)`

Fixed-Fixed Beam with Uniform Load
----------------------------------

- Configuration: Fixed-Fixed-Fixed-Fixed
- Load: :math:`q_1 = q_2 = q`
- End moments: :math:`M(0) = M(L) = -qL^2/12` (negative = hogging)
- Midspan moment: :math:`M(L/2) = qL^2/24` (positive = sagging)
- End shears: :math:`V(0) = qL/2`, :math:`V(L) = -qL/2`
- Midspan deflection: :math:`w(L/2) = qL^4/(384EI)`

Propped Cantilever with Uniform Load
------------------------------------

- Configuration: Fixed-Fixed-Fixed-Free (fixed at start, pinned at end)
- Load: :math:`q_1 = q_2 = q`
- Fixed end moment: :math:`M(0) = -qL^2/8`
- Pinned end moment: :math:`M(L) = 0`
- Maximum positive moment: :math:`M_{\max} = 9qL^2/128` at :math:`x = 3L/8`


Additional Release Combinations
===============================

Beyond the eight most common release combinations documented above, eight additional combinations are possible but rarely used in practice:

- **Fixed-Free-Free-Fixed**: :math:`w_1`, :math:`\phi_2` fixed; :math:`\phi_1`, :math:`w_2` free
- **Free-Fixed-Free-Fixed**: :math:`\phi_1`, :math:`\phi_2` fixed; :math:`w_1`, :math:`w_2` free
- **Free-Fixed-Fixed-Free**: :math:`\phi_1`, :math:`w_2` fixed; :math:`w_1`, :math:`\phi_2` free
- **Free-Fixed-Free-Free**: Only :math:`\phi_1` fixed
- **Fixed-Free-Free-Free**: Only :math:`w_1` fixed
- **Free-Free-Free-Fixed**: Only :math:`\phi_2` fixed
- **Free-Free-Fixed-Free**: Only :math:`w_2` fixed
- **Free-Free-Free-Free**: All free (rigid body motion, internal forces are zero)

These combinations can represent unusual support conditions or elements within complex multi-element structures. The analytical formulas follow the same derivation approach from the governing differential equations with the appropriate boundary conditions.


Warping Torsion (14-DOF Elements)
=================================

For thin-walled open sections (I-beams, channels, angles), torsion induces non-uniform warping of the cross-section. This requires a 14-DOF element formulation that includes the warping degree of freedom at each node.

Governing Differential Equation
-------------------------------

The mixed torsion problem for thin-walled open sections is governed by:

.. math::

   EI_\omega \frac{d^4\theta}{dx^4} - GJ \frac{d^2\theta}{dx^2} + m_t = 0

where:

- :math:`\theta`: Twist angle
- :math:`EI_\omega`: Warping stiffness (warping constant :math:`I_\omega` times elastic modulus)
- :math:`GJ`: St. Venant torsional stiffness
- :math:`m_t`: Distributed torque (assumed zero in the following derivations)

Introducing the warping parameter :math:`k`:

.. math::

   k = \sqrt{\frac{GJ}{EI_\omega}}

the governing equation (without distributed torque) becomes:

.. math::

   \frac{d^4\theta}{dx^4} - k^2 \frac{d^2\theta}{dx^2} = 0

The characteristic equation :math:`r^4 - k^2 r^2 = r^2(r^2 - k^2) = 0` has roots :math:`r = 0, 0, k, -k`, giving the general solution:

.. math::

   \theta(x) = A + Bx + C\cosh(kx) + D\sinh(kx)

Internal Actions
----------------

The internal actions for warping torsion are:

**Rate of twist:**

.. math::

   \phi' = \frac{d\theta}{dx}

**Bimoment:**

.. math::

   B_\omega = -EI_\omega \frac{d^2\theta}{dx^2}

**St. Venant torsion:**

.. math::

   T_{sv} = GJ \frac{d\theta}{dx}

**Warping torsion:**

.. math::

   T_\omega = \frac{dB_\omega}{dx} = -EI_\omega \frac{d^3\theta}{dx^3}

**Total torsion:**

.. math::

   T = T_{sv} + T_\omega

**Warping normal stress:**

.. math::

   \sigma_\omega = -\frac{B_\omega \cdot \omega}{I_\omega}

where :math:`\omega` is the sectorial coordinate at the point of interest.

Boundary Conditions
-------------------

For warping, each end can be either:

- **Fixed (warping restrained)**: The cross-section cannot warp freely. This is modeled as :math:`d\theta/dx = 0` at the support.
- **Free (warping free)**: The cross-section can warp freely, meaning bimoment is zero. This is modeled as :math:`d^2\theta/dx^2 = 0` (i.e., :math:`B_\omega = 0`).

With twist angle :math:`\theta` prescribed at both ends, this gives four combinations for warping boundary conditions.

.. list-table:: Warping Release Combinations
   :header-rows: 1
   :widths: 30 70

   * - Combination
     - Description
   * - Fixed-Fixed
     - Both ends warping restrained
   * - Fixed-Free
     - Start warping restrained, end warping free
   * - Free-Fixed
     - Start warping free, end warping restrained
   * - Free-Free
     - Both ends warping free (pure St. Venant torsion)

Twist Angle Solutions
---------------------

**Fixed-Fixed** (both ends warping restrained):

Let :math:`\Delta = kL\sinh(kL) - 2\cosh(kL) + 2`

.. math::

   \theta(x) = \frac{1}{\Delta} \Bigl[ kL\theta_1\sinh(kL) - kx(\theta_1 - \theta_2)\sinh(kL) \\
   - \theta_1\cosh(kL) + \theta_1\cosh(kx) - \theta_1\cosh(k(L-x)) + \theta_1 \\
   - \theta_2\cosh(kL) - \theta_2\cosh(kx) + \theta_2\cosh(k(L-x)) + \theta_2 \Bigr]

**Fixed-Free** (start warping restrained, end warping free):

Let :math:`\Delta' = kL\cosh(kL) - \sinh(kL)`

.. math::

   \theta(x) = \frac{1}{\Delta'} \Bigl[ kL\theta_1\cosh(kL) - kx(\theta_1 - \theta_2)\cosh(kL) \\
   - \theta_1\sinh(k(L-x)) - \theta_2\sinh(kL) + \theta_2\sinh(k(L-x)) \Bigr]

**Free-Fixed** (start warping free, end warping restrained):

.. math::

   \theta(x) = \frac{1}{\Delta'} \Bigl[ \theta_1 \Delta' - kx(\theta_1 - \theta_2)\cosh(kL) + (\theta_1 - \theta_2)\sinh(kx) \Bigr]

**Free-Free** (both ends warping free):

.. math::

   \theta(x) = \theta_1 + \frac{(\theta_2 - \theta_1)x}{L}

This is simply linear interpolation (pure St. Venant torsion).

Rate of Twist Solutions
-----------------------

**Fixed-Fixed**:

.. math::

   \frac{d\theta}{dx} = \frac{-k(\theta_1 - \theta_2)}{\Delta} \Bigl[ \sinh(kL) - \sinh(kx) - \sinh(k(L-x)) \Bigr]

**Fixed-Free**:

.. math::

   \frac{d\theta}{dx} = \frac{-k(\theta_1 - \theta_2)}{\Delta'} \Bigl[ \cosh(kL) - \cosh(k(L-x)) \Bigr]

**Free-Fixed**:

.. math::

   \frac{d\theta}{dx} = \frac{k(\theta_1 - \theta_2)}{\Delta'} \Bigl[ \cosh(kx) - \cosh(kL) \Bigr]

**Free-Free**:

.. math::

   \frac{d\theta}{dx} = \frac{\theta_2 - \theta_1}{L}

Bimoment Solutions
------------------

**Fixed-Fixed**:

.. math::

   B_\omega(x) = \frac{-EI_\omega k^2 (\theta_1 - \theta_2)}{\Delta} \Bigl[ \cosh(kx) - \cosh(k(L-x)) \Bigr]

At the boundaries:

.. math::

   B_\omega(0) = \frac{EI_\omega k^2 (\theta_1 - \theta_2)(\cosh(kL) - 1)}{\Delta}

.. math::

   B_\omega(L) = \frac{-EI_\omega k^2 (\theta_1 - \theta_2)(\cosh(kL) - 1)}{\Delta}

**Fixed-Free**:

.. math::

   B_\omega(x) = \frac{EI_\omega k^2 (\theta_1 - \theta_2) \sinh(k(L-x))}{\Delta'}

At the boundaries:

.. math::

   B_\omega(0) = \frac{EI_\omega k^2 (\theta_1 - \theta_2) \sinh(kL)}{\Delta'}

.. math::

   B_\omega(L) = 0

**Free-Fixed**:

.. math::

   B_\omega(x) = \frac{-EI_\omega k^2 (\theta_1 - \theta_2) \sinh(kx)}{\Delta'}

At the boundaries:

.. math::

   B_\omega(0) = 0

.. math::

   B_\omega(L) = \frac{-EI_\omega k^2 (\theta_1 - \theta_2) \sinh(kL)}{\Delta'}

**Free-Free**:

.. math::

   B_\omega(x) = 0

No bimoment exists when both ends are free to warp (pure St. Venant torsion).

St. Venant and Warping Torsion Components
-----------------------------------------

The St. Venant torsion component is:

.. math::

   T_{sv}(x) = GJ \frac{d\theta}{dx}

The warping torsion component is:

.. math::

   T_\omega(x) = \frac{dB_\omega}{dx}

For **Fixed-Fixed**:

.. math::

   T_\omega(x) = \frac{-EI_\omega k^3 (\theta_1 - \theta_2)}{\Delta} \Bigl[ \sinh(kx) + \sinh(k(L-x)) \Bigr]

For **Fixed-Free**:

.. math::

   T_\omega(x) = \frac{-EI_\omega k^3 (\theta_1 - \theta_2) \cosh(k(L-x))}{\Delta'}

For **Free-Fixed**:

.. math::

   T_\omega(x) = \frac{-EI_\omega k^3 (\theta_1 - \theta_2) \cosh(kx)}{\Delta'}

For **Free-Free**:

.. math::

   T_\omega(x) = 0

Limiting Cases
--------------

**Short beams** (:math:`kL \to 0`): Warping effects dominate, and the behavior approaches that of a beam with restrained warping throughout.

**Long beams** (:math:`kL \to \infty`): St. Venant torsion dominates, and bimoment effects are localized near the restrained ends.

The transition occurs around :math:`kL \approx 1`, where both mechanisms contribute significantly.


References
==========

1. Timoshenko, S. P., & Gere, J. M. (1961). *Theory of Elastic Stability* (2nd ed.). McGraw-Hill.

2. Weaver, W., & Gere, J. M. (1990). *Matrix Analysis of Framed Structures* (3rd ed.). Van Nostrand Reinhold.

3. Cook, R. D., Malkus, D. S., Plesha, M. E., & Witt, R. J. (2001). *Concepts and Applications of Finite Element Analysis* (4th ed.). Wiley.

4. Przemieniecki, J. S. (1968). *Theory of Matrix Structural Analysis*. McGraw-Hill.
