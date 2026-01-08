=======
Grillex
=======

**Grillex 2.0** is a structural analysis and design platform for offshore
and heavy-lift structures, featuring beam, plate, and grillage analysis
with cargo modeling capabilities.

.. note::

    This documentation includes executable code examples (doctests) that
    are verified against the codebase during builds to ensure accuracy.

Key Features
============

- **Multi-formulation beam elements**: Euler-Bernoulli, Timoshenko, and
  14-DOF warping elements
- **Comprehensive load types**: Point loads, distributed loads, and
  acceleration (gravity) loads
- **Design code support**: Eurocode 3 cross-section checks
- **LLM/Agent integration**: Designed for AI-assisted structural engineering
- **YAML model definition**: Human-readable input format

Quick Start
===========

.. code-block:: python

    from grillex.core import StructuralModel, DOFIndex

    # Create model
    model = StructuralModel(name="My First Model")

    # Add material and section
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)

    # Create beam, fix support, apply load
    beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    model.fix_node_at([0, 0, 0])
    model.add_point_load([6, 0, 0], force=[0, 0, -10.0])

    # Analyze and get results
    model.analyze()
    displacement = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)

Contents
========

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   user/index

.. toctree::
   :maxdepth: 2
   :caption: Verification

   verification/index

.. toctree::
   :maxdepth: 2
   :caption: Reference

   Internal Actions <internal_actions>
   Error Reference <errors>
   Module Reference <api/modules>

.. toctree::
   :maxdepth: 1
   :caption: Project Information

   Overview <readme>
   Contributions & Help <contributing>
   License <license>
   Authors <authors>
   Changelog <changelog>


Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


.. _toctree: https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html
.. _reStructuredText: https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html
.. _references: https://www.sphinx-doc.org/en/stable/markup/inline.html
.. _Python domain syntax: https://www.sphinx-doc.org/en/master/usage/restructuredtext/domains.html#the-python-domain
.. _Sphinx: https://www.sphinx-doc.org/
.. _Python: https://docs.python.org/
.. _Numpy: https://numpy.org/doc/stable
.. _SciPy: https://docs.scipy.org/doc/scipy/reference/
.. _matplotlib: https://matplotlib.org/contents.html#
.. _Pandas: https://pandas.pydata.org/pandas-docs/stable
.. _Scikit-Learn: https://scikit-learn.org/stable
.. _autodoc: https://www.sphinx-doc.org/en/master/ext/autodoc.html
.. _Google style: https://google.github.io/styleguide/pyguide.html#38-comments-and-docstrings
.. _NumPy style: https://numpydoc.readthedocs.io/en/latest/format.html
.. _classical style: https://www.sphinx-doc.org/en/master/domains.html#info-field-lists
