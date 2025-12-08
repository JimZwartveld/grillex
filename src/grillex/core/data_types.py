"""
Core data types for Grillex structural analysis.

This module provides Python-friendly access to the C++ core data structures:
- Node: Point in 3D space with up to 6 DOFs
- NodeRegistry: Manager for nodes with automatic merging
- Material: Material properties (E, G, nu, rho)
- Section: Cross-section properties for beams
"""

from grillex._grillex_cpp import Node, NodeRegistry, Material, Section

__all__ = ['Node', 'NodeRegistry', 'Material', 'Section']
