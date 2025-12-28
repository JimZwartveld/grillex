"""
Mesh generation utilities for Grillex.

This module provides mesh generation for plate elements using gmsh.
"""

from .gmsh_mesher import (
    GmshPlateMesher,
    MeshResult,
    MeshingError
)

__all__ = [
    'GmshPlateMesher',
    'MeshResult',
    'MeshingError'
]
