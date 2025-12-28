"""
Test gmsh with OpenGL libraries.

This module tests that gmsh can be properly initialized and used with
OpenGL libraries for mesh generation and visualization capabilities.

Task 19.3: Test gmsh with OpenGL libraries
"""

import os
import tempfile

import pytest


def test_gmsh_import():
    """Test that gmsh can be imported successfully."""
    import gmsh
    assert hasattr(gmsh, 'initialize')
    assert hasattr(gmsh, 'finalize')


def test_gmsh_initialization():
    """Test gmsh initialization and finalization."""
    import gmsh
    gmsh.initialize()
    try:
        # Verify gmsh is initialized
        gmsh.model.add('test')
        assert True
    finally:
        gmsh.finalize()


def test_gmsh_api_version():
    """Test that gmsh API version is accessible."""
    import gmsh
    gmsh.initialize()
    try:
        version = gmsh.GMSH_API_VERSION
        assert version is not None
        assert len(version) > 0
    finally:
        gmsh.finalize()


def test_gmsh_geometry_creation():
    """Test that gmsh can create basic geometry."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.model.add('geometry_test')

        # Create a simple box
        tag = gmsh.model.occ.addBox(0, 0, 0, 1, 1, 1)
        gmsh.model.occ.synchronize()

        assert tag > 0

        # Verify entities were created
        entities = gmsh.model.getEntities(3)  # 3D entities
        assert len(entities) > 0
    finally:
        gmsh.finalize()


def test_gmsh_mesh_generation():
    """Test that gmsh can generate 3D meshes."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.model.add('mesh_test')

        # Create geometry
        gmsh.model.occ.addBox(0, 0, 0, 1, 1, 1)
        gmsh.model.occ.synchronize()

        # Set mesh size
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.1)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 0.3)

        # Generate 3D mesh
        gmsh.model.mesh.generate(3)

        # Verify mesh was generated
        node_tags, coords, _ = gmsh.model.mesh.getNodes()
        elem_types, elem_tags, _ = gmsh.model.mesh.getElements()

        assert len(node_tags) > 0, "No nodes generated"
        assert len(elem_types) > 0, "No elements generated"
        assert 4 in elem_types, "No tetrahedral elements (type 4) generated"
    finally:
        gmsh.finalize()


def test_gmsh_opengl_graphics_options():
    """Test that OpenGL-related graphics options are accessible."""
    import gmsh
    gmsh.initialize()
    try:
        # Test reading graphics options
        graphics_options = [
            'General.GraphicsWidth',
            'General.GraphicsHeight',
            'General.BackgroundGradient',
            'General.Axes',
            'General.SmallAxes',
            'General.Trackball',
        ]

        for opt in graphics_options:
            val = gmsh.option.getNumber(opt)
            assert val is not None, f"Option {opt} returned None"
    finally:
        gmsh.finalize()


def test_gmsh_mesh_visualization_options():
    """Test that mesh visualization options can be set."""
    import gmsh
    gmsh.initialize()
    try:
        # Test setting visualization options
        viz_options = [
            ('Mesh.SurfaceEdges', 1),
            ('Mesh.SurfaceFaces', 1),
            ('Mesh.VolumeEdges', 1),
            ('Mesh.VolumeFaces', 1),
            ('Mesh.ColorCarousel', 1),
        ]

        for opt, val in viz_options:
            gmsh.option.setNumber(opt, val)
            new_val = gmsh.option.getNumber(opt)
            assert new_val == val, f"Option {opt} not set correctly"
    finally:
        gmsh.finalize()


def test_gmsh_color_setting():
    """Test that colors can be set for mesh elements."""
    import gmsh
    gmsh.initialize()
    try:
        # Test setting a color
        gmsh.option.setColor("Mesh.Color.Triangles", 100, 100, 255, 255)
        # If no exception, the test passes
        assert True
    finally:
        gmsh.finalize()


def test_gmsh_mesh_export():
    """Test that meshes can be exported to files."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.model.add('export_test')

        # Create and mesh geometry
        gmsh.model.occ.addBox(0, 0, 0, 1, 1, 1)
        gmsh.model.occ.synchronize()
        gmsh.model.mesh.generate(3)

        # Export to temporary file
        with tempfile.TemporaryDirectory() as tmpdir:
            mesh_file = os.path.join(tmpdir, "test.msh")
            gmsh.write(mesh_file)

            assert os.path.exists(mesh_file), "Mesh file not created"
            assert os.path.getsize(mesh_file) > 0, "Mesh file is empty"
    finally:
        gmsh.finalize()


def test_gmsh_beam_like_geometry():
    """Test creating beam-like geometry relevant to Grillex."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.model.add('beam_test')

        # Create a beam-like geometry (elongated box)
        length = 6.0  # meters
        width = 0.3   # meters
        height = 0.3  # meters

        gmsh.model.occ.addBox(0, -width/2, -height/2, length, width, height)
        gmsh.model.occ.synchronize()

        # Set mesh parameters for structural analysis
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.05)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 0.15)

        # Generate mesh
        gmsh.model.mesh.generate(3)

        # Verify mesh quality
        node_tags, coords, _ = gmsh.model.mesh.getNodes()

        assert len(node_tags) > 100, "Expected more nodes for fine mesh"
    finally:
        gmsh.finalize()


def test_gmsh_window_size_setting():
    """Test that graphics window size can be configured."""
    import gmsh
    gmsh.initialize()
    try:
        # Set custom window size
        gmsh.option.setNumber("General.GraphicsWidth", 1024)
        gmsh.option.setNumber("General.GraphicsHeight", 768)

        # Verify settings
        width = gmsh.option.getNumber("General.GraphicsWidth")
        height = gmsh.option.getNumber("General.GraphicsHeight")

        assert width == 1024, f"Expected width 1024, got {width}"
        assert height == 768, f"Expected height 768, got {height}"
    finally:
        gmsh.finalize()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
