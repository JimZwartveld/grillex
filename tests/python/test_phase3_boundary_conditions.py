"""
Test suite for Phase 3 Task 3.3: Boundary Conditions

Tests the BCHandler class for managing fixed DOFs and prescribed displacements,
including support for standard 6-DOF and 7-DOF (with warping) nodes.
"""

import pytest
import numpy as np
from grillex.core import (
    NodeRegistry, Material, Section, BeamElement, BeamConfig,
    DOFHandler, Assembler, BCHandler, DOFIndex, FixedDOF
)


class TestDOFIndex:
    """Test the DOFIndex enum"""

    def test_dof_index_values(self):
        """Test that DOF indices have correct values"""
        assert DOFIndex.UX == 0
        assert DOFIndex.UY == 1
        assert DOFIndex.UZ == 2
        assert DOFIndex.RX == 3
        assert DOFIndex.RY == 4
        assert DOFIndex.RZ == 5
        assert DOFIndex.WARP == 6

    def test_dof_index_usage(self):
        """Test that DOF index enum can be used with BCHandler"""
        bc = BCHandler()
        bc.add_fixed_dof(1, DOFIndex.UX, 0.0)
        bc.add_fixed_dof(1, DOFIndex.WARP, 0.0)
        assert bc.num_fixed_dofs() == 2


class TestFixedDOF:
    """Test the FixedDOF struct"""

    def test_fixed_dof_creation(self):
        """Test creating a FixedDOF"""
        fixed = FixedDOF(1, 2, 0.5)
        assert fixed.node_id == 1
        assert fixed.local_dof == 2
        assert fixed.value == 0.5

    def test_fixed_dof_default_value(self):
        """Test FixedDOF with default value"""
        fixed = FixedDOF(1, 2)
        assert fixed.value == 0.0

    def test_fixed_dof_repr(self):
        """Test FixedDOF __repr__"""
        fixed = FixedDOF(1, 2, 0.5)
        repr_str = repr(fixed)
        assert "node=1" in repr_str
        assert "dof=2" in repr_str
        assert "value=0.5" in repr_str


class TestBCHandler:
    """Test basic BCHandler functionality"""

    def test_bc_handler_creation(self):
        """Test creating an empty BCHandler"""
        bc = BCHandler()
        assert bc.num_fixed_dofs() == 0

    def test_add_fixed_dof(self):
        """Test adding a single fixed DOF"""
        bc = BCHandler()
        bc.add_fixed_dof(1, 0, 0.0)  # Fix UX at node 1
        assert bc.num_fixed_dofs() == 1

        fixed_dofs = bc.get_fixed_dofs()
        assert len(fixed_dofs) == 1
        assert fixed_dofs[0].node_id == 1
        assert fixed_dofs[0].local_dof == 0
        assert fixed_dofs[0].value == 0.0

    def test_add_duplicate_fixed_dof(self):
        """Test that adding duplicate DOF doesn't create duplicates"""
        bc = BCHandler()
        bc.add_fixed_dof(1, 0, 0.0)
        bc.add_fixed_dof(1, 0, 0.0)  # Same DOF
        assert bc.num_fixed_dofs() == 1

    def test_fix_node(self):
        """Test fixing all 6 standard DOFs at a node"""
        bc = BCHandler()
        bc.fix_node(1)
        assert bc.num_fixed_dofs() == 6

        # Check that all 6 standard DOFs are fixed
        fixed_dofs = bc.get_fixed_dofs()
        local_dofs = [fd.local_dof for fd in fixed_dofs]
        assert set(local_dofs) == {0, 1, 2, 3, 4, 5}

    def test_fix_node_with_warping(self):
        """Test fixing all 7 DOFs including warping"""
        bc = BCHandler()
        bc.fix_node_with_warping(1)
        assert bc.num_fixed_dofs() == 7

        # Check that all 7 DOFs are fixed
        fixed_dofs = bc.get_fixed_dofs()
        local_dofs = [fd.local_dof for fd in fixed_dofs]
        assert set(local_dofs) == {0, 1, 2, 3, 4, 5, 6}

    def test_pin_node(self):
        """Test pinning a node (fix translations only)"""
        bc = BCHandler()
        bc.pin_node(1)
        assert bc.num_fixed_dofs() == 3

        # Check that only translations are fixed
        fixed_dofs = bc.get_fixed_dofs()
        local_dofs = [fd.local_dof for fd in fixed_dofs]
        assert set(local_dofs) == {0, 1, 2}  # UX, UY, UZ

    def test_fork_support(self):
        """Test fork support (fix translations, free rotations and warping)"""
        bc = BCHandler()
        bc.fork_support(1)
        assert bc.num_fixed_dofs() == 3

        # Check that only translations are fixed
        fixed_dofs = bc.get_fixed_dofs()
        local_dofs = [fd.local_dof for fd in fixed_dofs]
        assert set(local_dofs) == {0, 1, 2}  # UX, UY, UZ

    def test_clear(self):
        """Test clearing all boundary conditions"""
        bc = BCHandler()
        bc.fix_node(1)
        assert bc.num_fixed_dofs() == 6
        bc.clear()
        assert bc.num_fixed_dofs() == 0

    def test_repr(self):
        """Test BCHandler __repr__"""
        bc = BCHandler()
        bc.fix_node(1)
        repr_str = repr(bc)
        assert "num_fixed=6" in repr_str


class TestBCHandlerWithDOFHandler:
    """Test BCHandler integration with DOFHandler"""

    def setup_method(self):
        """Set up a simple two-node model for testing"""
        self.registry = NodeRegistry()
        self.node1 = self.registry.get_or_create_node(0.0, 0.0, 0.0)
        self.node2 = self.registry.get_or_create_node(6.0, 0.0, 0.0)

        self.dof_handler = DOFHandler()
        self.dof_handler.number_dofs(self.registry)

    def test_get_fixed_global_dofs(self):
        """Test getting global DOF indices for fixed DOFs"""
        bc = BCHandler()
        bc.fix_node(self.node1.id)

        global_dofs = bc.get_fixed_global_dofs(self.dof_handler)
        assert len(global_dofs) == 6
        assert global_dofs == [0, 1, 2, 3, 4, 5]  # First 6 DOFs

    def test_get_fixed_global_dofs_with_warping(self):
        """Test getting global DOF indices with warping DOF"""
        # Enable warping on both nodes
        self.node1.enable_warping_dof()
        self.node2.enable_warping_dof()

        # Renumber DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(self.registry)

        bc = BCHandler()
        bc.fix_node_with_warping(self.node1.id)

        global_dofs = bc.get_fixed_global_dofs(dof_handler)
        assert len(global_dofs) == 7
        assert global_dofs == [0, 1, 2, 3, 4, 5, 6]  # First 7 DOFs

    def test_get_fixed_global_dofs_pin_support(self):
        """Test getting global DOFs for pin support"""
        bc = BCHandler()
        bc.pin_node(self.node1.id)

        global_dofs = bc.get_fixed_global_dofs(self.dof_handler)
        assert len(global_dofs) == 3
        assert global_dofs == [0, 1, 2]  # UX, UY, UZ at node 1


class TestPenaltyMethod:
    """Test penalty method application to system matrices"""

    def setup_method(self):
        """Set up a simple cantilever beam model"""
        # Create nodes
        self.registry = NodeRegistry()
        self.node1 = self.registry.get_or_create_node(0.0, 0.0, 0.0)  # Fixed end
        self.node2 = self.registry.get_or_create_node(6.0, 0.0, 0.0)  # Free end

        # Create material (steel)
        self.mat = Material(1, "Steel", E=200e6, nu=0.3, rho=7.85e-9)

        # Create section (W200x46.1)
        self.sec = Section(
            1, "W200x46.1",
            A=5880e-6,  # m²
            Iy=45.5e-6,  # m⁴
            Iz=15.3e-6,  # m⁴
            J=213e-9    # m⁴
        )

        # Create beam element
        self.elem = BeamElement(1, self.node1, self.node2, self.mat, self.sec)

        # Number DOFs
        self.dof_handler = DOFHandler()
        self.dof_handler.number_dofs(self.registry)

        # Assemble system matrices
        assembler = Assembler(self.dof_handler)
        self.K = assembler.assemble_stiffness([self.elem])
        self.M = assembler.assemble_mass([self.elem])

        # Create force vector (apply downward load at free end)
        self.F = np.zeros(self.dof_handler.total_dofs())

    def test_apply_to_stiffness_matrix(self):
        """Test that penalty method modifies stiffness matrix correctly"""
        bc = BCHandler()
        bc.fix_node(self.node1.id)

        K_original = self.K.todense().copy()
        F_original = self.F.copy()

        # Apply BCs (returns modified K and F)
        K_mod, F_mod = bc.apply_to_system(self.K, self.F, self.dof_handler)

        K_modified = K_mod.todense()

        # Check that fixed DOF diagonals are much larger
        for i in range(6):  # First 6 DOFs are fixed
            assert K_modified[i, i] > K_original[i, i] * 1e10

    def test_system_remains_solvable(self):
        """Test that system remains solvable after BC application"""
        bc = BCHandler()
        bc.fix_node(self.node1.id)

        # Apply BCs (returns modified K and F)
        K_mod, F_mod = bc.apply_to_system(self.K, self.F, self.dof_handler)

        # System should remain solvable (no NaN in matrix)
        K_dense = K_mod.todense()
        assert not np.any(np.isnan(K_dense))

        # Check that matrix is not singular (det != 0)
        # For a well-conditioned system, we can solve it
        try:
            u = np.linalg.solve(K_dense, F_mod)
            # If solve succeeds, system is solvable
            assert u.shape == (12,)
        except np.linalg.LinAlgError:
            pytest.fail("System became singular after BC application")

    def test_fixed_dofs_result_in_zero_displacement(self):
        """Test that fixed DOFs result in zero displacement"""
        bc = BCHandler()
        bc.fix_node(self.node1.id)

        # Apply downward load at free end (global DOF 7 = UY at node 2)
        self.F[7] = -10.0  # kN

        # Apply BCs (returns modified K and F)
        K_mod, F_mod = bc.apply_to_system(self.K, self.F, self.dof_handler)

        # Solve system
        K_dense = K_mod.todense()
        u = np.linalg.solve(K_dense, F_mod)

        # Check that fixed DOFs have near-zero displacement
        for i in range(6):  # First 6 DOFs are fixed
            assert abs(u[i]) < 1e-10

    def test_prescribed_displacement(self):
        """Test prescribed displacement (non-zero value)"""
        bc = BCHandler()
        # Fix node 1 with prescribed displacement in UX
        bc.add_fixed_dof(self.node1.id, DOFIndex.UX, 0.01)  # 10mm prescribed
        # Fix other DOFs to zero
        for dof in [DOFIndex.UY, DOFIndex.UZ, DOFIndex.RX, DOFIndex.RY, DOFIndex.RZ]:
            bc.add_fixed_dof(self.node1.id, dof, 0.0)

        # Apply BCs (returns modified K and F)
        K_mod, F_mod = bc.apply_to_system(self.K, self.F, self.dof_handler)

        # Solve system
        K_dense = K_mod.todense()
        u = np.linalg.solve(K_dense, F_mod)

        # Check that prescribed DOF has the prescribed value
        # (within tolerance due to penalty method approximation)
        assert abs(u[0] - 0.01) < 1e-6


class TestSimplySupportedBeam:
    """Test simply supported beam with pin supports at both ends"""

    def setup_method(self):
        """Set up a simply supported beam"""
        # Create nodes
        self.registry = NodeRegistry()
        self.node1 = self.registry.get_or_create_node(0.0, 0.0, 0.0)  # Pin support
        self.node2 = self.registry.get_or_create_node(6.0, 0.0, 0.0)  # Pin support

        # Create material and section
        self.mat = Material(1, "Steel", E=200e6, nu=0.3, rho=7.85e-9)
        self.sec = Section(1, "W200x46.1", A=5880e-6, Iy=45.5e-6, Iz=15.3e-6, J=213e-9)

        # Create beam element
        self.elem = BeamElement(1, self.node1, self.node2, self.mat, self.sec)

        # Number DOFs
        self.dof_handler = DOFHandler()
        self.dof_handler.number_dofs(self.registry)

        # Assemble system matrices
        assembler = Assembler(self.dof_handler)
        self.K = assembler.assemble_stiffness([self.elem])

        # Create force vector
        self.F = np.zeros(self.dof_handler.total_dofs())

    def test_simply_supported_beam_bcs(self):
        """Test simply supported beam boundary conditions"""
        bc = BCHandler()
        bc.pin_node(self.node1.id)  # Pin at left end
        bc.pin_node(self.node2.id)  # Pin at right end

        # Check that 6 DOFs are fixed (3 at each node)
        assert bc.num_fixed_dofs() == 6

        # Get global DOFs
        global_dofs = bc.get_fixed_global_dofs(self.dof_handler)
        assert len(global_dofs) == 6
        # Expect: UX, UY, UZ at node 1 (0,1,2) and UX, UY, UZ at node 2 (6,7,8)
        assert set(global_dofs) == {0, 1, 2, 6, 7, 8}


class TestWarpingBoundaryConditions:
    """Test boundary conditions with warping DOF"""

    def setup_method(self):
        """Set up a model with warping DOF"""
        # Create nodes with warping DOF
        self.registry = NodeRegistry()
        self.node1 = self.registry.get_or_create_node(0.0, 0.0, 0.0)
        self.node2 = self.registry.get_or_create_node(6.0, 0.0, 0.0)

        self.node1.enable_warping_dof()
        self.node2.enable_warping_dof()

        # Create material and section
        self.mat = Material(1, "Steel", E=200e6, nu=0.3, rho=7.85e-9)
        self.sec = Section(1, "I-section", A=5880e-6, Iy=45.5e-6, Iz=15.3e-6, J=213e-9)
        self.sec.enable_warping(Iw=2.0e-9, omega_max=0.1)

        # Create warping element
        config = BeamConfig()
        config.include_warping = True
        self.elem = BeamElement(1, self.node1, self.node2, self.mat, self.sec, config)

        # Number DOFs
        self.dof_handler = DOFHandler()
        self.dof_handler.number_dofs(self.registry)

    def test_warping_dof_can_be_fixed(self):
        """Test that warping DOF can be fixed"""
        bc = BCHandler()
        bc.add_fixed_dof(self.node1.id, DOFIndex.WARP, 0.0)

        assert bc.num_fixed_dofs() == 1

        global_dofs = bc.get_fixed_global_dofs(self.dof_handler)
        # Warping DOF is 7th DOF at node 1, so global DOF 6
        assert 6 in global_dofs

    def test_warping_dof_left_free(self):
        """Test that warping DOF can be left free"""
        bc = BCHandler()
        # Fix all standard DOFs but not warping
        bc.fix_node(self.node1.id)

        assert bc.num_fixed_dofs() == 6  # Only standard DOFs

        global_dofs = bc.get_fixed_global_dofs(self.dof_handler)
        # Warping DOF (global 6) should NOT be in list
        assert 6 not in global_dofs

    def test_fork_support_leaves_warping_free(self):
        """Test that fork support leaves warping free"""
        bc = BCHandler()
        bc.fork_support(self.node1.id)

        # Only translations should be fixed (3 DOFs)
        assert bc.num_fixed_dofs() == 3

        global_dofs = bc.get_fixed_global_dofs(self.dof_handler)
        # Should fix UX, UY, UZ (0, 1, 2) but not rotations or warping
        assert set(global_dofs) == {0, 1, 2}

    def test_built_in_with_warping_restrains_warping(self):
        """Test that built-in support with warping restrains warping"""
        bc = BCHandler()
        bc.fix_node_with_warping(self.node1.id)

        # All 7 DOFs should be fixed
        assert bc.num_fixed_dofs() == 7

        global_dofs = bc.get_fixed_global_dofs(self.dof_handler)
        # Should fix all 7 DOFs at node 1 (0-6)
        assert set(global_dofs) == {0, 1, 2, 3, 4, 5, 6}


class TestAcceptanceCriteria:
    """Test all acceptance criteria for Task 3.3"""

    def setup_method(self):
        """Set up a cantilever beam for testing"""
        self.registry = NodeRegistry()
        self.node1 = self.registry.get_or_create_node(0.0, 0.0, 0.0)
        self.node2 = self.registry.get_or_create_node(6.0, 0.0, 0.0)

        self.mat = Material(1, "Steel", E=200e6, nu=0.3, rho=7.85e-9)
        self.sec = Section(1, "W200", A=5880e-6, Iy=45.5e-6, Iz=15.3e-6, J=213e-9)

        self.elem = BeamElement(1, self.node1, self.node2, self.mat, self.sec)

        self.dof_handler = DOFHandler()
        self.dof_handler.number_dofs(self.registry)

        assembler = Assembler(self.dof_handler)
        self.K = assembler.assemble_stiffness([self.elem])
        self.F = np.zeros(self.dof_handler.total_dofs())

    def test_ac1_fixed_dofs_zero_displacement(self):
        """AC1: Fixed DOFs result in zero (or prescribed) displacement"""
        bc = BCHandler()
        bc.fix_node(self.node1.id)

        # Apply load at free end
        self.F[7] = -10.0  # Downward load

        K_mod, F_mod = bc.apply_to_system(self.K, self.F, self.dof_handler)

        K_dense = K_mod.todense()
        u = np.linalg.solve(K_dense, F_mod)

        # Fixed DOFs should be near zero
        for i in range(6):
            assert abs(u[i]) < 1e-10

    def test_ac2_reactions_recoverable(self):
        """AC2: Reactions can be recovered from K * u - F"""
        bc = BCHandler()
        bc.fix_node(self.node1.id)

        # Apply load
        self.F[7] = -10.0

        # Store original K before BC application
        K_original = self.K.copy()

        K_mod, F_mod = bc.apply_to_system(self.K, self.F, self.dof_handler)

        # Solve for displacements
        K_dense = K_mod.todense()
        u = np.linalg.solve(K_dense, F_mod)

        # Compute reactions: R = K_original * u - F_applied
        # (F_applied is external forces only, without penalty modifications)
        F_applied = np.zeros(self.dof_handler.total_dofs())
        F_applied[7] = -10.0

        K_orig_dense = K_original.todense()
        reactions = K_orig_dense @ u - F_applied

        # Reaction at fixed DOFs should be non-zero (supporting the load)
        # Reaction in Y direction at node 1 should oppose applied load
        # reactions is a matrix, so need to flatten or access with [0, i]
        reactions_vec = np.asarray(reactions).flatten()
        assert abs(reactions_vec[1]) > 1e-6  # UY reaction at node 1

    def test_ac3_system_remains_solvable(self):
        """AC3: System remains solvable after BC application"""
        bc = BCHandler()
        bc.fix_node(self.node1.id)

        K_mod, F_mod = bc.apply_to_system(self.K, self.F, self.dof_handler)

        # System should be solvable
        K_dense = K_mod.todense()
        assert not np.any(np.isnan(K_dense))

        # Should be able to solve without error
        u = np.linalg.solve(K_dense, F_mod)
        assert u.shape == (12,)
        assert not np.any(np.isnan(u))

    def test_ac4_warping_dof_can_be_fixed_or_free(self):
        """AC4: Warping DOF (index 6) can be fixed or left free"""
        # Enable warping
        self.node1.enable_warping_dof()
        self.node2.enable_warping_dof()

        dof_handler = DOFHandler()
        dof_handler.number_dofs(self.registry)

        # Test fixing warping
        bc1 = BCHandler()
        bc1.add_fixed_dof(self.node1.id, DOFIndex.WARP, 0.0)
        global_dofs = bc1.get_fixed_global_dofs(dof_handler)
        assert 6 in global_dofs  # Warping DOF is fixed

        # Test leaving warping free
        bc2 = BCHandler()
        bc2.fix_node(self.node1.id)  # Fix standard DOFs only
        global_dofs = bc2.get_fixed_global_dofs(dof_handler)
        assert 6 not in global_dofs  # Warping DOF is free

    def test_ac5_fork_support_leaves_warping_free(self):
        """AC5: Fork support correctly leaves warping free"""
        # Enable warping
        self.node1.enable_warping_dof()

        dof_handler = DOFHandler()
        dof_handler.number_dofs(self.registry)

        bc = BCHandler()
        bc.fork_support(self.node1.id)

        global_dofs = bc.get_fixed_global_dofs(dof_handler)
        # Only translations fixed, not rotations or warping
        assert set(global_dofs) == {0, 1, 2}

    def test_ac6_built_in_with_warping_restrains_warping(self):
        """AC6: Built-in support with warping correctly restrains warping"""
        # Enable warping
        self.node1.enable_warping_dof()

        dof_handler = DOFHandler()
        dof_handler.number_dofs(self.registry)

        bc = BCHandler()
        bc.fix_node_with_warping(self.node1.id)

        global_dofs = bc.get_fixed_global_dofs(dof_handler)
        # All 7 DOFs should be fixed
        assert set(global_dofs) == {0, 1, 2, 3, 4, 5, 6}
