/**
 * @file test_constraints.cpp
 * @brief C++ tests for ConstraintHandler MPC functionality
 *
 * These tests were moved from Python (test_phase6_constraints.py) because
 * pybind11 cannot handle Eigen::SparseMatrix as input parameters.
 *
 * Tests include:
 * - Equality constraint analysis
 * - Rigid link analysis with kinematics
 * - Full MPC workflow with boundary conditions
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "grillex/node_registry.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/beam_element.hpp"
#include "grillex/dof_handler.hpp"
#include "grillex/assembler.hpp"
#include "grillex/boundary_condition.hpp"
#include "grillex/constraints.hpp"
#include "grillex/solver.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>

using namespace grillex;
using Catch::Matchers::WithinAbs;

// =============================================================================
// Test Fixtures
// =============================================================================

/**
 * @brief Helper to create a test beam with nodes and material
 */
struct TwoNodeBeamFixture {
    NodeRegistry registry;
    Node* n1;
    Node* n2;
    std::unique_ptr<Material> mat;
    std::unique_ptr<Section> sec;
    std::unique_ptr<BeamElement> beam;
    DOFHandler dof_handler;

    TwoNodeBeamFixture(double x1 = 0.0, double y1 = 0.0, double z1 = 0.0,
                       double x2 = 6.0, double y2 = 0.0, double z2 = 0.0)
        : registry(1e-6) {
        n1 = registry.get_or_create_node(x1, y1, z1);
        n2 = registry.get_or_create_node(x2, y2, z2);

        mat = std::make_unique<Material>(1, "Steel", 210e6, 0.3, 7.85e-6);
        sec = std::make_unique<Section>(1, "Test", 0.01, 1e-5, 2e-5, 1.5e-5);

        BeamConfig config;
        beam = std::make_unique<BeamElement>(1, n1, n2, mat.get(), sec.get(), config);

        dof_handler.number_dofs(registry);
    }
};

/**
 * @brief Helper to create a three-node cantilever with slave node
 */
struct CantileverWithSlaveFixture {
    NodeRegistry registry;
    Node* n1;  // Fixed end
    Node* n2;  // Cantilever tip (master)
    Node* n3;  // Slave node (offset from n2)
    std::unique_ptr<Material> mat;
    std::unique_ptr<Section> sec;
    std::unique_ptr<BeamElement> beam;
    DOFHandler dof_handler;

    CantileverWithSlaveFixture(double offset_x = 0.0, double offset_y = 0.0, double offset_z = 0.0)
        : registry(1e-6) {
        n1 = registry.get_or_create_node(0.0, 0.0, 0.0);
        n2 = registry.get_or_create_node(6.0, 0.0, 0.0);
        n3 = registry.get_or_create_node(6.0 + offset_x, offset_y, offset_z);

        mat = std::make_unique<Material>(1, "Steel", 210e6, 0.3, 7.85e-6);
        sec = std::make_unique<Section>(1, "Test", 0.01, 1e-5, 2e-5, 1.5e-5);

        BeamConfig config;
        beam = std::make_unique<BeamElement>(1, n1, n2, mat.get(), sec.get(), config);

        dof_handler.number_dofs(registry);
    }
};

// =============================================================================
// Equality Constraint Analysis Tests
// =============================================================================

TEST_CASE("Equality constraint: two cantilevers with tied tips", "[ConstraintHandler][equality]") {
    /**
     * Test tying two cantilever tips together (from TestEqualityConstraintAnalysis)
     *
     * Two parallel cantilevers, both fixed at one end.
     * Tip DOFs tied together via equality constraints.
     * Apply load to one tip, both should deflect equally.
     */
    NodeRegistry registry(1e-6);

    // Create two parallel cantilevers
    Node* n1 = registry.get_or_create_node(0, 0, 0);
    Node* n2 = registry.get_or_create_node(6, 0, 0);
    Node* n3 = registry.get_or_create_node(0, 1, 0);
    Node* n4 = registry.get_or_create_node(6, 1, 0);

    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "Test", 0.01, 1e-5, 2e-5, 1.5e-5);

    BeamConfig config;
    BeamElement beam1(1, n1, n2, &mat, &sec, config);
    BeamElement beam2(2, n3, n4, &mat, &sec, config);

    DOFHandler dof_handler;
    dof_handler.number_dofs(registry);

    Assembler assembler(dof_handler);
    std::vector<BeamElement*> elements = {&beam1, &beam2};
    auto K = assembler.assemble_stiffness(elements);

    // Force vector: load on tip of first cantilever
    Eigen::VectorXd F = Eigen::VectorXd::Zero(dof_handler.total_dofs());
    int n2_uz = dof_handler.get_global_dof(n2->id, DOFIndex::UZ);
    F[n2_uz] = -10.0;  // 10 kN downward

    // Constraints: tie n2 and n4 in UZ direction
    ConstraintHandler ch;
    ch.add_equality_constraint(n4->id, DOFIndex::UZ, n2->id, DOFIndex::UZ);

    // Apply boundary conditions first, then reduce
    BCHandler bc;
    bc.fix_node(n1->id);
    bc.fix_node(n3->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, dof_handler);

    // Reduce with constraints
    auto reduced = ch.reduce_system(K_bc, F_bc, dof_handler);

    // Solve
    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(reduced.K_reduced, reduced.F_reduced);

    // Expand
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, reduced.T);

    // Check that tied DOFs have same displacement
    int n4_uz = dof_handler.get_global_dof(n4->id, DOFIndex::UZ);
    REQUIRE_THAT(u_full[n2_uz], WithinAbs(u_full[n4_uz], 1e-10));
}

// =============================================================================
// Rigid Link Analysis Tests
// =============================================================================

TEST_CASE("Rigid link with offset: rotation coupling", "[ConstraintHandler][rigid_link]") {
    /**
     * Test rigid link with offset - verifies rotation coupling
     * (from TestRigidLinkAnalysis.test_rigid_link_with_offset)
     *
     * Create a structure where rotation at master causes
     * translation at slave due to rigid link offset.
     */
    CantileverWithSlaveFixture fixture(0.0, 2.0, 0.0);  // Slave offset by 2 in Y

    Assembler assembler(fixture.dof_handler);
    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = assembler.assemble_stiffness(elements);

    // Apply moment at tip to cause rotation
    Eigen::VectorXd F = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    int n2_rz = fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RZ);
    F[n2_rz] = 10.0;  // Moment about Z

    // Rigid link: n3 is offset from n2 by (0, 2, 0)
    ConstraintHandler ch;
    ch.add_rigid_link(fixture.n3->id, fixture.n2->id, Eigen::Vector3d(0.0, 2.0, 0.0));

    // Apply BCs and reduce
    BCHandler bc;
    bc.fix_node(fixture.n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, fixture.dof_handler);
    auto reduced = ch.reduce_system(K_bc, F_bc, fixture.dof_handler);

    // Solve
    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(reduced.K_reduced, reduced.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, reduced.T);

    // Get displacements
    double n2_ux = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UX)];
    double n2_rz_disp = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RZ)];
    double n3_ux = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::UX)];
    double n3_rz_disp = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::RZ)];

    // Check rotation is same
    REQUIRE_THAT(n3_rz_disp, WithinAbs(n2_rz_disp, 1e-10));

    // Check translation coupling: n3_ux = n2_ux + 2 * n2_rz
    // (offset ry=2, so u_sx = u_mx + ry * theta_mz)
    double expected_n3_ux = n2_ux + 2.0 * n2_rz_disp;
    REQUIRE_THAT(n3_ux, WithinAbs(expected_n3_ux, 1e-10));
}

// =============================================================================
// Task 6.2 Rigid Link Kinematics Tests
// =============================================================================

TEST_CASE("AC1: Slave moves with master translation", "[ConstraintHandler][kinematics][acceptance]") {
    /**
     * AC1: Slave node moves correctly with master (translation)
     * When master translates, slave translates the same amount.
     */
    CantileverWithSlaveFixture fixture(0.0, 1.0, 0.0);  // Slave offset by 1 in Y

    Assembler assembler(fixture.dof_handler);
    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = assembler.assemble_stiffness(elements);

    // Apply vertical force at n2 (master)
    Eigen::VectorXd F = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    F[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UZ)] = -10.0;

    // Rigid link: n3 tied to n2
    ConstraintHandler ch;
    ch.add_rigid_link(fixture.n3->id, fixture.n2->id, Eigen::Vector3d(0.0, 1.0, 0.0));

    BCHandler bc;
    bc.fix_node(fixture.n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, fixture.dof_handler);

    auto reduced = ch.reduce_system(K_bc, F_bc, fixture.dof_handler);
    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(reduced.K_reduced, reduced.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, reduced.T);

    // Master and slave should have same UZ displacement (same Z translation)
    // With offset in Y only, uz should be coupled via -ry*theta_mx
    double n2_uz = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UZ)];
    double n2_rx = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RX)];
    double n3_uz = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::UZ)];

    // u_sz = u_mz - ry * theta_mx (ry = 1.0)
    double expected_n3_uz = n2_uz - 1.0 * n2_rx;
    REQUIRE_THAT(n3_uz, WithinAbs(expected_n3_uz, 1e-10));
}

TEST_CASE("AC2: Rotation produces translation at slave", "[ConstraintHandler][kinematics][acceptance]") {
    /**
     * AC2: Rotation at master produces translation at slave
     *
     * With offset r = [0, L, 0], rotation theta_z at master produces:
     * u_sx = L * theta_z (translation in X due to rotation about Z)
     */
    double L = 2.0;
    CantileverWithSlaveFixture fixture(0.0, L, 0.0);  // Slave offset by L in Y

    Assembler assembler(fixture.dof_handler);
    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = assembler.assemble_stiffness(elements);

    // Apply moment about Z at master (n2)
    Eigen::VectorXd F = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    F[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RZ)] = 10.0;

    // Rigid link: offset in Y direction
    ConstraintHandler ch;
    ch.add_rigid_link(fixture.n3->id, fixture.n2->id, Eigen::Vector3d(0.0, L, 0.0));

    BCHandler bc;
    bc.fix_node(fixture.n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, fixture.dof_handler);

    auto reduced = ch.reduce_system(K_bc, F_bc, fixture.dof_handler);
    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(reduced.K_reduced, reduced.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, reduced.T);

    // Get displacements
    double n2_ux = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UX)];
    double n2_rz = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RZ)];
    double n3_ux = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::UX)];

    // From rigid link kinematics: u_sx = u_mx + ry * theta_mz
    double expected_n3_ux = n2_ux + L * n2_rz;
    REQUIRE_THAT(n3_ux, WithinAbs(expected_n3_ux, 1e-10));

    // Rotation should be non-zero (moment applied)
    REQUIRE(std::abs(n2_rz) > 1e-10);
}

TEST_CASE("AC3: Force transfer through rigid link", "[ConstraintHandler][kinematics][acceptance]") {
    /**
     * AC3: Forces transfer correctly through rigid link
     *
     * Apply force to slave node, verify master responds correctly.
     */
    double ry = 0.5;
    CantileverWithSlaveFixture fixture(0.0, ry, 0.0);

    Assembler assembler(fixture.dof_handler);
    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = assembler.assemble_stiffness(elements);

    // Apply force to slave node (not connected to beam directly)
    Eigen::VectorXd F = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    F[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::UZ)] = -10.0;

    // Rigid link transfers force from slave to master
    ConstraintHandler ch;
    ch.add_rigid_link(fixture.n3->id, fixture.n2->id, Eigen::Vector3d(0.0, ry, 0.0));

    BCHandler bc;
    bc.fix_node(fixture.n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, fixture.dof_handler);

    auto reduced = ch.reduce_system(K_bc, F_bc, fixture.dof_handler);
    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(reduced.K_reduced, reduced.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, reduced.T);

    // Force was transferred: master (n2) should have displacement
    double n2_uz = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UZ)];
    REQUIRE(std::abs(n2_uz) > 1e-10);

    // Check rigid body kinematics: u_sz = u_mz - ry*theta_mx
    double n2_rx = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RX)];
    double n3_uz = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::UZ)];

    double expected_n3_uz = n2_uz - ry * n2_rx;
    REQUIRE_THAT(n3_uz, WithinAbs(expected_n3_uz, 1e-10));
}

// =============================================================================
// Task 6.1 Acceptance Criteria Tests
// =============================================================================

TEST_CASE("AC1: Simple equality constraints work", "[ConstraintHandler][acceptance]") {
    /**
     * AC1: Simple equality constraints work
     * Verify that equality constraint u_slave = u_master is enforced.
     */
    NodeRegistry registry(1e-6);

    Node* n1 = registry.get_or_create_node(0, 0, 0);
    Node* n2 = registry.get_or_create_node(3, 0, 0);
    Node* n3 = registry.get_or_create_node(6, 0, 0);

    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "Test", 0.01, 1e-5, 2e-5, 1.5e-5);

    BeamConfig config;
    BeamElement beam1(1, n1, n2, &mat, &sec, config);
    BeamElement beam2(2, n2, n3, &mat, &sec, config);

    DOFHandler dof_handler;
    dof_handler.number_dofs(registry);

    Assembler assembler(dof_handler);
    std::vector<BeamElement*> elements = {&beam1, &beam2};
    auto K = assembler.assemble_stiffness(elements);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(dof_handler.total_dofs());
    F[dof_handler.get_global_dof(n3->id, DOFIndex::UZ)] = -10.0;

    // Constraint: tie n2 UY to n1 UY
    ConstraintHandler ch;
    ch.add_equality_constraint(n2->id, DOFIndex::UY, n1->id, DOFIndex::UY);

    BCHandler bc;
    bc.fix_node(n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, dof_handler);

    auto reduced = ch.reduce_system(K_bc, F_bc, dof_handler);

    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(reduced.K_reduced, reduced.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, reduced.T);

    // Since n1 is fixed (UY=0), n2 UY should also be 0 due to constraint
    int n1_uy = dof_handler.get_global_dof(n1->id, DOFIndex::UY);
    int n2_uy = dof_handler.get_global_dof(n2->id, DOFIndex::UY);
    REQUIRE_THAT(u_full[n1_uy], WithinAbs(u_full[n2_uy], 1e-10));
}

TEST_CASE("AC2: Rigid links transfer forces correctly", "[ConstraintHandler][acceptance]") {
    /**
     * AC2: Rigid links transfer forces correctly
     * Apply force to slave node, verify it's transferred to master.
     */
    NodeRegistry registry(1e-6);

    Node* n1 = registry.get_or_create_node(0, 0, 0);
    Node* n2 = registry.get_or_create_node(3, 0, 0);
    Node* n3 = registry.get_or_create_node(3.001, 0, 0);  // Slightly offset for unique ID

    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "Test", 0.01, 1e-5, 2e-5, 1.5e-5);

    BeamConfig config;
    BeamElement beam(1, n1, n2, &mat, &sec, config);

    DOFHandler dof_handler;
    dof_handler.number_dofs(registry);

    Assembler assembler(dof_handler);
    std::vector<BeamElement*> elements = {&beam};
    auto K = assembler.assemble_stiffness(elements);

    // Apply load to slave node (n3)
    Eigen::VectorXd F = Eigen::VectorXd::Zero(dof_handler.total_dofs());
    int n3_uz = dof_handler.get_global_dof(n3->id, DOFIndex::UZ);
    F[n3_uz] = -10.0;

    // Rigid link: n3 tied to n2 with small offset
    ConstraintHandler ch;
    ch.add_rigid_link(n3->id, n2->id, Eigen::Vector3d(0.001, 0.0, 0.0));

    BCHandler bc;
    bc.fix_node(n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, dof_handler);

    auto reduced = ch.reduce_system(K_bc, F_bc, dof_handler);

    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(reduced.K_reduced, reduced.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, reduced.T);

    // Force on slave should cause displacement at master
    int n2_uz = dof_handler.get_global_dof(n2->id, DOFIndex::UZ);
    REQUIRE(std::abs(u_full[n2_uz]) > 1e-10);
}

// =============================================================================
// Task 6.3: Apply MPC to Global System Tests
// =============================================================================

TEST_CASE("AC2: Full displacements recovered correctly (equality)", "[ConstraintHandler][recovery][acceptance]") {
    /**
     * AC2: Full displacements are recovered correctly (equality)
     * u_full = T * u_reduced should give correct slave values.
     */
    TwoNodeBeamFixture fixture;

    Assembler assembler(fixture.dof_handler);
    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = assembler.assemble_stiffness(elements);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    F[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UZ)] = -10.0;

    // Tie UX together
    ConstraintHandler ch;
    ch.add_equality_constraint(fixture.n2->id, DOFIndex::UX, fixture.n1->id, DOFIndex::UX);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, fixture.dof_handler);

    auto result = ch.reduce_system(K_bc, F_bc, fixture.dof_handler);

    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(result.K_reduced, result.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, result.T);

    // Verify u_full = T * u_reduced
    Eigen::VectorXd u_full_check = result.T * u_reduced;
    REQUIRE((u_full - u_full_check).norm() < 1e-10);

    // Verify dimensions
    REQUIRE(u_full.size() == fixture.dof_handler.total_dofs());
}

TEST_CASE("AC2: Full displacements recovered correctly (rigid link)", "[ConstraintHandler][recovery][acceptance]") {
    /**
     * AC2: Full displacements are recovered correctly (rigid link)
     * Slave DOFs should follow rigid body kinematics from master.
     */
    double ry = 2.0;
    CantileverWithSlaveFixture fixture(0.0, ry, 0.0);

    Assembler assembler(fixture.dof_handler);
    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = assembler.assemble_stiffness(elements);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    F[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UZ)] = -10.0;

    // Rigid link with Y offset
    ConstraintHandler ch;
    ch.add_rigid_link(fixture.n3->id, fixture.n2->id, Eigen::Vector3d(0.0, ry, 0.0));

    BCHandler bc;
    bc.fix_node(fixture.n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, fixture.dof_handler);

    auto result = ch.reduce_system(K_bc, F_bc, fixture.dof_handler);

    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(result.K_reduced, result.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, result.T);

    // Get master DOFs
    double m_ux = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UX)];
    double m_uy = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UY)];
    double m_uz = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UZ)];
    double m_rx = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RX)];
    double m_ry = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RY)];
    double m_rz = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RZ)];

    // Get slave DOFs
    double s_ux = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::UX)];
    double s_uy = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::UY)];
    double s_uz = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::UZ)];
    double s_rx = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::RX)];
    double s_ry_disp = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::RY)];
    double s_rz = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, DOFIndex::RZ)];

    // Verify rigid body kinematics (offset r = [0, ry, 0])
    // u_sx = u_mx + ry*theta_mz
    REQUIRE_THAT(s_ux, WithinAbs(m_ux + ry * m_rz, 1e-10));
    // u_sy = u_my - rx*theta_mz = u_my (rx=0)
    REQUIRE_THAT(s_uy, WithinAbs(m_uy, 1e-10));
    // u_sz = u_mz - ry*theta_mx
    REQUIRE_THAT(s_uz, WithinAbs(m_uz - ry * m_rx, 1e-10));
    // Rotations equal
    REQUIRE_THAT(s_rx, WithinAbs(m_rx, 1e-10));
    REQUIRE_THAT(s_ry_disp, WithinAbs(m_ry, 1e-10));
    REQUIRE_THAT(s_rz, WithinAbs(m_rz, 1e-10));
}

TEST_CASE("AC3: Equality constraints satisfied", "[ConstraintHandler][constraint_satisfaction][acceptance]") {
    /**
     * AC3: Constrained DOFs satisfy constraint equations (equality)
     * For equality constraint: u_slave = u_master
     */
    NodeRegistry registry(1e-6);

    Node* n1 = registry.get_or_create_node(0, 0, 0);
    Node* n2 = registry.get_or_create_node(3, 0, 0);
    Node* n3 = registry.get_or_create_node(6, 0, 0);

    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "Test", 0.01, 1e-5, 2e-5, 1.5e-5);

    BeamConfig config;
    BeamElement beam1(1, n1, n2, &mat, &sec, config);
    BeamElement beam2(2, n2, n3, &mat, &sec, config);

    DOFHandler dof_handler;
    dof_handler.number_dofs(registry);

    Assembler assembler(dof_handler);
    std::vector<BeamElement*> elements = {&beam1, &beam2};
    auto K = assembler.assemble_stiffness(elements);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(dof_handler.total_dofs());
    F[dof_handler.get_global_dof(n3->id, DOFIndex::UZ)] = -20.0;

    // Multiple equality constraints
    ConstraintHandler ch;
    ch.add_equality_constraint(n2->id, DOFIndex::RX, n1->id, DOFIndex::RX);
    ch.add_equality_constraint(n3->id, DOFIndex::RY, n2->id, DOFIndex::RY);

    BCHandler bc;
    bc.fix_node(n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, dof_handler);

    auto result = ch.reduce_system(K_bc, F_bc, dof_handler);

    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(result.K_reduced, result.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, result.T);

    // Verify constraint 1: n2.RX = n1.RX
    double n1_rx = u_full[dof_handler.get_global_dof(n1->id, DOFIndex::RX)];
    double n2_rx = u_full[dof_handler.get_global_dof(n2->id, DOFIndex::RX)];
    REQUIRE_THAT(n2_rx, WithinAbs(n1_rx, 1e-10));

    // Verify constraint 2: n3.RY = n2.RY
    double n2_ry = u_full[dof_handler.get_global_dof(n2->id, DOFIndex::RY)];
    double n3_ry = u_full[dof_handler.get_global_dof(n3->id, DOFIndex::RY)];
    REQUIRE_THAT(n3_ry, WithinAbs(n2_ry, 1e-10));
}

TEST_CASE("AC3: Rigid link constraints satisfied", "[ConstraintHandler][constraint_satisfaction][acceptance]") {
    /**
     * AC3: Constrained DOFs satisfy constraint equations (rigid link)
     * All 6 DOFs should follow rigid body kinematics.
     */
    Eigen::Vector3d offset(1.0, 2.0, 0.5);
    CantileverWithSlaveFixture fixture(offset.x(), offset.y(), offset.z());

    Assembler assembler(fixture.dof_handler);
    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = assembler.assemble_stiffness(elements);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    // Apply force and moment to cause all DOF types to be non-zero
    F[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::UZ)] = -10.0;
    F[fixture.dof_handler.get_global_dof(fixture.n2->id, DOFIndex::RZ)] = 5.0;

    ConstraintHandler ch;
    ch.add_rigid_link(fixture.n3->id, fixture.n2->id, offset);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, fixture.dof_handler);

    auto result = ch.reduce_system(K_bc, F_bc, fixture.dof_handler);

    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(result.K_reduced, result.F_reduced);
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, result.T);

    // Get master and slave displacements
    Eigen::VectorXd master(6), slave(6);
    for (int i = 0; i < 6; ++i) {
        master[i] = u_full[fixture.dof_handler.get_global_dof(fixture.n2->id, i)];
        slave[i] = u_full[fixture.dof_handler.get_global_dof(fixture.n3->id, i)];
    }

    // Compute expected slave using transformation block
    RigidLink rl(fixture.n3->id, fixture.n2->id, offset);
    Eigen::Matrix<double, 6, 6> T_block = rl.transformation_block_6x6();
    Eigen::VectorXd expected_slave = T_block * master;

    REQUIRE((slave - expected_slave).norm() < 1e-10);
}

TEST_CASE("Complete MPC workflow", "[ConstraintHandler][workflow][acceptance]") {
    /**
     * Test complete workflow: Model -> Assemble -> MPC -> Solve -> Results
     */
    NodeRegistry registry(1e-6);

    // Create a simple frame with 4 nodes
    Node* n1 = registry.get_or_create_node(0, 0, 0);  // Fixed support
    Node* n2 = registry.get_or_create_node(4, 0, 0);  // Joint
    Node* n3 = registry.get_or_create_node(4, 3, 0);  // Free end (will be loaded)
    Node* n4 = registry.get_or_create_node(4, 0, 2);  // Slave node (rigid link to n2)

    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "IPE200", 0.00285, 1.94e-5, 1.42e-6, 6.98e-8);

    BeamConfig config;
    BeamElement beam1(1, n1, n2, &mat, &sec, config);  // Horizontal beam
    BeamElement beam2(2, n2, n3, &mat, &sec, config);  // Vertical beam

    // Step 1: DOF numbering
    DOFHandler dof_handler;
    dof_handler.number_dofs(registry);
    int n_full = dof_handler.total_dofs();

    // Step 2: Assemble global system
    Assembler assembler(dof_handler);
    std::vector<BeamElement*> elements = {&beam1, &beam2};
    auto K = assembler.assemble_stiffness(elements);

    // Step 3: Apply loads
    Eigen::VectorXd F = Eigen::VectorXd::Zero(n_full);
    F[dof_handler.get_global_dof(n3->id, DOFIndex::UX)] = 5.0;  // Horizontal force

    // Step 4: Define MPC constraints
    ConstraintHandler ch;
    // Rigid link from n4 to n2 (offset in Z)
    ch.add_rigid_link(n4->id, n2->id, Eigen::Vector3d(0.0, 0.0, 2.0));
    // Equality constraint: tie n3.RZ to n2.RZ
    ch.add_equality_constraint(n3->id, DOFIndex::RZ, n2->id, DOFIndex::RZ);

    // Step 5: Apply boundary conditions
    BCHandler bc;
    bc.fix_node(n1->id);
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, dof_handler);

    // Step 6: Reduce system with MPC
    auto result = ch.reduce_system(K_bc, F_bc, dof_handler);

    // Verify reduction
    int n_constraints = 6 + 1;  // 6 from rigid link + 1 from equality
    REQUIRE(result.n_reduced == n_full - n_constraints);

    // Step 7: Solve reduced system
    LinearSolver solver;
    Eigen::VectorXd u_reduced = solver.solve(result.K_reduced, result.F_reduced);

    // Step 8: Expand to full displacements
    Eigen::VectorXd u_full = ch.expand_displacements(u_reduced, result.T);

    // Verify results
    // 1. Fixed node should have zero displacements
    for (int dof = 0; dof < 6; ++dof) {
        double n1_dof = u_full[dof_handler.get_global_dof(n1->id, dof)];
        REQUIRE_THAT(n1_dof, WithinAbs(0.0, 1e-10));
    }

    // 2. Equality constraint satisfied
    double n2_rz = u_full[dof_handler.get_global_dof(n2->id, DOFIndex::RZ)];
    double n3_rz = u_full[dof_handler.get_global_dof(n3->id, DOFIndex::RZ)];
    REQUIRE_THAT(n3_rz, WithinAbs(n2_rz, 1e-10));

    // 3. Rigid link constraints satisfied (check one example)
    double n2_uz = u_full[dof_handler.get_global_dof(n2->id, DOFIndex::UZ)];
    double n4_uz = u_full[dof_handler.get_global_dof(n4->id, DOFIndex::UZ)];
    // With offset in Z only and no rotation coupling in z-direction:
    // u_sz = u_mz - ry*theta_mx + rx*theta_my (ry=0, rx=0 for this offset)
    REQUIRE_THAT(n4_uz, WithinAbs(n2_uz, 1e-10));

    // 4. Free end should have displacement (load applied)
    double n3_ux = u_full[dof_handler.get_global_dof(n3->id, DOFIndex::UX)];
    REQUIRE(std::abs(n3_ux) > 1e-10);
}
