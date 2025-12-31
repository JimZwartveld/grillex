/**
 * @file test_boundary_conditions.cpp
 * @brief C++ tests for BCHandler.apply_to_system functionality
 *
 * These tests were moved from Python because pybind11 cannot handle
 * Eigen::SparseMatrix as input parameters.
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
#include "grillex/solver.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>

using namespace grillex;
using Catch::Matchers::WithinAbs;

/**
 * @brief Helper class to set up a simple cantilever beam model for testing
 */
class CantileverBeamFixture {
public:
    NodeRegistry registry;
    Node* node1;
    Node* node2;
    std::unique_ptr<Material> mat;
    std::unique_ptr<Section> sec;
    std::unique_ptr<BeamElement> elem;
    DOFHandler dof_handler;
    Eigen::SparseMatrix<double> K;
    Eigen::VectorXd F;

    CantileverBeamFixture() {
        // Create nodes
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0);  // Fixed end
        node2 = registry.get_or_create_node(6.0, 0.0, 0.0);  // Free end

        // Create material and section
        mat = std::make_unique<Material>(1, "Steel", 200e6, 0.3, 7.85e-9);
        sec = std::make_unique<Section>(1, "W200x46.1", 5880e-6, 45.5e-6, 15.3e-6, 213e-9);

        // Create beam element
        BeamConfig config;
        elem = std::make_unique<BeamElement>(1, node1, node2, mat.get(), sec.get(), config);

        // Number DOFs
        dof_handler.number_dofs(registry);

        // Assemble stiffness matrix
        Assembler assembler(dof_handler);
        std::vector<BeamElement*> elements = {elem.get()};
        K = assembler.assemble_stiffness(elements);

        // Create zero force vector
        F = Eigen::VectorXd::Zero(dof_handler.total_dofs());
    }
};

TEST_CASE("Penalty method modifies stiffness matrix correctly", "[BCHandler][apply_to_system]") {
    CantileverBeamFixture fixture;

    BCHandler bc;
    bc.fix_node(fixture.node1->id);

    // Store original diagonal values
    Eigen::MatrixXd K_original = Eigen::MatrixXd(fixture.K);

    // Apply BCs
    auto [K_mod, F_mod] = bc.apply_to_system(fixture.K, fixture.F, fixture.dof_handler);

    Eigen::MatrixXd K_modified = Eigen::MatrixXd(K_mod);

    // Check that fixed DOF diagonals are much larger (penalty method)
    for (int i = 0; i < 6; ++i) {  // First 6 DOFs are fixed
        REQUIRE(K_modified(i, i) > K_original(i, i) * 1e10);
    }
}

TEST_CASE("System remains solvable after BC application", "[BCHandler][apply_to_system]") {
    CantileverBeamFixture fixture;

    BCHandler bc;
    bc.fix_node(fixture.node1->id);

    // Apply BCs
    auto [K_mod, F_mod] = bc.apply_to_system(fixture.K, fixture.F, fixture.dof_handler);

    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_mod);

    // Check no NaN in matrix
    REQUIRE(!K_dense.hasNaN());

    // Should be able to solve without error
    Eigen::VectorXd u = K_dense.ldlt().solve(F_mod);
    REQUIRE(u.size() == 12);
    REQUIRE(!u.hasNaN());
}

TEST_CASE("Fixed DOFs result in zero displacement", "[BCHandler][apply_to_system]") {
    CantileverBeamFixture fixture;

    BCHandler bc;
    bc.fix_node(fixture.node1->id);

    // Apply downward load at free end (global DOF 7 = UY at node 2)
    fixture.F[7] = -10.0;  // kN

    // Apply BCs
    auto [K_mod, F_mod] = bc.apply_to_system(fixture.K, fixture.F, fixture.dof_handler);

    // Solve system
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_mod);
    Eigen::VectorXd u = K_dense.ldlt().solve(F_mod);

    // Check that fixed DOFs have near-zero displacement
    for (int i = 0; i < 6; ++i) {  // First 6 DOFs are fixed
        REQUIRE_THAT(u[i], WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("Prescribed displacement is applied correctly", "[BCHandler][apply_to_system]") {
    CantileverBeamFixture fixture;

    BCHandler bc;
    // Fix node 1 with prescribed displacement in UX
    bc.add_fixed_dof(fixture.node1->id, DOFIndex::UX, 0.01);  // 10mm prescribed
    // Fix other DOFs to zero
    bc.add_fixed_dof(fixture.node1->id, DOFIndex::UY, 0.0);
    bc.add_fixed_dof(fixture.node1->id, DOFIndex::UZ, 0.0);
    bc.add_fixed_dof(fixture.node1->id, DOFIndex::RX, 0.0);
    bc.add_fixed_dof(fixture.node1->id, DOFIndex::RY, 0.0);
    bc.add_fixed_dof(fixture.node1->id, DOFIndex::RZ, 0.0);

    // Apply BCs
    auto [K_mod, F_mod] = bc.apply_to_system(fixture.K, fixture.F, fixture.dof_handler);

    // Solve system
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_mod);
    Eigen::VectorXd u = K_dense.ldlt().solve(F_mod);

    // Check that prescribed DOF has the prescribed value
    // (within tolerance due to penalty method approximation)
    REQUIRE_THAT(u[0], WithinAbs(0.01, 1e-6));
}

TEST_CASE("Reactions can be recovered from K * u - F", "[BCHandler][apply_to_system]") {
    CantileverBeamFixture fixture;

    BCHandler bc;
    bc.fix_node(fixture.node1->id);

    // Apply load
    fixture.F[7] = -10.0;

    // Store original K before BC application
    Eigen::SparseMatrix<double> K_original = fixture.K;

    auto [K_mod, F_mod] = bc.apply_to_system(fixture.K, fixture.F, fixture.dof_handler);

    // Solve for displacements
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_mod);
    Eigen::VectorXd u = K_dense.ldlt().solve(F_mod);

    // Compute reactions: R = K_original * u - F_applied
    Eigen::VectorXd F_applied = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    F_applied[7] = -10.0;

    Eigen::VectorXd reactions = K_original * u - F_applied;

    // Reaction at fixed DOFs should be non-zero (supporting the load)
    // Reaction in Y direction at node 1 should oppose applied load
    REQUIRE(std::abs(reactions[1]) > 1e-6);  // UY reaction at node 1
}

// Acceptance Criteria Tests

TEST_CASE("AC1: Fixed DOFs result in zero (or prescribed) displacement", "[BCHandler][acceptance]") {
    CantileverBeamFixture fixture;

    BCHandler bc;
    bc.fix_node(fixture.node1->id);

    // Apply load at free end
    fixture.F[7] = -10.0;  // Downward load

    auto [K_mod, F_mod] = bc.apply_to_system(fixture.K, fixture.F, fixture.dof_handler);

    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_mod);
    Eigen::VectorXd u = K_dense.ldlt().solve(F_mod);

    // Fixed DOFs should be near zero
    for (int i = 0; i < 6; ++i) {
        REQUIRE_THAT(u[i], WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("AC2: Reactions can be recovered from K * u - F", "[BCHandler][acceptance]") {
    CantileverBeamFixture fixture;

    BCHandler bc;
    bc.fix_node(fixture.node1->id);

    // Apply load
    fixture.F[7] = -10.0;

    // Store original K before BC application
    Eigen::SparseMatrix<double> K_original = fixture.K;

    auto [K_mod, F_mod] = bc.apply_to_system(fixture.K, fixture.F, fixture.dof_handler);

    // Solve for displacements
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_mod);
    Eigen::VectorXd u = K_dense.ldlt().solve(F_mod);

    // Compute reactions: R = K_original * u - F_applied
    Eigen::VectorXd F_applied = Eigen::VectorXd::Zero(fixture.dof_handler.total_dofs());
    F_applied[7] = -10.0;

    Eigen::VectorXd reactions = K_original * u - F_applied;

    // Reaction at fixed DOFs should be non-zero (supporting the load)
    REQUIRE(std::abs(reactions[1]) > 1e-6);  // UY reaction at node 1
}

TEST_CASE("AC3: System remains solvable after BC application", "[BCHandler][acceptance]") {
    CantileverBeamFixture fixture;

    BCHandler bc;
    bc.fix_node(fixture.node1->id);

    auto [K_mod, F_mod] = bc.apply_to_system(fixture.K, fixture.F, fixture.dof_handler);

    // System should be solvable
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_mod);
    REQUIRE(!K_dense.hasNaN());

    // Should be able to solve without error
    Eigen::VectorXd u = K_dense.ldlt().solve(F_mod);
    REQUIRE(u.size() == 12);
    REQUIRE(!u.hasNaN());
}
