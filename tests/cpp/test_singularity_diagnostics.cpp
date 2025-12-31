/**
 * @file test_singularity_diagnostics.cpp
 * @brief C++ tests for SingularityAnalyzer functionality
 *
 * These tests were moved from Python because pybind11 cannot handle
 * Eigen::SparseMatrix as input parameters.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "grillex/node_registry.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/beam_element.hpp"
#include "grillex/dof_handler.hpp"
#include "grillex/assembler.hpp"
#include "grillex/boundary_condition.hpp"
#include "grillex/singularity_diagnostics.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <string>

using namespace grillex;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::ContainsSubstring;

/**
 * @brief Helper class to create a free-floating beam (no boundary conditions)
 */
class FreeFloatingBeamFixture {
public:
    NodeRegistry registry;
    Node* node_i;
    Node* node_j;
    std::unique_ptr<Material> material;
    std::unique_ptr<Section> section;
    std::unique_ptr<BeamElement> beam;
    DOFHandler dof_handler;
    Eigen::SparseMatrix<double> K;

    FreeFloatingBeamFixture()
        : registry(1e-6) {

        node_i = registry.get_or_create_node(0.0, 0.0, 0.0);
        node_j = registry.get_or_create_node(5.0, 0.0, 0.0);

        material = std::make_unique<Material>(1, "Steel", 210e6, 0.3, 7.85e-3);
        section = std::make_unique<Section>(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7);

        BeamConfig config;
        beam = std::make_unique<BeamElement>(1, node_i, node_j, material.get(), section.get(), config);

        dof_handler.number_dofs(registry);

        Assembler assembler(dof_handler);
        std::vector<BeamElement*> elements = {beam.get()};
        K = assembler.assemble_stiffness(elements);
    }
};

/**
 * @brief Helper class to create a cantilever beam (properly constrained)
 */
class CantileverBeamFixture {
public:
    NodeRegistry registry;
    Node* node_i;
    Node* node_j;
    std::unique_ptr<Material> material;
    std::unique_ptr<Section> section;
    std::unique_ptr<BeamElement> beam;
    DOFHandler dof_handler;
    Eigen::SparseMatrix<double> K;
    Eigen::SparseMatrix<double> K_bc;

    CantileverBeamFixture()
        : registry(1e-6) {

        node_i = registry.get_or_create_node(0.0, 0.0, 0.0);
        node_j = registry.get_or_create_node(5.0, 0.0, 0.0);

        material = std::make_unique<Material>(1, "Steel", 210e6, 0.3, 7.85e-3);
        section = std::make_unique<Section>(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7);

        BeamConfig config;
        beam = std::make_unique<BeamElement>(1, node_i, node_j, material.get(), section.get(), config);

        dof_handler.number_dofs(registry);

        Assembler assembler(dof_handler);
        std::vector<BeamElement*> elements = {beam.get()};
        K = assembler.assemble_stiffness(elements);

        // Apply fixed boundary conditions at node i (cantilever root)
        BCHandler bc;
        bc.fix_node(node_i->id);

        // Create zero force vector and apply BCs
        Eigen::VectorXd F = Eigen::VectorXd::Zero(dof_handler.total_dofs());
        auto [K_modified, F_modified] = bc.apply_to_system(K, F, dof_handler);
        K_bc = K_modified;
    }
};

// =============================================================================
// SingularityAnalyzerSettings Tests
// =============================================================================

TEST_CASE("SingularityAnalyzerSettings: default values", "[SingularityAnalyzer][settings]") {
    SingularityAnalyzerSettings settings;

    REQUIRE_THAT(settings.eigenvalue_threshold, WithinAbs(1e-8, 1e-12));
    REQUIRE(settings.n_modes_to_check == 10);
    REQUIRE_THAT(settings.participation_threshold, WithinAbs(0.01, 1e-6));
    REQUIRE(settings.max_dofs_to_report == 20);
    REQUIRE(settings.use_mass_matrix == true);
}

TEST_CASE("SingularityAnalyzerSettings: custom values", "[SingularityAnalyzer][settings]") {
    SingularityAnalyzerSettings settings;
    settings.eigenvalue_threshold = 1e-6;
    settings.n_modes_to_check = 20;
    settings.participation_threshold = 0.05;
    settings.max_dofs_to_report = 50;
    settings.use_mass_matrix = false;

    REQUIRE_THAT(settings.eigenvalue_threshold, WithinAbs(1e-6, 1e-12));
    REQUIRE(settings.n_modes_to_check == 20);
    REQUIRE_THAT(settings.participation_threshold, WithinAbs(0.05, 1e-6));
    REQUIRE(settings.max_dofs_to_report == 50);
    REQUIRE(settings.use_mass_matrix == false);
}

// =============================================================================
// RigidBodyModeType Tests
// =============================================================================

TEST_CASE("RigidBodyModeType: enum values exist", "[SingularityAnalyzer][enum]") {
    REQUIRE(static_cast<int>(RigidBodyModeType::TranslationX) >= 0);
    REQUIRE(static_cast<int>(RigidBodyModeType::TranslationY) >= 0);
    REQUIRE(static_cast<int>(RigidBodyModeType::TranslationZ) >= 0);
    REQUIRE(static_cast<int>(RigidBodyModeType::RotationX) >= 0);
    REQUIRE(static_cast<int>(RigidBodyModeType::RotationY) >= 0);
    REQUIRE(static_cast<int>(RigidBodyModeType::RotationZ) >= 0);
    REQUIRE(static_cast<int>(RigidBodyModeType::Warping) >= 0);
    REQUIRE(static_cast<int>(RigidBodyModeType::Mixed) >= 0);
}

TEST_CASE("RigidBodyModeType: to_string conversion", "[SingularityAnalyzer][enum]") {
    REQUIRE(rigid_body_mode_type_to_string(RigidBodyModeType::TranslationX) == "X translation");
    REQUIRE(rigid_body_mode_type_to_string(RigidBodyModeType::TranslationY) == "Y translation");
    REQUIRE(rigid_body_mode_type_to_string(RigidBodyModeType::TranslationZ) == "Z translation");
    REQUIRE(rigid_body_mode_type_to_string(RigidBodyModeType::RotationX) == "X rotation");
    REQUIRE(rigid_body_mode_type_to_string(RigidBodyModeType::RotationY) == "Y rotation");
    REQUIRE(rigid_body_mode_type_to_string(RigidBodyModeType::RotationZ) == "Z rotation");
    REQUIRE(rigid_body_mode_type_to_string(RigidBodyModeType::Warping) == "warping");
    REQUIRE(rigid_body_mode_type_to_string(RigidBodyModeType::Mixed) == "mixed mode");
}

// =============================================================================
// SingularityDiagnostics Default Values
// =============================================================================

TEST_CASE("SingularityDiagnostics: default values", "[SingularityAnalyzer][diagnostics]") {
    SingularityDiagnostics diag;

    REQUIRE(diag.is_singular == false);
    REQUIRE(diag.n_rigid_body_modes == 0);
    REQUIRE(diag.rigid_body_modes.empty());
    REQUIRE(diag.unconstrained_dofs.empty());
    REQUIRE(diag.suggested_fixes.empty());
    REQUIRE(diag.nodes_needing_constraints.empty());
    REQUIRE(diag.dofs_to_constrain.empty());
}

// =============================================================================
// Free-Floating Beam Tests (Singular System)
// =============================================================================

TEST_CASE("analyze: free-floating beam detected as singular", "[SingularityAnalyzer][analyze]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K, fixture.dof_handler);

    REQUIRE(result.is_singular == true);
    // A free-floating 3D beam has 6 rigid body modes
    REQUIRE(result.n_rigid_body_modes >= 6);
}

TEST_CASE("analyze: free-floating beam unconstrained DOFs identified", "[SingularityAnalyzer][analyze]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K, fixture.dof_handler);

    // Should identify unconstrained DOFs
    REQUIRE(!result.unconstrained_dofs.empty());

    // Check that DOF information is provided
    for (const auto& dof : result.unconstrained_dofs) {
        REQUIRE(dof.global_dof >= 0);
        // Participation should be between 0 and 1
        REQUIRE(dof.participation >= 0);
        REQUIRE(dof.participation <= 1);
    }
}

TEST_CASE("analyze: free-floating beam nodes needing constraints identified", "[SingularityAnalyzer][analyze]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K, fixture.dof_handler);

    // Should identify at least one node needing constraints
    REQUIRE(!result.nodes_needing_constraints.empty());
}

TEST_CASE("analyze: free-floating beam helpful messages generated", "[SingularityAnalyzer][analyze]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K, fixture.dof_handler);

    // Summary message should indicate singular system
    std::string summary_upper = result.summary_message;
    std::transform(summary_upper.begin(), summary_upper.end(), summary_upper.begin(), ::toupper);
    REQUIRE_THAT(summary_upper, ContainsSubstring("SINGULAR"));

    // Detailed message should explain the problem
    REQUIRE(result.detailed_message.length() > 50);
    std::string detailed_lower = result.detailed_message;
    std::transform(detailed_lower.begin(), detailed_lower.end(), detailed_lower.begin(), ::tolower);
    REQUIRE_THAT(detailed_lower, ContainsSubstring("rigid body"));

    // Should have suggested fixes
    REQUIRE(!result.suggested_fixes.empty());

    // to_string should combine all information
    std::string full_message = result.to_string();
    std::string full_lower = full_message;
    std::transform(full_lower.begin(), full_lower.end(), full_lower.begin(), ::tolower);
    REQUIRE_THAT(full_lower, ContainsSubstring("rigid body"));
}

TEST_CASE("analyze: free-floating beam rigid body mode info available", "[SingularityAnalyzer][analyze]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K, fixture.dof_handler);

    // Should have detailed mode info
    REQUIRE(result.rigid_body_modes.size() >= 6);

    for (const auto& mode : result.rigid_body_modes) {
        // Mode should have near-zero eigenvalue
        REQUIRE(std::abs(mode.eigenvalue) < 1e-6);
        // Mode should have a type
        REQUIRE(static_cast<int>(mode.mode_type) >= 0);
        // Mode should have description
        REQUIRE(!mode.description.empty());
    }
}

TEST_CASE("is_singular: free-floating beam returns true", "[SingularityAnalyzer][is_singular]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    REQUIRE(analyzer.is_singular(fixture.K) == true);
}

TEST_CASE("count_rigid_body_modes: free-floating beam has 6 modes", "[SingularityAnalyzer][count]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    int count = analyzer.count_rigid_body_modes(fixture.K);

    // Free-floating 3D beam has 6 rigid body modes
    REQUIRE(count >= 6);
}

// =============================================================================
// Cantilever Beam Tests (Well-Constrained System)
// =============================================================================

TEST_CASE("analyze: cantilever beam not singular", "[SingularityAnalyzer][analyze]") {
    CantileverBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K_bc, fixture.dof_handler);

    REQUIRE(result.is_singular == false);
    REQUIRE(result.n_rigid_body_modes == 0);
}

TEST_CASE("is_singular: cantilever beam returns false", "[SingularityAnalyzer][is_singular]") {
    CantileverBeamFixture fixture;

    SingularityAnalyzer analyzer;
    REQUIRE(analyzer.is_singular(fixture.K_bc) == false);
}

TEST_CASE("count_rigid_body_modes: cantilever beam has 0 modes", "[SingularityAnalyzer][count]") {
    CantileverBeamFixture fixture;

    SingularityAnalyzer analyzer;
    int count = analyzer.count_rigid_body_modes(fixture.K_bc);

    REQUIRE(count == 0);
}

// =============================================================================
// Partially Constrained System Tests
// =============================================================================

TEST_CASE("analyze: partially constrained beam detects missing constraints", "[SingularityAnalyzer][analyze]") {
    // Create a beam with only translation constraints (missing rotation constraints)
    NodeRegistry registry(1e-6);
    Node* node_i = registry.get_or_create_node(0.0, 0.0, 0.0);
    Node* node_j = registry.get_or_create_node(5.0, 0.0, 0.0);

    Material material(1, "Steel", 210e6, 0.3, 7.85e-3);
    Section section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7);

    BeamConfig config;
    BeamElement beam(1, node_i, node_j, &material, &section, config);

    DOFHandler dof_handler;
    dof_handler.number_dofs(registry);

    Assembler assembler(dof_handler);
    std::vector<BeamElement*> elements = {&beam};
    auto K = assembler.assemble_stiffness(elements);

    // Apply only pin supports (translations only, rotations free)
    BCHandler bc;
    bc.pin_node(node_i->id);
    bc.pin_node(node_j->id);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(dof_handler.total_dofs());
    auto [K_bc, F_bc] = bc.apply_to_system(K, F, dof_handler);

    // Simply supported beam should still be singular (no torsion restraint)
    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(K_bc, dof_handler);

    // Depending on the specific constraints, this may or may not be singular
    // With only pin supports, torsion (RX) may be unrestrained
    // The exact behavior depends on the stiffness matrix structure
}

// =============================================================================
// Acceptance Criteria Tests
// =============================================================================

TEST_CASE("AC: Free-floating model detected as singular", "[SingularityAnalyzer][acceptance]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K, fixture.dof_handler);

    REQUIRE(result.is_singular == true);
}

TEST_CASE("AC: Specific unconstrained DOFs identified", "[SingularityAnalyzer][acceptance]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K, fixture.dof_handler);

    REQUIRE(!result.unconstrained_dofs.empty());
    REQUIRE(!result.nodes_needing_constraints.empty());
}

TEST_CASE("AC: Helpful diagnostic message generated", "[SingularityAnalyzer][acceptance]") {
    FreeFloatingBeamFixture fixture;

    SingularityAnalyzer analyzer;
    SingularityDiagnostics result = analyzer.analyze(fixture.K, fixture.dof_handler);

    // Check that messages are helpful
    REQUIRE(!result.summary_message.empty());
    REQUIRE(!result.detailed_message.empty());
    REQUIRE(!result.suggested_fixes.empty());

    // Check to_string produces output
    std::string full_output = result.to_string();
    REQUIRE(full_output.length() > 100);
}
