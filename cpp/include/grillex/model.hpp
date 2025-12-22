#pragma once

#include "grillex/node_registry.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/beam_element.hpp"
#include "grillex/spring_element.hpp"
#include "grillex/point_mass.hpp"
#include "grillex/plate_element.hpp"
#include "grillex/dof_handler.hpp"
#include "grillex/assembler.hpp"
#include "grillex/boundary_condition.hpp"
#include "grillex/solver.hpp"
#include "grillex/load_case.hpp"
#include "grillex/constraints.hpp"
#include "grillex/nonlinear_solver.hpp"
#include "grillex/eigenvalue_solver.hpp"

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace grillex {

/**
 * @brief Top-level Model class for structural analysis
 *
 * The Model class provides a high-level interface for creating and analyzing
 * structural models. It manages all model entities (nodes, materials, sections,
 * elements, load cases) and orchestrates the analysis workflow:
 *
 * 1. Model definition (create entities)
 * 2. Load cases and boundary conditions
 * 3. Analysis (DOF numbering, assembly, solving for all load cases)
 * 4. Results extraction (displacements, reactions, element forces)
 *
 * Usage:
 *   Model model;
 *   auto mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6);
 *   auto sec = model.create_section("I_300", 0.01, 1e-5, 2e-5, 1.5e-5);
 *   auto n1 = model.nodes.get_or_create_node(0, 0, 0);
 *   auto n2 = model.nodes.get_or_create_node(6, 0, 0);
 *   auto elem = model.create_beam(n1, n2, mat, sec);
 *   model.boundary_conditions.fix_node(n1->id);
 *   auto lc = model.create_load_case("Dead Load");
 *   lc->add_nodal_load(n2->id, DOFIndex::UY, -10.0);
 *   bool success = model.analyze();  // Analyzes all load cases
 *   if (success) {
 *       model.set_active_load_case(lc);
 *       Eigen::VectorXd u = model.get_displacements();
 *   }
 */
class Model {
public:
    /**
     * @brief Public members for model entities
     */
    NodeRegistry nodes;                                    ///< Node registry with merging
    std::vector<std::unique_ptr<Material>> materials;      ///< Material library
    std::vector<std::unique_ptr<Section>> sections;        ///< Section library
    std::vector<std::unique_ptr<BeamElement>> elements;    ///< Beam elements
    std::vector<std::unique_ptr<SpringElement>> spring_elements;  ///< Spring elements
    std::vector<std::unique_ptr<PointMass>> point_masses;  ///< Point mass elements
    std::vector<std::unique_ptr<PlateElement>> plate_elements;  ///< Plate elements
    BCHandler boundary_conditions;                         ///< Boundary conditions
    ConstraintHandler constraints;                         ///< MPC and rigid link constraints

    /**
     * @brief Get node registry (for Python bindings)
     * @return Pointer to node registry
     */
    NodeRegistry* get_node_registry() { return &nodes; }

    /**
     * @brief Construct an empty model
     * @param node_tolerance Tolerance for node merging (default: 1e-6 m)
     * @param solver_method Solver method to use (default: SimplicialLDLT)
     */
    explicit Model(double node_tolerance = 1e-6,
                   LinearSolver::Method solver_method = LinearSolver::Method::SimplicialLDLT);

    /**
     * @brief Create a material and add to model
     * @param name Material name
     * @param E Young's modulus [kN/m²]
     * @param nu Poisson's ratio [-]
     * @param rho Density [mT/m³]
     * @return Pointer to created material (owned by model)
     */
    Material* create_material(const std::string& name, double E, double nu, double rho);

    /**
     * @brief Create a section and add to model
     * @param name Section name
     * @param A Cross-sectional area [m²]
     * @param Iy Second moment about y-axis [m⁴]
     * @param Iz Second moment about z-axis [m⁴]
     * @param J Torsional constant [m⁴]
     * @return Pointer to created section (owned by model)
     */
    Section* create_section(const std::string& name, double A, double Iy, double Iz, double J);

    /**
     * @brief Create a beam element and add to model
     * @param node_i Start node
     * @param node_j End node
     * @param material Material pointer
     * @param section Section pointer
     * @param config Beam configuration (formulation, warping, etc.)
     * @return Pointer to created element (owned by model)
     */
    BeamElement* create_beam(Node* node_i, Node* node_j,
                             Material* material, Section* section,
                             const BeamConfig& config = BeamConfig{});

    /**
     * @brief Create a spring element and add to model
     * @param node_i Start node
     * @param node_j End node
     * @return Pointer to created spring element (owned by model)
     *
     * Example:
     *   auto spring = model.create_spring(n1, n2);
     *   spring->kx = 1000.0;  // Axial stiffness [kN/m]
     *   spring->kz = 500.0;   // Vertical stiffness [kN/m]
     */
    SpringElement* create_spring(Node* node_i, Node* node_j);

    /**
     * @brief Create a point mass element and add to model
     * @param node Node where mass is located
     * @return Pointer to created point mass (owned by model)
     *
     * Example:
     *   auto pm = model.create_point_mass(node);
     *   pm->mass = 10.0;  // 10 mT
     *   pm->set_inertia(5.0, 5.0, 3.0);  // Rotational inertias
     */
    PointMass* create_point_mass(Node* node);

    /**
     * @brief Create a plate element and add to model
     * @param n1 Node 1 (corner at ξ=-1, η=-1)
     * @param n2 Node 2 (corner at ξ=+1, η=-1)
     * @param n3 Node 3 (corner at ξ=+1, η=+1)
     * @param n4 Node 4 (corner at ξ=-1, η=+1)
     * @param thickness Plate thickness [m]
     * @param material Material pointer
     * @return Pointer to created plate element (owned by model)
     *
     * Creates a 4-node Mindlin plate element (MITC4 formulation).
     *
     * Example:
     *   auto plate = model.create_plate(n1, n2, n3, n4, 0.01, steel);
     */
    PlateElement* create_plate(Node* n1, Node* n2, Node* n3, Node* n4,
                               double thickness, Material* material);

    /**
     * @brief Add a rigid link constraint between two nodes
     * @param slave_node Slave (constrained) node - its motion follows master
     * @param master_node Master (independent) node
     * @param offset Offset vector from master to slave in global coordinates [m]
     *
     * Creates a kinematic constraint where the slave node's motion is
     * completely determined by the master node's motion:
     *
     * u_slave = u_master + θ_master × offset
     * θ_slave = θ_master
     *
     * This is used for modeling rigid connections between nodes, such as
     * cargo COG to footprint connections.
     *
     * Example:
     *   auto cog_node = model.nodes.get_or_create_node(5, 5, 2);
     *   auto foot_node = model.nodes.get_or_create_node(5, 5, 0);
     *   model.add_rigid_link(foot_node, cog_node, Eigen::Vector3d(0, 0, -2));
     */
    void add_rigid_link(Node* slave_node, Node* master_node,
                        const Eigen::Vector3d& offset);

    /**
     * @brief Remove a beam element from the model by ID
     * @param element_id ID of the element to remove
     * @return true if element was found and removed, false otherwise
     *
     * Used for beam subdivision when a beam needs to be split at internal nodes.
     * The element is removed from the elements vector and destroyed.
     */
    bool remove_element(int element_id);

    /**
     * @brief Get a beam element by ID
     * @param element_id ID of the element to find
     * @return Pointer to element, or nullptr if not found
     */
    BeamElement* get_element(int element_id) const;

    // Load case management

    /**
     * @brief Create a new load case
     * @param name Descriptive name (e.g., "Dead Load", "Wind +X")
     * @param type Load case type classification
     * @return Pointer to created LoadCase (owned by model)
     *
     * Example:
     *   auto lc = model.create_load_case("Dead Load", LoadCaseType::Permanent);
     *   lc->add_nodal_load(node_id, DOFIndex::UY, -10.0);
     */
    LoadCase* create_load_case(const std::string& name,
                               LoadCaseType type = LoadCaseType::Permanent);

    /**
     * @brief Get the default load case (auto-created if needed)
     * @return Pointer to default LoadCase
     *
     * Convenience method for simple models. Returns a load case named "Default"
     * that is automatically created on first access.
     */
    LoadCase* get_default_load_case();

    /**
     * @brief Set the active load case for result queries
     * @param load_case Pointer to load case (must be owned by this model)
     *
     * Determines which load case's results are returned by:
     * - get_displacements()
     * - get_node_displacement()
     * - get_reactions()
     *
     * Throws std::runtime_error if load_case not found in model.
     */
    void set_active_load_case(LoadCase* load_case);

    /**
     * @brief Get the currently active load case
     * @return Pointer to active LoadCase (nullptr if none set)
     */
    LoadCase* get_active_load_case() const { return active_load_case_; }

    /**
     * @brief Get all load cases in the model
     * @return Vector of LoadCase pointers
     */
    std::vector<LoadCase*> get_load_cases() const;

    /**
     * @brief Get result for a specific load case
     * @param load_case Pointer to load case
     * @return LoadCaseResult reference
     * @throws std::runtime_error if load case not analyzed
     */
    const LoadCaseResult& get_result(LoadCase* load_case) const;

    /**
     * @brief Get all load case results (for use with LoadCombination)
     * @return Map of load case ID to LoadCaseResult
     *
     * Use this to get combined results from a LoadCombination:
     *   auto combo = LoadCombination(1, "ULS", 1.35, 1.5);
     *   combo.add_load_case(dead_load);
     *   combo.add_load_case(live_load);
     *   model.analyze();
     *   auto combined_u = combo.get_combined_displacements(model.get_all_results());
     */
    const std::map<int, LoadCaseResult>& get_all_results() const { return results_; }

    /**
     * @brief Run analysis for all load cases
     * @return true if ALL load cases analyzed successfully
     *
     * Analysis workflow:
     * 1. Number DOFs (once - same for all cases)
     * 2. Assemble stiffness matrix (once - same for all cases)
     * 3. For each load case:
     *    a. Assemble load vector for this case
     *    b. Apply boundary conditions (same K, different F)
     *    c. Solve K*u = F
     *    d. Store displacements and reactions
     *
     * After successful analysis, use set_active_load_case() to choose which
     * case's results to query via get_displacements(), etc.
     *
     * Note: For models with nonlinear springs, this uses the linear solver.
     * Use analyze_nonlinear() for proper iterative nonlinear analysis.
     */
    bool analyze();

    /**
     * @brief Check if model has nonlinear (tension/compression-only) springs
     * @return true if any spring has non-Linear behavior
     *
     * Use this to determine whether nonlinear analysis is needed.
     * If false, the efficient linear solver can be used.
     */
    bool has_nonlinear_springs() const;

    /**
     * @brief Run nonlinear analysis for all load cases
     * @return true if ALL load cases converged successfully
     *
     * Uses iterative solver for tension/compression-only springs.
     * If no nonlinear springs exist, falls back to efficient linear analysis.
     *
     * Analysis workflow:
     * 1. Number DOFs (once - same for all cases)
     * 2. Assemble base stiffness matrix (beams, plates - excludes springs)
     * 3. For each load case:
     *    a. Use NonlinearSolver to iterate until spring states converge
     *    b. Store displacements, reactions, and spring states
     */
    bool analyze_nonlinear();

    // =========================================================================
    // Eigenvalue Analysis
    // =========================================================================

    /**
     * @brief Run eigenvalue analysis to compute natural frequencies and mode shapes
     * @param settings Solver configuration (default: 10 modes, dense solver)
     * @return true if analysis converged successfully
     *
     * Solves the generalized eigenvalue problem:
     *   K × φ = ω² × M × φ
     *
     * Where:
     * - K: Global stiffness matrix (from beams, springs, plates)
     * - M: Global mass matrix (from beams and point masses)
     * - ω: Natural circular frequency [rad/s]
     * - φ: Mode shape (eigenvector)
     *
     * Analysis workflow:
     * 1. Number DOFs (reuse from static analysis if already done)
     * 2. Assemble K and M matrices (including point masses)
     * 3. Reduce system (eliminate fixed DOFs)
     * 4. Solve eigenvalue problem
     * 5. Expand mode shapes to full DOF vector
     * 6. Compute participation factors
     * 7. Store results
     *
     * Results are accessible via:
     * - get_eigenvalue_result() for full results
     * - get_natural_frequencies() for frequency list [Hz]
     * - get_periods() for period list [s]
     * - get_mode_shape(n) for mode shape vector
     *
     * @code
     * EigensolverSettings settings;
     * settings.n_modes = 20;
     * settings.method = EigensolverMethod::SubspaceIteration;
     * if (model.analyze_eigenvalues(settings)) {
     *     auto freqs = model.get_natural_frequencies();
     *     auto mode1 = model.get_mode_shape(1);
     * }
     * @endcode
     */
    bool analyze_eigenvalues(const EigensolverSettings& settings = EigensolverSettings{});

    /**
     * @brief Check if eigenvalue results are available
     * @return true if analyze_eigenvalues() has been called successfully
     */
    bool has_eigenvalue_results() const;

    /**
     * @brief Get eigenvalue analysis results
     * @return Reference to EigensolverResult
     * @throws std::runtime_error if no eigenvalue results available
     */
    const EigensolverResult& get_eigenvalue_result() const;

    /**
     * @brief Get natural frequencies from eigenvalue analysis [Hz]
     * @return Vector of frequencies, sorted ascending (lowest first)
     * @throws std::runtime_error if no eigenvalue results available
     */
    std::vector<double> get_natural_frequencies() const;

    /**
     * @brief Get natural periods from eigenvalue analysis [s]
     * @return Vector of periods, sorted by frequency (longest first)
     * @throws std::runtime_error if no eigenvalue results available
     */
    std::vector<double> get_periods() const;

    /**
     * @brief Get mode shape for a specific mode
     * @param mode_number Mode number (1-based: 1 = first mode)
     * @return Full mode shape vector (size = total_dofs) with zeros at fixed DOFs
     * @throws std::runtime_error if no eigenvalue results or mode not found
     *
     * Mode shapes are mass-normalized (φᵀMφ = 1) if settings.mass_normalize is true.
     */
    Eigen::VectorXd get_mode_shape(int mode_number) const;

    /**
     * @brief Analyze a specific load combination with nonlinear spring support
     * @param combo Load combination to analyze
     * @param settings Optional solver settings (uses model defaults if empty)
     * @return LoadCombinationResult with displacements and convergence info
     *
     * **IMPORTANT**: With nonlinear springs, load combinations cannot use
     * superposition (summing individual load case results). Each combination
     * must be solved directly as a single nonlinear problem.
     *
     * **Static→Dynamic Sequencing**:
     * For proper physical behavior, this method:
     * 1. First solves the "static base" (Permanent load cases only)
     *    to establish the baseline contact pattern (gaps close, cargo settles)
     * 2. Then solves the full combination starting from the static state
     *
     * This is essential for gap springs where gaps must close under gravity
     * before dynamic loads are applied.
     *
     * @code
     * LoadCombination combo(1, "ULS-Roll", 1.35, 1.5, 1.5, 1.0);
     * combo.add_load_case(dead_load);  // Permanent - solved first as baseline
     * combo.add_load_case(roll_moment); // Environmental - applied on top
     *
     * auto result = model.analyze_combination(combo);
     * if (result.converged) {
     *     // result.displacements, result.spring_states available
     * }
     * @endcode
     */
    LoadCombinationResult analyze_combination(
        const LoadCombination& combo,
        const NonlinearSolverSettings& settings = {});

    /**
     * @brief Get nonlinear solver settings
     * @return Reference to settings (can be modified)
     */
    NonlinearSolverSettings& nonlinear_settings() { return nl_settings_; }
    const NonlinearSolverSettings& nonlinear_settings() const { return nl_settings_; }

    /**
     * @brief Check if model has been analyzed
     * @return true if analyze() has been called successfully
     */
    bool is_analyzed() const { return analyzed_; }

    /**
     * @brief Get the global displacement vector (for active load case)
     * @return Displacement vector (size = total_dofs)
     * @throws std::runtime_error if no active load case or not analyzed
     *
     * Use set_active_load_case() to select which load case's results to query.
     */
    Eigen::VectorXd get_displacements() const;

    /**
     * @brief Get displacement at a specific node and DOF (for active load case)
     * @param node_id Node ID
     * @param local_dof Local DOF index (0-6)
     * @return Displacement value [m] or [rad]
     * @throws std::runtime_error if no active load case or not analyzed
     *
     * Use set_active_load_case() to select which load case's results to query.
     */
    double get_node_displacement(int node_id, int local_dof) const;

    /**
     * @brief Get the global reaction vector (for active load case)
     * @return Reaction vector at fixed DOFs (size = total_dofs)
     * @throws std::runtime_error if no active load case or not analyzed
     *
     * Reactions are computed as: R = K * u - F_applied
     * Non-zero values appear only at constrained DOFs.
     *
     * Use set_active_load_case() to select which load case's results to query.
     */
    Eigen::VectorXd get_reactions() const;

    /**
     * @brief Get total number of DOFs in the model
     * @return Number of DOFs (only valid after analyze())
     */
    int total_dofs() const { return total_dofs_; }

    /**
     * @brief Get the DOF handler (for advanced users)
     * @return Reference to DOFHandler
     */
    const DOFHandler& get_dof_handler() const { return dof_handler_; }

    /**
     * @brief Get the linear solver (for configuration)
     * @return Reference to LinearSolver
     */
    LinearSolver& get_solver() { return solver_; }
    const LinearSolver& get_solver() const { return solver_; }

    /**
     * @brief Get analysis error message (if analyze() failed)
     * @return Error message string (empty if no error)
     */
    std::string get_error_message() const { return error_msg_; }

    /**
     * @brief Clear the model (reset to empty state)
     *
     * Removes all materials, sections, elements, load cases, and BCs.
     * Does NOT clear nodes (use nodes.clear() if needed).
     */
    void clear();

private:
    // Analysis components
    DOFHandler dof_handler_;
    std::unique_ptr<Assembler> assembler_;  // Created during analyze()
    LinearSolver solver_;
    NonlinearSolverSettings nl_settings_;   // Settings for nonlinear analysis

    // Load case management
    std::vector<std::unique_ptr<LoadCase>> load_cases_;
    LoadCase* default_load_case_ = nullptr;        // Lazy-created on first access
    LoadCase* active_load_case_ = nullptr;         // For result queries

    // Results (one per load case)
    bool analyzed_ = false;
    int total_dofs_ = 0;
    std::map<int, LoadCaseResult> results_;        // Keyed by LoadCase::id()
    std::string error_msg_;

    // Eigenvalue analysis results
    std::unique_ptr<EigensolverResult> eigenvalue_result_;

    // ID counters
    int next_material_id_ = 1;
    int next_section_id_ = 1;
    int next_element_id_ = 1;
    int next_spring_id_ = 1;
    int next_point_mass_id_ = 1;
    int next_plate_id_ = 1;
    int next_load_case_id_ = 1;

    /**
     * @brief Ensure default load case exists
     */
    void ensure_default_load_case();

    /**
     * @brief Compute reactions after solving
     * @param K_original Original stiffness matrix (before BCs)
     * @param F_applied Applied load vector (before BCs)
     */
    void compute_reactions(const Eigen::SparseMatrix<double>& K_original,
                          const Eigen::VectorXd& F_applied);

    /**
     * @brief Check if any element requires warping DOF
     * @return true if warping analysis is needed
     */
    bool needs_warping_analysis() const;
};

} // namespace grillex
