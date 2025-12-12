#pragma once

#include "grillex/node_registry.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/beam_element.hpp"
#include "grillex/dof_handler.hpp"
#include "grillex/assembler.hpp"
#include "grillex/boundary_condition.hpp"
#include "grillex/solver.hpp"

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace grillex {

/**
 * @brief Top-level Model class for structural analysis
 *
 * The Model class provides a high-level interface for creating and analyzing
 * structural models. It manages all model entities (nodes, materials, sections,
 * elements) and orchestrates the analysis workflow:
 *
 * 1. Model definition (create entities)
 * 2. Boundary conditions and loads
 * 3. Analysis (DOF numbering, assembly, solving)
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
 *   model.add_nodal_load(n2->id, DOFIndex::UY, -10.0);
 *   bool success = model.analyze();
 *   if (success) {
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
    BCHandler boundary_conditions;                         ///< Boundary conditions

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
     * @brief Add a nodal load to the model
     * @param node_id Node ID
     * @param local_dof Local DOF index (0-6: UX, UY, UZ, RX, RY, RZ, WARP)
     * @param value Load value [kN] or [kN·m] for moments
     *
     * Loads are accumulated if multiple calls are made for the same DOF.
     */
    void add_nodal_load(int node_id, int local_dof, double value);

    /**
     * @brief Clear all nodal loads
     */
    void clear_loads();

    /**
     * @brief Run the analysis
     * @return true if analysis succeeded, false if error occurred
     *
     * Analysis steps:
     * 1. Number DOFs (standard or with element-specific warping)
     * 2. Assemble global stiffness matrix
     * 3. Build load vector from nodal loads
     * 4. Apply boundary conditions
     * 5. Solve linear system K * u = F
     * 6. Store displacements
     * 7. Compute reactions
     *
     * After successful analysis, results can be queried via:
     * - get_displacements()
     * - get_reactions()
     * - get_node_displacement()
     * - is_analyzed()
     */
    bool analyze();

    /**
     * @brief Check if model has been analyzed
     * @return true if analyze() has been called successfully
     */
    bool is_analyzed() const { return analyzed_; }

    /**
     * @brief Get the global displacement vector
     * @return Displacement vector (size = total_dofs)
     * @throws std::runtime_error if model not analyzed
     */
    Eigen::VectorXd get_displacements() const;

    /**
     * @brief Get displacement at a specific node and DOF
     * @param node_id Node ID
     * @param local_dof Local DOF index (0-6)
     * @return Displacement value [m] or [rad]
     * @throws std::runtime_error if model not analyzed
     */
    double get_node_displacement(int node_id, int local_dof) const;

    /**
     * @brief Get the global reaction vector
     * @return Reaction vector at fixed DOFs (size = total_dofs)
     * @throws std::runtime_error if model not analyzed
     *
     * Reactions are computed as: R = K * u - F_applied
     * Non-zero values appear only at constrained DOFs.
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
     * Removes all materials, sections, elements, loads, and BCs.
     * Does NOT clear nodes (use nodes.clear() if needed).
     */
    void clear();

private:
    // Analysis components
    DOFHandler dof_handler_;
    std::unique_ptr<Assembler> assembler_;  // Created during analyze()
    LinearSolver solver_;

    // Load tracking
    std::vector<std::tuple<int, int, double>> nodal_loads_;  // (node_id, local_dof, value)

    // Results
    bool analyzed_ = false;
    int total_dofs_ = 0;
    Eigen::VectorXd displacements_;
    Eigen::VectorXd reactions_;
    std::string error_msg_;

    // ID counters
    int next_material_id_ = 1;
    int next_section_id_ = 1;
    int next_element_id_ = 1;

    /**
     * @brief Build the global load vector from nodal loads
     * @return Load vector F (size = total_dofs)
     */
    Eigen::VectorXd build_load_vector() const;

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
