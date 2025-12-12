#include "grillex/model.hpp"
#include <stdexcept>
#include <sstream>

namespace grillex {

Model::Model(double node_tolerance, LinearSolver::Method solver_method)
    : nodes(node_tolerance), solver_(solver_method) {}

Material* Model::create_material(const std::string& name, double E, double nu, double rho) {
    auto material = std::make_unique<Material>(next_material_id_++, name, E, nu, rho);
    Material* ptr = material.get();
    materials.push_back(std::move(material));
    return ptr;
}

Section* Model::create_section(const std::string& name, double A, double Iy, double Iz, double J) {
    auto section = std::make_unique<Section>(next_section_id_++, name, A, Iy, Iz, J);
    Section* ptr = section.get();
    sections.push_back(std::move(section));
    return ptr;
}

BeamElement* Model::create_beam(Node* node_i, Node* node_j,
                                 Material* material, Section* section,
                                 const BeamConfig& config) {
    auto element = std::make_unique<BeamElement>(next_element_id_++, node_i, node_j,
                                                  material, section, config);
    BeamElement* ptr = element.get();
    elements.push_back(std::move(element));
    return ptr;
}

void Model::add_nodal_load(int node_id, int local_dof, double value) {
    // Check if load already exists for this node/DOF
    bool found = false;
    for (auto& load : nodal_loads_) {
        if (std::get<0>(load) == node_id && std::get<1>(load) == local_dof) {
            // Accumulate load
            std::get<2>(load) += value;
            found = true;
            break;
        }
    }

    if (!found) {
        nodal_loads_.push_back(std::make_tuple(node_id, local_dof, value));
    }

    // Mark as not analyzed (need to re-analyze)
    analyzed_ = false;
}

void Model::clear_loads() {
    nodal_loads_.clear();
    analyzed_ = false;
}

bool Model::analyze() {
    // Reset state
    analyzed_ = false;
    error_msg_ = "";
    displacements_ = Eigen::VectorXd();
    reactions_ = Eigen::VectorXd();
    total_dofs_ = 0;

    try {
        // Validation
        if (elements.empty()) {
            error_msg_ = "Model has no elements";
            return false;
        }

        // Step 1: Number DOFs
        // Check if we need element-specific warping DOF handling
        bool has_warping = needs_warping_analysis();

        if (has_warping) {
            // Use element-specific warping DOF numbering
            std::vector<BeamElement*> elem_ptrs;
            elem_ptrs.reserve(elements.size());
            for (const auto& elem : elements) {
                elem_ptrs.push_back(elem.get());
            }
            dof_handler_.number_dofs_with_elements(nodes, elem_ptrs);
        } else {
            // Standard nodal DOF numbering
            dof_handler_.number_dofs(nodes);
        }

        total_dofs_ = dof_handler_.total_dofs();

        if (total_dofs_ == 0) {
            error_msg_ = "Model has no active DOFs";
            return false;
        }

        // Step 2: Assemble global stiffness matrix
        assembler_ = std::make_unique<Assembler>(dof_handler_);

        std::vector<BeamElement*> elem_ptrs;
        elem_ptrs.reserve(elements.size());
        for (const auto& elem : elements) {
            elem_ptrs.push_back(elem.get());
        }

        Eigen::SparseMatrix<double> K = assembler_->assemble_stiffness(elem_ptrs);

        // Step 3: Build load vector
        Eigen::VectorXd F = build_load_vector();

        // Store original K and F for reaction computation
        Eigen::SparseMatrix<double> K_original = K;
        Eigen::VectorXd F_applied = F;

        // Step 4: Apply boundary conditions
        auto [K_mod, F_mod] = boundary_conditions.apply_to_system(K, F, dof_handler_);

        // Step 5: Solve linear system
        displacements_ = solver_.solve(K_mod, F_mod);

        if (solver_.is_singular()) {
            error_msg_ = "Solver detected singular system: " + solver_.get_error_message();
            return false;
        }

        // Step 6: Compute reactions
        compute_reactions(K_original, F_applied);

        // Success
        analyzed_ = true;
        return true;

    } catch (const std::exception& e) {
        error_msg_ = std::string("Analysis failed: ") + e.what();
        return false;
    }
}

Eigen::VectorXd Model::build_load_vector() const {
    Eigen::VectorXd F = Eigen::VectorXd::Zero(total_dofs_);

    for (const auto& load : nodal_loads_) {
        int node_id = std::get<0>(load);
        int local_dof = std::get<1>(load);
        double value = std::get<2>(load);

        // Get global DOF number
        int global_dof = dof_handler_.get_global_dof(node_id, local_dof);

        if (global_dof >= 0 && global_dof < total_dofs_) {
            F(global_dof) += value;
        }
    }

    return F;
}

void Model::compute_reactions(const Eigen::SparseMatrix<double>& K_original,
                              const Eigen::VectorXd& F_applied) {
    // Reactions: R = K * u - F_applied
    // Only non-zero at constrained DOFs
    reactions_ = K_original * displacements_ - F_applied;
}

bool Model::needs_warping_analysis() const {
    for (const auto& elem : elements) {
        if (elem->has_warping()) {
            return true;
        }
    }
    return false;
}

Eigen::VectorXd Model::get_displacements() const {
    if (!analyzed_) {
        throw std::runtime_error("Model not analyzed. Call analyze() first.");
    }
    return displacements_;
}

double Model::get_node_displacement(int node_id, int local_dof) const {
    if (!analyzed_) {
        throw std::runtime_error("Model not analyzed. Call analyze() first.");
    }

    int global_dof = dof_handler_.get_global_dof(node_id, local_dof);

    if (global_dof < 0) {
        // DOF is inactive at this node
        return 0.0;
    }

    if (global_dof >= displacements_.size()) {
        throw std::runtime_error("Invalid global DOF index");
    }

    return displacements_(global_dof);
}

Eigen::VectorXd Model::get_reactions() const {
    if (!analyzed_) {
        throw std::runtime_error("Model not analyzed. Call analyze() first.");
    }
    return reactions_;
}

void Model::clear() {
    materials.clear();
    sections.clear();
    elements.clear();
    nodal_loads_.clear();
    boundary_conditions.clear();

    analyzed_ = false;
    total_dofs_ = 0;
    displacements_ = Eigen::VectorXd();
    reactions_ = Eigen::VectorXd();
    error_msg_ = "";

    next_material_id_ = 1;
    next_section_id_ = 1;
    next_element_id_ = 1;
}

} // namespace grillex
