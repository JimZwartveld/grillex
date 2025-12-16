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

bool Model::remove_element(int element_id) {
    // Find element with matching ID
    for (auto it = elements.begin(); it != elements.end(); ++it) {
        if ((*it)->id == element_id) {
            // Found - erase and return true
            elements.erase(it);
            // Mark as not analyzed since model changed
            analyzed_ = false;
            return true;
        }
    }
    // Not found
    return false;
}

BeamElement* Model::get_element(int element_id) const {
    for (const auto& elem : elements) {
        if (elem->id == element_id) {
            return elem.get();
        }
    }
    return nullptr;
}

// Load case management

LoadCase* Model::create_load_case(const std::string& name, LoadCaseType type) {
    auto lc = std::make_unique<LoadCase>(next_load_case_id_++, name, type);
    LoadCase* ptr = lc.get();
    load_cases_.push_back(std::move(lc));

    // If this is the first load case, make it active
    if (load_cases_.size() == 1) {
        active_load_case_ = ptr;
    }

    // Mark as not analyzed
    analyzed_ = false;

    return ptr;
}

LoadCase* Model::get_default_load_case() {
    ensure_default_load_case();
    return default_load_case_;
}

void Model::ensure_default_load_case() {
    if (default_load_case_ == nullptr) {
        default_load_case_ = create_load_case("Default", LoadCaseType::Permanent);
    }
}

void Model::set_active_load_case(LoadCase* load_case) {
    // Verify load case belongs to this model
    bool found = false;
    for (const auto& lc : load_cases_) {
        if (lc.get() == load_case) {
            found = true;
            break;
        }
    }

    if (!found) {
        throw std::runtime_error("LoadCase does not belong to this Model");
    }

    active_load_case_ = load_case;
}

std::vector<LoadCase*> Model::get_load_cases() const {
    std::vector<LoadCase*> result;
    result.reserve(load_cases_.size());
    for (const auto& lc : load_cases_) {
        result.push_back(lc.get());
    }
    return result;
}

const LoadCaseResult& Model::get_result(LoadCase* load_case) const {
    auto it = results_.find(load_case->id());
    if (it == results_.end()) {
        throw std::runtime_error("Result not found for load case: " + load_case->name() +
                                ". Make sure analyze() has been called.");
    }

    return it->second;
}

bool Model::analyze() {
    // Reset state
    analyzed_ = false;
    error_msg_ = "";
    results_.clear();
    total_dofs_ = 0;

    try {
        // Validation
        if (elements.empty()) {
            error_msg_ = "Model has no elements";
            return false;
        }

        if (load_cases_.empty()) {
            error_msg_ = "No load cases defined. Create at least one load case.";
            return false;
        }

        // Step 1: Number DOFs (same for all cases)
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

        // Step 2: Assemble global stiffness matrix (same for all cases)
        assembler_ = std::make_unique<Assembler>(dof_handler_);

        std::vector<BeamElement*> elem_ptrs;
        elem_ptrs.reserve(elements.size());
        for (const auto& elem : elements) {
            elem_ptrs.push_back(elem.get());
        }

        Eigen::SparseMatrix<double> K = assembler_->assemble_stiffness(elem_ptrs);

        // Step 3: Analyze each load case
        bool all_success = true;

        for (const auto& lc_ptr : load_cases_) {
            LoadCase* lc = lc_ptr.get();
            LoadCaseResult result(lc);

            try {
                // Assemble load vector for this case
                Eigen::VectorXd F = lc->assemble_load_vector(*this, dof_handler_);
                Eigen::VectorXd F_applied = F;  // Store for reactions

                // Apply boundary conditions (same K structure, modifies values)
                Eigen::SparseMatrix<double> K_bc = K;  // Copy K for modification
                Eigen::VectorXd F_bc = F;              // Copy F for modification
                auto [K_mod, F_mod] = boundary_conditions.apply_to_system(K_bc, F_bc, dof_handler_);

                // Solve
                result.displacements = solver_.solve(K_mod, F_mod);

                if (solver_.is_singular()) {
                    result.success = false;
                    result.error_message = "Solver detected singular system: " + solver_.get_error_message();
                    all_success = false;
                } else {
                    result.success = true;

                    // Compute reactions: R = K * u - F_applied
                    result.reactions = K * result.displacements - F_applied;
                }

            } catch (const std::exception& e) {
                result.success = false;
                result.error_message = std::string("Load case analysis failed: ") + e.what();
                all_success = false;
            }

            results_[lc->id()] = result;
        }

        analyzed_ = all_success;

        if (!all_success) {
            error_msg_ = "One or more load cases failed to analyze";
        }

        // Ensure there's an active load case
        if (active_load_case_ == nullptr && !load_cases_.empty()) {
            active_load_case_ = load_cases_[0].get();
        }

        return all_success;

    } catch (const std::exception& e) {
        error_msg_ = std::string("Analysis failed: ") + e.what();
        return false;
    }
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
    if (active_load_case_ == nullptr) {
        throw std::runtime_error("No active load case. Call set_active_load_case() first.");
    }

    const auto& result = get_result(active_load_case_);
    if (!result.success) {
        throw std::runtime_error("Active load case analysis failed: " + result.error_message);
    }

    return result.displacements;
}

double Model::get_node_displacement(int node_id, int local_dof) const {
    if (!analyzed_) {
        throw std::runtime_error("Model not analyzed. Call analyze() first.");
    }
    if (active_load_case_ == nullptr) {
        throw std::runtime_error("No active load case. Call set_active_load_case() first.");
    }

    const auto& result = get_result(active_load_case_);
    if (!result.success) {
        throw std::runtime_error("Active load case analysis failed: " + result.error_message);
    }

    int global_dof = dof_handler_.get_global_dof(node_id, local_dof);

    if (global_dof < 0) {
        // DOF is inactive at this node
        return 0.0;
    }

    if (global_dof >= result.displacements.size()) {
        throw std::runtime_error("Invalid global DOF index");
    }

    return result.displacements(global_dof);
}

Eigen::VectorXd Model::get_reactions() const {
    if (!analyzed_) {
        throw std::runtime_error("Model not analyzed. Call analyze() first.");
    }
    if (active_load_case_ == nullptr) {
        throw std::runtime_error("No active load case. Call set_active_load_case() first.");
    }

    const auto& result = get_result(active_load_case_);
    if (!result.success) {
        throw std::runtime_error("Active load case analysis failed: " + result.error_message);
    }

    return result.reactions;
}

void Model::clear() {
    materials.clear();
    sections.clear();
    elements.clear();
    load_cases_.clear();
    boundary_conditions.clear();

    analyzed_ = false;
    total_dofs_ = 0;
    results_.clear();
    error_msg_ = "";
    default_load_case_ = nullptr;
    active_load_case_ = nullptr;

    next_material_id_ = 1;
    next_section_id_ = 1;
    next_element_id_ = 1;
    next_load_case_id_ = 1;
}

} // namespace grillex
