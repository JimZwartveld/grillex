#include "grillex/model.hpp"
#include <stdexcept>
#include <sstream>
#include <cmath>
#include <optional>

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

SpringElement* Model::create_spring(Node* node_i, Node* node_j) {
    auto spring = std::make_unique<SpringElement>(next_spring_id_++, node_i, node_j);
    SpringElement* ptr = spring.get();
    spring_elements.push_back(std::move(spring));
    // Mark as not analyzed
    analyzed_ = false;
    return ptr;
}

PointMass* Model::create_point_mass(Node* node) {
    auto pm = std::make_unique<PointMass>(next_point_mass_id_++, node);
    PointMass* ptr = pm.get();
    point_masses.push_back(std::move(pm));
    // Mark as not analyzed
    analyzed_ = false;
    return ptr;
}

PlateElement* Model::create_plate(Node* n1, Node* n2, Node* n3, Node* n4,
                                   double thickness, Material* material) {
    auto plate = std::make_unique<PlateElement>(next_plate_id_++, n1, n2, n3, n4,
                                                 thickness, material);
    PlateElement* ptr = plate.get();
    plate_elements.push_back(std::move(plate));
    // Mark as not analyzed
    analyzed_ = false;
    return ptr;
}

void Model::add_rigid_link(Node* slave_node, Node* master_node,
                            const Eigen::Vector3d& offset) {
    if (!slave_node || !master_node) {
        throw std::runtime_error("add_rigid_link: null node pointer");
    }
    constraints.add_rigid_link(slave_node->id, master_node->id, offset);
    // Mark as not analyzed since constraints changed
    analyzed_ = false;
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
        if (elements.empty() && spring_elements.empty() && plate_elements.empty()) {
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

        // Step 2: Assemble global stiffness matrix
        assembler_ = std::make_unique<Assembler>(dof_handler_);

        std::vector<BeamElement*> elem_ptrs;
        elem_ptrs.reserve(elements.size());
        for (const auto& elem : elements) {
            elem_ptrs.push_back(elem.get());
        }

        Eigen::SparseMatrix<double> K_base = assembler_->assemble_stiffness(elem_ptrs);

        // Check if any springs have conditional loading (static/dynamic)
        bool has_conditional_springs = false;
        for (const auto& spring : spring_elements) {
            if (spring->loading_condition != LoadingCondition::All) {
                has_conditional_springs = true;
                break;
            }
        }

        // Lambda to add spring stiffness to K matrix with optional load case filter
        auto add_spring_stiffness = [&](Eigen::SparseMatrix<double>& K,
                                        std::optional<LoadCaseType> filter_type) {
            for (const auto& spring : spring_elements) {
                if (!spring->has_stiffness()) continue;

                // Check if spring should be included based on filter
                if (filter_type.has_value()) {
                    if (!spring->is_active_for_load_case(*filter_type)) {
                        continue;  // Skip this spring for this load case type
                    }
                }

                auto K_spring = spring->global_stiffness_matrix();

                // Build location array for 12-DOF spring (6 DOFs per node)
                std::vector<int> loc(12);
                for (int i = 0; i < 6; ++i) {
                    loc[i] = dof_handler_.get_global_dof(spring->node_i->id, i);
                    loc[i + 6] = dof_handler_.get_global_dof(spring->node_j->id, i);
                }

                // Add to global K using triplets
                std::vector<Eigen::Triplet<double>> triplets;
                for (int i = 0; i < 12; ++i) {
                    if (loc[i] < 0) continue;
                    for (int j = 0; j < 12; ++j) {
                        if (loc[j] < 0) continue;
                        double val = K_spring(i, j);
                        if (std::abs(val) > 1e-20) {
                            triplets.emplace_back(loc[i], loc[j], val);
                        }
                    }
                }

                // Add triplets to K
                Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
                K_add.setFromTriplets(triplets.begin(), triplets.end());
                K += K_add;
            }
        };

        // Build stiffness matrices based on whether we have conditional springs
        // K_static: For Permanent load cases (excludes Dynamic springs)
        // K_dynamic: For Variable/Environmental/Accidental load cases (includes all springs)
        Eigen::SparseMatrix<double> K_static = K_base;
        Eigen::SparseMatrix<double> K_dynamic = K_base;

        if (has_conditional_springs) {
            // Build separate matrices for static and dynamic cases
            // K_static: Only includes Static and All springs (excludes Dynamic)
            add_spring_stiffness(K_static, LoadCaseType::Permanent);
            // K_dynamic: Includes ALL springs (Static, Dynamic, and All)
            // because in environmental/variable load cases, both static and dynamic
            // connections resist the loads
            add_spring_stiffness(K_dynamic, std::nullopt);
        } else {
            // All springs have LoadingCondition::All, use single matrix
            add_spring_stiffness(K_static, std::nullopt);
            K_dynamic = K_static;  // Same matrix for both
        }

        // Add plate element stiffness contributions (to both matrices)
        for (const auto& plate : plate_elements) {
            auto K_plate = plate->global_stiffness_matrix();

            // Build location array for 24-DOF plate (6 DOFs per node × 4 nodes)
            std::vector<int> loc(24);
            for (int node_idx = 0; node_idx < 4; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            // Add to global K using triplets
            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 24; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 24; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            // Add triplets to both K matrices (plates are always active)
            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_static += K_add;
            if (has_conditional_springs) {
                K_dynamic += K_add;
            }
        }

        // Step 3: Handle constraints if present
        // The transformation matrix T maps reduced DOFs to full DOFs: u_full = T * u_reduced
        bool has_constraints = constraints.has_constraints();
        Eigen::SparseMatrix<double> T;  // Transformation matrix (only used if has_constraints)

        // If we have constraints, we need to reduce K_static and K_dynamic
        Eigen::SparseMatrix<double> K_static_reduced;
        Eigen::SparseMatrix<double> K_dynamic_reduced;

        if (has_constraints) {
            // Build transformation matrix once
            T = constraints.build_transformation_matrix(dof_handler_);

            // Reduce both stiffness matrices: K_reduced = T^T * K * T
            K_static_reduced = T.transpose() * K_static * T;
            K_dynamic_reduced = T.transpose() * K_dynamic * T;
        }

        // Step 4: Analyze each load case
        bool all_success = true;

        for (const auto& lc_ptr : load_cases_) {
            LoadCase* lc = lc_ptr.get();
            LoadCaseResult result(lc);

            try {
                // Select appropriate stiffness matrix based on load case type
                // Static load cases (Permanent) use K_static (excludes Dynamic springs)
                // Dynamic load cases (Variable/Environmental/Accidental) use K_dynamic (all springs)
                bool use_static = (lc->type() == LoadCaseType::Permanent);

                // Assemble load vector for this case
                Eigen::VectorXd F = lc->assemble_load_vector(*this, dof_handler_);
                Eigen::VectorXd F_applied = F;  // Store for reactions

                if (has_constraints) {
                    // Use reduced system
                    const Eigen::SparseMatrix<double>& K_reduced =
                        use_static ? K_static_reduced : K_dynamic_reduced;

                    // Reduce F: F_reduced = T^T * F
                    Eigen::VectorXd F_reduced = T.transpose() * F;

                    // Apply boundary conditions to reduced system
                    // Note: BCs need to reference reduced DOF indices
                    // For now, we apply BCs to the full K first, then reduce
                    // This is a simplification - proper handling would track
                    // which reduced DOFs correspond to constrained full DOFs

                    // Apply boundary conditions to reduced system
                    Eigen::SparseMatrix<double> K_bc = K_reduced;
                    Eigen::VectorXd F_bc = F_reduced;

                    // For constraints, we need to apply BCs differently
                    // The slave DOFs are eliminated, so BCs on master DOFs
                    // need to be applied to the reduced system
                    // For simplicity, we apply BCs to full system first, then reduce
                    // (This handles the common case where BCs are on independent DOFs)

                    // Rebuild with BCs applied to full system first
                    const Eigen::SparseMatrix<double>& K_full =
                        use_static ? K_static : K_dynamic;
                    Eigen::SparseMatrix<double> K_full_bc = K_full;
                    Eigen::VectorXd F_full_bc = F;
                    auto [K_with_bc, F_with_bc] = boundary_conditions.apply_to_system(
                        K_full_bc, F_full_bc, dof_handler_);

                    // Now reduce the BC-modified system
                    Eigen::SparseMatrix<double> K_final = T.transpose() * K_with_bc * T;
                    Eigen::VectorXd F_final = T.transpose() * F_with_bc;

                    // Solve reduced system
                    Eigen::VectorXd u_reduced = solver_.solve(K_final, F_final);

                    if (solver_.is_singular()) {
                        result.success = false;
                        result.error_message = "Solver detected singular system: " + solver_.get_error_message();
                        all_success = false;
                    } else {
                        result.success = true;

                        // Expand to full DOFs: u_full = T * u_reduced
                        result.displacements = T * u_reduced;

                        // Compute reactions: R = K * u - F_applied (using full K)
                        const Eigen::SparseMatrix<double>& K_full_orig =
                            use_static ? K_static : K_dynamic;
                        result.reactions = K_full_orig * result.displacements - F_applied;
                    }
                } else {
                    // No constraints - use original system directly
                    const Eigen::SparseMatrix<double>& K =
                        use_static ? K_static : K_dynamic;

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
    spring_elements.clear();
    point_masses.clear();
    plate_elements.clear();
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
    next_spring_id_ = 1;
    next_point_mass_id_ = 1;
    next_plate_id_ = 1;
    next_load_case_id_ = 1;
}

} // namespace grillex
