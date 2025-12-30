#include "grillex/model.hpp"
#include <stdexcept>
#include <sstream>
#include <cmath>
#include <optional>

namespace grillex {

Model::Model(double node_tolerance, LinearSolver::Method solver_method)
    : nodes(node_tolerance), solver_(solver_method) {}

Material* Model::create_material(const std::string& name, double E, double nu, double rho,
                                  double fy, double fu) {
    auto material = std::make_unique<Material>(next_material_id_++, name, E, nu, rho, fy, fu);
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

PlateElement8* Model::create_plate_8(Node* n1, Node* n2, Node* n3, Node* n4,
                                      Node* n5, Node* n6, Node* n7, Node* n8,
                                      double thickness, Material* material) {
    auto plate = std::make_unique<PlateElement8>(next_plate_id_++,
                                                  n1, n2, n3, n4, n5, n6, n7, n8,
                                                  thickness, material);
    PlateElement8* ptr = plate.get();
    plate_elements_8.push_back(std::move(plate));
    // Mark as not analyzed
    analyzed_ = false;
    return ptr;
}

PlateElement9* Model::create_plate_9(Node* n1, Node* n2, Node* n3, Node* n4,
                                      Node* n5, Node* n6, Node* n7, Node* n8, Node* n9,
                                      double thickness, Material* material) {
    auto plate = std::make_unique<PlateElement9>(next_plate_id_++,
                                                  n1, n2, n3, n4, n5, n6, n7, n8, n9,
                                                  thickness, material);
    PlateElement9* ptr = plate.get();
    plate_elements_9.push_back(std::move(plate));
    // Mark as not analyzed
    analyzed_ = false;
    return ptr;
}

PlateElementTri* Model::create_plate_tri(Node* n1, Node* n2, Node* n3,
                                          double thickness, Material* material) {
    auto plate = std::make_unique<PlateElementTri>(next_plate_id_++, n1, n2, n3,
                                                    thickness, material);
    PlateElementTri* ptr = plate.get();
    plate_elements_tri.push_back(std::move(plate));
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
        // Validation - check all element types
        bool has_elements = !elements.empty() ||
                            !spring_elements.empty() ||
                            !plate_elements.empty() ||
                            !plate_elements_8.empty() ||
                            !plate_elements_9.empty() ||
                            !plate_elements_tri.empty();
        if (!has_elements) {
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
        // MITC4 (4-node) plate elements
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

        // MITC8 (8-node) plate elements
        for (const auto& plate : plate_elements_8) {
            auto K_plate = plate->global_stiffness_matrix();

            // Build location array for 48-DOF plate (6 DOFs per node × 8 nodes)
            std::vector<int> loc(48);
            for (int node_idx = 0; node_idx < 8; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 48; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 48; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_static += K_add;
            if (has_conditional_springs) {
                K_dynamic += K_add;
            }
        }

        // MITC9 (9-node) plate elements
        for (const auto& plate : plate_elements_9) {
            auto K_plate = plate->global_stiffness_matrix();

            // Build location array for 54-DOF plate (6 DOFs per node × 9 nodes)
            std::vector<int> loc(54);
            for (int node_idx = 0; node_idx < 9; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 54; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 54; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_static += K_add;
            if (has_conditional_springs) {
                K_dynamic += K_add;
            }
        }

        // DKT (3-node triangular) plate elements
        for (const auto& plate : plate_elements_tri) {
            auto K_plate = plate->global_stiffness_matrix();

            // Build location array for 18-DOF plate (6 DOFs per node × 3 nodes)
            std::vector<int> loc(18);
            for (int node_idx = 0; node_idx < 3; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 18; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 18; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

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

bool Model::has_nonlinear_springs() const {
    for (const auto& spring : spring_elements) {
        if (spring->is_nonlinear() || spring->has_gap()) {
            return true;
        }
    }
    return false;
}

bool Model::analyze_nonlinear() {
    // If no nonlinear springs, fall back to efficient linear analysis
    if (!has_nonlinear_springs()) {
        return analyze();
    }

    // Reset state
    analyzed_ = false;
    error_msg_ = "";
    results_.clear();
    total_dofs_ = 0;

    try {
        // Validation - check all element types
        bool has_elements = !elements.empty() ||
                            !spring_elements.empty() ||
                            !plate_elements.empty() ||
                            !plate_elements_8.empty() ||
                            !plate_elements_9.empty() ||
                            !plate_elements_tri.empty();
        if (!has_elements) {
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
            std::vector<BeamElement*> elem_ptrs;
            elem_ptrs.reserve(elements.size());
            for (const auto& elem : elements) {
                elem_ptrs.push_back(elem.get());
            }
            dof_handler_.number_dofs_with_elements(nodes, elem_ptrs);
        } else {
            dof_handler_.number_dofs(nodes);
        }

        total_dofs_ = dof_handler_.total_dofs();

        if (total_dofs_ == 0) {
            error_msg_ = "Model has no active DOFs";
            return false;
        }

        // Step 2: Assemble BASE stiffness matrix (beams + plates, NOT springs)
        assembler_ = std::make_unique<Assembler>(dof_handler_);

        std::vector<BeamElement*> elem_ptrs;
        elem_ptrs.reserve(elements.size());
        for (const auto& elem : elements) {
            elem_ptrs.push_back(elem.get());
        }

        Eigen::SparseMatrix<double> K_base = assembler_->assemble_stiffness(elem_ptrs);

        // Add plate element stiffness to base
        // MITC4 (4-node) plate elements
        for (const auto& plate : plate_elements) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(24);
            for (int node_idx = 0; node_idx < 4; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

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

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_base += K_add;
        }

        // MITC8 (8-node) plate elements
        for (const auto& plate : plate_elements_8) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(48);
            for (int node_idx = 0; node_idx < 8; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 48; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 48; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_base += K_add;
        }

        // MITC9 (9-node) plate elements
        for (const auto& plate : plate_elements_9) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(54);
            for (int node_idx = 0; node_idx < 9; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 54; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 54; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_base += K_add;
        }

        // DKT (3-node triangular) plate elements
        for (const auto& plate : plate_elements_tri) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(18);
            for (int node_idx = 0; node_idx < 3; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 18; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 18; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_base += K_add;
        }

        // Apply boundary conditions to base K
        Eigen::VectorXd dummy_F = Eigen::VectorXd::Zero(total_dofs_);
        auto [K_base_bc, _] = boundary_conditions.apply_to_system(K_base, dummy_F, dof_handler_);

        // Create nonlinear solver
        NonlinearSolver nl_solver(nl_settings_);

        // Get raw spring pointers for solver
        std::vector<SpringElement*> spring_ptrs;
        spring_ptrs.reserve(spring_elements.size());
        for (auto& spring : spring_elements) {
            spring_ptrs.push_back(spring.get());
        }

        // Step 3: Analyze each load case
        bool all_success = true;

        for (const auto& lc_ptr : load_cases_) {
            LoadCase* lc = lc_ptr.get();
            LoadCaseResult result(lc);

            try {
                // Assemble load vector for this case
                Eigen::VectorXd F = lc->assemble_load_vector(*this, dof_handler_);
                Eigen::VectorXd F_applied = F;

                // Apply boundary conditions to F
                auto [_, F_bc] = boundary_conditions.apply_to_system(K_base, F, dof_handler_);

                // Solve using nonlinear solver
                auto nl_result = nl_solver.solve(K_base_bc, F_bc, spring_ptrs, dof_handler_);

                result.displacements = nl_result.displacements;
                result.success = nl_result.converged;
                result.iterations = nl_result.iterations;
                result.solver_message = nl_result.message;
                result.spring_states = nl_result.spring_states;
                result.spring_forces = nl_result.spring_forces;

                if (!nl_result.converged) {
                    result.error_message = nl_result.message;
                    all_success = false;
                } else {
                    // Compute reactions with final spring stiffness
                    // Assemble current spring stiffness
                    Eigen::SparseMatrix<double> K_springs(total_dofs_, total_dofs_);
                    std::vector<Eigen::Triplet<double>> triplets;

                    for (const auto* spring : spring_ptrs) {
                        auto K_s = spring->current_stiffness_matrix();
                        std::array<int, 12> loc;
                        for (int d = 0; d < 6; ++d) {
                            loc[d] = dof_handler_.get_global_dof(spring->node_i->id, d);
                            loc[d + 6] = dof_handler_.get_global_dof(spring->node_j->id, d);
                        }
                        for (int i = 0; i < 12; ++i) {
                            if (loc[i] < 0) continue;
                            for (int j = 0; j < 12; ++j) {
                                if (loc[j] < 0) continue;
                                double val = K_s(i, j);
                                if (std::abs(val) > 1e-20) {
                                    triplets.emplace_back(loc[i], loc[j], val);
                                }
                            }
                        }
                    }
                    K_springs.setFromTriplets(triplets.begin(), triplets.end());

                    Eigen::SparseMatrix<double> K_total = K_base + K_springs;
                    result.reactions = K_total * result.displacements - F_applied;
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
            error_msg_ = "One or more load cases failed to converge";
        }

        // Ensure there's an active load case
        if (active_load_case_ == nullptr && !load_cases_.empty()) {
            active_load_case_ = load_cases_[0].get();
        }

        return all_success;

    } catch (const std::exception& e) {
        error_msg_ = std::string("Nonlinear analysis failed: ") + e.what();
        return false;
    }
}

LoadCombinationResult Model::analyze_combination(
    const LoadCombination& combo,
    const NonlinearSolverSettings& settings)
{
    LoadCombinationResult result;

    try {
        // Validation - check all element types
        bool has_elements = !elements.empty() ||
                            !spring_elements.empty() ||
                            !plate_elements.empty() ||
                            !plate_elements_8.empty() ||
                            !plate_elements_9.empty() ||
                            !plate_elements_tri.empty();
        if (!has_elements) {
            result.message = "Model has no elements";
            return result;
        }

        if (combo.empty()) {
            result.message = "Load combination is empty";
            return result;
        }

        // Ensure DOFs are numbered (reuse if already done)
        if (total_dofs_ == 0) {
            bool has_warping = needs_warping_analysis();
            if (has_warping) {
                std::vector<BeamElement*> elem_ptrs;
                elem_ptrs.reserve(elements.size());
                for (const auto& elem : elements) {
                    elem_ptrs.push_back(elem.get());
                }
                dof_handler_.number_dofs_with_elements(nodes, elem_ptrs);
            } else {
                dof_handler_.number_dofs(nodes);
            }
            total_dofs_ = dof_handler_.total_dofs();
        }

        if (total_dofs_ == 0) {
            result.message = "Model has no active DOFs";
            return result;
        }

        // Assemble BASE stiffness matrix (beams + plates, NOT springs)
        if (!assembler_) {
            assembler_ = std::make_unique<Assembler>(dof_handler_);
        }

        std::vector<BeamElement*> elem_ptrs;
        elem_ptrs.reserve(elements.size());
        for (const auto& elem : elements) {
            elem_ptrs.push_back(elem.get());
        }

        Eigen::SparseMatrix<double> K_base = assembler_->assemble_stiffness(elem_ptrs);

        // Add plate element stiffness to base
        // MITC4 (4-node) plate elements
        for (const auto& plate : plate_elements) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(24);
            for (int node_idx = 0; node_idx < 4; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

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

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_base += K_add;
        }

        // MITC8 (8-node) plate elements
        for (const auto& plate : plate_elements_8) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(48);
            for (int node_idx = 0; node_idx < 8; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 48; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 48; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_base += K_add;
        }

        // MITC9 (9-node) plate elements
        for (const auto& plate : plate_elements_9) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(54);
            for (int node_idx = 0; node_idx < 9; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 54; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 54; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_base += K_add;
        }

        // DKT (3-node triangular) plate elements
        for (const auto& plate : plate_elements_tri) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(18);
            for (int node_idx = 0; node_idx < 3; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 18; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 18; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K_base += K_add;
        }

        // Apply boundary conditions to base K
        Eigen::VectorXd dummy_F = Eigen::VectorXd::Zero(total_dofs_);
        auto [K_base_bc, _] = boundary_conditions.apply_to_system(K_base, dummy_F, dof_handler_);

        // Get raw spring pointers
        std::vector<SpringElement*> spring_ptrs;
        spring_ptrs.reserve(spring_elements.size());
        for (auto& spring : spring_elements) {
            spring_ptrs.push_back(spring.get());
        }

        // Create nonlinear solver with provided settings (or model defaults)
        NonlinearSolverSettings effective_settings = settings;
        if (settings.max_iterations == 50 && nl_settings_.max_iterations != 50) {
            // Use model defaults if settings appear to be default
            effective_settings = nl_settings_;
        }
        NonlinearSolver nl_solver(effective_settings);

        // =====================================================================
        // STATIC→DYNAMIC SEQUENCING
        //
        // For proper physical behavior:
        // 1. First solve the "static base" (Permanent loads only)
        //    to establish the baseline contact pattern (gaps close, cargo settles)
        // 2. Then solve the full combination starting from the static state
        // =====================================================================

        // Step 1: Identify and assemble static base (Permanent loads only)
        Eigen::VectorXd F_static = Eigen::VectorXd::Zero(total_dofs_);
        bool has_permanent = false;

        for (const auto& term : combo.get_terms()) {
            if (term.load_case->type() == LoadCaseType::Permanent) {
                Eigen::VectorXd F_case = term.load_case->assemble_load_vector(*this, dof_handler_);
                F_static += term.factor * F_case;
                has_permanent = true;
            }
        }

        NonlinearInitialState initial_state;
        int total_iterations = 0;

        if (has_permanent && has_nonlinear_springs()) {
            // Solve static base to establish baseline contact pattern
            Eigen::VectorXd F_static_bc;
            {
                auto [_, F_bc] = boundary_conditions.apply_to_system(K_base, F_static, dof_handler_);
                F_static_bc = F_bc;
            }

            auto static_result = nl_solver.solve(K_base_bc, F_static_bc, spring_ptrs, dof_handler_);
            total_iterations += static_result.iterations;

            if (!static_result.converged) {
                result.converged = false;
                result.iterations = total_iterations;
                result.message = "Static base solve failed: " + static_result.message;
                return result;
            }

            // Use static solution as starting point for full combination
            initial_state.displacement = static_result.displacements;
            initial_state.spring_states = static_result.spring_states;
        }

        // Step 2: Assemble full combined load vector
        Eigen::VectorXd F_combined = Eigen::VectorXd::Zero(total_dofs_);

        for (const auto& term : combo.get_terms()) {
            Eigen::VectorXd F_case = term.load_case->assemble_load_vector(*this, dof_handler_);
            F_combined += term.factor * F_case;
        }

        Eigen::VectorXd F_applied = F_combined;

        // Apply boundary conditions to combined F
        Eigen::VectorXd F_combined_bc;
        {
            auto [_, F_bc] = boundary_conditions.apply_to_system(K_base, F_combined, dof_handler_);
            F_combined_bc = F_bc;
        }

        // Step 3: Solve full combination starting from static state
        auto nl_result = nl_solver.solve(K_base_bc, F_combined_bc, spring_ptrs,
                                          dof_handler_, initial_state);
        total_iterations += nl_result.iterations;

        result.displacements = nl_result.displacements;
        result.converged = nl_result.converged;
        result.iterations = total_iterations;
        result.spring_states = nl_result.spring_states;
        result.spring_forces = nl_result.spring_forces;

        if (!nl_result.converged) {
            result.message = nl_result.message;
        } else {
            // Compute reactions with final spring stiffness
            Eigen::SparseMatrix<double> K_springs(total_dofs_, total_dofs_);
            std::vector<Eigen::Triplet<double>> triplets;

            for (const auto* spring : spring_ptrs) {
                auto K_s = spring->current_stiffness_matrix();
                std::array<int, 12> loc;
                for (int d = 0; d < 6; ++d) {
                    loc[d] = dof_handler_.get_global_dof(spring->node_i->id, d);
                    loc[d + 6] = dof_handler_.get_global_dof(spring->node_j->id, d);
                }
                for (int i = 0; i < 12; ++i) {
                    if (loc[i] < 0) continue;
                    for (int j = 0; j < 12; ++j) {
                        if (loc[j] < 0) continue;
                        double val = K_s(i, j);
                        if (std::abs(val) > 1e-20) {
                            triplets.emplace_back(loc[i], loc[j], val);
                        }
                    }
                }
            }
            K_springs.setFromTriplets(triplets.begin(), triplets.end());

            Eigen::SparseMatrix<double> K_total = K_base + K_springs;
            result.reactions = K_total * result.displacements - F_applied;
            result.message = nl_result.message;
        }

        return result;

    } catch (const std::exception& e) {
        result.converged = false;
        result.message = std::string("Combination analysis failed: ") + e.what();
        return result;
    }
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
    plate_elements_8.clear();
    plate_elements_9.clear();
    plate_elements_tri.clear();
    load_cases_.clear();
    boundary_conditions.clear();

    analyzed_ = false;
    total_dofs_ = 0;
    results_.clear();
    error_msg_ = "";
    default_load_case_ = nullptr;
    active_load_case_ = nullptr;
    eigenvalue_result_.reset();

    next_material_id_ = 1;
    next_section_id_ = 1;
    next_element_id_ = 1;
    next_spring_id_ = 1;
    next_point_mass_id_ = 1;
    next_plate_id_ = 1;
    next_load_case_id_ = 1;
}

// =============================================================================
// Eigenvalue Analysis Methods
// =============================================================================

bool Model::analyze_eigenvalues(const EigensolverSettings& settings) {
    // Reset eigenvalue results
    eigenvalue_result_.reset();
    error_msg_ = "";

    try {
        // Validation - check all element types
        bool has_structural_elements = !elements.empty() ||
                                       !plate_elements.empty() ||
                                       !plate_elements_8.empty() ||
                                       !plate_elements_9.empty() ||
                                       !plate_elements_tri.empty();
        if (!has_structural_elements) {
            error_msg_ = "Model has no structural elements for eigenvalue analysis";
            return false;
        }

        // Step 1: Number DOFs (reuse from static analysis if already done)
        if (total_dofs_ == 0) {
            bool has_warping = needs_warping_analysis();

            if (has_warping) {
                std::vector<BeamElement*> elem_ptrs;
                elem_ptrs.reserve(elements.size());
                for (const auto& elem : elements) {
                    elem_ptrs.push_back(elem.get());
                }
                dof_handler_.number_dofs_with_elements(nodes, elem_ptrs);
            } else {
                dof_handler_.number_dofs(nodes);
            }
            total_dofs_ = dof_handler_.total_dofs();
        }

        if (total_dofs_ == 0) {
            error_msg_ = "Model has no active DOFs";
            return false;
        }

        // Step 2: Assemble global stiffness matrix K
        if (!assembler_) {
            assembler_ = std::make_unique<Assembler>(dof_handler_);
        }

        std::vector<BeamElement*> elem_ptrs;
        elem_ptrs.reserve(elements.size());
        for (const auto& elem : elements) {
            elem_ptrs.push_back(elem.get());
        }

        Eigen::SparseMatrix<double> K = assembler_->assemble_stiffness(elem_ptrs);

        // Add spring stiffness (linear springs only for eigenvalue analysis)
        for (const auto& spring : spring_elements) {
            if (!spring->has_stiffness()) continue;

            auto K_spring = spring->global_stiffness_matrix();

            std::vector<int> loc(12);
            for (int i = 0; i < 6; ++i) {
                loc[i] = dof_handler_.get_global_dof(spring->node_i->id, i);
                loc[i + 6] = dof_handler_.get_global_dof(spring->node_j->id, i);
            }

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

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K += K_add;
        }

        // Add plate element stiffness
        // MITC4 (4-node) plate elements
        for (const auto& plate : plate_elements) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(24);
            for (int node_idx = 0; node_idx < 4; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

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

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K += K_add;
        }

        // MITC8 (8-node) plate elements
        for (const auto& plate : plate_elements_8) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(48);
            for (int node_idx = 0; node_idx < 8; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 48; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 48; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K += K_add;
        }

        // MITC9 (9-node) plate elements
        for (const auto& plate : plate_elements_9) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(54);
            for (int node_idx = 0; node_idx < 9; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 54; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 54; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K += K_add;
        }

        // DKT (3-node triangular) plate elements
        for (const auto& plate : plate_elements_tri) {
            auto K_plate = plate->global_stiffness_matrix();

            std::vector<int> loc(18);
            for (int node_idx = 0; node_idx < 3; ++node_idx) {
                for (int dof = 0; dof < 6; ++dof) {
                    loc[node_idx * 6 + dof] = dof_handler_.get_global_dof(
                        plate->nodes[node_idx]->id, dof);
                }
            }

            std::vector<Eigen::Triplet<double>> triplets;
            for (int i = 0; i < 18; ++i) {
                if (loc[i] < 0) continue;
                for (int j = 0; j < 18; ++j) {
                    if (loc[j] < 0) continue;
                    double val = K_plate(i, j);
                    if (std::abs(val) > 1e-20) {
                        triplets.emplace_back(loc[i], loc[j], val);
                    }
                }
            }

            Eigen::SparseMatrix<double> K_add(total_dofs_, total_dofs_);
            K_add.setFromTriplets(triplets.begin(), triplets.end());
            K += K_add;
        }

        // Step 3: Assemble global mass matrix M (including point masses)
        std::vector<PointMass*> pm_ptrs;
        pm_ptrs.reserve(point_masses.size());
        for (const auto& pm : point_masses) {
            pm_ptrs.push_back(pm.get());
        }

        Eigen::SparseMatrix<double> M = assembler_->assemble_mass(elem_ptrs, pm_ptrs);

        // Check for zero mass (will cause division by zero in eigenvalue problem)
        double total_mass = assembler_->compute_total_mass(elem_ptrs, pm_ptrs);
        if (total_mass < 1e-14) {
            error_msg_ = "Model has zero or negligible mass";
            return false;
        }

        // Step 4: Reduce system (eliminate fixed DOFs)
        EigenvalueSolver eigensolver;
        auto [K_reduced, M_reduced, dof_mapping] = eigensolver.reduce_system(
            K, M, boundary_conditions, dof_handler_);

        int n_free_dofs = static_cast<int>(dof_mapping.size());
        if (n_free_dofs == 0) {
            error_msg_ = "All DOFs are constrained - no free DOFs for eigenvalue analysis";
            return false;
        }

        // Step 5: Solve eigenvalue problem
        EigensolverResult result = eigensolver.solve(K_reduced, M_reduced, settings);

        if (!result.converged) {
            error_msg_ = "Eigenvalue solver did not converge: " + result.message;
            return false;
        }

        // Store DOF mapping info for mode shape expansion
        result.reduced_to_full = dof_mapping;
        result.total_dofs = total_dofs_;

        // Step 6: Compute participation factors if requested (BEFORE expanding mode shapes)
        // Participation factors require reduced-size mode shapes for proper matrix multiplication
        if (settings.compute_participation) {
            // Convert M_reduced to dense for participation factor calculation
            Eigen::MatrixXd M_reduced_dense = Eigen::MatrixXd(M_reduced);
            eigensolver.compute_participation_factors(
                result, M_reduced_dense, dof_mapping, dof_handler_, total_mass);
        }

        // Step 7: Expand mode shapes to full DOF vector
        for (auto& mode : result.modes) {
            Eigen::VectorXd expanded = EigenvalueSolver::expand_mode_shape(
                mode.mode_shape, dof_mapping, total_dofs_);
            mode.mode_shape = expanded;
        }

        // Store results
        eigenvalue_result_ = std::make_unique<EigensolverResult>(std::move(result));

        return true;

    } catch (const std::exception& e) {
        error_msg_ = std::string("Eigenvalue analysis failed: ") + e.what();
        return false;
    }
}

bool Model::has_eigenvalue_results() const {
    return eigenvalue_result_ != nullptr && eigenvalue_result_->converged;
}

const EigensolverResult& Model::get_eigenvalue_result() const {
    if (!eigenvalue_result_) {
        throw std::runtime_error("No eigenvalue results available. Call analyze_eigenvalues() first.");
    }
    return *eigenvalue_result_;
}

std::vector<double> Model::get_natural_frequencies() const {
    if (!eigenvalue_result_) {
        throw std::runtime_error("No eigenvalue results available. Call analyze_eigenvalues() first.");
    }
    return eigenvalue_result_->get_frequencies();
}

std::vector<double> Model::get_periods() const {
    if (!eigenvalue_result_) {
        throw std::runtime_error("No eigenvalue results available. Call analyze_eigenvalues() first.");
    }
    return eigenvalue_result_->get_periods();
}

Eigen::VectorXd Model::get_mode_shape(int mode_number) const {
    if (!eigenvalue_result_) {
        throw std::runtime_error("No eigenvalue results available. Call analyze_eigenvalues() first.");
    }

    const ModeResult* mode = eigenvalue_result_->get_mode(mode_number);
    if (!mode) {
        throw std::runtime_error("Mode " + std::to_string(mode_number) + " not found");
    }

    // Mode shapes are already expanded in analyze_eigenvalues()
    return mode->mode_shape;
}

} // namespace grillex
