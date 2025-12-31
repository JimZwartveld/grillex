# Grillex Test Suite Overview

This document provides a comprehensive overview of all tests in the Grillex project, organized by Python and C++ test files.

*Last updated: 2025-12-31*

---

## Summary

| Language | Test Files | Test Cases | Notes |
|----------|------------|------------|-------|
| Python | 41 | ~550+ | Main test suite |
| C++ | 6 | 57 | Tests for pybind11-incompatible functions |

---

## C++ Tests (Catch2)

Tests moved to C++ because pybind11 cannot handle `Eigen::SparseMatrix` as input parameters.

### test_boundary_conditions.cpp (8 tests)

| Test Case | Tags |
|-----------|------|
| Penalty method modifies stiffness matrix correctly | [BCHandler][apply_to_system] |
| System remains solvable after BC application | [BCHandler][apply_to_system] |
| Fixed DOFs result in zero displacement | [BCHandler][apply_to_system] |
| Prescribed displacement is applied correctly | [BCHandler][apply_to_system] |
| Reactions can be recovered from K * u - F | [BCHandler][apply_to_system] |
| AC1: Fixed DOFs result in zero (or prescribed) displacement | [BCHandler][acceptance] |
| AC2: Reactions can be recovered from K * u - F | [BCHandler][acceptance] |
| AC3: System remains solvable after BC application | [BCHandler][acceptance] |

### test_eigenvalue_solver.cpp (12 tests)

| Test Case | Tags |
|-----------|------|
| reduce_system: fixed node reduces DOF count | [EigenvalueSolver][reduce_system] |
| reduce_system: preserves matrix symmetry | [EigenvalueSolver][reduce_system] |
| reduce_system: DOF mapping excludes fixed DOFs | [EigenvalueSolver][reduce_system] |
| solve: cantilever beam first mode frequency | [EigenvalueSolver][solve] |
| solve: all eigenvalues are non-negative | [EigenvalueSolver][solve] |
| solve: modes are sorted by frequency | [EigenvalueSolver][solve] |
| solve: mode shapes are mass normalized | [EigenvalueSolver][solve] |
| solve: mode shapes satisfy eigenvalue equation | [EigenvalueSolver][solve] |
| EigensolverResult: get_frequencies returns sorted list | [EigenvalueSolver][result] |
| EigensolverResult: expand_mode_shape works correctly | [EigenvalueSolver][result] |
| AC: EigenvalueSolver.reduce_system properly eliminates fixed DOFs | [EigenvalueSolver][acceptance] |
| AC: Eigenvalue solver produces positive eigenvalues for positive-definite system | [EigenvalueSolver][acceptance] |

### test_singularity_diagnostics.cpp (19 tests)

| Test Case | Tags |
|-----------|------|
| SingularityAnalyzerSettings: default values | [SingularityAnalyzer][settings] |
| SingularityAnalyzerSettings: custom values | [SingularityAnalyzer][settings] |
| RigidBodyModeType: enum values exist | [SingularityAnalyzer][enum] |
| RigidBodyModeType: to_string conversion | [SingularityAnalyzer][enum] |
| SingularityDiagnostics: default values | [SingularityAnalyzer][diagnostics] |
| analyze: free-floating beam detected as singular | [SingularityAnalyzer][analyze] |
| analyze: free-floating beam unconstrained DOFs identified | [SingularityAnalyzer][analyze] |
| analyze: free-floating beam nodes needing constraints identified | [SingularityAnalyzer][analyze] |
| analyze: free-floating beam helpful messages generated | [SingularityAnalyzer][analyze] |
| analyze: free-floating beam rigid body mode info available | [SingularityAnalyzer][analyze] |
| is_singular: free-floating beam returns true | [SingularityAnalyzer][is_singular] |
| count_rigid_body_modes: free-floating beam has 6 modes | [SingularityAnalyzer][count] |
| analyze: cantilever beam not singular | [SingularityAnalyzer][analyze] |
| is_singular: cantilever beam returns false | [SingularityAnalyzer][is_singular] |
| count_rigid_body_modes: cantilever beam has 0 modes | [SingularityAnalyzer][count] |
| analyze: partially constrained beam detects missing constraints | [SingularityAnalyzer][analyze] |
| AC: Free-floating model detected as singular | [SingularityAnalyzer][acceptance] |
| AC: Specific unconstrained DOFs identified | [SingularityAnalyzer][acceptance] |
| AC: Helpful diagnostic message generated | [SingularityAnalyzer][acceptance] |

### test_constraints.cpp (12 tests)

| Test Case | Tags |
|-----------|------|
| Equality constraint: two cantilevers with tied tips | [ConstraintHandler][equality] |
| Rigid link with offset: rotation coupling | [ConstraintHandler][rigid_link] |
| AC1: Slave moves with master translation | [ConstraintHandler][kinematics][acceptance] |
| AC2: Rotation produces translation at slave | [ConstraintHandler][kinematics][acceptance] |
| AC3: Force transfer through rigid link | [ConstraintHandler][kinematics][acceptance] |
| AC1: Simple equality constraints work | [ConstraintHandler][acceptance] |
| AC2: Rigid links transfer forces correctly | [ConstraintHandler][acceptance] |
| AC2: Full displacements recovered correctly (equality) | [ConstraintHandler][recovery][acceptance] |
| AC2: Full displacements recovered correctly (rigid link) | [ConstraintHandler][recovery][acceptance] |
| AC3: Equality constraints satisfied | [ConstraintHandler][constraint_satisfaction][acceptance] |
| AC3: Rigid link constraints satisfied | [ConstraintHandler][constraint_satisfaction][acceptance] |
| Complete MPC workflow | [ConstraintHandler][workflow][acceptance] |

### test_internal_actions.cpp (3 tests)

| Test Case | Tags |
|-----------|------|
| Bimoment continuous at internal support | [InternalActions][bimoment][continuity] |
| Cantilever bimoment matches analytical solution | [InternalActions][bimoment][analytical] |
| Bimoment distribution matches analytical along beam | [InternalActions][bimoment][distribution] |

### test_warping.cpp (3 tests)

| Test Case | Tags |
|-----------|------|
| T-joint with torque shows no warping coupling between orthogonal beams | [Warping][decoupling][T-joint] |
| Continuous beam shows warping continuity at internal nodes | [Warping][continuity] |
| Warping continuity produces coupled bimoments | [Warping][continuity][bimoments] |

---

## Python Tests (pytest)

### Phase 1: Core Data Structures

#### test_phase1_data_structures.py

| Test Class | Test Method |
|------------|-------------|
| TestNodeClass | test_node_construction_with_id_and_coordinates |
| TestNodeClass | test_position_as_eigen_vector |
| TestNodeClass | test_dof_arrays_initialized |
| TestNodeRegistry | test_creating_node_twice_returns_same_node |
| TestNodeRegistry | test_node_within_tolerance_returns_existing |
| TestNodeRegistry | test_node_outside_tolerance_creates_new |
| TestNodeRegistry | test_node_ids_sequential_and_unique |
| TestNodeRegistry | test_get_node_by_id |
| TestNodeRegistry | test_get_node_by_invalid_id |
| TestNodeRegistry | test_all_nodes |
| TestNodeRegistry | test_tolerance_setter_getter |
| TestMaterialClass | test_material_construction |
| TestMaterialClass | test_shear_modulus_computed |
| TestMaterialClass | test_compute_G_static_method |
| TestMaterialClass | test_units_documented |
| TestSectionClass | test_section_construction |
| TestSectionClass | test_optional_properties_default_to_zero |
| TestSectionClass | test_set_warping_constant |
| TestSectionClass | test_set_shear_areas |
| TestSectionClass | test_set_fibre_distances |
| TestPythonBindings | test_import_from_core |
| TestPythonBindings | test_create_instances_from_python |
| TestPythonBindings | test_properties_accessible |
| TestPythonBindings | test_repr_methods |

### Phase 2: Beam Elements

#### test_phase2_beam_element.py

| Test Class | Test Method |
|------------|-------------|
| TestLocalAxes | test_horizontal_beam_along_x |
| TestLocalAxes | test_vertical_beam_no_nan |
| TestLocalAxes | test_roll_angle_rotates_y_and_z |
| TestLocalAxes | test_rotation_matrix_orthonormal |
| TestLocalAxes | test_to_local_to_global_roundtrip |
| TestBeamElementStiffness | test_stiffness_matrix_is_symmetric |
| TestBeamElementStiffness | test_stiffness_matrix_positive_semidefinite |
| TestBeamElementStiffness | test_cantilever_deflection |
| TestBeamElementStiffness | test_axial_stiffness |
| TestBeamElementStiffness | test_transformation_matrix_structure |
| TestBeamElementMass | test_mass_matrix_is_symmetric |
| TestBeamElementMass | test_mass_matrix_positive_semidefinite |
| TestBeamElementMass | test_total_mass_equals_rho_A_L |
| TestBeamElementOffsets | test_beam_without_offsets_has_no_offsets |
| TestBeamElementOffsets | test_beam_with_offsets_reports_correctly |
| TestBeamElementOffsets | test_effective_length_with_axial_offsets |
| TestBeamElementOffsets | test_effective_length_with_transverse_offsets |
| TestBeamElementOffsets | test_offset_affects_stiffness_matrix |
| TestBeamElementOffsets | test_offset_affects_mass_matrix |
| TestBeamElementOffsets | test_cantilever_with_offset_tip_load |
| TestPythonBindings | test_import_beam_classes |
| TestPythonBindings | test_create_beam_element_from_python |
| TestPythonBindings | test_beam_element_matrices_accessible |
| TestPythonBindings | test_local_axes_accessible |
| TestTimoshenkoBeam | test_slender_beam_timoshenko_matches_euler_bernoulli |
| TestTimoshenkoBeam | test_deep_beam_timoshenko_softer_than_euler_bernoulli |
| TestTimoshenkoBeam | test_timoshenko_stiffness_symmetric |
| TestTimoshenkoBeam | test_timoshenko_stiffness_positive_semidefinite |
| TestTimoshenkoBeam | test_timoshenko_mass_matrix_symmetric |
| TestTimoshenkoBeam | test_timoshenko_cantilever_deflection |
| TestTimoshenkoBeam | test_shear_deformation_factor_calculation |
| TestTimoshenkoBeam | test_axial_and_torsion_unchanged |
| TestWarpingBeam | test_node_warping_dof_control |
| TestWarpingBeam | test_section_warping_configuration |
| TestWarpingBeam | test_warping_stiffness_matrix_is_symmetric |
| TestWarpingBeam | test_warping_mass_matrix_is_symmetric |
| TestWarpingBeam | test_warping_transformation_matrix_structure |
| TestWarpingBeam | test_zero_warping_constant_behaves_like_12dof |
| TestWarpingBeam | test_warping_increases_torsional_stiffness |
| TestWarpingBeam | test_warping_stiffness_positive_semidefinite |
| TestWarpingBeam | test_cantilever_with_warping_restrained |
| TestWarpingBeam | test_warping_dof_arrays_size_seven |
| TestEndReleases | test_end_release_struct_creation |
| TestEndReleases | test_release_moment_convenience_methods |
| TestEndReleases | test_release_all_rotations_convenience_methods |
| TestEndReleases | test_get_released_indices_12dof |
| TestEndReleases | test_get_released_indices_14dof |
| TestEndReleases | test_simply_supported_beam_stiffness |
| TestEndReleases | test_pinned_fixed_beam_stiffness |
| TestEndReleases | test_axial_release_sliding_joint |
| TestEndReleases | test_torsion_release |
| TestEndReleases | test_mass_matrix_with_releases |
| TestEndReleases | test_warping_release_14dof |
| TestEndReleases | test_multiple_releases_combined |
| TestEndReleases | test_timoshenko_beam_with_releases |
| TestEndReleases | test_beam_element_has_releases_member |
| TestWarpingOffsetTransformation | test_offset_transformation_matrix_warping_size |
| TestWarpingOffsetTransformation | test_offset_transformation_matrix_warping_no_offsets |
| TestWarpingOffsetTransformation | test_offset_transformation_warping_dof_uncoupled |
| TestWarpingOffsetTransformation | test_offset_transformation_translation_rotation_coupling |
| TestWarpingOffsetTransformation | test_stiffness_with_offsets_and_warping |
| TestWarpingOffsetTransformation | test_mass_with_offsets_and_warping |
| TestWarpingOffsetTransformation | test_global_matrices_with_offsets_and_warping |
| TestWarpingOffsetTransformation | test_offset_consistency_12dof_vs_14dof |

#### test_beam_factory.py

| Test Class | Test Method |
|------------|-------------|
| TestBeamConfig | test_default_config |
| TestBeamConfig | test_config_get_formulation |
| TestBeamConfig | test_config_explicit_timoshenko |
| TestBeamConfig | test_config_with_warping |
| TestBeamElementFactory | test_factory_creates_euler_bernoulli_element |
| TestBeamElementFactory | test_factory_creates_timoshenko_element |
| TestBeamElementFactory | test_factory_creates_warping_element |
| TestBeamElementFactory | test_factory_creates_timoshenko_with_warping |
| TestBeamElementFactory | test_factory_with_include_shear_deformation |
| TestPolymorphicInterface | test_polymorphic_stiffness_matrix_12dof |
| TestPolymorphicInterface | test_polymorphic_stiffness_matrix_14dof |
| TestPolymorphicInterface | test_polymorphic_mass_matrix_12dof |
| TestPolymorphicInterface | test_polymorphic_mass_matrix_14dof |
| TestPolymorphicInterface | test_polymorphic_transformation_matrix_12dof |
| TestPolymorphicInterface | test_polymorphic_transformation_matrix_14dof |
| TestBackwardCompatibility | test_old_constructor_still_works |
| TestBackwardCompatibility | test_old_methods_still_work |
| TestBackwardCompatibility | test_new_constructor_with_config |

### Phase 3: Assembly & Solver

#### test_phase3_assembler.py

| Test Class | Test Method |
|------------|-------------|
| TestAssembler | test_assembler_creation |
| TestAssembler | test_assemble_single_12dof_element_stiffness |
| TestAssembler | test_assemble_single_12dof_element_mass |
| TestAssembler | test_assemble_single_14dof_element_stiffness |
| TestAssembler | test_assemble_single_14dof_element_mass |
| TestAssembler | test_assemble_two_12dof_elements |
| TestAssembler | test_assemble_mixed_12dof_14dof_elements |
| TestAssemblerAcceptanceCriteria | test_assembled_matrix_is_sparse |
| TestAssemblerAcceptanceCriteria | test_assembled_matrix_is_symmetric |
| TestAssemblerAcceptanceCriteria | test_single_12dof_element_matches_element_matrix |
| TestAssemblerAcceptanceCriteria | test_single_14dof_element_matches_element_matrix |
| TestAssemblerAcceptanceCriteria | test_mixed_12dof_14dof_assembly_works |

#### test_phase3_dof_handler.py

| Test Class | Test Method |
|------------|-------------|
| TestDOFHandler | test_dof_handler_creation |
| TestDOFHandler | test_simple_two_node_numbering |
| TestDOFHandler | test_inactive_dof_returns_minus_one |
| TestDOFHandler | test_warping_dof_numbering |
| TestDOFHandler | test_mixed_warping_numbering |
| TestDOFHandler | test_location_array_12dof_element |
| TestDOFHandler | test_location_array_14dof_element |
| TestDOFHandler | test_clear_method |
| TestDOFHandler | test_renumbering_after_modifications |
| TestDOFHandlerAcceptanceCriteria | test_unique_global_numbers |
| TestDOFHandlerAcceptanceCriteria | test_location_arrays_correct_mapping |
| TestDOFHandlerAcceptanceCriteria | test_inactive_dofs_not_numbered |
| TestDOFHandlerAcceptanceCriteria | test_warping_numbered_only_when_active |
| TestDOFHandlerAcceptanceCriteria | test_mixed_12dof_14dof_elements |

#### test_phase3_boundary_conditions.py

| Test Class | Test Method | Uses apply_to_system |
|------------|-------------|---------------------|
| TestDOFIndex | test_dof_index_values | No |
| TestDOFIndex | test_dof_index_usage | No |
| TestFixedDOF | test_fixed_dof_creation | No |
| TestFixedDOF | test_fixed_dof_default_value | No |
| TestFixedDOF | test_fixed_dof_repr | No |
| TestBCHandler | test_bc_handler_creation | No |
| TestBCHandler | test_add_fixed_dof | No |
| TestBCHandler | test_add_duplicate_fixed_dof | No |
| TestBCHandler | test_fix_node | No |
| TestBCHandler | test_fix_node_with_warping | No |
| TestBCHandler | test_pin_node | No |
| TestBCHandler | test_fork_support | No |
| TestBCHandler | test_clear | No |
| TestBCHandler | test_repr | No |
| TestBCHandlerWithDOFHandler | test_get_fixed_global_dofs | No |
| TestBCHandlerWithDOFHandler | test_get_fixed_global_dofs_with_warping | No |
| TestBCHandlerWithDOFHandler | test_get_fixed_global_dofs_pin_support | No |
| TestSimplySupportedBeam | test_simply_supported_beam_bcs | No |
| TestWarpingBoundaryConditions | test_warping_dof_can_be_fixed | No |
| TestWarpingBoundaryConditions | test_warping_dof_left_free | No |
| TestWarpingBoundaryConditions | test_fork_support_leaves_warping_free | No |
| TestWarpingBoundaryConditions | test_built_in_with_warping_restrains_warping | No |
| TestAcceptanceCriteria | test_ac4_warping_dof_can_be_fixed_or_free | No |
| TestAcceptanceCriteria | test_ac5_fork_support_leaves_warping_free | No |
| TestAcceptanceCriteria | test_ac6_built_in_with_warping_restrains_warping | No |

**Note:** Tests for AC1-AC3 (penalty method, reactions, solvability) moved to C++ `test_boundary_conditions.cpp`.

#### test_phase3_solver.py

| Test Class | Test Method | Uses apply_to_system |
|------------|-------------|---------------------|
| TestLinearSolverBasics | test_solver_creation_default | No |
| TestLinearSolverBasics | test_solver_creation_with_method | No |
| TestLinearSolverBasics | test_solver_method_change | No |
| TestLinearSolverBasics | test_solver_repr | No |
| TestSingularityDetection | test_singular_system_all_dofs_free | No |
| TestIterativeSolverSettings | test_set_solver_parameters | No |
| TestAcceptanceCriteria | test_ac2_singular_systems_detected | No |
| TestErrorHandling | test_dimension_mismatch_k_not_square | No |
| TestErrorHandling | test_dimension_mismatch_k_f_incompatible | No |

**Note:** Tests for AC1, AC3 (correct displacement, performance) moved to C++ `test_boundary_conditions.cpp`.

#### test_phase3_model.py

| Test Class | Test Method |
|------------|-------------|
| TestModelCreation | test_model_default_creation |
| TestModelCreation | test_model_custom_parameters |
| TestModelCreation | test_create_material |
| TestModelCreation | test_create_section |
| TestModelCreation | test_create_beam |
| TestModelLoadsAndBCs | test_add_nodal_load |
| TestModelLoadsAndBCs | test_add_multiple_loads_same_dof |
| TestModelLoadsAndBCs | test_clear_loads |
| TestModelLoadsAndBCs | test_boundary_conditions |
| TestSimpleCantileverAnalysis | test_cantilever_beam_analysis |
| TestErrorHandling | test_analyze_empty_model |
| TestErrorHandling | test_analyze_without_boundary_conditions |
| TestErrorHandling | test_get_displacements_before_analysis |
| TestErrorHandling | test_get_node_displacement_before_analysis |
| TestErrorHandling | test_get_reactions_before_analysis |
| TestModelClear | test_clear_model |
| TestAcceptanceCriteria | test_ac1_complete_workflow_runs_without_errors |
| TestAcceptanceCriteria | test_ac2_results_match_hand_calculations |
| TestAcceptanceCriteria | test_ac3_error_handling_invalid_models |
| TestMultiElementModel | test_three_span_beam |

### Phase 4: Python Front-End & I/O

#### test_phase4_yaml_loader.py

| Test Class | Test Method |
|------------|-------------|
| TestYAMLLoading | test_load_simple_cantilever |
| TestYAMLLoading | test_load_multi_span_beam |
| TestYAMLLoading | test_loaded_model_can_be_analyzed |
| TestYAMLLoading | test_loaded_model_with_multiple_load_cases |
| TestEntityTypes | test_materials_loaded |
| TestEntityTypes | test_sections_loaded |
| TestEntityTypes | test_beams_loaded |
| TestEntityTypes | test_boundary_conditions_loaded |
| TestEntityTypes | test_boundary_condition_types |
| TestEntityTypes | test_load_cases_loaded |
| TestErrorHandling | test_file_not_found |
| TestErrorHandling | test_empty_yaml_file |
| TestErrorHandling | test_invalid_yaml_syntax |
| TestErrorHandling | test_yaml_root_not_dict |
| TestErrorHandling | test_missing_required_material_field |
| TestErrorHandling | test_missing_required_section_field |
| TestErrorHandling | test_missing_required_beam_field |
| TestErrorHandling | test_invalid_beam_coordinates |
| TestErrorHandling | test_beam_references_nonexistent_material |
| TestErrorHandling | test_invalid_dof_name |
| TestErrorHandling | test_invalid_load_case_type |
| TestErrorHandling | test_invalid_bc_type |
| TestDefaultValues | test_model_name_defaults_to_filename |
| TestDefaultValues | test_load_case_type_defaults_to_variable |
| TestAcceptanceCriteria | test_ac1_valid_yaml_files_load_without_error |
| TestAcceptanceCriteria | test_ac2_all_entity_types_supported |
| TestAcceptanceCriteria | test_ac3_clear_error_messages_for_invalid_yaml |

#### test_phase4_python_wrapper.py

| Test Class | Test Method |
|------------|-------------|
| TestBeamClass | test_beam_creation |
| TestBeamClass | test_beam_length_calculation |
| TestBeamClass | test_beam_direction_vector |
| TestBeamClass | test_beam_midpoint |
| TestBeamClass | test_beam_repr |

#### test_phase4_beam_subdivision.py

| Test Class | Test Method |
|------------|-------------|
| TestPointOnLineSegment | test_point_on_horizontal_line |
| TestPointOnLineSegment | test_point_at_endpoints_not_detected |
| TestPointOnLineSegment | test_point_off_line_not_detected |
| TestPointOnLineSegment | test_point_outside_segment_not_detected |
| TestPointOnLineSegment | test_diagonal_line_segment |

#### test_phase4_json_export.py

| Test Class | Test Method |
|------------|-------------|
| TestDataClasses | test_node_result_creation |
| TestDataClasses | test_element_result_creation |
| TestDataClasses | test_load_case_info_creation |
| TestDataClasses | test_model_info_creation |
| TestResultCase | test_result_case_creation |

### Phase 5: Loads

#### test_phase5_line_loads.py

| Test Class | Test Method |
|------------|-------------|
| TestDistributedLoadStruct | test_zero_load |
| TestDistributedLoadStruct | test_uniform_load |
| TestDistributedLoadStruct | test_trapezoidal_load |
| TestEquivalentNodalForces | test_uniform_vertical_load_simple |

#### test_phase5_acceleration_loads.py

| Test Class | Test Method |
|------------|-------------|
| TestGravityLoads | test_cantilever_gravity_basic |
| TestGravityLoads | test_cantilever_gravity_reactions |

#### test_phase5_load_combinations.py

| Test Class | Test Method |
|------------|-------------|
| TestLoadCombinationBasics | test_create_combination_default_factors |
| TestLoadCombinationBasics | test_create_combination_with_type_factors |
| TestLoadCombinationBasics | test_set_type_factor |
| TestAddLoadCases | test_add_load_case_type_based_factor |

### Phase 6: Constraints

#### test_phase6_constraints.py

| Test Class | Test Method | Uses apply_to_system |
|------------|-------------|---------------------|
| TestConstraintCreation | test_create_constraint_handler | No |
| TestConstraintCreation | test_add_equality_constraint | No |
| TestConstraintCreation | test_add_rigid_link | No |
| TestConstraintCreation | test_clear_constraints | No |
| TestConstraintCreation | test_equality_constraint_self_reference_error | No |
| TestConstraintCreation | test_rigid_link_same_node_error | No |
| TestSkewMatrix | test_skew_matrix_computation | No |
| TestSkewMatrix | test_skew_matrix_antisymmetric | No |
| TestTransformationMatrix | test_no_constraints_identity | No |
| TestTransformationMatrix | test_equality_constraint_matrix_dimensions | No |
| TestTransformationMatrix | test_rigid_link_matrix_dimensions | No |
| TestSystemReduction | test_reduce_system_basic | No (uses reduce_system) |
| TestSystemReduction | test_reduce_system_no_constraints | No (uses reduce_system) |
| TestDisplacementExpansion | test_expand_displacements_identity | No |
| TestDisplacementExpansion | test_expand_displacements_with_constraints | No |
| TestEqualityConstraintAnalysis | test_simple_equality_two_cantilevers | **YES** |
| TestRigidLinkAnalysis | test_rigid_link_zero_offset | No |
| TestRigidLinkAnalysis | test_rigid_link_with_offset | **YES** |
| TestTask62RigidLinkKinematics | test_transformation_block_6x6_structure | No |
| TestTask62RigidLinkKinematics | test_transformation_block_zero_offset | No |
| TestTask62RigidLinkKinematics | test_ac1_slave_moves_with_master_translation | **YES** |
| TestTask62RigidLinkKinematics | test_ac2_rotation_produces_translation | **YES** |
| TestTask62RigidLinkKinematics | test_ac3_force_transfer_through_rigid_link | **YES** |
| TestTask62RigidLinkKinematics | test_full_6dof_rigid_link_coupling | No |
| TestAcceptanceCriteria | test_ac1_simple_equality_constraints_work | **YES** |
| TestAcceptanceCriteria | test_ac2_rigid_links_transfer_forces_correctly | **YES** |
| TestAcceptanceCriteria | test_ac3_transformation_matrix_correct | No |
| TestNumSlaveDofs | test_num_slave_dofs_equality | No |
| TestNumSlaveDofs | test_num_slave_dofs_rigid_link | No |
| TestTask63ApplyMPCToGlobalSystem | test_ac1_reduced_system_smaller_equality | No |
| TestTask63ApplyMPCToGlobalSystem | test_ac1_reduced_system_smaller_rigid_link | No |
| TestTask63ApplyMPCToGlobalSystem | test_ac2_full_displacements_recovered_equality | **YES** |
| TestTask63ApplyMPCToGlobalSystem | test_ac2_full_displacements_recovered_rigid_link | **YES** |
| TestTask63ApplyMPCToGlobalSystem | test_ac3_equality_constraints_satisfied | **YES** |
| TestTask63ApplyMPCToGlobalSystem | test_ac3_rigid_link_constraints_satisfied | **YES** |
| TestTask63ApplyMPCToGlobalSystem | test_complete_workflow_with_mpc | **YES** |

**Note:** 13 tests use `apply_to_system` - C++ equivalents created in `test_constraints.cpp`.

### Phase 7: Internal Actions & Results

#### test_phase7_internal_actions.py

| Test Class | Test Method | Uses apply_to_system |
|------------|-------------|---------------------|
| TestCantileverTipLoad | test_moment_at_base | No |
| TestCantileverTipLoad | test_moment_at_tip | No |
| TestCantileverTipLoad | test_shear_is_constant | No |
| TestTask72bBimomentContinuity | test_bimoment_continuous_at_internal_support | **YES** (line 1454) |
| TestCantileverIBeamAnalyticalComparison | test_cantilever_bimoment_analytical | **YES** (line 1543) |
| TestCantileverIBeamAnalyticalComparison | test_bimoment_distribution_along_beam | **YES** (line 1634) |

**Note:** 3 tests use `apply_to_system` - C++ equivalents created in `test_internal_actions.cpp`.

#### test_phase7_end_forces.py

| Test Class | Test Method |
|------------|-------------|
| TestEndForcesStruct | test_default_construction |
| TestEndForcesStruct | test_construction_with_components |
| TestEndForcesStruct | test_construction_without_bimoment |
| TestEndForcesStruct | test_to_vector6 |
| TestEndForcesStruct | test_to_vector7 |
| TestEndForcesStruct | test_repr |
| TestInternalActionsStruct | test_default_construction |
| TestInternalActionsStruct | test_construction_with_position |

#### test_phase7_python_bindings.py

| Test Class | Test Method |
|------------|-------------|
| TestPhase7StructsAccessible | test_internal_actions_accessible |
| TestPhase7StructsAccessible | test_end_forces_accessible |
| TestPhase7StructsAccessible | test_displacement_line_accessible |

#### test_phase7_check_locations.py

| Test Class | Test Method |
|------------|-------------|
| TestCheckLocationManagement | test_add_check_location_valid |
| TestCheckLocationManagement | test_add_check_location_invalid |
| TestCheckLocationManagement | test_add_check_location_sorted |
| TestCheckLocationManagement | test_add_check_location_no_duplicates |

#### test_phase7_distributed_load_query.py

| Test Class | Test Method |
|------------|-------------|
| TestDistributedLoadQueryBasics | test_no_loads_returns_zero |
| TestDistributedLoadQueryBasics | test_load_on_different_element_returns_zero |
| TestHorizontalBeamAlongX | (various tests) |

#### test_phase7_displacement_multielem.py

| Test Class | Test Method |
|------------|-------------|
| TestDisplacementAtElementEnds | test_cantilever_tip_displacement_matches_nodal |
| TestDisplacementAtElementEnds | test_cantilever_fixed_end_displacement_zero |

### Warping Tests

#### test_warping_decoupling.py

| Test Class | Test Method | Uses apply_to_system |
|------------|-------------|---------------------|
| TestDirectionVector | test_horizontal_beam_x_direction | No |
| TestDirectionVector | test_horizontal_beam_y_direction | No |
| TestDirectionVector | test_vertical_beam | No |
| TestDirectionVector | test_diagonal_beam | No |
| TestDirectionVector | test_direction_is_unit_vector | No |
| TestWarpingDOFDecoupling | test_independent_warping_at_tjoint | **YES** (line 792) |
| TestContinuousBeamWarpingContinuity | test_continuous_beam_warping_continuity | **YES** (line 867) |
| TestContinuousBeamWarpingContinuity | test_warping_continuity_produces_coupled_bimoments | **YES** (line 927) |

**Note:** 3 tests use `apply_to_system` - C++ equivalents created in `test_warping.cpp`.

### Phase 11: Error Handling & Diagnostics

#### test_phase11_singularity_diagnostics.py

| Test Class | Test Method |
|------------|-------------|
| TestSingularityAnalyzerSettings | test_default_values |
| TestSingularityAnalyzerSettings | test_custom_settings |
| TestRigidBodyModeType | test_translation_modes |
| TestRigidBodyModeType | test_rotation_modes |
| TestRigidBodyModeType | test_special_modes |
| TestSingularityDiagnostics | test_default_values |
| TestSingularityDiagnostics | test_bool_conversion |
| TestSingularityAnalyzer | test_analyzer_creation |

**Note:** Tests using `analyze()`, `is_singular()`, `count_rigid_body_modes()` moved to C++ `test_singularity_diagnostics.cpp`.

### Phase 16: Eigenvalue Analysis

#### test_phase16_eigenvalue.py

| Test Class | Test Method |
|------------|-------------|
| TestEigensolverSettings | test_default_settings |
| TestEigensolverSettings | test_modify_settings |
| TestEigensolverSettings | test_repr |
| TestEigensolverMethod | test_enum_values |
| TestEigensolverMethod | test_enum_assignment |
| TestModeResult | test_default_mode_result |
| TestModeResult | test_participation_factors |
| TestModeResult | test_effective_mass_fields |
| TestEigensolverResult | test_default_result |
| TestEigensolverResult | test_get_frequencies |
| TestEigensolverResult | test_get_periods |
| TestExpandModeShape | test_expand_mode_shape |
| TestPointMassAssembly | test_point_mass_creation |
| TestPointMassAssembly | test_assemble_mass_with_point_masses |
| TestPointMassAssembly | test_compute_total_mass |
| TestModelIntegration | test_model_analyze_eigenvalues_basic |
| TestModelIntegration | test_model_get_natural_frequencies |
| TestModelIntegration | test_model_get_mode_shape |
| TestModelIntegration | test_model_eigenvalue_result_access |
| TestModelIntegration | test_model_eigenvalue_with_participation |
| TestStructuralModelEigenvalue | test_structural_model_analyze_modes |
| TestStructuralModelEigenvalue | test_structural_model_get_frequencies |
| TestStructuralModelEigenvalue | test_structural_model_get_periods |
| TestStructuralModelEigenvalue | test_structural_model_get_mode_shape |
| TestStructuralModelEigenvalue | test_structural_model_get_mode_displacement_at |
| TestStructuralModelEigenvalue | test_structural_model_eigenvalue_methods |
| TestStructuralModelEigenvalue | test_structural_model_frequency_period_consistency |
| TestEigenvalueAnalytical | test_simply_supported_beam_first_mode |
| TestEigenvalueAnalytical | test_simply_supported_beam_higher_modes |
| TestEigenvalueAnalytical | test_cantilever_beam_first_mode |
| TestEigenvalueAnalytical | test_cantilever_beam_higher_modes |
| TestEigenvalueAnalytical | test_cantilever_mode_shape |
| TestEigenvalueAnalytical | test_sdof_spring_mass_system |
| TestEigenvalueAnalytical | test_two_dof_spring_mass |
| TestEigenvalueAnalytical | test_free_free_beam_rigid_body_modes |
| TestParticipationFactorsValidation | test_cantilever_z_participation_dominates |
| TestParticipationFactorsValidation | test_symmetric_structure_symmetric_participation |
| TestParticipationFactorsValidation | test_cumulative_mass_approaches_100 |
| TestParticipationFactorsValidation | test_point_mass_increases_modal_mass |
| TestParticipationFactorsValidation | test_participation_cumulative_monotonic |
| TestParticipationFactorsValidation | test_effective_mass_nonzero |
| TestEigenvalueIntegration | test_grillage_natural_frequencies |
| TestEigenvalueIntegration | test_mixed_elements_beams_and_point_masses |
| TestEigenvalueIntegration | test_mixed_elements_with_springs |
| TestEigenvalueIntegration | test_warping_elements |
| TestEigenvalueIntegration | test_large_model_performance |
| TestEigenvalueIntegration | test_yaml_model_eigenvalue |
| TestEigenvalueIntegration | test_mode_shape_continuity |
| TestEigenvalueIntegration | test_cantilever_with_subdivision |
| TestEigenvalueIntegration | test_frame_structure |

### Phase 19: Plate Elements

#### test_phase19_plate_meshing.py

| Test Class | Test Method |
|------------|-------------|
| TestPlateElementAPI | test_add_plate_element_creates_plate |
| TestPlateElementAPI | test_add_plate_element_returns_plate_element |
| TestPlateElementAPI | test_get_plate_elements |
| TestPlateElementAPI | test_add_plate_element_missing_material |
| TestPlateElementAPI | test_add_plate_element_nodes_created |
| TestPlateElementAPI | test_plate_stiffness_matrix_shape |
| TestPlateElementAPI | test_plate_mass_matrix_shape |
| TestPlateGeometry | test_plate_creation_quad |
| TestPlateGeometry | test_plate_creation_triangle |
| TestPlateGeometry | test_plate_creation_pentagon |
| TestPlateGeometry | test_plate_minimum_corners |
| TestPlateGeometry | test_plate_corner_validation |
| TestPlateGeometry | test_get_edge |
| TestPlateGeometry | test_get_edge_length |
| TestPlateGeometry | test_get_edge_direction |
| TestPlateGeometry | test_get_normal_horizontal_plate |
| TestPlateGeometry | test_get_normal_vertical_plate |
| TestPlateGeometry | test_is_planar_horizontal |
| TestPlateGeometry | test_is_planar_pentagon |
| TestPlateGeometry | test_is_planar_non_planar |
| TestPlateGeometry | test_get_area_square |
| TestPlateGeometry | test_get_area_rectangle |
| TestPlateGeometry | test_get_centroid |
| TestPlateGeometry | test_get_bounding_box |
| TestPlateGeometry | test_is_convex_square |
| TestPlateGeometry | test_is_convex_non_convex |
| TestPlateGeometry | test_edge_mesh_control |
| TestPlateGeometry | test_plate_default_values |
| TestGmshMesher | test_gmsh_import_error_message |
| TestGmshMesher | test_mesh_result_properties |
| TestGmshMesher | test_mesh_quad_plate |
| TestGmshMesher | test_mesh_with_edge_divisions |
| TestGmshMesher | test_mesh_triangle_plate |
| TestGmshMesher | test_mesh_pentagon |
| TestGmshMesher | test_edge_nodes_tracked |
| TestGmshMesher | test_context_manager |
| TestGmshMesher | test_mesh_from_plate_geometry |
| TestGmshMesher | test_invalid_corners |
| TestGmshMesher | test_invalid_corner_coordinates |
| TestPlateIntegration | test_plate_element_analysis |

#### test_task19_7_add_plate.py

| Test Class | Test Method |
|------------|-------------|
| TestAddPlate | test_add_simple_quad_plate |
| TestAddPlate | test_add_triangular_plate |
| TestAddPlate | test_add_plate_with_name |
| TestAddPlate | test_add_plate_with_different_element_types |
| TestAddPlate | test_add_plate_missing_material_raises |
| TestAddPlate | test_add_plate_invalid_element_type_raises |
| TestAddPlate | test_add_plate_nonplanar_raises |
| TestAddPlate | test_add_plate_too_few_corners_raises |
| TestSetEdgeDivisions | test_set_edge_divisions |
| TestSetEdgeDivisions | test_set_edge_divisions_all_edges |
| TestSetEdgeDivisions | test_set_edge_divisions_invalid_index_raises |
| TestSetEdgeDivisions | test_set_edge_divisions_negative_index_raises |
| TestSetEdgeDivisions | test_set_edge_divisions_zero_elements_raises |
| TestGetPlates | test_get_plates_empty |
| TestGetPlates | test_get_plates_single |
| TestGetPlates | test_get_plates_multiple |
| TestGetPlates | test_get_plates_returns_copy |
| TestPlateNaming | test_auto_naming_sequential |
| TestPlateNaming | test_custom_name_does_not_affect_counter |
| TestPlateIntegration | test_plate_and_beams_in_same_model |
| TestPlateIntegration | test_multiple_materials_for_plates |

#### test_task19_8_plate_beam_coupling.py

| Test Class | Test Method |
|------------|-------------|
| TestCouplePlateToBeam | test_basic_coupling |
| TestCouplePlateToBeam | test_coupling_with_offset |
| TestCouplePlateToBeam | test_coupling_with_releases |
| TestCouplePlateToBeam | test_coupling_stored_in_plate |
| TestCouplePlateToBeam | test_multiple_couplings |
| TestCouplePlateToBeam | test_coupling_different_edges |
| TestCouplePlateToBeam | test_coupling_invalid_edge_index_raises |
| TestCouplePlateToBeam | test_coupling_negative_edge_index_raises |
| TestCouplePlateToBeam | test_coupling_triangular_plate |
| TestSupportCurve | test_basic_support |
| TestSupportCurve | test_full_support |
| TestSupportCurve | test_support_stored_in_plate |
| TestSupportCurve | test_multiple_supports |
| TestSupportCurve | test_support_invalid_edge_index_raises |
| TestCouplingAndSupportIntegration | test_plate_with_coupling_and_support |
| TestCouplingAndSupportIntegration | test_multiple_plates_with_connections |

#### test_task19_10_mesh.py

| Test Class | Test Method |
|------------|-------------|
| TestMeshBasic | test_mesh_returns_statistics |
| TestMeshBasic | test_mesh_empty_model |
| TestMeshBasic | test_mesh_verbose |
| TestMeshMITC4 | test_mesh_mitc4_quad |
| TestMeshMITC4 | test_mesh_mitc4_structured |
| TestMeshDKT | test_mesh_dkt_triangular_plate |
| TestMeshSupportCurves | test_support_curve_applied_during_mesh |
| TestMeshSupportCurves | test_multiple_supports |
| TestMeshPlateBeamCoupling | test_coupling_creates_rigid_links |
| TestMeshPlateBeamCoupling | test_coupling_no_links_when_colocated |
| TestMeshMultiplePlates | test_mesh_multiple_plates |
| TestMeshMultiplePlates | test_mesh_different_element_types |
| TestMeshEdgeNodes | test_edge_nodes_stored_in_plate |
| TestMeshStatisticsAccuracy | test_statistics_counts_match |
| TestMeshIntegration | test_mesh_then_analyze_structure |

#### test_task19_11_plate_results.py

| Test Class | Test Method |
|------------|-------------|
| TestGetPlateDisplacement | test_displacement_at_center |
| TestGetPlateDisplacement | test_displacement_at_corner |
| TestGetPlateDisplacement | test_displacement_returns_dict |
| TestGetPlateMoments | test_moments_at_center |
| TestGetPlateMoments | test_moments_structure |
| TestGetPlateStress | test_stress_at_surfaces |
| TestGetPlateStress | test_stress_symmetry |
| TestGetPlateStress | test_middle_surface_zero_stress |
| TestGetPlateStress | test_invalid_surface_raises |
| TestGetPlateStress | test_stress_returns_dict |
| TestGetPlateElements | test_returns_all_elements |
| TestGetPlateElements | test_empty_model_returns_empty_list |
| TestDifferentElementTypes | test_mitc4_displacement |
| TestDifferentElementTypes | test_dkt_displacement |
| TestResultsRequireAnalysis | test_displacement_after_analysis |
| TestResultsRequireAnalysis | test_moments_after_analysis |
| TestResultsRequireAnalysis | test_stress_after_analysis |

#### test_plate_element_tri.py

| Test Class | Test Method |
|------------|-------------|
| TestPlateElementTriBasic | test_create_element |
| TestPlateElementTriBasic | test_nodes_stored_correctly |
| TestPlateElementTriBasic | test_material_reference |
| TestPlateElementTriBasic | test_area_equilateral |
| TestPlateElementTriBasic | test_area_right_triangle |
| TestPlateElementTriBasic | test_centroid |
| TestPlateElementTriLocalAxes | test_local_axes_flat_triangle |
| TestPlateElementTriLocalAxes | test_local_axes_inclined |
| TestPlateElementTriLocalAxes | test_transformation |
| TestPlateElementTriStiffnessMatrix | test_stiffness_matrix_shape |
| TestPlateElementTriStiffnessMatrix | test_stiffness_matrix_symmetric |
| TestPlateElementTriStiffnessMatrix | test_stiffness_matrix_positive_semidefinite |
| TestPlateElementTriStiffnessMatrix | test_stiffness_scales_with_thickness_cubed |
| TestPlateElementTriStiffnessMatrix | test_stiffness_scales_with_E |
| TestPlateElementTriMassMatrix | test_mass_matrix_shape |
| TestPlateElementTriMassMatrix | test_mass_matrix_symmetric |
| TestPlateElementTriMassMatrix | test_mass_matrix_positive_semidefinite |
| TestPlateElementTriMassMatrix | test_total_mass_translational |
| TestPlateElementTriDifferentGeometries | test_thin_triangle |
| TestPlateElementTriDifferentGeometries | test_vertical_triangle |
| TestPlateElementTriDifferentGeometries | test_arbitrary_orientation |
| TestPlateElementTriRepr | test_repr |

#### test_plate_elements_higher_order.py

| Test Class | Test Method |
|------------|-------------|
| TestPlateElement8 | test_construction |
| TestPlateElement8 | test_stiffness_matrix_shape |
| TestPlateElement8 | test_stiffness_matrix_symmetry |
| TestPlateElement8 | test_mass_matrix_shape |
| TestPlateElement8 | test_area_calculation |
| TestPlateElement8 | test_centroid |
| TestPlateElement8 | test_local_axes |
| TestPlateElement9 | test_construction |
| TestPlateElement9 | test_stiffness_matrix_shape |
| TestPlateElement9 | test_stiffness_matrix_symmetry |
| TestPlateElement9 | test_mass_matrix_shape |
| TestPlateElement9 | test_area_calculation |
| TestPlateElement9 | test_centroid |
| TestPlateElement9 | test_local_axes |
| TestPlateElementComparison | test_stiffness_increases_with_order |

#### test_element_types.py

| Test Class | Test Method |
|------------|-------------|
| TestPlateElementTypeEnum | test_enum_values |
| TestPlateElementTypeEnum | test_enum_count |
| TestGetElementType | test_get_element_type_uppercase |
| TestGetElementType | test_get_element_type_lowercase |
| TestGetElementType | test_get_element_type_mixed_case |
| TestGetElementType | test_get_element_type_invalid |
| TestElementTypeInfo | test_all_types_have_info |
| TestElementTypeInfo | test_info_has_required_fields |
| TestElementTypeInfo | test_mitc4_info |
| TestElementTypeInfo | test_mitc8_info |
| TestElementTypeInfo | test_mitc9_info |
| TestElementTypeInfo | test_dkt_info |
| TestGetAvailableElementTypes | test_returns_list |
| TestGetAvailableElementTypes | test_contains_all_types |
| TestCreatePlateElement | test_create_mitc4 |
| TestCreatePlateElement | test_create_mitc8 |
| TestCreatePlateElement | test_create_mitc9 |
| TestCreatePlateElement | test_create_dkt |
| TestCreatePlateElement | test_wrong_node_count_mitc4 |
| TestCreatePlateElement | test_wrong_node_count_dkt |
| TestCreatePlateElement | test_invalid_element_type |
| TestGeometryHelpers | test_is_quad_element |
| TestGeometryHelpers | test_is_triangle_element |
| TestGeometryHelpers | test_supports_shear_deformation |
| TestCreatedElementsWork | test_mitc4_stiffness_valid |
| TestCreatedElementsWork | test_mitc8_stiffness_valid |
| TestCreatedElementsWork | test_mitc9_stiffness_valid |
| TestCreatedElementsWork | test_dkt_stiffness_valid |

### Utility & Misc Tests

#### test_basic_imports.py

| Test | Description |
|------|-------------|
| test_grillex_imports | Tests basic grillex imports |
| test_cpp_module_import | Tests C++ module import |
| test_cpp_greeting | Tests C++ greeting function |
| test_cpp_add_numbers | Tests C++ add function |
| test_cpp_module_version | Tests C++ module version |

#### test_gmsh_opengl.py

| Test | Description |
|------|-------------|
| test_gmsh_import | Tests gmsh import |
| test_gmsh_initialization | Tests gmsh initialization |
| test_gmsh_api_version | Tests gmsh API version |
| test_gmsh_geometry_creation | Tests geometry creation |
| test_gmsh_mesh_generation | Tests mesh generation |
| test_gmsh_opengl_graphics_options | Tests OpenGL graphics options |
| test_gmsh_mesh_visualization_options | Tests mesh visualization |
| test_gmsh_color_setting | Tests color setting |
| test_gmsh_mesh_export | Tests mesh export |
| test_gmsh_beam_like_geometry | Tests beam-like geometry |
| test_gmsh_window_size_setting | Tests window size setting |

---

## Tests Migrated to C++

The following Python tests used `apply_to_system` (pybind11-incompatible) and have been migrated to C++:

### test_phase6_constraints.py -> test_constraints.cpp (13 tests)

| Python Test | C++ Equivalent |
|-------------|----------------|
| TestEqualityConstraintAnalysis.test_simple_equality_two_cantilevers | Equality constraint: two cantilevers with tied tips |
| TestRigidLinkAnalysis.test_rigid_link_with_offset | Rigid link with offset: rotation coupling |
| TestTask62RigidLinkKinematics.test_ac1_slave_moves_with_master_translation | AC1: Slave moves with master translation |
| TestTask62RigidLinkKinematics.test_ac2_rotation_produces_translation | AC2: Rotation produces translation at slave |
| TestTask62RigidLinkKinematics.test_ac3_force_transfer_through_rigid_link | AC3: Force transfer through rigid link |
| TestAcceptanceCriteria.test_ac1_simple_equality_constraints_work | AC1: Simple equality constraints work |
| TestAcceptanceCriteria.test_ac2_rigid_links_transfer_forces_correctly | AC2: Rigid links transfer forces correctly |
| TestTask63ApplyMPCToGlobalSystem.test_ac2_full_displacements_recovered_equality | AC2: Full displacements recovered correctly (equality) |
| TestTask63ApplyMPCToGlobalSystem.test_ac2_full_displacements_recovered_rigid_link | AC2: Full displacements recovered correctly (rigid link) |
| TestTask63ApplyMPCToGlobalSystem.test_ac3_equality_constraints_satisfied | AC3: Equality constraints satisfied |
| TestTask63ApplyMPCToGlobalSystem.test_ac3_rigid_link_constraints_satisfied | AC3: Rigid link constraints satisfied |
| TestTask63ApplyMPCToGlobalSystem.test_complete_workflow_with_mpc | Complete MPC workflow |

### test_phase7_internal_actions.py -> test_internal_actions.cpp (3 tests)

| Python Test | C++ Equivalent |
|-------------|----------------|
| TestTask72bBimomentContinuity.test_bimoment_continuous_at_internal_support | Bimoment continuous at internal support |
| TestCantileverIBeamAnalyticalComparison.test_cantilever_bimoment_analytical | Cantilever bimoment matches analytical solution |
| TestCantileverIBeamAnalyticalComparison.test_bimoment_distribution_along_beam | Bimoment distribution matches analytical along beam |

### test_warping_decoupling.py -> test_warping.cpp (3 tests)

| Python Test | C++ Equivalent |
|-------------|----------------|
| TestWarpingDOFDecoupling.test_independent_warping_at_tjoint | T-joint with torque shows no warping coupling between orthogonal beams |
| TestContinuousBeamWarpingContinuity.test_continuous_beam_warping_continuity | Continuous beam shows warping continuity at internal nodes |
| TestContinuousBeamWarpingContinuity.test_warping_continuity_produces_coupled_bimoments | Warping continuity produces coupled bimoments |

---

## Migration Complete

All Python tests that used `apply_to_system` have been migrated to C++:
- **test_constraints.cpp**: 12 tests (constraint handler, rigid links, MPC workflow)
- **test_internal_actions.cpp**: 3 tests (bimoment calculations, analytical comparisons)
- **test_warping.cpp**: 3 tests (warping DOF decoupling and continuity)

**Status:** Migration complete. Python tests using `apply_to_system` have been removed and replaced with C++ equivalents.

All tests pass:
- C++ tests: 57 tests across 6 test files
- Python tests: ~530 tests (reduced from ~550 after migration)
