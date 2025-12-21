/**
 * @file errors.hpp
 * @brief Structured error handling for Grillex.
 *
 * This file defines error codes and error structures for reporting
 * analysis failures in a machine-readable format that can be parsed
 * by both humans and LLM agents.
 */

#ifndef GRILLEX_ERRORS_HPP
#define GRILLEX_ERRORS_HPP

#include <string>
#include <vector>
#include <map>
#include <optional>

namespace grillex {

/**
 * @brief Error codes for Grillex analysis failures.
 *
 * These codes provide machine-readable error identification.
 * Each code corresponds to a specific type of failure.
 */
enum class ErrorCode {
    /// No error - analysis completed successfully
    OK = 0,

    // === Structural/Constraint Errors (100-199) ===

    /// System has unconstrained degrees of freedom (rigid body modes)
    UNCONSTRAINED_SYSTEM = 100,

    /// Stiffness matrix is singular (cannot be inverted)
    SINGULAR_MATRIX = 101,

    /// Insufficient boundary conditions for stability
    INSUFFICIENT_CONSTRAINTS = 102,

    /// Redundant or conflicting constraints
    REDUNDANT_CONSTRAINTS = 103,

    // === Element Errors (200-299) ===

    /// Invalid element definition (e.g., zero length beam)
    INVALID_ELEMENT = 200,

    /// Element has invalid or missing material
    INVALID_MATERIAL = 201,

    /// Element has invalid or missing section
    INVALID_SECTION = 202,

    /// Element references non-existent node
    INVALID_NODE_REFERENCE = 203,

    /// Element property is invalid (e.g., negative area)
    INVALID_PROPERTY = 204,

    /// Element stiffness matrix is not positive definite
    INVALID_ELEMENT_STIFFNESS = 205,

    // === Load Errors (300-399) ===

    /// Load references non-existent node
    INVALID_LOAD_NODE = 300,

    /// Load references non-existent element
    INVALID_LOAD_ELEMENT = 301,

    /// Load case is empty (no loads defined)
    EMPTY_LOAD_CASE = 302,

    /// Load combination references non-existent load case
    INVALID_LOAD_COMBINATION = 303,

    // === Model Errors (400-499) ===

    /// Model has no elements
    EMPTY_MODEL = 400,

    /// Model has no nodes
    NO_NODES = 401,

    /// Model has disconnected parts
    DISCONNECTED_MODEL = 402,

    /// Analysis was not performed before querying results
    NOT_ANALYZED = 403,

    // === Solver Errors (500-599) ===

    /// Solver failed to converge
    SOLVER_CONVERGENCE_FAILED = 500,

    /// Numerical overflow during computation
    NUMERICAL_OVERFLOW = 501,

    /// Out of memory
    OUT_OF_MEMORY = 502,

    // === Generic Errors (900-999) ===

    /// Unknown or unspecified error
    UNKNOWN_ERROR = 999
};

/**
 * @brief Convert error code to string representation.
 */
inline std::string error_code_to_string(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK: return "OK";
        case ErrorCode::UNCONSTRAINED_SYSTEM: return "UNCONSTRAINED_SYSTEM";
        case ErrorCode::SINGULAR_MATRIX: return "SINGULAR_MATRIX";
        case ErrorCode::INSUFFICIENT_CONSTRAINTS: return "INSUFFICIENT_CONSTRAINTS";
        case ErrorCode::REDUNDANT_CONSTRAINTS: return "REDUNDANT_CONSTRAINTS";
        case ErrorCode::INVALID_ELEMENT: return "INVALID_ELEMENT";
        case ErrorCode::INVALID_MATERIAL: return "INVALID_MATERIAL";
        case ErrorCode::INVALID_SECTION: return "INVALID_SECTION";
        case ErrorCode::INVALID_NODE_REFERENCE: return "INVALID_NODE_REFERENCE";
        case ErrorCode::INVALID_PROPERTY: return "INVALID_PROPERTY";
        case ErrorCode::INVALID_ELEMENT_STIFFNESS: return "INVALID_ELEMENT_STIFFNESS";
        case ErrorCode::INVALID_LOAD_NODE: return "INVALID_LOAD_NODE";
        case ErrorCode::INVALID_LOAD_ELEMENT: return "INVALID_LOAD_ELEMENT";
        case ErrorCode::EMPTY_LOAD_CASE: return "EMPTY_LOAD_CASE";
        case ErrorCode::INVALID_LOAD_COMBINATION: return "INVALID_LOAD_COMBINATION";
        case ErrorCode::EMPTY_MODEL: return "EMPTY_MODEL";
        case ErrorCode::NO_NODES: return "NO_NODES";
        case ErrorCode::DISCONNECTED_MODEL: return "DISCONNECTED_MODEL";
        case ErrorCode::NOT_ANALYZED: return "NOT_ANALYZED";
        case ErrorCode::SOLVER_CONVERGENCE_FAILED: return "SOLVER_CONVERGENCE_FAILED";
        case ErrorCode::NUMERICAL_OVERFLOW: return "NUMERICAL_OVERFLOW";
        case ErrorCode::OUT_OF_MEMORY: return "OUT_OF_MEMORY";
        case ErrorCode::UNKNOWN_ERROR: return "UNKNOWN_ERROR";
        default: return "UNKNOWN_ERROR";
    }
}

/**
 * @brief Structured error information for Grillex.
 *
 * Contains machine-readable error code, human-readable message,
 * and diagnostic information about involved DOFs and elements.
 */
struct GrillexError {
    /// Machine-readable error code
    ErrorCode code;

    /// Human-readable error message
    std::string message;

    /// DOF indices involved in the error (global DOF numbers)
    std::vector<int> involved_dofs;

    /// Element IDs involved in the error
    std::vector<int> involved_elements;

    /// Node IDs involved in the error
    std::vector<int> involved_nodes;

    /// Additional key-value details for diagnostics
    std::map<std::string, std::string> details;

    /// Suggested fix for the error (for LLM agents)
    std::string suggestion;

    /**
     * @brief Default constructor creates OK status.
     */
    GrillexError()
        : code(ErrorCode::OK), message("OK") {}

    /**
     * @brief Construct error with code and message.
     */
    GrillexError(ErrorCode code, const std::string& message)
        : code(code), message(message) {}

    /**
     * @brief Check if this represents a successful state.
     */
    bool is_ok() const { return code == ErrorCode::OK; }

    /**
     * @brief Check if this represents an error state.
     */
    bool is_error() const { return code != ErrorCode::OK; }

    /**
     * @brief Get string representation of the error code.
     */
    std::string code_string() const { return error_code_to_string(code); }

    /**
     * @brief Get formatted error string for display.
     */
    std::string to_string() const {
        if (is_ok()) return "OK";

        std::string result = "[" + code_string() + "] " + message;

        if (!involved_nodes.empty()) {
            result += "\n  Involved nodes: ";
            for (size_t i = 0; i < involved_nodes.size(); ++i) {
                if (i > 0) result += ", ";
                result += std::to_string(involved_nodes[i]);
            }
        }

        if (!involved_dofs.empty()) {
            result += "\n  Involved DOFs: ";
            for (size_t i = 0; i < involved_dofs.size(); ++i) {
                if (i > 0) result += ", ";
                result += std::to_string(involved_dofs[i]);
            }
        }

        if (!involved_elements.empty()) {
            result += "\n  Involved elements: ";
            for (size_t i = 0; i < involved_elements.size(); ++i) {
                if (i > 0) result += ", ";
                result += std::to_string(involved_elements[i]);
            }
        }

        if (!suggestion.empty()) {
            result += "\n  Suggestion: " + suggestion;
        }

        return result;
    }

    // === Factory methods for common errors ===

    /**
     * @brief Create error for unconstrained system.
     */
    static GrillexError unconstrained(const std::vector<int>& dofs,
                                       const std::vector<int>& nodes = {}) {
        GrillexError err(ErrorCode::UNCONSTRAINED_SYSTEM,
            "System has unconstrained degrees of freedom (rigid body modes)");
        err.involved_dofs = dofs;
        err.involved_nodes = nodes;
        err.suggestion = "Add boundary conditions to constrain the unconstrained DOFs. "
                        "Common fixes: fix all 6 DOFs at one support, or add torsional restraints.";
        return err;
    }

    /**
     * @brief Create error for singular matrix.
     */
    static GrillexError singular(const std::string& details = "") {
        GrillexError err(ErrorCode::SINGULAR_MATRIX,
            "Stiffness matrix is singular and cannot be inverted");
        if (!details.empty()) {
            err.details["solver_details"] = details;
        }
        err.suggestion = "Check boundary conditions - the model may have rigid body modes. "
                        "Also check for zero-stiffness elements or disconnected parts.";
        return err;
    }

    /**
     * @brief Create error for invalid element.
     */
    static GrillexError invalid_element(int element_id, const std::string& reason) {
        GrillexError err(ErrorCode::INVALID_ELEMENT,
            "Invalid element: " + reason);
        err.involved_elements.push_back(element_id);
        return err;
    }

    /**
     * @brief Create error for invalid node reference.
     */
    static GrillexError invalid_node(int node_id, const std::string& context = "") {
        GrillexError err(ErrorCode::INVALID_NODE_REFERENCE,
            "Invalid node reference" + (context.empty() ? "" : ": " + context));
        err.involved_nodes.push_back(node_id);
        return err;
    }

    /**
     * @brief Create error for empty model.
     */
    static GrillexError empty_model() {
        GrillexError err(ErrorCode::EMPTY_MODEL, "Model has no elements");
        err.suggestion = "Add at least one element (beam, spring, plate) to the model.";
        return err;
    }

    /**
     * @brief Create error for not analyzed state.
     */
    static GrillexError not_analyzed() {
        GrillexError err(ErrorCode::NOT_ANALYZED,
            "Analysis was not performed before querying results");
        err.suggestion = "Call model.analyze() before querying displacements or reactions.";
        return err;
    }
};

}  // namespace grillex

#endif  // GRILLEX_ERRORS_HPP
