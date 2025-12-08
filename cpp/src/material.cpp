#include "grillex/material.hpp"

namespace grillex {

Material::Material(int id, std::string name, double E, double nu, double rho)
    : id(id), name(std::move(name)), E(E), nu(nu), rho(rho) {
    // Compute shear modulus from E and nu
    G = compute_G(E, nu);
}

double Material::compute_G(double E, double nu) {
    return E / (2.0 * (1.0 + nu));
}

} // namespace grillex
