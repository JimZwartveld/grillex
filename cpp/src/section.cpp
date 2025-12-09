#include "grillex/section.hpp"

namespace grillex {

Section::Section(int id, std::string name, double A, double Iy, double Iz, double J)
    : id(id), name(std::move(name)), A(A), Iy(Iy), Iz(Iz), J(J),
      Iw(0.0), Asy(0.0), Asz(0.0),
      requires_warping(false), omega_max(0.0),
      zy_top(0.0), zy_bot(0.0), zz_top(0.0), zz_bot(0.0) {
}

void Section::set_warping_constant(double Iw) {
    this->Iw = Iw;
}

void Section::set_shear_areas(double Asy, double Asz) {
    this->Asy = Asy;
    this->Asz = Asz;
}

void Section::set_fibre_distances(double zy_top, double zy_bot, double zz_top, double zz_bot) {
    this->zy_top = zy_top;
    this->zy_bot = zy_bot;
    this->zz_top = zz_top;
    this->zz_bot = zz_bot;
}

void Section::enable_warping(double Iw, double omega_max) {
    this->Iw = Iw;
    this->omega_max = omega_max;
    this->requires_warping = true;
}

} // namespace grillex
