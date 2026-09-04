#include "mosfetfreebody.h"
#include "constants.h"
#include <cmath>


// ============================================================================
// Funciones auxiliares privad
// Usamos un espacio de nombres anónimo para que solo sean visibles en este archivo.
// Al ser funciones inline pequeñas, el compilador las optimizará completamente.
// ============================================================================
namespace {

inline double __lf_impl(double lfactor, double Vg, double Vd,double n, double Vth,double V) {
    return lfactor * std::exp(Vg / (n * Vth)) * std::exp(-Vd / Vth) * std::exp(V / Vth);
}

inline double __lr_impl(double lfactor, double Vg, double Vs,double n, double Vth, double V) {
    return lfactor * std::exp(Vg / (n * Vth)) * std::exp(-Vs / Vth) * std::exp(-V / Vth);
}

} // namespace anónimo

// ============================================================================
// Implementación de la clase Mosfet
// ============================================================================

Mosfetfree::Mosfetfree(
    double I0,
    double VT,
    double n,
    double Vth,
    const std::vector<std::pair<int, int>>& control
) : TwoTermDevice(Vth, control), // Inicialización de la clase base
    I0(I0),
    VT(VT),
    n(n)
{
    // Cálculo de lfactor equivalente al constructor de Python:
    // self.lfactor = (self.I0 / qe) * np.exp(-self.VT / (self.n * self.Vth))
    this->lfactor = (this->I0 / qe) * std::exp(-this->VT / (this->n * this->Vth));
}

double Mosfetfree::lf(double V, const Eigen::VectorXd& ctrl) const {
    // ctrl[0] equivale a ctrl[0] en Python
    double Vg = ctrl[0];
    double Vd = ctrl[1];
    return __lf_impl(this->lfactor, Vg, Vd, this->n, this->Vth, V);
}

double Mosfetfree::lr(double V, const Eigen::VectorXd& ctrl) const {
    // ctrl[0] equivale a ctrl[0] en Python
    double Vg = ctrl[0];
    double Vs = ctrl[2];
    return __lr_impl(this->lfactor, Vg, Vs, this->n, this->Vth, V);
}