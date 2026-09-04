#include "shockleydiode.h"
#include "constants.h"
#include <cmath>

ShockleyDiode::ShockleyDiode(
    double I0,
    double n,
    double Vth,
    const std::vector<std::pair<int,int>>& control
): TwoTermDevice(Vth, control),
    I0(I0),
    n(n)
{
}

double ShockleyDiode::lf(
    double V,
    const Eigen::VectorXd& ctrl
) const
{
    return (I0 / qe) * std::exp(V / (n * Vth));
}

double ShockleyDiode::lr(
    double V,
    const Eigen::VectorXd& ctrl
) const
{
    return I0 / qe;
}