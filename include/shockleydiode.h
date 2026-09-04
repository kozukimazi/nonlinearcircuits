#ifndef SHOCKLEYDIODE_H
#define SHOCKLEYDIODE_H

#include <Eigen/Dense>
#include <vector>
#include "twotermdevice.h"


class ShockleyDiode : public TwoTermDevice
{//necesitmos iniciar las variables I0 y n en el constructor de la clase hija, 
//ya que son privadas y no se pueden acceder desde la clase padre
private:
    double I0;
    double n;

public:
    ShockleyDiode(
        double I0,
        double n,
        double Vth,
        const std::vector<std::pair<int,int>>& control
    );

    double lf(
        double V,
        const Eigen::VectorXd& ctrl
    ) const override;

    double lr(
        double V,
        const Eigen::VectorXd& ctrl
    ) const override;
};

#endif