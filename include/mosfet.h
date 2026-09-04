#ifndef MOSFET_H
#define MOSFET_H

#include <Eigen/Dense>
#include <vector>
#include "twotermdevice.h"

class Mosfet : public TwoTermDevice
{
private:
    double I0;
    double VT;
    double n;
    double lfactor;

public:
    Mosfet(
        double I0,
        double VT,
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