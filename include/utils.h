#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <vector>

void av_deltaV(
    const Eigen::VectorXd& V,
    const Eigen::VectorXd& dV,
    const std::pair<int, int>& dev_term,
    const std::vector<int>& control_a,
    const std::vector<int>& control_b,
    double t,
    double& av_DV_f,
    double& av_DV_r,
    std::vector<double>& av_DV_ctrl_f,
    std::vector<double>& av_DV_ctrl_r
);

#endif