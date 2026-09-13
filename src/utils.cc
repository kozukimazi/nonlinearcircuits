#include "utils.h"

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
)
{
    // Voltajes promedio forward y reverse
    Eigen::VectorXd av_V_f = V + dV / 2.0;
    Eigen::VectorXd av_V_r = V - dV / 2.0;

    // Diferencia de potencial en el dispositivo
    av_DV_f =
        av_V_f(dev_term.first - 1)
        - av_V_f(dev_term.second - 1);

    av_DV_r =
        av_V_r(dev_term.first - 1)
        - av_V_r(dev_term.second - 1);

    // Diferencias de potencial en los controles
    av_DV_ctrl_f.clear();
    av_DV_ctrl_r.clear();

    for (std::size_t i = 0; i < control_a.size(); ++i)
    {
        double av_DV_control_f =
            av_V_f(control_a[i] - 1)
            - av_V_f(control_b[i] - 1);

        double av_DV_control_r =
            av_V_r(control_a[i] - 1)
            - av_V_r(control_b[i] - 1);

        av_DV_ctrl_f.push_back(av_DV_control_f);
        av_DV_ctrl_r.push_back(av_DV_control_r);
    }
}