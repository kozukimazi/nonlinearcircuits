#ifndef SIMULATION_H
#define SIMULATION_H

#include "circuits.h"
#include <Eigen/Dense>
#include <vector>
#include <random>

// Declaración previa de la función av_deltaV definida en tu proyecto
struct DeltaVResult {
    double av_DV_f;
    double av_DV_r;
    double ctrl_f;
    double ctrl_r;
};

DeltaVResult av_deltaV(
    const Eigen::VectorXd& V,
    const Eigen::VectorXd& dV,
    //template que permite pasar cualquier tipo de contenedor que tenga dos elementos
    //El & permite pasar por referencia, evitando copias innecesarias
    const std::pair<int, int>& dev_term,
    const Eigen::VectorXi& ctrl_a,
    const Eigen::VectorXi& ctrl_b,
    double t
);

class Simulation {
public:
    const Circuit& circuit;
    int Ndev;

    Eigen::VectorXd rates_f;
    Eigen::VectorXd rates_r;
    Eigen::VectorXd av_dv_f;
    Eigen::VectorXd av_dv_r;

    std::vector<Eigen::VectorXi> controls_a;
    std::vector<Eigen::VectorXi> controls_b;
    std::vector<Eigen::VectorXd> dV;

    // Resultados Tau-Leaping
    double dt;
    double tf;
    Eigen::VectorXd times;
    Eigen::MatrixXd traj;   // Filas: Nf, Columnas: pasos de tiempo
    Eigen::MatrixXd jumps;  // Filas: Ndev, Columnas: pasos de tiempo - 1
    Eigen::MatrixXd Q;      // Filas: Ndev, Columnas: pasos de tiempo - 1

    // Resultados Gillespie
    std::vector<double> gill_times;
    std::vector<Eigen::VectorXd> gill_traj;
    std::vector<int> gill_jumps;

    explicit Simulation(const Circuit& circuit);

    void run_tl(double dt, double tf, const Eigen::VectorXd* init_st = nullptr);
    void run_gill(double tf, const Eigen::VectorXd* init_st = nullptr);

private:
    std::mt19937 rng; // Generador de números aleatorios C++

    void update_tau_leaping(
        const Eigen::VectorXd& q, 
        double t, 
        Eigen::VectorXd& dq, 
        Eigen::VectorXd& Nj, 
        Eigen::VectorXd& Q_out
    );

    void update_gillespie(
        const Eigen::VectorXd& q, 
        double t, 
        double& dt_out, 
        Eigen::VectorXd& dq_out, 
        int& jump_out
    );
};

#endif // SIMULATION_H