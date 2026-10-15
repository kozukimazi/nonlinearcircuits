#include "simulation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>


// =========================================================================
// DEFINICIÓN DE av_deltaV (Coincide 100% con simulation.h)
// =========================================================================
DeltaVResult av_deltaV(
    const Eigen::VectorXd& V,
    const Eigen::VectorXd& dV,
    const std::pair<int, int>& dev_term,
    const Eigen::VectorXi& ctrl_a,
    const Eigen::VectorXi& ctrl_b,
    double t
) {
    DeltaVResult res;

    // Promedio de voltajes antes y después del salto[cite: 1]
    Eigen::VectorXd av_V_f = V + dV / 2.0;
    Eigen::VectorXd av_V_r = V - dV / 2.0;

    // Ajuste de índices 1-based de Python a 0-based de C++[cite: 1]
    res.av_DV_f = av_V_f(dev_term.first - 1) - av_V_f(dev_term.second - 1);
    res.av_DV_r = av_V_r(dev_term.first - 1) - av_V_r(dev_term.second - 1);

    // Voltajes de control (compuerta/gate)
    if (ctrl_a.size() > 0 && ctrl_b.size() > 0) {
        res.ctrl_f = av_V_f(ctrl_a(0) - 1) - av_V_f(ctrl_b(0) - 1);
        res.ctrl_r = av_V_r(ctrl_a(0) - 1) - av_V_r(ctrl_b(0) - 1);
    } else {
        res.ctrl_f = 0.0;
        res.ctrl_r = 0.0;
    }

    return res;
}


// =========================================================================
// CONSTRUCTOR DE LA SIMULACIÓN
// =========================================================================
Simulation::Simulation(const Circuit& circ) 
    : circuit(circ), rng(std::random_device{}()) {
    
    // Al ser Simulation 'friend class' de Circuit, accedemos directamente a dev.
    Ndev = circuit.dev.size(); 
    
    // Inicializamos los vectores termodinámicos y de tasas
    rates_f = Eigen::VectorXd::Zero(Ndev);
    rates_r = Eigen::VectorXd::Zero(Ndev);
    av_dv_f = Eigen::VectorXd::Zero(Ndev);
    av_dv_r = Eigen::VectorXd::Zero(Ndev);

    // Precálculo de los vectores de control y variaciones de voltaje (dV)
    for (int d = 0; d < Ndev; ++d) {
        // Almacenamos el cambio de voltaje (dV) cuando hay un salto en este dispositivo
        dV.push_back(circuit.dV(d));
        
        // ¡Aquí extraemos los controles faltantes!
        // Llamamos al getter de TwoTermDevice para obtener los pares (ctrl_a, ctrl_b)
        auto control_pairs = circuit.dev[d].device->get_control();
        int num_controls = control_pairs.size();
        
        Eigen::VectorXi ca(num_controls);
        Eigen::VectorXi cb(num_controls);
        
        for (int i = 0; i < num_controls; ++i) {
            ca(i) = control_pairs[i].first;
            cb(i) = control_pairs[i].second;
        }
        
        controls_a.push_back(ca);
        controls_b.push_back(cb);
    }
}

// =========================================================================
// ALGORITMO EXACTO: GILLESPIE (run_gill)
// =========================================================================
void Simulation::run_gill(double tf_in, const Eigen::VectorXd* init_st) {
    this->tf = tf_in;
    
    gill_times.clear();
    gill_traj.clear();
    gill_jumps.clear();

    gill_times.push_back(0.0);
    
    if (init_st != nullptr) {
        gill_traj.push_back(*init_st);
    } else {
        gill_traj.push_back(Eigen::VectorXd::Zero(circuit.Nf)); 
    }

    // Bucle hasta alcanzar tf
    while (gill_times.back() < this->tf) {
        double dt_out;
        Eigen::VectorXd dq_out;
        int jump_out;

        update_gillespie(gill_traj.back(), gill_times.back(), dt_out, dq_out, jump_out);

        gill_times.push_back(gill_times.back() + dt_out);
        gill_traj.push_back(gill_traj.back() + dq_out);
        gill_jumps.push_back(jump_out);
    }
}

// =========================================================================
// PASO INTERNO DE GILLESPIE (update_gillespie)
// =========================================================================
void Simulation::update_gillespie(
    const Eigen::VectorXd& q, 
    double t, 
    double& dt_out, 
    Eigen::VectorXd& dq_out, 
    int& jump_out) {

    // 1. Calculamos los voltajes V(q, t) usando la inversa de Maxwell
    Eigen::VectorXd V = circuit.V(q, t);

    // 2. Evaluamos termodinámica y tasas de transición
    for (int d = 0; d < Ndev; ++d) {
        std::pair<int, int> dev_term = circuit.dev[d].terminals;

        // Calculamos voltajes promedio del salto (y voltajes de control)
        DeltaVResult res = av_deltaV(V, dV[d], dev_term, controls_a[d], controls_b[d], t);
        
        // Transformamos el double del control a Eigen::VectorXd para la interfaz del dispositivo
        Eigen::VectorXd ctrl_vec_f = Eigen::VectorXd::Constant(1, res.ctrl_f);
        Eigen::VectorXd ctrl_vec_r = Eigen::VectorXd::Constant(1, res.ctrl_r);

        // Tasas de Poisson \lambda_f y \lambda_r usando los voltajes de control[cite: 13]
        rates_f(d) = circuit.dev[d].device->lf(res.av_DV_f, ctrl_vec_f);
        rates_r(d) = circuit.dev[d].device->lr(res.av_DV_r, ctrl_vec_r);
    }

    // 3. Generar tiempos de salto
    std::vector<double> dt_f(Ndev), dt_r(Ndev);
    for (int d = 0; d < Ndev; ++d) {
        double rate_f_safe = (rates_f(d) > 0) ? rates_f(d) : 1e-30;
        double rate_r_safe = (rates_r(d) > 0) ? rates_r(d) : 1e-30;

        std::exponential_distribution<double> exp_f(rate_f_safe);
        std::exponential_distribution<double> exp_r(rate_r_safe);
        
        dt_f[d] = exp_f(rng);
        dt_r[d] = exp_r(rng);
    }

    // 4. Seleccionamos el salto inminente
    auto min_f_it = std::min_element(dt_f.begin(), dt_f.end());
    auto min_r_it = std::min_element(dt_r.begin(), dt_r.end());
    
    int amin_dt_f = std::distance(dt_f.begin(), min_f_it);
    int amin_dt_r = std::distance(dt_r.begin(), min_r_it);

    if (*min_f_it <= *min_r_it) {
        jump_out = amin_dt_f;
        dt_out = *min_f_it;
        dq_out = circuit.free_Delta.col(amin_dt_f).cast<double>(); 
    } else {
        jump_out = -amin_dt_r;
        dt_out = *min_r_it;
        dq_out = -1.0 * circuit.free_Delta.col(amin_dt_r).cast<double>(); 
    }
}

// =========================================================================
// ALGORITMO APROXIMADO: TAU-LEAPING (run_tl)
// =========================================================================
void Simulation::run_tl(double dt_in, double tf_in, const Eigen::VectorXd* init_st) {
    this->dt = dt_in;
    this->tf = tf_in;
    
    int num_steps = static_cast<int>(std::ceil(tf / dt));
    
    times = Eigen::VectorXd::LinSpaced(num_steps, 0.0, tf);
    traj = Eigen::MatrixXd::Zero(circuit.Nf, num_steps);
    jumps = Eigen::MatrixXd::Zero(Ndev, num_steps - 1);
    Q = Eigen::MatrixXd::Zero(Ndev, num_steps - 1);

    if (init_st != nullptr) {
        traj.col(0) = *init_st;
    }

    for (int s = 0; s < num_steps - 1; ++s) {
        Eigen::VectorXd dq;
        Eigen::VectorXd Nj;
        Eigen::VectorXd Q_step;

        update_tau_leaping(traj.col(s), times(s), dq, Nj, Q_step);

        traj.col(s + 1) = traj.col(s) + dq;
        jumps.col(s) = Nj;
        Q.col(s) = Q_step;
    }
}

// =========================================================================
// PASO INTERNO DE TAU-LEAPING (update_tau_leaping)
// =========================================================================
void Simulation::update_tau_leaping(
    const Eigen::VectorXd& q, 
    double t, 
    Eigen::VectorXd& dq, 
    Eigen::VectorXd& Nj, 
    Eigen::VectorXd& Q_out) {

    Eigen::VectorXd V = circuit.V(q, t);

    // 1. Termodinámica en cada dispositivo y voltajes de control
    for (int d = 0; d < Ndev; ++d) {
        std::pair<int, int> dev_term = circuit.dev[d].terminals;

        DeltaVResult res = av_deltaV(V, dV[d], dev_term, controls_a[d], controls_b[d], t);
        
        Eigen::VectorXd ctrl_vec_f = Eigen::VectorXd::Constant(1, res.ctrl_f);
        Eigen::VectorXd ctrl_vec_r = Eigen::VectorXd::Constant(1, res.ctrl_r);

        rates_f(d) = circuit.dev[d].device->lf(res.av_DV_f, ctrl_vec_f);
        rates_r(d) = circuit.dev[d].device->lr(res.av_DV_r, ctrl_vec_r);

        // Guardamos los diferenciales de voltaje promedio para el cálculo de Q[cite: 13]
        av_dv_f(d) = res.av_DV_f;
        av_dv_r(d) = res.av_DV_r;
    }

    // 2. Muestreo de Poisson usando el paso de tiempo discreto
    Eigen::VectorXd Nf_vec(Ndev), Nr_vec(Ndev);
    for (int d = 0; d < Ndev; ++d) {
        std::poisson_distribution<int> pois_f(rates_f(d) * this->dt);
        std::poisson_distribution<int> pois_r(rates_r(d) * this->dt);
        
        Nf_vec(d) = pois_f(rng);
        Nr_vec(d) = pois_r(rng);
    }

    // 3. Modificaciones de estado y calor (Q)
    Nj = Nf_vec - Nr_vec;
    dq = circuit.free_Delta.cast<double>() * Nj; 

    Q_out = Nf_vec.cast<double>().cwiseProduct(av_dv_f) - Nr_vec.cast<double>().cwiseProduct(av_dv_r); 
}