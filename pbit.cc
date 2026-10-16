#include <iostream>
#include <vector>
#include <memory>
#include <fstream>
#include <Eigen/Dense>

#include "circuits.h"
#include "mosfet.h"
#include "simulation.h"

int main() {
    // =========================================================================
    // 1. DEFINICIÓN DE PARÁMETROS DEL CIRCUITO (Equivalente a Celda 2)
    // =========================================================================
    
    const double VT_val = 26e-3;        // Voltaje térmico (26 mV)
    const double Vdd = 1.1 * VT_val;    // Voltaje de alimentación

    const double C0 = 0.16;             // Unidad de capacitancia (1V / qe = 0.16 aF)
    const double Ci = 50.0 / C0;        // Capacitancia de compuerta (gate) de 50 aF
    const double Co = Ci / 100.0;       // Capacitancia de salida (output)

    std::vector<Source> src;
    std::vector<Capacitor> cap;
    std::vector<Device> dev;

    // Fuentes de Voltaje (Nodos 1 y 2 regulados)
    src.push_back({1, Vdd});   // Fuente positiva (+Vdd)
    src.push_back({2, -Vdd});  // Fuente negativa (-Vdd)

 
    // Transistores MOSFET (I0=1, VT=0, n=1, Vth=VT_val)
    // 1ra etapa
    cap.push_back({1, 3, Ci}); // Capacitancia de compuerta del pMOS 1ª etapa
    cap.push_back({2, 3, Ci}); // Capacitancia de compuerta del nMOS 1ª etapa
    cap.push_back({1, 4, Co}); // Capacitancia de salida del pMOS 1ª etapa
    cap.push_back({2, 4, Co}); // Capacitancia de salida del nMOS 1ª etapa

    // 2da etapa inversora
    cap.push_back({1, 4, Ci}); // Capacitancia de compuerta del pMOS 2ª etapa
    cap.push_back({2, 4, Ci}); // Capacitancia de compuerta del nMOS 2ª etapa
    cap.push_back({1, 3, Co}); // Capacitancia de salida del pMOS 2ª etapa
    cap.push_back({2, 3, Co}); // Capacitancia de salida del nMOS 2ª etapa

    // Un shared_ptr necesita almacenar dos cosas en la memoria dinámica (heap): el objeto en sí y un "bloque de control". Si usas new, haces dos llamadas separadas a la memoria. 
    //Con make_shared, el compilador reserva todo en un único bloque continuo, lo que es mucho más rápido
    auto pmos1 = std::make_shared<Mosfet>(1.0, 0.0, 1.0, VT_val, std::vector<std::pair<int,int>>{{{1, 3}}});
    dev.push_back({{1, 4}, pmos1}); // pMOS 1ª etapa (Source: 1, Drain: 4, Gate: 1-3)

    auto nmos1 = std::make_shared<Mosfet>(1.0, 0.0, 1.0, VT_val, std::vector<std::pair<int,int>>{{{3, 2}}});
    dev.push_back({{4, 2}, nmos1}); // nMOS 1ª etapa (Drain: 4, Source: 2, Gate: 3-2)

    // 2da etapa
    auto pmos2 = std::make_shared<Mosfet>(1.0, 0.0, 1.0, VT_val, std::vector<std::pair<int,int>>{{{1, 4}}});
    dev.push_back({{1, 3}, pmos2}); // pMOS 2ª etapa (Source: 1, Drain: 3, Gate: 1-4)

    auto nmos2 = std::make_shared<Mosfet>(1.0, 0.0, 1.0, VT_val, std::vector<std::pair<int,int>>{{{4, 2}}});
    dev.push_back({{3, 2}, nmos2}); // nMOS 2ª etapa (Drain: 3, Source: 2, Gate: 4-2)

    // Construcción del objeto Circuito electrostático
    Circuit circuit(cap, dev, src);

    // =========================================================================
    // 2. EJECUCIÓN DE LA SIMULACIÓN ESTOCÁSTICA (Equivalente a Celda 3)
    // =========================================================================
    
    std::cout << "Iniciando simulación Tau-Leaping..." << std::endl;
    Simulation sim(circuit);

    // Parámetros de simulación: dt = 0.2, tf = 50000.0
    double dt_sim = 0.2;
    double tf_sim = 50000.0;
    
    sim.run_tl(dt_sim, tf_sim);

    // Impresión de la media de saltos para cada transistor k=0,1,2,3
    std::cout << "\n--- Promedio de saltos netos por dispositivo (jumps) ---" << std::endl;
    for (int k = 0; k < sim.Ndev; ++k) {
        double mean_jump = sim.jumps.row(k).mean();
        std::cout << "Dispositivo " << k + 1 << ": " << mean_jump << std::endl;
    }

    // =========================================================================
    // 3. CÁLCULO DE VOLTAJE DE SALIDA (Vout en Nodo 3) Y EXPORTACIÓN
    // =========================================================================
    
    int num_steps = sim.times.size();
    Eigen::VectorXd Vout(num_steps);

    // En Python: Vout = [circuit.V(traj[:,k], times[k])[3-1] for k in range(...)]
    // El nodo 3 en base-1 equivale al índice 2 en Eigen (base-0)
    for (int k = 0; k < num_steps; ++k) {
        Eigen::VectorXd q_k = sim.traj.col(k);
        double t_k = sim.times(k);
        
        Eigen::VectorXd V_all = circuit.V(q_k, t_k);
        Vout(k) = V_all(2); // Nodo 3 (índice 2)
    }

    // Exportar datos a CSV para graficar posteriormente (con Python, gnuplot, etc.)
    std::ofstream outfile("simulation_results.csv");
    outfile << "time,Vout,Vout_normalized\n";
    for (int k = 0; k < num_steps; ++k) {
        outfile << sim.times(k) << "," << Vout(k) << "," << (Vout(k) / Vdd) << "\n";
    }
    outfile.close();

    std::cout << "\nResultados guardados exitosamente en 'simulation_results.csv'." << std::endl;

    return 0;
}