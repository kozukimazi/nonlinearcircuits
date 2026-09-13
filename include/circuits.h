#ifndef CIRCUITS_H
#define CIRCUITS_H

#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <memory>
#include <variant>
#include <functional>

#include "twotermdevice.h"


// Capacitor: (nodo_a, nodo_b, capacitancia)
struct Capacitor
{
    int node_a;
    int node_b;
    double C;
};


// Fuente: nodo regulado + voltaje
// El voltaje puede ser un double o una función de t
struct Source
{
    int node;

    std::variant<
        double,
        std::function<double(double)>
    > voltage;
};


// Dispositivo: terminales + dispositivo
struct Device
{
    std::pair<int,int> terminals;
//std::shared_ptr is a smart pointer that retains shared ownership of an object 
//through a pointer.
// Several shared_ptr objects may own the same object.
    std::shared_ptr<TwoTermDevice> device;
};


class Circuit
{
private:

    // Datos originales del circuito
    std::vector<Capacitor> cap;
    std::vector<Device> dev;
    std::vector<Source> src;


    // Número de conductores
    int N;
    int Nr;
    int Nf;


    // Matrices de capacitancia
    Eigen::MatrixXd lumped_C;
    Eigen::MatrixXd maxwell_C;

    Eigen::MatrixXd Cr;
    Eigen::MatrixXd C;
    Eigen::MatrixXd Cx;
    Eigen::MatrixXd invC;


    // Índices de conductores
    std::vector<int> reg_idx;
    std::vector<int> free_idx;


    // Voltajes regulados
    Eigen::VectorXd Vr;


    // Matrices de incidencia
    Eigen::MatrixXd Delta;
    Eigen::MatrixXd free_Delta;
    Eigen::MatrixXd reg_Delta;


    void set_N();
    void build_C_matrix();
    void split_C();
    void build_Delta();


public:

    Circuit(
        const std::vector<Capacitor>& cap,
        const std::vector<Device>& dev,
        const std::vector<Source>& src
    );


    void update_C();

    void update_Vr();


    Eigen::VectorXd V(
        const Eigen::VectorXd& q,
        double t
    ) const;


    Eigen::VectorXd dV(
        int dev_idx
    ) const;
};

#endif