#include "circuits.h"
#include "constants.h"

#include <algorithm>
#include <numeric>
#include <cmath>


Circuit::Circuit(
    const std::vector<Capacitor>& cap,
    const std::vector<Device>& dev,
    const std::vector<Source>& src
)
    : cap(cap),
      dev(dev),
      src(src)
{
    set_N();
    build_C_matrix();
    split_C();
    build_Delta();
}

void Circuit::set_N()
{
    N = 0;
//
//auto& sirve para deducir 
//el tipo de una variable manteniendo su condición de referencia
//recorre cada elemento del vector cap y lo asigna a la variable c, que es una referencia constante al elemento actual del vector.
    for (const auto& c : cap)
    {
        N = std::max(N, c.node_a);
        N = std::max(N, c.node_b);
    }
//finalmente tomará el nodo más grande, para saber cuántos nodos hay en total en el circuito, y lo almacenará en la variable N.
    for (const auto& d : dev)
    {   //tomara el primer o segundo nodo del par de terminales del dispositivo
        N = std::max(N, d.terminals.first);
        N = std::max(N, d.terminals.second);
    }

    for (const auto& s : src)
    {
        N = std::max(N, s.node);
    }

//static_cast es un operador de conversión de tipo en C++ 
//que realiza una conversión de tipo en tiempo de compilación,
    Nr = static_cast<int>(src.size());
    Nf = N - Nr;
}

void Circuit::build_C_matrix()
{
    lumped_C = Eigen::MatrixXd::Zero(N, N);

    for (const auto& c : cap)
    {
        int a = c.node_a - 1;
        int b = c.node_b - 1;

        lumped_C(a, b) += c.C;
        lumped_C(b, a) += c.C;
    }

    maxwell_C = -lumped_C;

    for (int n = 0; n < N; n++)
    {
        maxwell_C(n, n) = lumped_C.col(n).sum();
    }
}

void Circuit::split_C()
{
    Vr.resize(Nr);

    reg_idx.clear();

    for (int k = 0; k < Nr; k++)
    {
        reg_idx.push_back(src[k].node - 1);
//holds_alternative es una función de la biblioteca estándar de C++ que se utiliza para verificar 
//si un objeto std::variant contiene un valor de un tipo específico double.

        if (std::holds_alternative<double>(src[k].voltage))
//obtenemos un double del std::variant usando std::get<double>
        {
            Vr(k) = std::get<double>(src[k].voltage);
        }
        else
        {
            Vr(k) = 0.0;
        }
    }

    free_idx.clear();

    for (int k = 0; k < N; k++)
    {
        if (std::find(reg_idx.begin(), reg_idx.end(), k)
            == reg_idx.end())
        {
            free_idx.push_back(k);
        }
    }

    C.resize(Nf, Nf);
    Cx.resize(Nf, Nr);
    Cr.resize(Nr, Nr);

    for (int i = 0; i < Nf; i++)
    {
        for (int j = 0; j < Nf; j++)
        {
            C(i,j) = maxwell_C(free_idx[i], free_idx[j]);
        }
    }

    for (int i = 0; i < Nf; i++)
    {
        for (int j = 0; j < Nr; j++)
        {
            Cx(i,j) = maxwell_C(free_idx[i], reg_idx[j]);
        }
    }

    for (int i = 0; i < Nr; i++)
    {
        for (int j = 0; j < Nr; j++)
        {
            Cr(i,j) = maxwell_C(reg_idx[i], reg_idx[j]);
        }
    }

    invC = C.inverse();
}


void Circuit::build_Delta()
{
    int ndev = static_cast<int>(dev.size());

    Delta = Eigen::MatrixXd::Zero(N, ndev);

    for (int k = 0; k < ndev; k++)
    {
        int a = dev[k].terminals.first - 1;
        int b = dev[k].terminals.second - 1;


        Delta(a, k) = -1.0;
        Delta(b, k) = 1.0;
    }

    free_Delta.resize(Nf, ndev);
    reg_Delta.resize(Nr, ndev);

    for (int i = 0; i < Nf; i++)
        free_Delta.row(i) = Delta.row(free_idx[i]);

    for (int i = 0; i < Nr; i++)
        reg_Delta.row(i) = Delta.row(reg_idx[i]);
}

Eigen::VectorXd Circuit::V(
    const Eigen::VectorXd& q,
    double t
) const
{
    Eigen::VectorXd Vr_t(Nr);

    for (int k = 0; k < Nr; k++)
    {
        if (std::holds_alternative<double>(src[k].voltage))
        {
            Vr_t(k) = std::get<double>(src[k].voltage);
        }
        else
        {
            const auto& f =
                std::get<std::function<double(double)>>(src[k].voltage);

            Vr_t(k) = f(t);
        }
    }

    Eigen::VectorXd Vf =
        invC * (q - Cx * Vr_t);

    Eigen::VectorXd voltage =
        Eigen::VectorXd::Zero(N);

    for (int i = 0; i < Nf; i++)
        voltage(free_idx[i]) = Vf(i);

    for (int i = 0; i < Nr; i++)
        voltage(reg_idx[i]) = Vr_t(i);

    return voltage;
}


Eigen::VectorXd Circuit::dV(int dev_idx) const
{
    Eigen::VectorXd delta =
        free_Delta.col(dev_idx);

    Eigen::VectorXd dVf =
        qe * invC * delta;

    Eigen::VectorXd dV =
        Eigen::VectorXd::Zero(N);

    for (int i = 0; i < Nf; i++)
        dV(free_idx[i]) = dVf(i);

    return dV;
}