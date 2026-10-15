#ifndef TWOTERMDEVICE_H
#define TWOTERMDEVICE_H

#include <Eigen/Dense>
#include <vector>

class TwoTermDevice
{
protected:
    double Vth;
    std::vector<std::pair<int,int>> control;
//Private members are only accessible within the class defining them.
//Protected members are accessible in the class that defines them and in classes that inherit from that class.
//Both are also accessible by friends of their class, and in the case of protected members, by friends of their derived classes

public:
    TwoTermDevice(
        double Vth,
        const std::vector<std::pair<int,int>>& control
    );

    // Agrégalo en la sección public:
    const std::vector<std::pair<int,int>>& get_control() const { return control; }
    virtual ~TwoTermDevice() = default;
//garantiza que los objetos de las clases hijas se destruyan 
//correctamente cuando se manejan a través de un puntero de TwoTermDevice.
    virtual double lf(
        double V,
        const Eigen::VectorXd& ctrl
    ) const = 0;

    virtual double lr(
        double V,
        const Eigen::VectorXd& ctrl
    ) const = 0;
};

#endif