#include <iostream>
#include <Eigen/Dense>
#include "twotermdevice.h"
#include "mosfet.h"

int main()
{
    Eigen::VectorXd v(3);

    v << 1.0, 2.0, 3.0;

    std::cout << v << std::endl;

    return 0;
}