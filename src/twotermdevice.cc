#include "twotermdevice.h"

TwoTermDevice::TwoTermDevice(
    double Vth,
    const std::vector<std::pair<int,int>>& control
)
    : Vth(Vth), control(control)
{
}