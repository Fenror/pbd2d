#ifndef PBD_FACTORY_H_
#define PBD_FACTORY_H_

#include "pbd_system.hpp"
#include "point_cloud.hpp"
#include <memory>

namespace pbd
{

std::unique_ptr<PbdSystem> MakeRod(
    double length, double mass, int num_edges,
    double stretch_resistance, double bend_resistance);

std::unique_ptr<PbdSystem> MakeSquare(
    double side_length, double stiffness);

}

#endif
