#ifndef PBD_FACTORY_H_
#define PBD_FACTORY_H_

#include "pbd_system.hpp"
#include "point_cloud.hpp"
#include <memory>

namespace pbd
{

std::unique_ptr<PbdSystem> MakeRod(
    double length, int num_edges, double stiffness);

}

#endif
