#include "pbd_factory.hpp"


namespace pbd
{

std::unique_ptr<PbdSystem> MakeRod(
    double length, int num_edges, double stiffness)
{
  std::unique_ptr<PbdSystem> rod =
      std::make_unique<PbdSystem>(num_edges+1);

  const auto edge_len = length/num_edges;
  const double angle = 0;
  const int power = 1;

  for (int i = 0; i < num_edges; ++i)
  {
    rod->AddLengthConstraint(i,i+1,edge_len,power);
  }

  for (int i = 1; i < num_edges; ++i)
  {
    rod->AddAngleConstraint(i-1,i,i+1,angle,stiffness,power);
  }

  for (int i = 0; i < num_edges+1; ++i)
  {
    rod->GetPointCloud()
       ->SetPoint(i, {i*edge_len, 0.0});
  }

  return rod;
}

}
