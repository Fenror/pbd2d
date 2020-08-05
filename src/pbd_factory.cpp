#include "pbd_factory.hpp"


namespace pbd
{

std::unique_ptr<PbdSystem> MakeRod(double length, int num_edges)
{
  std::unique_ptr<PbdSystem> rod =
      std::make_unique<PbdSystem>(num_edges+1);

  const auto edge_len = length/num_edges;
  const int power = 1;

  for (int i = 0; i < num_edges; ++i)
  {
    rod->AddLengthConstraint(i,i+1,edge_len,power);
  }

  for (int i = 0; i < num_edges+1; ++i)
  {
    rod->GetDynamicalSystem()
       ->SetPoint(i, {i*edge_len, 0.0});
  }

  return rod;
}

}
