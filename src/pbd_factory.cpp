#include "pbd_factory.hpp"

#include <glm/gtc/constants.hpp>


namespace pbd
{

std::unique_ptr<PbdSystem> MakeRod(
    double length, double mass, int num_edges,
    double stretch_resistance, double bend_resistance)
{
  std::unique_ptr<PbdSystem> rod =
      std::make_unique<PbdSystem>(num_edges+1);

  const auto edge_len = length/num_edges;
  const double angle = glm::pi<double>();
  const int num_iter = 1;

  for (int i = 0; i < num_edges; ++i)
  {
    rod->AddLengthConstraint(i,i+1,edge_len,stretch_resistance,num_iter);
  }

  for (int i = 1; i < num_edges; ++i)
  {
    rod->AddBendConstraint(i-1,i,i+1,2*edge_len,bend_resistance,num_iter);
  }

  for (int i = 0; i < num_edges+1; ++i)
  {
    rod->SetPoint(i, {i*edge_len, 0.5});
    rod->SetMass(i, mass/(num_edges+1));
  }

  return rod;
}

std::unique_ptr<PbdSystem> MakeSquare(
    double side_length, double stiffness)
{
  std::unique_ptr<PbdSystem> square =
      std::make_unique<PbdSystem>(4);

  const int num_iter = 1;

  const double c = glm::length(glm::dvec2{side_length, side_length});
  square->AddLengthConstraint(0,1,side_length,stiffness,num_iter);
  square->AddLengthConstraint(1,2,side_length,stiffness,num_iter);
  square->AddLengthConstraint(2,3,side_length,stiffness,num_iter);
  square->AddLengthConstraint(3,0,side_length,stiffness,num_iter);
  square->AddLengthConstraint(0,2,c,stiffness,num_iter);
  square->AddLengthConstraint(1,3,c,stiffness,num_iter);

  square->SetPoint(0, {0.0, 0.0});
  square->SetPoint(1, {side_length, 0.0});
  square->SetPoint(2, {side_length, side_length});
  square->SetPoint(3, {0.0, side_length});

  return square;
}

}
