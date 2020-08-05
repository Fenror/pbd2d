#include "pbd_factory.hpp"

#include <glm/gtc/constants.hpp>


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

std::unique_ptr<PbdSystem> MakeSquare(
    double side_length, double stiffness)
{
  std::unique_ptr<PbdSystem> square =
      std::make_unique<PbdSystem>(4);

  const double angle = glm::half_pi<double>();
  const int power = 1;

  square->AddLengthConstraint(0,1,side_length,power);
  square->AddLengthConstraint(1,2,side_length,power);
  square->AddLengthConstraint(2,3,side_length,power);
  square->AddLengthConstraint(3,0,side_length,power);

  square->AddAngleConstraint(0,1,2,angle,stiffness,power);
  square->AddAngleConstraint(1,2,3,angle,stiffness,power);
  square->AddAngleConstraint(2,3,0,angle,stiffness,power);
  square->AddAngleConstraint(3,0,1,angle,stiffness,power);

  square->GetPointCloud()->SetPoint(0, {0.0, 0.0});
  square->GetPointCloud()->SetPoint(1, {side_length, 0.0});
  square->GetPointCloud()->SetPoint(2, {side_length, side_length});
  square->GetPointCloud()->SetPoint(3, {0.0, side_length});

  return square;
}

}
