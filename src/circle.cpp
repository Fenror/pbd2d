#include "circle.hpp"

Circle::Circle(glm::dvec2 center, double radius)
  : center_(center)
  , radius_(radius)
{

}

glm::dvec2 Circle::GetCenter() const
{
  return center_;
}

double Circle::GetRadius() const
{
  return radius_;
}
