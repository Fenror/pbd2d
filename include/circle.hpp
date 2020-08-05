#ifndef CIRCLE_DISC_H_
#define CIRCLE_DISC_H_

#include <glm/glm.hpp>

class Circle
{
public:
  Circle(glm::dvec2 center, double radius);

  glm::dvec2 GetCenter() const;
  double GetRadius() const;
private:
  glm::dvec2 center_;
  double radius_;
};

#endif
