#ifndef CAMERA_H_
#define CAMERA_H_

#include <glm/glm.hpp>

class Camera
{
public:
  Camera();
  ~Camera();
  void Zoom(double z);
  void Displace(glm::dvec2 d);

  void SetZoom(double z);
  void SetCenter(glm::dvec2 c);
  glm::dvec2 GetCenter() const;
  double GetWidth() const;
  void GetLimits(double *left, double *right,
                 double *top, double *bottom) const;
  void SetAspectRatio(double a);

private:
  double aspect_ratio_; // width / height
  double width_ = 2.0;
  glm::dvec2 center_ = {0.0, 0.0};
};

#endif
