#include "camera.hpp"

Camera::Camera()
{
  aspect_ratio_ = 16.0/9.0;
}

Camera::~Camera() { }

void Camera::Zoom(double z)
{
  width_ = width_/z;
}

void Camera::Displace(glm::dvec2 d)
{
  center_ += d;
}

void Camera::GetLimits(double *left, double *right,
                       double *top, double *bottom) const
{
  double height = width_/aspect_ratio_;

  *left   = center_.x - 0.5*width_;
  *right  = center_.x + 0.5*width_;
  *top    = center_.y + 0.5*height;
  *bottom = center_.y - 0.5*height;
}

void Camera::SetCenter(glm::dvec2 c)
{
  center_ = c;
}

glm::dvec2 Camera::GetCenter() const
{
  return center_;
}

double Camera::GetWidth() const
{
  return width_;
}

void Camera::SetAspectRatio(double a)
{
  aspect_ratio_ = a;
}
