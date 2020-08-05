#include "rod.hpp"
#include "point_cloud.hpp"
#include <glm/gtc/constants.hpp>
#include <glm/gtx/vector_angle.hpp>
#define GLM_FORCE_RADIANS
#include <algorithm>

Rod::Rod(double rod_length, int num_edges)
  : rod_length_(rod_length)
  , num_edges_(num_edges)
{
  ds_ = std::make_unique<PointCloud>(num_edges+1);
  for (int i = 0; i < num_edges+1; ++i)
  {
    ds_->SetPoint(i,{i*GetRestingEdgeLength(), 0.0});
    ds_->SetMass(i, 1.0/ds_->GetNumPoints());
  }
  length_iterations_ = 20;
  bend_iterations_ = 1;
  bend_stiffness_ = 1.0;
}

void Rod::Integrate(double dt)
{
  ds_->Integrate(dt);
  DampVelocities(0.001);
  EnforceConstraints(dt);
}

void Rod::SpawnBetweenPoints(
    glm::dvec2 p1, glm::dvec2 p2, int num_edges)
{
  num_edges_ = num_edges;
  rod_length_ = glm::distance(p1,p2);
  std::vector<glm::dvec2> vertices(num_edges+1);

  for (int i = 0; i < num_edges+1; ++i)
  { 
    vertices[i] = p1 + static_cast<double>(i)/num_edges*(p2 - p1);
  }

  ds_->SpawnNewPoints(vertices);
}

void Rod::Destroy()
{
  ds_->RemoveAllPoints();

  num_edges_ = 0;
  rod_length_ = 0;
}

void Rod::EnforceConstraints(double dt)
{
  auto old_vertices = ds_->GetPoints();
  for (int ns = 0; ns < length_iterations_; ++ns)
  {
    EnforceLengthConstraints();
  }
  for (int ns = 0; ns < bend_iterations_; ++ns)
  {
    EnforceBendConstraints();
  }

  for (int i = 0; i < ds_->GetNumPoints(); ++i)
  {
    auto old_vertex = old_vertices[i];
    auto new_vertex = ds_->GetPoint(i);
    ds_->AddVelocity(i, (new_vertex - old_vertex)/dt);
  }
}

void Rod::EnforceLengthConstraints()
{
  for (int i = 0; i < num_edges_; ++i)
  {
    const auto edge = GetEdge(i);
    const auto edge_len = glm::length(edge);
    const auto edge_dir = glm::normalize(edge);
    const double wl = 1/ds_->GetMass(i);
    const double wr = 1/ds_->GetMass(i+1);
    const auto dl = wl/(wl+wr)*0.5*(edge_len - GetRestingEdgeLength())*edge_dir;
    const auto dr = wr/(wl+wr)*0.5*(GetRestingEdgeLength() - edge_len)*edge_dir;
    ds_->DisplacePoint(i, dl);
    ds_->DisplacePoint(i+1, dr);
  }
}

void Rod::EnforceBendConstraints()
{
  for (int i = 1; i < num_edges_; ++i)
  {
    const auto edge_l = glm::normalize(GetEdge(i-1));
    const auto edge_r = glm::normalize(GetEdge(i));
    const double C = glm::angle(edge_l, edge_r);

    if (glm::abs(C) < 0.01/num_edges_)
      continue;

    const double d = glm::dot(edge_l,edge_r);
    const auto l = (edge_r - d*edge_l)/GetEdgeLength(i-1);
    const auto r = (edge_l - d*edge_r)/GetEdgeLength(i);
    const auto ls = -glm::sqrt(std::max(1e-10,1-d*d))*l;
    const auto rs = -glm::sqrt(std::max(1e-10,1-d*d))*r;
    const auto wprev = 1/ds_->GetMass(i-1);
    const auto wcurr = 1/ds_->GetMass(i);
    const auto wnext = 1/ds_->GetMass(i+1);
    const double bla =
      wprev*glm::length2(l) +
      wcurr*glm::length2(l-r) +
      wnext*glm::length2(r);

    const double step_length = 0.5*bend_stiffness_;
    const auto dprev =
        wprev*step_length*C*ls/bla;
    const auto dcurr =
        wcurr*step_length*C*(rs-ls)/bla;
    const auto dnext =
        -wnext*step_length*C*rs/bla;
    ds_->DisplacePoint(i-1, dprev);
    ds_->DisplacePoint(i, dcurr);
    ds_->DisplacePoint(i+1, dnext);
  }
}

void Rod::DampVelocities(double damping)
{
  auto global_momentum = GetGlobalMomentum();
  for (int i = 0; i < ds_->GetNumPoints(); ++i)
  {
    const auto cur_velocity = ds_->GetVelocity(i);
    ds_->AddVelocity(i, damping*(global_momentum - cur_velocity));
  }
}

glm::dvec2 Rod::GetGlobalMomentum() const
{
  glm::dvec2 global_momentum{0.0,0.0};

  for (int i = 0; i < ds_->GetNumPoints(); ++i)
    global_momentum += ds_->GetVelocity(i);

  global_momentum /= ds_->GetNumPoints();

  return global_momentum;
}

glm::dvec2 Rod::GetEdge(int i) const
{
  return ds_->GetPoint(i+1) - ds_->GetPoint(i);
}

//setters and getters
PointCloud* Rod::GetPointCloud() const
{
  return ds_.get();
}

int Rod::GetNumEdges() const
{
  return ds_->GetNumPoints()-1;
}

double Rod::GetLength() const
{
  return rod_length_;
}

void Rod::SetLength(double l)
{
  rod_length_ = l;
}

double Rod::GetStretch(unsigned int i) const
{
  return GetEdgeLength(i)/GetRestingEdgeLength();
}

void Rod::SetStretchResistance(int r)
{
  length_iterations_ = r;
}

void Rod::SetBendResistance(double r)
{
  const double min = 0.0;
  const double max = 1.0;
  bend_stiffness_ = glm::clamp(r,min,max);
}
