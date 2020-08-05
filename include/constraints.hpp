#include <glm/glm.hpp>

namespace pbd
{

void GetLengthConstraintDelta(
    glm::dvec2 p, double pmass,
    glm::dvec2 q, double qmass,
    double target_len,
    glm::dvec2* dp,
    glm::dvec2* dq);

void GetAngleConstraintDelta(
    glm::dvec2 p, double pmass,
    glm::dvec2 q, double qmass,
    glm::dvec2 r, double rmass,
    double target_angle,
    double stiffness,
    glm::dvec2* dp,
    glm::dvec2* dq,
    glm::dvec2* dr);

}
