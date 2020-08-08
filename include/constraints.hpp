#include <glm/glm.hpp>

namespace pbd
{

void GetLengthConstraintDelta(
    glm::dvec2 p, double pmass,
    glm::dvec2 q, double qmass,
    double target_len,
    double stiffness,
    glm::dvec2* dp,
    glm::dvec2* dq);

void GetBendConstraintDelta(
    glm::dvec2 p, double pmass,
    glm::dvec2 q, double qmass,
    glm::dvec2 r, double rmass,
    double segment_length,
    double stiffness,
    glm::dvec2* dp,
    glm::dvec2* dq,
    glm::dvec2* dr);

}
