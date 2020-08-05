#include <glm/glm.hpp>

namespace pbd
{

void GetLengthConstraintDelta(
    glm::dvec2 p, double pmass,
    glm::dvec2 q, double qmass,
    double target_len,
    glm::dvec2* dp,
    glm::dvec2* dq);

}
