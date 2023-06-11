#include "math_util.h"
#include "vector.h"

const Vector3 kZeroVector(0.0f, 0.0f, 0.0f);

float wrapPi(float theta) {
    theta += kPi;
    theta -= floor(theta * k1Over2Pi) * k2Pi;
    theta -= kPi;
    return theta;
}
