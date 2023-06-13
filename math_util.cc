#include "math_util.h"
#include "vector3d.h"
#include <vector>

const Vector3 kZeroVector(0.0f, 0.0f, 0.0f);

float wrap_pi(float theta) {
    theta += kPi;
    theta -= floor(theta * k1Over2Pi) * k2Pi;
    theta -= kPi;
    return theta;
}

float safe_acos(float x) {
    if (x <= -1.0f) {
        return kPi;
    }
    if (x >= 1.0f) {
        return 0.0f;
    }
    return acos(x);
}
