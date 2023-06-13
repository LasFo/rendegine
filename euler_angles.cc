#include "euler_angles.h"
#include "math_util.h"
#include "quaternion.h"
#include "matrix_4x3.h"
#include "rotation_matrix.h"

const EulerAngles kEulerAnglesIdentity(0.0f, 0.0f, 0.0f);
void EulerAngles::canonize() {
    pitch = wrap_pi(pitch);

    if (pitch < -kPiOver2) {
        pitch = -kPi - pitch;
        heading += kPi;
        bank += kPi;
    } else if (pitch > kPiOver2) {
        pitch = kPi - pitch;
        heading += kPi;
        bank += kPi;
    }

    if (fabs(pitch) > kPiOver2 - 1e-4) {
        // gimbel lock wrap bank angle in canocial range
        heading += bank;
        bank = 0.0f;
    } else {
        bank = wrap_pi(bank);
    }
    heading = wrap_pi(heading);
}

void EulerAngles::fromObjectToInertialQuaternion(const Quaternion &q) {
    float sp = -2.0f*(q.y*q.z - q.w*q.x);
    if (fabs(sp) > 0.9999f) {
        // gimbel lock
        pitch = kPiOver2 * sp;
        heading = atan2(-q.x*q.z + q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
        bank = 0.0f;
    } else {
        pitch    = asin(sp);
        heading    = atan2(q.x*q.z + q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
        bank    = atan2(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
    }
}

void EulerAngles::fromInertialToObjectQuaternion(const Quaternion &q) {
    float sp = -2.0f * (q.y*q.z + q.w*q.x);
    if (fabs(sp) > 0.9999f) {
        // gimbel lock
        pitch = kPiOver2 * sp;
        heading = atan2(-q.x*q.z - q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
        bank = 0.0f;
    } else {
        pitch    = asin(sp);
        heading    = atan2(q.x*q.z - q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
        bank    = atan2(q.x*q.y - q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
    }
}

void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m) {
    float sp = -m.m32;
    if (fabs(sp) > 9.99999f) {
        // gimbel lock
        pitch = kPiOver2 * sp;
        heading = atan2(-m.m23, m.m11);
        bank = 0.0f;
    } else {
        heading = atan2(m.m31, m.m33);
        pitch = asin(sp);
        bank = atan2(m.m12, m.m22);
    }
}

void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3 &m) {
    float sp = -m.m23;
    if (fabs(sp) > 9.99999f) {
        // gimbel lock
        pitch = kPiOver2 * sp;
        heading = atan2(-m.m31, m.m11);
        bank = 0.0f;
    } else {
        pitch = asin(sp);
        heading = atan2(m.m13, m.m33);
        bank = atan2(m.m21, m.m22);
    }
}

void EulerAngles::fromRotationMatrix(const RotationMatrix &m) {
    float sp = -m.m23;
    if (fabs(sp) > 9.99999f) {
        // gimbel lock
        pitch = kPiOver2 * sp;
        heading = atan2(-m.m31, m.m11);
        bank = 0.0f;
    } else {
        pitch = asin(sp);
        heading = atan2(m.m13, m.m33);
        bank = atan2(m.m21, m.m22);
    }
}
