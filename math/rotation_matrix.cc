#include "rotation_matrix.h"
#include "euler_angles.h"
#include "quaternion.h"
#include "math_util.h"

void RotationMatix::identity() {
    m11 =  m22 =  m33 = 1;
    m12 = m13= m21 = m23 = m31 = m32 = 0;
}

void RotationMatix::setup(const EulerAngles &orientation) {
    float h = orientation.heading;
    float p = orientation.pitch;
    float b = orientation.bank;
    float cosh, sinh;
    sin_cos(&sinh, &cosh, h);
    float cosp, sinp;
    sin_cos(&sinp, &cosp, p);
    float cosb, sinb;
    sin_cos(&sinb, &cosb, b);

    m11 = cosh*cosb + sinh*sinp*sinb;
    m12 = -cosh*sinb + sinh*sinp*cosb;
    m13 = sinh*cosp;

    m21 = sinb*cosp;
    m22 = cosb*cosp;
    m23 = -sinp;

    m31 = -sinh*cosb+cosh*sinp*sinb;
    m32 = sinb*sinh+cosh*sinp*cosb;
    m33 = cosh*cosp;
}

void RotationMatrix::fromInertialToObjectQuaternion(const Quaternion &q) {
    m11 = 1.f - 2.f*q.y*q.y - 2.f*q.z*q.z;
    m12 = 2.f*q.x*q.y + 2.f*q.w*q.z;
    m12 = 2.f*q.x*q.z - 2.f*q.w*q.y;

    m21 = 2.f*q.x*q.y - 2.f*q.w*q.z;
    m22 = 1.f -2.f*q.x*q.x - 2.f*q.z*q.z;
    m23 = 2.f*q.y*q.z + 2.f*q.w*q.x;

    m31 = 2.f*q.x*q.z + 2.f*q.w*q.y;
    m32 = 2.f*q.y*q.z - 2.f*q.w*q.x;
    m33 = 1.f - 2.f*q.x*q.x - 2.f*q.y*q.y;
}

void RotationMatrix::fromInertialToObjectQuaternion(const Quaternion &q) {
    m11 = 1.f - 2.f*q.y*q.y - 2.f*q.z*q.z;
    m12 = 2.f*q.x*q.y + 2.f*q.w*q.z;
    m13 = 2.f*q.x*q.z - 2.f*q.w*q.y;

    m21 = 2.f*q.x*q.y - 2.f*q.w*q.z;
    m22 = 1.f -2.f*q.x*q.x - 2.f*q.z*q.z;
    m23 = 2.f*q.y*q.z + 2.f*q.w*q.x;

    m31 = 2.f*q.x*q.z + 2.f*q.w*q.y;
    m32 = 2.f*q.y*q.z - 2.f*q.w*q.x;
    m33 = 1.f - 2.f*q.x*q.x - 2.f*q.y*q.y;
}

void RotationMatrix::fromObjectToInertialQuaternion(const Quaternion &q) {
    m11 = 1.f - 2.f*q.y*q.y - 2.f*q.z*q.z;
    m12 = 2.f*q.x*q.y - 2.f*q.w*q.z;
    m13 = 2.f*q.x*q.z + 2.f*q.w*q.y;

    m21 = 2.f*q.x*q.y + 2.f*q.w*q.z;
    m22 = 1.f -2.f*q.x*q.x - 2.f*q.z*q.z;
    m23 = 2.f*q.y*q.z - 2.f*q.w*q.x;

    m31 = 2.f*q.x*q.z - 2.f*q.w*q.y;
    m32 = 2.f*q.y*q.z + 2.f*q.w*q.x;
    m33 = 1.f - 2.f*q.x*q.x - 2.f*q.y*q.y;
}

Vector3	RotationMatrix::inertialToObject(const Vector3 &v) const {
	return Vector3(
		m11*v.x + m21*v.y + m31*v.z,
		m12*v.x + m22*v.y + m32*v.z,
		m13*v.x + m23*v.y + m33*v.z
	);
}

Vector3	RotationMatrix::objectToInertial(const Vector3 &v) const {

	return Vector3(
		m11*v.x + m12*v.y + m13*v.z,
		m21*v.x + m22*v.y + m23*v.z,
		m31*v.x + m32*v.y + m33*v.z
	);
}
