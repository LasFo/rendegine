#include "quaternion.h"
#include "euler_angles.h"
#include "vector3d.h"
#include "math_util.h"
#include <cassert>

const Quaternion kQuaternionIdentity{.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};

void Quaternion::setToRotateAboutX(float theta) {
    theta *= 0.5f;
    float sinT, cosT;
    sin_cos(&sinT, &cosT, theta);
    w = cosT, x = sinT, y = 0.0f, z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta) {
    theta *= 0.5f;
    float sinT, cosT;
    sin_cos(&sinT, &cosT, theta);
    w = cosT, x = 0.0f, y = sinT, z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float theta) {
    theta *= 0.5f;
    float sinT, cosT;
    sin_cos(&sinT, &cosT, theta);
    w = cosT, x = 0.0f, y = 0.0f, z = sinT;
}

void Quaternion::setToRotateAboutAxis(const Vector3 &axis, float theta) {
    theta *= 0.5f;
    float sinT, cosT;
    sin_cos(&sinT, &cosT, theta);
    w = cosT, x = axis.x*sinT, y = axis.y*sinT, z = axis.z*sinT;
}

// UNTESTED
void Quaternion::setToRotateObjectToInertial(const EulerAngles &orientation) {
	float	sp, sb, sh;
	float	cp, cb, ch;
	sin_cos(&sp, &cp, orientation.pitch * 0.5f);
	sin_cos(&sb, &cb, orientation.bank * 0.5f);
	sin_cos(&sh, &ch, orientation.heading * 0.5f);

	w =  ch*cp*cb + sh*sp*sb;
	x =  ch*sp*cb + sh*cp*sb;
	y = -ch*sp*sb + sh*cp*cb;
	z = -sh*sp*cb + ch*cp*sb;
}

// UNTESTED
void Quaternion::setToRotateInertialToObject(const EulerAngles &orientation) {
	float	sp, sb, sh;
	float	cp, cb, ch;
	sin_cos(&sp, &cp, orientation.pitch * 0.5f);
	sin_cos(&sb, &cb, orientation.bank * 0.5f);
	sin_cos(&sh, &ch, orientation.heading * 0.5f);

	w =  ch*cp*cb + sh*sp*sb;
	x = -ch*sp*cb - sh*cp*sb;
	y =  ch*sp*sb - sh*cb*cp;
	z =  sh*sp*cb - ch*cp*sb;
}

Quaternion Quaternion::operator *(const Quaternion &a) const {
    Quaternion res;
    res.w = w * a.w - (x * a.x + y * a.y + z * a.z);
    res.x = w * a.x + a.w * x + z * a.y - y * a.z;
    res.y = w * a.y + a.w * y + x * a.z - z * a.x;
    res.z = w * a.z + a.w * z + y * a.x - x * a.y;
    return res;
}

Quaternion &Quaternion::operator *=(const Quaternion &a){
    *this = *this * a;
    return *this;
}

void Quaternion::normalize() {
	float	mag = (float)sqrt(w*w + x*x + y*y + z*z);
	if (mag > 0.0f) {
		float	oneOverMag = 1.0f / mag;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	} else {
		assert(false && "Quaternion mag is 0.0f");
		identity();
	}
}

float Quaternion::getRotationAngle() const {
    return safe_acos(w) * 2.0f;
}

Vector3	Quaternion::getRotationAxis() const {
    float sinThetaOver2Sq = 1.0f - w*w;
	if (sinThetaOver2Sq <= 0.0f) {
		return Vector3(1.0f, 0.0f, 0.0f);
	}
	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);
	return Vector3(
		x * oneOverSinThetaOver2,
		y * oneOverSinThetaOver2,
		z * oneOverSinThetaOver2
	);
}

float dotProduct(const Quaternion &a, const Quaternion &b) {
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

// UNTESTED
extern Quaternion slerp(const Quaternion &q0, const Quaternion &q1, float t) {
	if (t <= 0.0f) return q0;
	if (t >= 1.0f) return q1;

	// Compute "cosine of angle between quaternions" using dot product

	float cosOmega = dotProduct(q0, q1);

	// If negative dot, use -q1.  Two quaternions q and -q
	// represent the same rotation, but may produce
	// different slerp.  We chose q or -q to rotate using
	// the acute angle.

	float q1w = q1.w;
	float q1x = q1.x;
	float q1y = q1.y;
	float q1z = q1.z;
	if (cosOmega < 0.0f) {
		q1w = -q1w;
		q1x = -q1x;
		q1y = -q1y;
		q1z = -q1z;
		cosOmega = -cosOmega;
	}

	// We should have two unit quaternions, so dot should be <= 1.0

	assert(cosOmega < 1.1f);

	// Compute interpolation fraction, checking for quaternions
	// almost exactly the same

	float k0, k1;
	if (cosOmega > 0.9999f) {

		// Very close - just use linear interpolation,
		// which will protect againt a divide by zero

		k0 = 1.0f-t;
		k1 = t;

	} else {

		// Compute the sin of the angle using the
		// trig identity sin^2(omega) + cos^2(omega) = 1

		float sinOmega = sqrt(1.0f - cosOmega*cosOmega);

		// Compute the angle from its sin and cosine

		float omega = atan2(sinOmega, cosOmega);

		// Compute inverse of denominator, so we only have
		// to divide once

		float oneOverSinOmega = 1.0f / sinOmega;

		// Compute interpolation parameters

		k0 = sin((1.0f - t) * omega) * oneOverSinOmega;
		k1 = sin(t * omega) * oneOverSinOmega;
	}

	// Interpolate

	Quaternion result;
	result.x = k0*q0.x + k1*q1x;
	result.y = k0*q0.y + k1*q1y;
	result.z = k0*q0.z + k1*q1z;
	result.w = k0*q0.w + k1*q1w;

	return result;
}

Quaternion conjugate(const Quaternion &q) {
    return Quaternion{.w = q.w, .x = -q.x, .y = -q.y, .z = -q.z};
}

// UNTESTED
extern Quaternion pow(const Quaternion &q, float exponent) {

	// Check for the case of an identity quaternion.
	// This will protect against divide by zero

	if (fabs(q.w) > .9999f) {
		return q;
	}

	float	alpha = acos(q.w);
	float	newAlpha = alpha * exponent;
	Quaternion result;
	result.w = cos(newAlpha);
	float	mult = sin(newAlpha) / sin(alpha);
	result.x = q.x * mult;
	result.y = q.y * mult;
	result.z = q.z * mult;
	return result;
}
