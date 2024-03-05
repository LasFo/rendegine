#include "geo/aabb3.h"
#include "cassert"
#include "math/vector3d.h"
#include "math/matrix_4x3.h"
#include <algorithm>

Vector3 AABB3::corner(int i) const {
  assert(i >= 0);
  assert(i <= 7);
  Vector3 res;
  res.x = i & 1 ? min.x : max.x;
  res.y = i & 2 ? min.y : max.y;
  res.z = i & 4 ? min.z : max.z;
  return res;
}

void AABB3::empty() {
  const float kBigNumber = 1e37f;
  min.x = min.y = min.z = kBigNumber;
  max.x = max.y = max.z = -kBigNumber;
}

void AABB3::add(const Vector3 &p) {
    min.x = std::min(p.x, min.x);
    min.y = std::min(p.y, min.y);
    min.z = std::min(p.z, min.z);
    max.x = std::max(p.x, max.x);
    max.y = std::max(p.y, max.y);
    max.z = std::max(p.z, max.z);
}

void AABB3::add(const AABB3 &box) {
    min.x = std::min(box.min.x, min.x);
    min.y = std::min(box.min.y, min.y);
    min.z = std::min(box.min.z, min.z);
    max.x = std::max(box.max.x, max.x);
    max.y = std::max(box.max.y, max.y);
    max.z = std::max(box.max.z, max.z);
}

void AABB3::setToTransformedBox(const AABB3 &box, const Matrix4x3 &m) {
    min = max = getTranslation(m);
    if (m.m11 > 0.0f) {
		min.x += m.m11 * box.min.x; max.x += m.m11 * box.max.x;
	} else {
		min.x += m.m11 * box.max.x; max.x += m.m11 * box.min.x;
	}

	if (m.m12 > 0.0f) {
		min.y += m.m12 * box.min.x; max.y += m.m12 * box.max.x;
	} else {
		min.y += m.m12 * box.max.x; max.y += m.m12 * box.min.x;
	}

	if (m.m13 > 0.0f) {
		min.z += m.m13 * box.min.x; max.z += m.m13 * box.max.x;
	} else {
		min.z += m.m13 * box.max.x; max.z += m.m13 * box.min.x;
	}

	if (m.m21 > 0.0f) {
		min.x += m.m21 * box.min.y; max.x += m.m21 * box.max.y;
	} else {
		min.x += m.m21 * box.max.y; max.x += m.m21 * box.min.y;
	}

	if (m.m22 > 0.0f) {
		min.y += m.m22 * box.min.y; max.y += m.m22 * box.max.y;
	} else {
		min.y += m.m22 * box.max.y; max.y += m.m22 * box.min.y;
	}

	if (m.m23 > 0.0f) {
		min.z += m.m23 * box.min.y; max.z += m.m23 * box.max.y;
	} else {
		min.z += m.m23 * box.max.y; max.z += m.m23 * box.min.y;
	}

	if (m.m31 > 0.0f) {
		min.x += m.m31 * box.min.z; max.x += m.m31 * box.max.z;
	} else {
		min.x += m.m31 * box.max.z; max.x += m.m31 * box.min.z;
	}

	if (m.m32 > 0.0f) {
		min.y += m.m32 * box.min.z; max.y += m.m32 * box.max.z;
	} else {
		min.y += m.m32 * box.max.z; max.y += m.m32 * box.min.z;
	}

	if (m.m33 > 0.0f) {
		min.z += m.m33 * box.min.z; max.z += m.m33 * box.max.z;
	} else {
		min.z += m.m33 * box.max.z; max.z += m.m33 * box.min.z;
	}
}
