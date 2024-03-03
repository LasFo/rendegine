#include "euler_angles.h"
#include "math_util.h"
#include "matrix_4x3.h"
#include "quaternion.h"
#include "vector3d.h"
#include "gtest/gtest.h"

void pri(Matrix4x3 m) {
  printf("EXPECT_NEAR(%.5f, sut.m11, 1.0e-5f);\n", m.m11);
  printf("EXPECT_NEAR(%.5f, sut.m12, 1.0e-5f);\n", m.m12);
  printf("EXPECT_NEAR(%.5f, sut.m13, 1.0e-5f);\n", m.m13);
  printf("EXPECT_NEAR(%.5f, sut.m21, 1.0e-5f);\n", m.m21);
  printf("EXPECT_NEAR(%.5f, sut.m22, 1.0e-5f);\n", m.m22);
  printf("EXPECT_NEAR(%.5f, sut.m23, 1.0e-5f);\n", m.m23);
  printf("EXPECT_NEAR(%.5f, sut.m31, 1.0e-5f);\n", m.m31);
  printf("EXPECT_NEAR(%.5f, sut.m32, 1.0e-5f);\n", m.m32);
  printf("EXPECT_NEAR(%.5f, sut.m33, 1.0e-5f);\n", m.m33);
  printf("EXPECT_NEAR(%.5f, sut.tx, 1.0e-5f);\n", m.tx);
  printf("EXPECT_NEAR(%.5f, sut.ty, 1.0e-5f);\n", m.ty);
  printf("EXPECT_NEAR(%.5f, sut.tz, 1.0e-5f);\n", m.tz);
}

TEST(matrix_4x3, setupLocalToParent) {
  EulerAngles orient(0.0f, 0.0f, 0.0f);
  Vector3 pos(0.0f, 0.0f, 0.0f);
  Matrix4x3 sut;

  sut.setupParentToLocal(pos, orient);
  EXPECT_NEAR(1.0f, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.m21, 1.0e-5f);
  EXPECT_NEAR(1.0f, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.m32, 1.0e-5f);
  EXPECT_NEAR(1.0f, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.tz, 1.0e-5f);

  orient = EulerAngles(1.0f, -1.0f, 0.0f);
  sut.setupParentToLocal(pos, orient);
  EXPECT_NEAR(0.54030f, sut.m11, 1.0e-5f);
  EXPECT_NEAR(-0.70807f, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.45464f, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.54030f, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.84147f, sut.m23, 1.0e-5f);
  EXPECT_NEAR(-0.84147f, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.45464f, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.29192f, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.tz, 1.0e-5f);

  pos = Vector3(5.0f, -13.5f, -0.6f);
  sut.setupParentToLocal(pos, orient);
  EXPECT_NEAR(0.54030f, sut.m11, 1.0e-5f);
  EXPECT_NEAR(-0.70807f, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.45464f, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.0f, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.54030f, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.84147f, sut.m23, 1.0e-5f);
  EXPECT_NEAR(-0.84147f, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.45464f, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.29192f, sut.m33, 1.0e-5f);
  EXPECT_NEAR(-3.20639f, sut.tx, 1.0e-5f);
  EXPECT_NEAR(10.56165f, sut.ty, 1.0e-5f);
  EXPECT_NEAR(9.26177f, sut.tz, 1.0e-5f);
}

TEST(matrix_4x3, setupParentToLocal) {
  EulerAngles orient(0.0f, 0.0f, 0.0f);
  Vector3 pos(0.0f, 0.0f, 0.0f);
  Matrix4x3 sut;

  sut.setupParentToLocal(pos, orient);
  EXPECT_NEAR(1.00000, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m22, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m32, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m33, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.tz, 1.0e-5f);

  orient = EulerAngles(1.0f, -1.0f, 0.0f);
  sut.setupParentToLocal(pos, orient);
  EXPECT_NEAR(0.54030, sut.m11, 1.0e-5f);
  EXPECT_NEAR(-0.70807, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.45465, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.54030, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.84147, sut.m23, 1.0e-5f);
  EXPECT_NEAR(-0.84147, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.45465, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.29193, sut.m33, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.tz, 1.0e-5f);

  pos = Vector3(5.0f, -13.5f, -0.6f);
  sut.setupParentToLocal(pos, orient);
  EXPECT_NEAR(0.54030, sut.m11, 1.0e-5f);
  EXPECT_NEAR(-0.70807, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.45465, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.54030, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.84147, sut.m23, 1.0e-5f);
  EXPECT_NEAR(-0.84147, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.45465, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.29193, sut.m33, 1.0e-5f);
  EXPECT_NEAR(-3.20639, sut.tx, 1.0e-5f);
  EXPECT_NEAR(10.56166, sut.ty, 1.0e-5f);
  EXPECT_NEAR(9.26177, sut.tz, 1.0e-5f);
}

TEST(matrix_4x3, setupRotate) {
  Matrix4x3 sut;
  sut.setupRotate(1, 0.5);
  EXPECT_NEAR(1.00000, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.47943, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.47943, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);

  sut.setupRotate(2, -8.5);
  EXPECT_NEAR(-0.60201, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.79849, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m23, 1.0e-5f);
  EXPECT_NEAR(-0.79849, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m32, 1.0e-5f);
  EXPECT_NEAR(-0.60201, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);

  sut.setupRotate(3, 0);
  EXPECT_NEAR(1.00000, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m32, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);
}

TEST(matrix_4x3, setupRotateVector) {
  Matrix4x3 sut;
  Vector3 axis(1.0f, 0.0f, 0.0f);
  sut.setupRotate(axis, 0.5);
  EXPECT_NEAR(1.00000, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.47943, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.47943, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);

  axis = Vector3(0.0f, 1.0f, 0.0f);
  sut.setupRotate(2, -8.5);
  EXPECT_NEAR(-0.60201, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.79849, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m23, 1.0e-5f);
  EXPECT_NEAR(-0.79849, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m32, 1.0e-5f);
  EXPECT_NEAR(-0.60201, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);

  axis = Vector3(0.0f, 0.0f, 1.0f);
  sut.setupRotate(3, 0);
  EXPECT_NEAR(1.00000, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m32, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);

  axis = Vector3(-1.0f, 0.5f, 1.0f);
  sut.setupRotate(3, 0.859f);
  EXPECT_NEAR(0.65319, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.75719, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m13, 1.0e-5f);
  EXPECT_NEAR(-0.75719, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.65319, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m32, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);
}

TEST(matrix_4x3, fromQuaternion) {
  Quaternion q;
  Matrix4x3 sut;

  q.setToRotateAboutX(0.5f);
  sut.fromQuaternion(q);
  EXPECT_NEAR(1.00000, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(-0.47943, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.47943, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.47943, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);

  q.setToRotateAboutX(-0.5f);
  sut.fromQuaternion(q);
  EXPECT_NEAR(1.00000, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.47943, sut.m13, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m22, 1.0e-5f);
  EXPECT_NEAR(-0.47943, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.47943, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);

  q.setToRotateAboutZ(kPi);
  sut.fromQuaternion(q);
  EXPECT_NEAR(-1.00000, sut.m11, 1.0e-5f);
  EXPECT_NEAR(-0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m13, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m21, 1.0e-5f);
  EXPECT_NEAR(-1.00000, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m31, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m32, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);
}

TEST(matrix_4x3, matrix_matrix_mul) {
  Matrix4x3 sut, mul;
  Vector3 axis(1.0f, 0.0f, 0.0f);

  sut.setupRotate(axis, 0.5);
  axis = Vector3(0.0f, 1.0f, 0.0f);
  mul.setupRotate(axis, -0.5);
  sut *= mul;
  EXPECT_NEAR(0.87758, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.m12, 1.0e-5f);
  EXPECT_NEAR(0.47943, sut.m13, 1.0e-5f);
  EXPECT_NEAR(-0.22985, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.42074, sut.m23, 1.0e-5f);
  EXPECT_NEAR(-0.42074, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.47943, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.77015, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);

  axis = Vector3(0.5f, 1.0f, 1.0f);
  axis.normalize();
  sut.setupRotate(axis, 1.5);
  axis = Vector3(5.0f, 1.0f, 0.0f);
  axis.normalize();
  mul.setupRotate(axis, 0.5);
  sut *= mul;
  EXPECT_NEAR(0.15058, sut.m11, 1.0e-5f);
  EXPECT_NEAR(0.98856, sut.m12, 1.0e-5f);
  EXPECT_NEAR(-0.00902, sut.m13, 1.0e-5f);
  EXPECT_NEAR(-0.37485, sut.m21, 1.0e-5f);
  EXPECT_NEAR(0.06554, sut.m22, 1.0e-5f);
  EXPECT_NEAR(0.92477, sut.m23, 1.0e-5f);
  EXPECT_NEAR(0.91477, sut.m31, 1.0e-5f);
  EXPECT_NEAR(-0.13587, sut.m32, 1.0e-5f);
  EXPECT_NEAR(0.38043, sut.m33, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tx, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.ty, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.tz, 1.0e-5f);
}

void pri(Vector3 m) {
  printf("EXPECT_NEAR(%.5f, sut.x, 1.0e-5f);\n", m.x);
  printf("EXPECT_NEAR(%.5f, sut.y, 1.0e-5f);\n", m.y);
  printf("EXPECT_NEAR(%.5f, sut.z, 1.0e-5f);\n", m.z);
}

TEST(matrix_4x3, vector_matrix_mul) {
  Matrix4x3 mul;
  Vector3 axis(1.0f, 0.0f, 0.0f);
  Vector3 sut(0.0f, 1.0f, 0.0f);

  mul.setupRotate(axis, 0.5);
  sut *= mul;
  EXPECT_NEAR(0.00000, sut.x, 1.0e-5f);
  EXPECT_NEAR(0.87758, sut.y, 1.0e-5f);
  EXPECT_NEAR(0.47943, sut.z, 1.0e-5f);

  axis = Vector3(0.0f, 1.0f, 0.0f);
  mul.setupRotate(axis, 2.5);
  sut = Vector3(0.0f, 1.0f, 0.0f);
  sut *= mul;
  EXPECT_NEAR(0.00000, sut.x, 1.0e-5f);
  EXPECT_NEAR(1.00000, sut.y, 1.0e-5f);
  EXPECT_NEAR(0.00000, sut.z, 1.0e-5f);

  axis = Vector3(1.0f, 1.0f, 1.0f);
  axis.normalize();
  mul.setupRotate(axis, 2.5);
  sut = Vector3(0.0f, 1.5f, -1.0f);
  sut *= mul;
  EXPECT_NEAR(-0.56363, sut.x, 1.0e-5f);
  EXPECT_NEAR(0.86287, sut.y, 1.0e-5f);
  EXPECT_NEAR(1.36477, sut.z, 1.0e-5f);
}
