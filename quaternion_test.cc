#include "gtest/gtest.h"
#include "quaternion.h"
#include "math_util.h"
#include "vector3d.h"


TEST(quaternion, setToRotateAboutX){
    Quaternion q;
    q.setToRotateAboutX(0.5f);
    EXPECT_NEAR(0.96891f, q.w, 1.0e-5f);
    EXPECT_NEAR(0.24740f, q.x, 1.0e-5f);
    EXPECT_NEAR(0.0f,     q.y, 1.0e-5f);
    EXPECT_NEAR(0.0f,     q.z, 1.0e-5f);

    q.setToRotateAboutX(-0.5f);
    EXPECT_NEAR(0.96891f,  q.w, 1.0e-5f);
    EXPECT_NEAR(-0.24740f, q.x, 1.0e-5f);
    EXPECT_NEAR(0.0f,      q.y, 1.0e-5f);
    EXPECT_NEAR(0.0f,      q.z, 1.0e-5f);

    q.setToRotateAboutX(kPi);
    EXPECT_NEAR(0.0f, q.w, 1.0e-5f);
    EXPECT_NEAR(1.0f, q.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, q.y, 1.0e-5f);
    EXPECT_NEAR(0.0f, q.z, 1.0e-5f);
}

TEST(quaternion, setToRotateAboutY){
    Quaternion q;
    q.setToRotateAboutY(0.5f);
    EXPECT_NEAR(0.96891f, q.w, 1.0e-5f);
    EXPECT_NEAR(0.0f,     q.x, 1.0e-5f);
    EXPECT_NEAR(0.24740f, q.y, 1.0e-5f);
    EXPECT_NEAR(0.0f,     q.z, 1.0e-5f);

    q.setToRotateAboutY(-0.5f);
    EXPECT_NEAR(0.96891f,  q.w, 1.0e-5f);
    EXPECT_NEAR(0.0f,      q.x, 1.0e-5f);
    EXPECT_NEAR(-0.24740f, q.y, 1.0e-5f);
    EXPECT_NEAR(0.0f,      q.z, 1.0e-5f);

    q.setToRotateAboutY(kPi);
    EXPECT_NEAR(0.0f, q.w, 1.0e-5f);
    EXPECT_NEAR(0.0f, q.x, 1.0e-5f);
    EXPECT_NEAR(1.0f, q.y, 1.0e-5f);
    EXPECT_NEAR(0.0f, q.z, 1.0e-5f);
}

TEST(quaternion, setToRotateAboutZ){
    Quaternion q;
    q.setToRotateAboutZ(0.5f);
    EXPECT_NEAR(0.96891f, q.w, 1.0e-5f);
    EXPECT_NEAR(0.0f,     q.x, 1.0e-5f);
    EXPECT_NEAR(0.0f,     q.y, 1.0e-5f);
    EXPECT_NEAR(0.24740f, q.z, 1.0e-5f);

    q.setToRotateAboutZ(-0.5f);
    EXPECT_NEAR(0.96891f,  q.w, 1.0e-5f);
    EXPECT_NEAR(0.0f,      q.x, 1.0e-5f);
    EXPECT_NEAR(0.0f,      q.y, 1.0e-5f);
    EXPECT_NEAR(-0.24740f, q.z, 1.0e-5f);

    q.setToRotateAboutZ(kPi);
    EXPECT_NEAR(0.0f, q.w, 1.0e-5f);
    EXPECT_NEAR(0.0f, q.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, q.y, 1.0e-5f);
    EXPECT_NEAR(1.0f, q.z, 1.0e-5f);
}

TEST(quaternion, setToRotateAboutAxis){
    Quaternion q;
    q.setToRotateAboutAxis(Vector3(1.0f, 0.0f, 0.0f), 0.5);
    Quaternion qx;
    qx.setToRotateAboutX(0.5f);
    EXPECT_NEAR(qx.w, q.w, 1.0e-5f);
    EXPECT_NEAR(qx.x, q.x, 1.0e-5f);
    EXPECT_NEAR(qx.y, q.y, 1.0e-5f);
    EXPECT_NEAR(qx.z, q.z, 1.0e-5f);

    q.setToRotateAboutAxis(Vector3(0.0f, 1.0f, 0.0f), 0.5);
    Quaternion qy;
    qy.setToRotateAboutY(0.5f);
    EXPECT_NEAR(qy.w, q.w, 1.0e-5f);
    EXPECT_NEAR(qy.x, q.x, 1.0e-5f);
    EXPECT_NEAR(qy.y, q.y, 1.0e-5f);
    EXPECT_NEAR(qy.z, q.z, 1.0e-5f);

    q.setToRotateAboutAxis(Vector3(0.0f, 0.0f, 1.0f), 0.5);
    Quaternion qz;
    qz.setToRotateAboutZ(0.5f);
    EXPECT_NEAR(qz.w, q.w, 1.0e-5f);
    EXPECT_NEAR(qz.x, q.x, 1.0e-5f);
    EXPECT_NEAR(qz.y, q.y, 1.0e-5f);
    EXPECT_NEAR(qz.z, q.z, 1.0e-5f);

    Vector3 v(3.2f, 4.7f, 0.35f);
    v.normalize();
    q.setToRotateAboutAxis(v, kPi-0.2f);
    EXPECT_NEAR(0.09983f, q.w, 1.0e-5f);
    EXPECT_NEAR(0.55892f, q.x, 1.0e-5f);
    EXPECT_NEAR(0.82091f, q.y, 1.0e-5f);
    EXPECT_NEAR(0.06113f, q.z, 1.0e-5f);
}

Quaternion referenceCrossProdImpl(const Quaternion &b ,const Quaternion &a) {
	Quaternion result;

	result.w = b.w*a.w - b.x*a.x - b.y*a.y - b.z*a.z;
	result.x = b.w*a.x + b.x*a.w + b.z*a.y - b.y*a.z;
	result.y = b.w*a.y + b.y*a.w + b.x*a.z - b.z*a.x;
	result.z = b.w*a.z + b.z*a.w + b.y*a.x - b.x*a.y;

	return result;
}

TEST(quaternion, CrossProduct){
    Quaternion q0, q1;
    q0.setToRotateAboutX(0.5);
    q1.setToRotateAboutY(0.5);
    Quaternion qn, qr;
    qn = q0 * q1;
    qr = referenceCrossProdImpl(q0, q1);
    EXPECT_NEAR(qr.w, qn.w, 1.0e-5f);
    EXPECT_NEAR(qr.x, qn.x, 1.0e-5f);
    EXPECT_NEAR(qr.y, qn.y, 1.0e-5f);
    EXPECT_NEAR(qr.z, qn.z, 1.0e-5f);

    Vector3 v0(3.2f, 4.7f, 0.35f);
    v0.normalize();
    q0.setToRotateAboutAxis(v0, kPi-0.2f);
    Vector3 v1(56.0f, 0.7f, -1.5f);
    v1.normalize();
    q1.setToRotateAboutAxis(v1, 0.752f);
    qn = q0 * q1;
    qr = referenceCrossProdImpl(q0, q1);
    EXPECT_NEAR(qr.w, qn.w, 1.0e-5f);
    EXPECT_NEAR(qr.x, qn.x, 1.0e-5f);
    EXPECT_NEAR(qr.y, qn.y, 1.0e-5f);
    EXPECT_NEAR(qr.z, qn.z, 1.0e-5f);
}

TEST(quaternion, CrossProductAssign){
    Quaternion q0, q1;
    q0.setToRotateAboutX(0.5);
    q1.setToRotateAboutY(0.5);
    Quaternion qn, qr;
    qn = q0; qn *= q1;
    qr = referenceCrossProdImpl(q0, q1);
    EXPECT_NEAR(qr.w, qn.w, 1.0e-5f);
    EXPECT_NEAR(qr.x, qn.x, 1.0e-5f);
    EXPECT_NEAR(qr.y, qn.y, 1.0e-5f);
    EXPECT_NEAR(qr.z, qn.z, 1.0e-5f);

    Vector3 v0(3.2f, 4.7f, 0.35f);
    v0.normalize();
    q0.setToRotateAboutAxis(v0, kPi-0.2f);
    Vector3 v1(56.0f, 0.7f, -1.5f);
    v1.normalize();
    q1.setToRotateAboutAxis(v1, 0.752f);
    qn = q0; qn *= q1;
    qr = referenceCrossProdImpl(q0, q1);
    EXPECT_NEAR(qr.w, qn.w, 1.0e-5f);
    EXPECT_NEAR(qr.x, qn.x, 1.0e-5f);
    EXPECT_NEAR(qr.y, qn.y, 1.0e-5f);
    EXPECT_NEAR(qr.z, qn.z, 1.0e-5f);
}

TEST(quaternion, Normalize){
    Quaternion q{.w = 1.5f, .x = 5.5, .y = 6.7, .z = -53.0f};
    q.normalize();
    EXPECT_NEAR(0.02791f,  q.w, 1.0e-5f);
    EXPECT_NEAR(0.10237f,  q.x, 1.0e-5f);
    EXPECT_NEAR(0.12470f,  q.y, 1.0e-5f);
    EXPECT_NEAR(-0.98650f, q.z, 1.0e-5f);

    q = Quaternion{.w = 0.0f, .x = -0.5, .y = 0.7, .z = 0.05f};
    q.normalize();
    EXPECT_NEAR(0.0f,      q.w, 1.0e-5f);
    EXPECT_NEAR(-0.58025f, q.x, 1.0e-5f);
    EXPECT_NEAR(0.81236f,  q.y, 1.0e-5f);
    EXPECT_NEAR(0.058025f, q.z, 1.0e-5f);
}

TEST(quaternion, GetRotationAngle){
    Quaternion q;
    q.setToRotateAboutX(0.5f);
    EXPECT_NEAR(0.5f, q.getRotationAngle(), 1.0e-5f);

    q.setToRotateAboutY(1.3f);
    EXPECT_NEAR(1.3f, q.getRotationAngle(), 1.0e-5f);

    q.setToRotateAboutZ(kPi);
    EXPECT_NEAR(kPi, q.getRotationAngle(), 1.0e-5f);

    q.setToRotateAboutAxis(Vector3(1.0f, 0.0f, 0.0f), -kPi);
    EXPECT_NEAR(kPi, q.getRotationAngle(), 1.0e-5f);
}

TEST(quaternion, GetRotationAxis){
    Quaternion q;
    q.setToRotateAboutX(0.5f);
    Vector3 v = q.getRotationAxis();
    EXPECT_NEAR(1.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.z, 1.0e-5f);

    q.setToRotateAboutY(0.5f);
    v = q.getRotationAxis();
    EXPECT_NEAR(0.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.z, 1.0e-5f);

    q.setToRotateAboutZ(0.5f);
    v = q.getRotationAxis();
    EXPECT_NEAR(0.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.z, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.y, 1.0e-5f);

    Vector3 v1(56.0f, 0.7f, -1.5f);
    v1.normalize();
    q.setToRotateAboutAxis(v1, 0.752f);
    v = q.getRotationAxis();
    EXPECT_NEAR(v1.x, v.x, 1.0e-5f);
    EXPECT_NEAR(v1.y, v.y, 1.0e-5f);
    EXPECT_NEAR(v1.z, v.z, 1.0e-5f);
}

TEST(quaternion, DotProduct){
    Quaternion q0, q1;
    q0.setToRotateAboutX(0.5);
    q1.setToRotateAboutY(0.5);
    float dp = dotProduct(q0, q1);
    EXPECT_NEAR(0.93879f, dp, 1.0e-5f);

    Vector3 v1(56.0f, 0.7f, -1.5f);
    v1.normalize();
    q0.setToRotateAboutAxis(v1, 0.752f);
    v1 = Vector3(0.02f, 5.7f, 3.5f);
    v1.normalize();
    q1.setToRotateAboutAxis(v1, 1.752f);
    dp = dotProduct(q0, q1);
    EXPECT_NEAR(0.59539f, dp, 1.0e-5f);
}

TEST(quaternion, Conjugate){
    Quaternion q{.w = -1.0, .x = 2.0f, .y = 3.0f, .z = -2.0f};
    Quaternion qr = conjugate(q);
    EXPECT_NEAR(-1.0f, qr.w, 1.0e-5f);
    EXPECT_NEAR(-2.0f, qr.x, 1.0e-5f);
    EXPECT_NEAR(-3.0f, qr.y, 1.0e-5f);
    EXPECT_NEAR(2.0f,  qr.z, 1.0e-5f);

    q = Quaternion{.w = 1.0, .x = -0.5f, .y = 0.0f, .z = 6.0f};
    qr = conjugate(q);
    EXPECT_NEAR(1.0f,  qr.w, 1.0e-5f);
    EXPECT_NEAR(0.5f,  qr.x, 1.0e-5f);
    EXPECT_NEAR(0.0f,  qr.y, 1.0e-5f);
    EXPECT_NEAR(-6.0f, qr.z, 1.0e-5f);
}
