#include "gtest/gtest.h"
#include "euler_angles.h"
#include "math_util.h"
#include "quaternion.h"
#include "rotation_matrix.h"
#include "vector3d.h"

TEST(rotation_matrix, setup) {
    EulerAngles ea(0.0f,0.0f,0.0f);
    RotationMatrix rm;
    rm.setup(ea);
    EXPECT_NEAR(1.0f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(1.0f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(1.0f, rm.m33, 1.0e-5f);

    ea = EulerAngles(1.0f,-1.0f,0.0f);
    rm.setup(ea);
    EXPECT_NEAR(0.540302f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(-0.70807f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(0.45465f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(0.54030f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(0.84147f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(-0.84147f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(-0.45465f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(0.29193f, rm.m33, 1.0e-5f);

    ea = EulerAngles(kPi,-k2Pi,kPi * 8);
    rm.setup(ea);
    EXPECT_NEAR(-1.0f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(1.0f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(-1.0f, rm.m33, 1.0e-5f);
}

TEST(rotation_matrix, from_inertial_to_object_quaternion) {
    Quaternion q;
    RotationMatrix rm;

    q.setToRotateAboutX(0.5f);
    rm.fromInertialToObjectQuaternion(q);
    EXPECT_NEAR(1.0f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(0.87758f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(0.47942f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(-0.47942f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(0.87758f, rm.m33, 1.0e-5f);

    q.setToRotateAboutX(-0.5f);
    rm.fromInertialToObjectQuaternion(q);
    EXPECT_NEAR(1.0f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(0.87758f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(-0.47942f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(0.47942f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(0.87758f, rm.m33, 1.0e-5f);

    q.setToRotateAboutY(kPi);
    rm.fromInertialToObjectQuaternion(q);
    EXPECT_NEAR(-1.0f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(1.0f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(-1.0f, rm.m33, 1.0e-5f);

    q.setToRotateAboutZ(kPi*2/3);
    rm.fromInertialToObjectQuaternion(q);
    EXPECT_NEAR(-0.5f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(0.86602f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(-0.86602f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(-0.5f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(1.0f, rm.m33, 1.0e-5f);
}

TEST(rotation_matrix, from_object_to_inertial_quaternion) {
    Quaternion q;
    RotationMatrix rm;

    q.setToRotateAboutX(1.5f);
    rm.fromObjectToInertialQuaternion(q);
    EXPECT_NEAR(1.0f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(0.07073f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(-0.99749f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(0.99749f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(0.07073f, rm.m33, 1.0e-5f);

    q.setToRotateAboutY(-0.5f);
    rm.fromObjectToInertialQuaternion(q);
    EXPECT_NEAR(0.87758f, rm.m11, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m12, 1.0e-5f);
    EXPECT_NEAR(-0.47942f, rm.m13, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m21, 1.0e-5f);
    EXPECT_NEAR(1.0f, rm.m22, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m23, 1.0e-5f);
    EXPECT_NEAR(0.47942f, rm.m31, 1.0e-5f);
    EXPECT_NEAR(0.0f, rm.m32, 1.0e-5f);
    EXPECT_NEAR(0.87758f, rm.m33, 1.0e-5f);
}

TEST(rotation_matrix, inertial_to_object) {
    Quaternion q;
    RotationMatrix rm;
    Vector3 v;

    q.setToRotateAboutX(0.5f);
    rm.fromObjectToInertialQuaternion(q);
    v.x=1.0f, v.y=0.0f, v.z=0.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(1.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.z, 1.0e-5f);

    q.setToRotateAboutY(kPi);
    rm.fromObjectToInertialQuaternion(q);
    v.x=1.0f, v.y=0.0f, v.z=0.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(-1.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.z, 1.0e-5f);

    q.setToRotateAboutY(kPi/2);
    rm.fromObjectToInertialQuaternion(q);
    v.x=1.0f, v.y=0.0f, v.z=0.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(0.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.z, 1.0e-5f);

    q.setToRotateAboutZ(-0.45);
    rm.fromObjectToInertialQuaternion(q);
    v.x=1.0f, v.y=1.0f, v.z=1.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(0.46548f, v.x, 1.0e-5f);
    EXPECT_NEAR(1.33541f, v.y, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.z, 1.0e-5f);

    q.setToRotateAboutAxis(Vector3(1.0f, 1.0f, 1.0f), 0.5);
    rm.fromObjectToInertialQuaternion(q);
    v.x=1.0f, v.y=1.0f, v.z=1.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(1.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.z, 1.0e-5f);

    q.setToRotateAboutAxis(Vector3(-1.0f, 1.0f, 1.0f), 0.5);
    rm.fromObjectToInertialQuaternion(q);
    v.x=1.0f, v.y=1.0f, v.z=1.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(0.51033f, v.x, 1.0e-5f);
    EXPECT_NEAR(-0.20368f, v.y, 1.0e-5f);
    EXPECT_NEAR(1.71401f, v.z, 1.0e-5f);
}

TEST(rotation_matrix, object_to_inertial) {
    Quaternion q;
    RotationMatrix rm;
    Vector3 v;

    q.setToRotateAboutX(0.5f);
    rm.fromInertialToObjectQuaternion(q);
    v.x=1.0f, v.y=0.0f, v.z=0.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(1.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.z, 1.0e-5f);

    q.setToRotateAboutY(kPi);
    rm.fromInertialToObjectQuaternion(q);
    v.x=1.0f, v.y=0.0f, v.z=0.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(-1.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.z, 1.0e-5f);

    q.setToRotateAboutY(kPi/2);
    rm.fromInertialToObjectQuaternion(q);
    v.x=1.0f, v.y=0.0f, v.z=0.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(0.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(0.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(-1.0f, v.z, 1.0e-5f);

    q.setToRotateAboutZ(-0.45);
    rm.fromInertialToObjectQuaternion(q);
    v.x=1.0f, v.y=1.0f, v.z=1.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(1.33541f, v.x, 1.0e-5f);
    EXPECT_NEAR(0.46548f, v.y, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.z, 1.0e-5f);

    q.setToRotateAboutAxis(Vector3(1.0f, 1.0f, 1.0f), 0.5);
    rm.fromInertialToObjectQuaternion(q);
    v.x=1.0f, v.y=1.0f, v.z=1.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(1.0f, v.x, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.y, 1.0e-5f);
    EXPECT_NEAR(1.0f, v.z, 1.0e-5f);

    q.setToRotateAboutAxis(Vector3(-1.0f, 1.0f, 1.0f), 0.5);
    rm.fromInertialToObjectQuaternion(q);
    v.x=1.0f, v.y=1.0f, v.z=1.0f;
    v = rm.inertialToObject(v);
    EXPECT_NEAR(0.51033f, v.x, 1.0e-5f);
    EXPECT_NEAR(1.71401f, v.y, 1.0e-5f);
    EXPECT_NEAR(-0.20368f, v.z, 1.0e-5f);
}
