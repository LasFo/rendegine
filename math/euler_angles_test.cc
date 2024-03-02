#include "gtest/gtest.h"
#include "euler_angles.h"
#include "math_util.h"
#include "quaternion.h"
#include "rotation_matrix.h"
#include "matrix_4x3.h"


TEST(euler_angles, canonize){
    EulerAngles ea(0.0f,0.0f,0.0f);
    ea.canonize();
    EXPECT_NEAR(0.0f, ea.heading, 1.0e-5f);
    EXPECT_NEAR(0.0f, ea.pitch,   1.0e-5f);
    EXPECT_NEAR(0.0f, ea.bank,    1.0e-5f);

    ea = EulerAngles(1.0f,-1.0f,0.0f);
    ea.canonize();
    EXPECT_NEAR(1.0f,  ea.heading, 1.0e-5f);
    EXPECT_NEAR(-1.0f, ea.pitch,   1.0e-5f);
    EXPECT_NEAR(0.0f,  ea.bank,    1.0e-5f);

    ea = EulerAngles(kPi,-k2Pi,kPi * 8);
    ea.canonize();
    EXPECT_NEAR(-kPi, ea.heading, 1.0e-5f);
    EXPECT_NEAR(0.0f, ea.pitch,   1.0e-5f);
    EXPECT_NEAR(0.0f, ea.bank,    1.0e-5f);

    ea = EulerAngles(-kPi,(kPi-0.1f)/2.0f,kPi/2.0f);
    ea.canonize();
    EXPECT_NEAR(-kPi, ea.heading, 1.0e-5f);
    EXPECT_NEAR((kPi-0.1f)/2.0f, ea.pitch,   1.0e-5f);
    EXPECT_NEAR(kPi/2.0f, ea.bank,    1.0e-5f);
}

TEST(euler_angles, fromObjectToInertialQuaternion){
    EulerAngles ea;
    Quaternion q{.w = 1.0f, .x = 1.0f, .y = 1.0f, .z = 1.0f};
    ea.fromObjectToInertialQuaternion(q);
    EXPECT_NEAR(2.21429f, ea.heading, 1.0e-5f);
    EXPECT_NEAR(0.0f,     ea.pitch,   1.0e-5f);
    EXPECT_NEAR(2.21429f, ea.bank,    1.0e-5f);
}

TEST(euler_angles, fromInertialToObjectQuaternion){
    EulerAngles ea;
    Quaternion q{.w = 1.0f, .x = 1.0f, .y = 1.0f, .z = 1.0f};
    ea.fromObjectToInertialQuaternion(q);
    EXPECT_NEAR(2.21429f, ea.heading, 1.0e-5f);
    EXPECT_NEAR(0.0f,     ea.pitch,   1.0e-5f);
    EXPECT_NEAR(2.21429f, ea.bank,    1.0e-5f);
}

TEST(euler_angles, fromObjectToWorldMatrix){
    EulerAngles ea;
    Matrix4x3 m{
        .m11 = 1.0f, .m12 = 0.0f, .m13 = 0.0f,
        .m21 = 0.0f, .m22 = 1.0f, .m23 = 0.0f,
        .m31 = 0.0f, .m32 = 0.0f, .m33 = 1.0f,
        .tx = 0.0f,  .ty = 0.0f,  .tz = 0.5f,
    };
    ea.fromObjectToWorldMatrix(m);
    EXPECT_NEAR(0.0f, ea.heading, 1.0e-5f);
    EXPECT_NEAR(0.0f, ea.pitch,   1.0e-5f);
    EXPECT_NEAR(0.0f, ea.bank,    1.0e-5f);
}

TEST(euler_angles, fromWorldToObjectMatrix){
    EulerAngles ea;
    Matrix4x3 m{
        .m11 = 1.0f, .m12 = 0.0f, .m13 = 0.0f,
        .m21 = 0.0f, .m22 = 1.0f, .m23 = 0.0f,
        .m31 = 0.0f, .m32 = 0.0f, .m33 = 1.0f,
        .tx = 5.0f,  .ty = 8.0f,  .tz = 105.0f,
    };
    ea.fromWorldToObjectMatrix(m);
    EXPECT_NEAR(0.0f, ea.heading, 1.0e-5f);
    EXPECT_NEAR(0.0f, ea.pitch,   1.0e-5f);
    EXPECT_NEAR(0.0f, ea.bank,    1.0e-5f);
}

TEST(euler_angles, fromtRotationMatrix){
    EulerAngles ea;
    RotationMatrix rm{
        .m11 = 1.0f, .m12 = 0.0f, .m13 = 0.0f,
        .m21 = 0.0f, .m22 = 1.0f, .m23 = 0.0f,
        .m31 = 0.0f, .m32 = 0.0f, .m33 = 1.0f,
    };
    ea.fromRotationMatrix(rm);
    EXPECT_NEAR(0.0f, ea.heading, 1.0e-5f);
    EXPECT_NEAR(0.0f, ea.pitch,   1.0e-5f);
    EXPECT_NEAR(0.0f, ea.bank,    1.0e-5f);

    float values[5] = {0.5f, kPi-0.1f, -kPi+0.1f, 1.154f, 0};
    for(float h : values) for(float p : values) for (float b : values) {
        EulerAngles ea_in(h, p, b);
        EulerAngles ea_out;
        ea_in.canonize();
        rm.setup(ea_in);
        ea_out.fromRotationMatrix(rm);

        ea_out.canonize();
        EXPECT_NEAR(wrap_pi(ea_in.heading+k2Pi), wrap_pi(ea_out.heading+k2Pi), 1.0e-5f);
        EXPECT_NEAR(wrap_pi(ea_in.pitch+k2Pi), wrap_pi(ea_out.pitch+k2Pi),   1.0e-5f);
        EXPECT_NEAR(wrap_pi(ea_in.bank+k2Pi), wrap_pi(ea_out.bank+k2Pi),    1.0e-5f);
    }
}

