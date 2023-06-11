#include "gtest/gtest.h"
#include "math_util.h"

TEST(math_util, wrap_pi){
    EXPECT_NEAR(0.0f,      wrapPi(k2Pi),      1.0e-5f);
    EXPECT_NEAR(-kPi,      wrapPi(kPi),       1.0e-5f);
    EXPECT_NEAR(-kPi,      wrapPi(-kPi),      1.0e-5f);
    EXPECT_NEAR(-kPi,      wrapPi(7*kPi),     1.0e-5f);
    EXPECT_NEAR(kPi,       wrapPi(7*(-kPi)),  1.0e-5f); // floating point inaccuracy
    EXPECT_NEAR(0.0f,      wrapPi(8*kPi),     1.0e-5f);
    EXPECT_NEAR(0.0f,      wrapPi(8*(-kPi)),  1.0e-5f);
    EXPECT_NEAR(-kPi+0.1f, wrapPi(kPi+0.1f),  1.0e-5f);
    EXPECT_NEAR(kPi-0.1f,  wrapPi(kPi-0.1f),  1.0e-5f);
    EXPECT_NEAR(kPi-0.1f,  wrapPi(-kPi-0.1f), 1.0e-5f);
}
