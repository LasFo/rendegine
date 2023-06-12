#include "gtest/gtest.h"
#include "math_util.h"

TEST(math_util, wrap_pi){
    EXPECT_NEAR(0.0f,      wrap_pi(k2Pi),      1.0e-5f);
    EXPECT_NEAR(-kPi,      wrap_pi(kPi),       1.0e-5f);
    EXPECT_NEAR(-kPi,      wrap_pi(-kPi),      1.0e-5f);
    EXPECT_NEAR(-kPi,      wrap_pi(7*kPi),     1.0e-5f);
    EXPECT_NEAR(kPi,       wrap_pi(7*(-kPi)),  1.0e-5f); // floating point inaccuracy. Should be -kPi.
    EXPECT_NEAR(0.0f,      wrap_pi(8*kPi),     1.0e-5f);
    EXPECT_NEAR(0.0f,      wrap_pi(8*(-kPi)),  1.0e-5f);
    EXPECT_NEAR(-kPi+0.1f, wrap_pi(kPi+0.1f),  1.0e-5f);
    EXPECT_NEAR(kPi-0.1f,  wrap_pi(kPi-0.1f),  1.0e-5f);
    EXPECT_NEAR(kPi-0.1f,  wrap_pi(-kPi-0.1f), 1.0e-5f);
}

TEST(math_util, safe_acos){
    EXPECT_NEAR(kPi/2,     safe_acos(0.0f),      1.0e-5f);
    EXPECT_NEAR(kPi,       safe_acos(-1.1f),     1.0e-5f);
    EXPECT_NEAR(0.0f,      safe_acos(1.1f),      1.0e-5f);
    EXPECT_NEAR(0.451027f, safe_acos(1.0f-0.1f), 1.0e-5f);
    EXPECT_NEAR(2.69057f,  safe_acos(0.1f-1.0f), 1.0e-5f);
}
