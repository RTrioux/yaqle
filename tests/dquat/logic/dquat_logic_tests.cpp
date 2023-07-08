#include "dquat.hpp"
#include <gtest/gtest.h>

using namespace yaqle;

// Test fixture for DQuat operators
class DualQuatLogicTest : public ::testing::Test
{
  protected:
    DQuat dq1{Quat{1.0, 2.0, 3.0, 4.0}, Quat{5.0, 6.0, 7.0, 8.0}};
    DQuat dq2{Quat{1.0, 2.0, 3.0, 4.0}, Quat{5.0, 6.0, 7.0, 8.0}};
    DQuat dq3{Quat{9.0, 10.0, 11.0, 12.0}, Quat{13.0, 14.0, 15.0, 16.0}};
};

// Test case for == operator of DQuat
TEST_F(DualQuatLogicTest, EqualityOperator)
{
    // Perform necessary checks/assertions
    EXPECT_TRUE(dq1 == dq2);
    EXPECT_FALSE(dq1 == dq3);
}

// Test case for != operator of DQuat
TEST_F(DualQuatLogicTest, InequalityOperator)
{
    // Perform necessary checks/assertions
    EXPECT_FALSE(dq1 != dq2);
    EXPECT_TRUE(dq1 != dq3);
}
