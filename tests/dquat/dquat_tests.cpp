#include "dquat.hpp"
#include <etl/array.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace yaqle;

// Helper function to compare dual quaternions for equality
bool compareDualQuat(const DQuat &dq1, const DQuat &dq2, double tolerance = 1e-6)
{
    for (int i = 0; i < 8; ++i)
    {
        if (std::abs(dq1[i] - dq2[i]) > tolerance)
            return false;
    }
    return true;
}

// Test fixture for Dual Quaternion library
class DualQuatTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
        // Initialize test dual quaternions
        dq1 = DQuat(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        dq2 = DQuat(-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0);
        dq3 = DQuat(2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0);

        rt1 = rigidTransform(unitQuat(0, 0, 0, 0), 0, 0, 0);
        rt2 = rigidTransform(unitQuat(M_PI, 0, 0, 1), 2, -1, 1);
    }

    // Test dual quaternions
    DQuat dq1, dq2, dq3;
    DQuat rt1, rt2;
};

/* Constructors */
// Test case for default constructor
TEST_F(DualQuatTest, DefaultConstructor)
{
    DQuat dq;

    // Perform necessary checks/assertions
    // For example, check that all elements are initialized to zero
    EXPECT_EQ(dq[0], 0.0);
    EXPECT_EQ(dq[1], 0.0);
    EXPECT_EQ(dq[2], 0.0);
    EXPECT_EQ(dq[3], 0.0);
    EXPECT_EQ(dq[4], 0.0);
    EXPECT_EQ(dq[5], 0.0);
    EXPECT_EQ(dq[6], 0.0);
    EXPECT_EQ(dq[7], 0.0);
}

// Test case for constructor with Quat arguments
TEST_F(DualQuatTest, QuatConstructor)
{
    Quat hRe(1.0, 2.0, 3.0, 4.0);
    Quat hIm(5.0, 6.0, 7.0, 8.0);

    DQuat dq(hRe, hIm);

    // Perform necessary checks/assertions
    // For example, check that the real and imaginary parts are correctly set
    EXPECT_EQ(dq[0], hRe[0]);
    EXPECT_EQ(dq[1], hRe[1]);
    EXPECT_EQ(dq[2], hRe[2]);
    EXPECT_EQ(dq[3], hRe[3]);
    EXPECT_EQ(dq[4], hIm[0]);
    EXPECT_EQ(dq[5], hIm[1]);
    EXPECT_EQ(dq[6], hIm[2]);
    EXPECT_EQ(dq[7], hIm[3]);
}

// Test case for constructor with float array argument
TEST_F(DualQuatTest, FloatArrayConstructor)
{
    float arr[8] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};

    DQuat dq(arr);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from the array
    EXPECT_EQ(dq[0], arr[0]);
    EXPECT_EQ(dq[1], arr[1]);
    EXPECT_EQ(dq[2], arr[2]);
    EXPECT_EQ(dq[3], arr[3]);
    EXPECT_EQ(dq[4], arr[4]);
    EXPECT_EQ(dq[5], arr[5]);
    EXPECT_EQ(dq[6], arr[6]);
    EXPECT_EQ(dq[7], arr[7]);
}

// Test case for constructor with etl::array arguments
TEST_F(DualQuatTest, EtlArrayConstructor)
{
    etl::array<float, 4> hRe = {1.0, 2.0, 3.0, 4.0};
    etl::array<float, 4> hIm = {5.0, 6.0, 7.0, 8.0};

    DQuat dq(hRe, hIm);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from the etl::array objects
    EXPECT_EQ(dq[0], hRe[0]);
    EXPECT_EQ(dq[1], hRe[1]);
    EXPECT_EQ(dq[2], hRe[2]);
    EXPECT_EQ(dq[3], hRe[3]);
    EXPECT_EQ(dq[4], hIm[0]);
    EXPECT_EQ(dq[5], hIm[1]);
    EXPECT_EQ(dq[6], hIm[2]);
    EXPECT_EQ(dq[7], hIm[3]);
}

// Test case for constructor with individual float arguments
TEST_F(DualQuatTest, IndividualFloatConstructor)
{
    float w0 = 1.0;
    float x0 = 2.0;
    float y0 = 3.0;
    float z0 = 4.0;
    float w1 = 5.0;
    float x1 = 6.0;
    float y1 = 7.0;
    float z1 = 8.0;

    DQuat dq(w0, x0, y0, z0, w1, x1, y1, z1);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from individual float arguments
    EXPECT_EQ(dq[0], w0);
    EXPECT_EQ(dq[1], x0);
    EXPECT_EQ(dq[2], y0);
    EXPECT_EQ(dq[3], z0);
    EXPECT_EQ(dq[4], w1);
    EXPECT_EQ(dq[5], x1);
    EXPECT_EQ(dq[6], y1);
    EXPECT_EQ(dq[7], z1);
}

// Test addition operator
TEST_F(DualQuatTest, AdditionOperator)
{
    DQuat result = dq1 + dq2;
    DQuat expected = DQuat(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_TRUE(compareDualQuat(result, expected));
}

// Test subtraction operator
TEST_F(DualQuatTest, SubtractionOperator)
{
    DQuat result = dq1 - dq2;
    DQuat expected = DQuat(2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0);
    EXPECT_TRUE(compareDualQuat(result, expected));
}

// Test quaternion product
TEST_F(DualQuatTest, QuaternionProduct)
{
    DQuat result = dq1 * dq2;
    DQuat expected = DQuat(28, -4.0, -6.0, -8, 120, -32, -44.0, -56.0);
    EXPECT_TRUE(compareDualQuat(result, expected));
}

// Test conjugate definitions
TEST_F(DualQuatTest, ConjugateDefinitions)
{
    DQuat conjResult = dq1.conj();
    DQuat dconjResult = dq1.dconj();
    DQuat mconResult = dq1.mconj();

    DQuat conjExpected = DQuat(1.0, -2.0, -3.0, -4.0, 5.0, -6.0, -7.0, -8.0);
    DQuat dconjExpected = DQuat(1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0);
    DQuat mconExpected = DQuat(1.0, -2.0, -3.0, -4.0, -5.0, 6.0, 7.0, 8.0);

    EXPECT_TRUE(compareDualQuat(conjResult, conjExpected));
    EXPECT_TRUE(compareDualQuat(dconjResult, dconjExpected));
    EXPECT_TRUE(compareDualQuat(mconResult, mconExpected));
}

TEST_F(DualQuatTest, Normalization)
{
    DQuat dqn = dq1.normalize();
    EXPECT_TRUE(dqn.isNormalized());
}

TEST_F(DualQuatTest, slerp)
{
    for (float t = 0; t < 1; t += 0.001)
    {
        DQuat rt = slerp(rt1, rt2, t);
        Vector3D pos = rt.pos();

        ASSERT_NEAR(pos[0], 2 * t, 1e-6);
        ASSERT_NEAR(pos[1], -t, 1e-6);
        ASSERT_NEAR(pos[2], t, 1e-6);
    }
}