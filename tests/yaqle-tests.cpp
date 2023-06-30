#include <gtest/gtest.h>
#include <cmath>

// Include the header files of the components you want to test
#include "quat.hpp"

using namespace yaqle;

void assertCoord(Quat q, float w, float x, float y, float z)
{
    ASSERT_FLOAT_EQ(q[0], w);
    ASSERT_FLOAT_EQ(q[1], x);
    ASSERT_FLOAT_EQ(q[2], y);
    ASSERT_FLOAT_EQ(q[3], z);
}

// Define your test cases
TEST(Instantiation, Basic)
{
    Quat q;
    q = Quat(1, 2, 3, 4);
    assertCoord(q, 1, 2, 3, 4);

    float arr[4] = {1, 2, 3, 4};
    q = Quat(arr);
    assertCoord(q, 1, 2, 3, 4);

    // Implicit declaration
    // This call the constructor Quat(float q0,float q1,float q2,float q3)
    q = 15;
    assertCoord(q, 15, 0, 0, 0);
}

TEST(Instantiation, UnitQuat)
{
    /** Create a unit quaternion describing a rotation `angle` along a given axis **/
    /* (float angle, float x, float y, float z, bool degree = false) */
    Quat Uq1 = unitQuat(M_PI_4, 1, 0, 0);   // Rotation of Pi/4 rad about X axis
    Quat Uq2 = unitQuat(45, 1, 0, 0, true); // Rotation of 45° about X axis

    /* (float real, Vector3D im, bool degree = false) */
    Quat Uq3 = unitQuat(M_PI_4, Vector3D(1, 0, 0));
    Quat Uq4 = unitQuat(45, Vector3D(1, 0, 0), true);

    // Norm test
    ASSERT_FLOAT_EQ(Uq1.norm(), 1.0f);
    ASSERT_FLOAT_EQ(Uq2.norm(), 1.0f);
    ASSERT_FLOAT_EQ(Uq3.norm(), 1.0f);
    ASSERT_FLOAT_EQ(Uq4.norm(), 1.0f);

    // Coordinates test
    float c1 = cosf(M_PI_4 / 2.0f);
    assertCoord(Uq1, c1, sqrtf(1 - c1 * c1), 0, 0);
    assertCoord(Uq2, c1, sqrtf(1 - c1 * c1), 0, 0);
    assertCoord(Uq3, c1, sqrtf(1 - c1 * c1), 0, 0);
    assertCoord(Uq4, c1, sqrtf(1 - c1 * c1), 0, 0);
}

TEST(Arithmetic, AddSub)
{
    Quat q1(2, 3, 5, 7);
    Quat q2(11, 13, 17, 19);

    Quat q3 = q1 + q2;
    assertCoord(q3, 13, 16, 22, 26);

    q3 = q2 - q1;
    assertCoord(q3, 9, 10, 12, 12);
}

TEST(Arithmetic, Multiplication)
{
    Quat q1(2, 3, 5, 7);
    Quat q2(11, 13, 17, 19);
    assertCoord(q1 * q2, -235, 35, 123, 101);
    assertCoord(q2 * q1, -235, 83, 55, 129);
}

TEST(Arithmetic, Division)
{
    Quat q1(2, 3, 5, 7);

    assertCoord(q1 / q1, 1, 0, 0, 0);
}

// Entry point for the test executable
int main(int argc, char **argv)
{
    // Initialize Google Test
    ::testing::InitGoogleTest(&argc, argv);

    // Run the tests
    return RUN_ALL_TESTS();
}
