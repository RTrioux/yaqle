#include "quat.hpp"
#include <gtest/gtest.h>

using namespace yaqle;

// Helper function to compare quaternions for equality
bool compareQuat(const Quat &q1, const Quat &q2, double tolerance = 1e-6)
{
    for (int i = 0; i < 4; ++i)
    {
        if (std::abs(q1[i] - q2[i]) > tolerance)
            return false;
    }
    return true;
}

// Test fixture for Quaternion library
class QuaternionTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
        // Initialize test quaternions
        q1 = Quat(1.0, 2.0, 3.0, 4.0);
        q2 = Quat(-1.0, -2.0, -3.0, -4.0);
        q3 = Quat(2.0, 4.0, 6.0, 8.0);

        uq1 = fromAxisAngle(0, 0, 0, 1);
        uq2 = fromAxisAngle(M_PI, 0, 0, 1);
    }

    // Test quaternions
    Quat q1, q2, q3;
    Quat uq1, uq2;
};

/* Constructors */

// Test case for default constructor
TEST_F(QuaternionTest, DefaultQuaternionConstructor)
{
    Quat q;

    // Perform necessary checks/assertions
    // For example, check that all elements are initialized to zero
    EXPECT_EQ(q[0], 0.0);
    EXPECT_EQ(q[1], 0.0);
    EXPECT_EQ(q[2], 0.0);
    EXPECT_EQ(q[3], 0.0);
}

// Test case for constructor with individual float arguments
TEST_F(QuaternionTest, IndividualFloatConstructor)
{
    float q0 = 1.0;
    float q1 = 2.0;
    float q2 = 3.0;
    float q3 = 4.0;

    Quat q(q0, q1, q2, q3);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from individual float arguments
    EXPECT_EQ(q[0], q0);
    EXPECT_EQ(q[1], q1);
    EXPECT_EQ(q[2], q2);
    EXPECT_EQ(q[3], q3);
}

// Test case for constructor with float array argument
TEST_F(QuaternionTest, FloatArrayConstructor)
{
    float arr[4] = {1.0, 2.0, 3.0, 4.0};

    Quat q(arr);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from the array
    EXPECT_EQ(q[0], arr[0]);
    EXPECT_EQ(q[1], arr[1]);
    EXPECT_EQ(q[2], arr[2]);
    EXPECT_EQ(q[3], arr[3]);
}

// Test case for constructor with float and Vector3D argument
TEST_F(QuaternionTest, FloatVector3DConstructor)
{
    float q0 = 1.0;
    float q1 = 2.0;
    float q2 = 3.0;
    float q3 = 4.0;
    etl::array<float, 3> arr3 = {5.0, 6.0, 7.0};
    Vector3D im(arr3);

    Quat q(q0, im);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from the float and Vector3D arguments
    EXPECT_EQ(q[0], q0);
    EXPECT_EQ(q[1], im[0]);
    EXPECT_EQ(q[2], im[1]);
    EXPECT_EQ(q[3], im[2]);
}

// Test case for constructor with etl::array argument
TEST_F(QuaternionTest, EtlArrayConstructor)
{
    etl::array<float, 4> arr = {1.0, 2.0, 3.0, 4.0};

    Quat q(arr);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from the etl::array
    EXPECT_EQ(q[0], arr[0]);
    EXPECT_EQ(q[1], arr[1]);
    EXPECT_EQ(q[2], arr[2]);
    EXPECT_EQ(q[3], arr[3]);
}

// Test case for Vector3D constructor with individual float arguments
TEST_F(QuaternionTest, Vector3DIndividualFloatConstructor)
{
    float x = 1.0;
    float y = 2.0;
    float z = 3.0;

    Vector3D v(x, y, z);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from individual float arguments
    EXPECT_EQ(v[0], x);
    EXPECT_EQ(v[1], y);
    EXPECT_EQ(v[2], z);
}

// Test case for Vector3D constructor with float array argument
TEST_F(QuaternionTest, Vector3DFloatArrayConstructor)
{
    float arr[3] = {1.0, 2.0, 3.0};

    Vector3D v(arr);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from the array
    EXPECT_EQ(v[0], arr[0]);
    EXPECT_EQ(v[1], arr[1]);
    EXPECT_EQ(v[2], arr[2]);
}

// Test case for Vector3D constructor with etl::array argument
TEST_F(QuaternionTest, Vector3DEtlArrayConstructor)
{
    etl::array<float, 3> arr = {1.0, 2.0, 3.0};

    Vector3D v(arr);

    // Perform necessary checks/assertions
    // For example, check that the elements are correctly set from the etl::array
    EXPECT_EQ(v[0], arr[0]);
    EXPECT_EQ(v[1], arr[1]);
    EXPECT_EQ(v[2], arr[2]);
}

/* Complex operations */

TEST_F(QuaternionTest, Inverse)
{
    Quat q_inv = q1.inverse();
    Quat identity = q1 * q_inv;

    // The result should be close to the identity quaternion
    EXPECT_NEAR(identity[0], 1.0, 1e-6);
    EXPECT_NEAR(identity[1], 0.0, 1e-6);
    EXPECT_NEAR(identity[2], 0.0, 1e-6);
    EXPECT_NEAR(identity[3], 0.0, 1e-6);
}

TEST_F(QuaternionTest, Conjugate)
{
    Quat q_conj = q1.conj();
    EXPECT_EQ(q_conj[0], q1[0]);
    EXPECT_EQ(q_conj[1], -q1[1]);
    EXPECT_EQ(q_conj[2], -q1[2]);
    EXPECT_EQ(q_conj[3], -q1[3]);
}

TEST_F(QuaternionTest, Real)
{
    EXPECT_EQ(q1.re(), q1[0]);
}

TEST_F(QuaternionTest, Imaginary)
{
    Vector3D im = q1.im();
    EXPECT_EQ(im[0], q1[1]);
    EXPECT_EQ(im[1], q1[2]);
    EXPECT_EQ(im[2], q1[3]);
}

TEST_F(QuaternionTest, Norm)
{
    float expected_norm = sqrtf(q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2] + q1[3] * q1[3]);
    EXPECT_NEAR(q1.norm(), expected_norm, 1e-6);
}

/* Arithmetics */
// Test addition operator
TEST_F(QuaternionTest, AdditionOperator)
{
    Quat result = q1 + q2;
    Quat expected = Quat(0.0, 0.0, 0.0, 0.0);
    EXPECT_TRUE(compareQuat(result, expected));
}

// Test subtraction operator
TEST_F(QuaternionTest, SubtractionOperator)
{
    Quat result = q1 - q2;
    Quat expected = Quat(2.0, 4.0, 6.0, 8.0);
    EXPECT_TRUE(compareQuat(result, expected));
}

// Test quaternion product
TEST_F(QuaternionTest, QuaternionProduct)
{
    Quat result = q1 * q2;
    Quat expected = Quat(28.0, -4.0, -6.0, -8.0);
    EXPECT_TRUE(compareQuat(result, expected));
}

/* Conversions tests */
TEST_F(QuaternionTest, fromAxisAngleIdentity)
{
    Quat q = fromAxisAngle(0, 0, 0, 1, false);
    EXPECT_NEAR(q.norm(), 1.0, 1e-6);
    EXPECT_NEAR(q[0], 1.0, 1e-6);
    EXPECT_NEAR(q[1], 0.0, 1e-6);
    EXPECT_NEAR(q[2], 0.0, 1e-6);
    EXPECT_NEAR(q[3], 0.0, 1e-6);
}

TEST_F(QuaternionTest, fromAxisAnglePiRotation)
{
    Quat q = fromAxisAngle(M_PI, 1, 0, 0, false);
    EXPECT_NEAR(q.norm(), 1.0, 1e-6);
    EXPECT_NEAR(q[0], 0.0, 1e-6);
    EXPECT_NEAR(q[1], 1.0, 1e-6);
    EXPECT_NEAR(q[2], 0.0, 1e-6);
    EXPECT_NEAR(q[3], 0.0, 1e-6);
}

TEST_F(QuaternionTest, fromAxisAngleNormalizationForced)
{
    Quat q = fromAxisAngle(M_PI, 10, 0, 0, true);
    EXPECT_NEAR(q.norm(), 1.0, 1e-6);
    // EXPECT_NEAR(q[0], 0.0, 1e-6);
    // EXPECT_NEAR(q[1], 1.0, 1e-6);
    EXPECT_NEAR(q[2], 0.0, 1e-6);
    EXPECT_NEAR(q[3], 0.0, 1e-6);
}

TEST_F(QuaternionTest, toEuler_fromEuler)
{
    Quat q = fromAxisAngle(M_PI / 4, 1, 1, 1, false);
    Vector3D euler = q.toEuler(Quat::ZYX, false, false);
    Quat q_test = fromEuler(euler, Quat::ZYX, false, false);

    EXPECT_TRUE(compareQuat(q, q_test, 1e-5));
}

TEST_F(QuaternionTest, toEulerXYZ)
{
    Quat q = fromAxisAngle(M_PI / 2, 1, 0, 0, false);
    Vector3D euler = q.toEuler(Quat::XYZ, false, false);
    EXPECT_NEAR(euler[0], M_PI / 2, 1e-5); // Roll
    EXPECT_NEAR(euler[1], 0.0, 1e-5);      // Pitch
    EXPECT_NEAR(euler[2], 0.0, 1e-5);      // Yaw
}

/* Spatial rotation tests */
TEST_F(QuaternionTest, RotateVector)
{
    Vector3D v(1, 0, 0);
    Quat q = fromAxisAngle(M_PI / 2, 0, 0, 1, false);
    Vector3D v_rot = q.rotate(v);
    EXPECT_NEAR(v_rot[0], 0.0, 1e-5);
    EXPECT_NEAR(v_rot[1], 1.0, 1e-5);
    EXPECT_NEAR(v_rot[2], 0.0, 1e-5);
}

// Test SLERP between two quaternions
TEST_F(QuaternionTest, QuaternionSLERP)
{
    for (float t = 0; t < 1; t += 0.01)
    {
        Quat q = slerp(uq1, uq2, t);
        float angle_ref = M_PI * t;
        float angle = 2.0f * acosf(q[0]);

        ASSERT_NEAR(angle, angle_ref, 1e-5);
    }
}