#include "quat.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <string>
#include <vector>

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

/* Euler conventions (intrinsic / extrinsic) */

namespace
{
struct SeqInfo
{
    Quat::Sequence seq;
    const char *name;
};

const SeqInfo ALL_SEQUENCES[12] = {{Quat::XYX, "XYX"}, {Quat::XYZ, "XYZ"}, {Quat::XZX, "XZX"}, {Quat::XZY, "XZY"},
                                   {Quat::YXY, "YXY"}, {Quat::YXZ, "YXZ"}, {Quat::YZX, "YZX"}, {Quat::YZY, "YZY"},
                                   {Quat::ZXY, "ZXY"}, {Quat::ZXZ, "ZXZ"}, {Quat::ZYX, "ZYX"}, {Quat::ZYZ, "ZYZ"}};

// Independent from the library: look the reversed name up (XYZ -> ZYX)
Quat::Sequence reverseSequence(const char *name)
{
    char rev[4] = {name[2], name[1], name[0], '\0'};
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        if (std::string(info.name) == rev)
            return info.seq;
    }
    return Quat::XYZ;
}

bool isProperEuler(const char *name)
{
    return name[0] == name[2];
}

// q and -q are the same rotation
bool sameRotation(const Quat &q1, const Quat &q2, double tolerance = 1e-5)
{
    return compareQuat(q1, q2, tolerance) || compareQuat(q1, -q2, tolerance);
}

using Mat3 = std::array<std::array<float, 3>, 3>;

Mat3 elementaryMatrix(char axis, float angle)
{
    const float c = cosf(angle), s = sinf(angle);
    switch (axis)
    {
    case 'X':
        return {{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
    case 'Y':
        return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
    default:
        return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
    }
}

Mat3 multiply(const Mat3 &A, const Mat3 &B)
{
    Mat3 C = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// Reference rotation matrix built from the definition of each convention
Mat3 referenceMatrix(const char *name, const float angles[3], bool isExtrinsic)
{
    Mat3 R1 = elementaryMatrix(name[0], angles[0]);
    Mat3 R2 = elementaryMatrix(name[1], angles[1]);
    Mat3 R3 = elementaryMatrix(name[2], angles[2]);
    // Intrinsic: R = R1.R2.R3 / Extrinsic: the first rotation is applied first: R = R3.R2.R1
    return isExtrinsic ? multiply(multiply(R3, R2), R1) : multiply(multiply(R1, R2), R3);
}

// Generic angles (no gimbal lock) valid for both Tait-Bryan and proper Euler sequences
const float GENERIC_ANGLES[4][3] = {{0.4f, 0.7f, -0.9f}, {-1.2f, 0.3f, 2.5f}, {2.9f, 1.1f, 0.2f}, {-0.1f, 1.4f, -2.8f}};
} // namespace

TEST_F(QuaternionTest, extrinsicIsReversedIntrinsic)
{
    // Extrinsic ABC (a,b,c) <=> Intrinsic CBA (c,b,a), and conversely
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        const Quat::Sequence rev = reverseSequence(info.name);
        for (const auto &ang : GENERIC_ANGLES)
        {
            SCOPED_TRACE(std::string(info.name) + " / " + std::to_string(ang[0]) + " " + std::to_string(ang[1]) + " " +
                         std::to_string(ang[2]));

            Quat qExt = fromEuler(ang[0], ang[1], ang[2], info.seq, false, true);
            Quat qIntRev = fromEuler(ang[2], ang[1], ang[0], rev, false, false);
            EXPECT_TRUE(compareQuat(qExt, qIntRev, 1e-6));

            Quat qInt = fromEuler(ang[0], ang[1], ang[2], info.seq, false, false);
            Quat qExtRev = fromEuler(ang[2], ang[1], ang[0], rev, false, true);
            EXPECT_TRUE(compareQuat(qInt, qExtRev, 1e-6));
        }
    }
}

TEST_F(QuaternionTest, extrinsicIsReversedIntrinsicDegrees)
{
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        SCOPED_TRACE(info.name);
        Quat qExt = fromEuler(30, 50, 70, info.seq, true, true);
        Quat qIntRev = fromEuler(70, 50, 30, reverseSequence(info.name), true, false);
        EXPECT_TRUE(compareQuat(qExt, qIntRev, 1e-6));
    }
}

TEST_F(QuaternionTest, toEulerExtrinsicIsReversedIntrinsic)
{
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        const Quat::Sequence rev = reverseSequence(info.name);
        for (const auto &ang : GENERIC_ANGLES)
        {
            SCOPED_TRACE(info.name);
            Quat q = fromEuler(ang[0], ang[1], ang[2], info.seq, false, false);
            Vector3D ext = q.toEuler(info.seq, false, true);
            Vector3D intRev = q.toEuler(rev, false, false);
            // Extrinsic ABC (a,b,c) <=> Intrinsic CBA (c,b,a)
            EXPECT_NEAR(ext[0], intRev[2], 1e-5);
            EXPECT_NEAR(ext[1], intRev[1], 1e-5);
            EXPECT_NEAR(ext[2], intRev[0], 1e-5);
        }
    }
}

TEST_F(QuaternionTest, fromEulerMatchesReferenceMatrix)
{
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        for (bool isExtrinsic : {false, true})
        {
            for (const auto &ang : GENERIC_ANGLES)
            {
                SCOPED_TRACE(std::string(info.name) + (isExtrinsic ? " extrinsic" : " intrinsic"));
                Quat q = fromEuler(ang[0], ang[1], ang[2], info.seq, false, isExtrinsic);
                Mat3 Rref = referenceMatrix(info.name, ang, isExtrinsic);
                Matrix<3, 3> R = q.toRotationMatrix(info.seq, isExtrinsic);
                for (size_t i = 0; i < 3; i++)
                    for (size_t j = 0; j < 3; j++)
                        EXPECT_NEAR(R(i, j), Rref[i][j], 1e-5) << "at (" << i << "," << j << ")";
            }
        }
    }
}

TEST_F(QuaternionTest, toEulerFromEulerRoundTrip)
{
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        for (bool isExtrinsic : {false, true})
        {
            for (const auto &ang : GENERIC_ANGLES)
            {
                // Tait-Bryan middle angle is in [-pi/2, pi/2], proper Euler one in [0, pi]
                if (!isProperEuler(info.name) && std::abs(ang[1]) > M_PI_2)
                    continue;
                SCOPED_TRACE(std::string(info.name) + (isExtrinsic ? " extrinsic" : " intrinsic"));

                Quat q = fromEuler(ang[0], ang[1], ang[2], info.seq, false, isExtrinsic);
                Vector3D euler = q.toEuler(info.seq, false, isExtrinsic);
                EXPECT_TRUE(sameRotation(q, fromEuler(euler, info.seq, false, isExtrinsic)));
                EXPECT_NEAR(euler[1], ang[1], 1e-4);
            }
        }
    }
}

TEST_F(QuaternionTest, toEulerReturnsOriginalAngles)
{
    // Middle angle in the principal range for both families: no ambiguity
    const float ang[3] = {0.4f, 0.7f, -0.9f};
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        for (bool isExtrinsic : {false, true})
        {
            SCOPED_TRACE(std::string(info.name) + (isExtrinsic ? " extrinsic" : " intrinsic"));
            Quat q = fromEuler(ang[0], ang[1], ang[2], info.seq, false, isExtrinsic);
            Vector3D euler = q.toEuler(info.seq, false, isExtrinsic);
            EXPECT_NEAR(euler[0], ang[0], 1e-4);
            EXPECT_NEAR(euler[1], ang[1], 1e-4);
            EXPECT_NEAR(euler[2], ang[2], 1e-4);
        }
    }
}

TEST_F(QuaternionTest, toEulerDegrees)
{
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        for (bool isExtrinsic : {false, true})
        {
            SCOPED_TRACE(std::string(info.name) + (isExtrinsic ? " extrinsic" : " intrinsic"));
            Quat q = fromEuler(20, 40, -60, info.seq, true, isExtrinsic);
            Vector3D euler = q.toEuler(info.seq, true, isExtrinsic);
            EXPECT_NEAR(euler[0], 20, 1e-2);
            EXPECT_NEAR(euler[1], 40, 1e-2);
            EXPECT_NEAR(euler[2], -60, 1e-2);
        }
    }
}

TEST_F(QuaternionTest, toEulerGimbalLock)
{
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        // Tait-Bryan: beta = +/-pi/2, proper Euler: beta = 0 or pi
        std::vector<float> betas;
        if (isProperEuler(info.name))
            betas = {0.0f, (float)M_PI};
        else
            betas = {(float)M_PI_2, -(float)M_PI_2};

        for (bool isExtrinsic : {false, true})
        {
            for (float beta : betas)
            {
                for (const auto &ang : GENERIC_ANGLES)
                {
                    SCOPED_TRACE(std::string(info.name) + (isExtrinsic ? " extrinsic" : " intrinsic") +
                                 " beta=" + std::to_string(beta));
                    Quat q = fromEuler(ang[0], beta, ang[2], info.seq, false, isExtrinsic);
                    Vector3D euler = q.toEuler(info.seq, false, isExtrinsic);

                    EXPECT_FALSE(std::isnan(euler[0]) || std::isnan(euler[1]) || std::isnan(euler[2]));
                    // The third angle is set to 0 and the rotation is preserved
                    EXPECT_NEAR(euler[2], 0.0, 1e-5);
                    EXPECT_TRUE(sameRotation(q, fromEuler(euler, info.seq, false, isExtrinsic), 1e-4));
                }
            }
        }
    }
}

TEST_F(QuaternionTest, toEulerSlightlyOutOfRangeDoesNotProduceNaN)
{
    // Rounding errors can push a rotation matrix coefficient slightly above 1
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        for (bool isExtrinsic : {false, true})
        {
            Quat q = fromEuler(0.3f, (float)M_PI_2, 0.5f, info.seq, false, isExtrinsic);
            Vector3D euler = q.toEuler(info.seq, false, isExtrinsic);
            EXPECT_FALSE(std::isnan(euler[0]) || std::isnan(euler[1]) || std::isnan(euler[2])) << info.name;
        }
    }
}

TEST_F(QuaternionTest, rotationMatrixIndependentOfSequence)
{
    Quat q = fromAxisAngle(0.8f, 1, 2, 3, true);
    Matrix<3, 3> ref = q.toRotationMatrix();
    for (const SeqInfo &info : ALL_SEQUENCES)
    {
        for (bool isExtrinsic : {false, true})
        {
            Matrix<3, 3> R = q.toRotationMatrix(info.seq, isExtrinsic);
            for (size_t i = 0; i < 3; i++)
                for (size_t j = 0; j < 3; j++)
                    EXPECT_NEAR(R(i, j), ref(i, j), 1e-6);
        }
    }

    // Columns of the matrix are the images of the basis vectors
    Vector3D ex = q.rotate(Vector3D(1, 0, 0));
    EXPECT_NEAR(ref(0, 0), ex[0], 1e-5);
    EXPECT_NEAR(ref(1, 0), ex[1], 1e-5);
    EXPECT_NEAR(ref(2, 0), ex[2], 1e-5);
}

TEST_F(QuaternionTest, intrinsicVsExtrinsicKnownRotation)
{
    // XYZ with (90, 0, 90) degrees
    Quat qInt = fromEuler(90, 0, 90, Quat::XYZ, true, false);
    Quat qExt = fromEuler(90, 0, 90, Quat::XYZ, true, true);

    // Intrinsic: R = Rx(90).Rz(90) -> ex goes to ez
    Vector3D vInt = qInt.rotate(Vector3D(1, 0, 0));
    EXPECT_NEAR(vInt[0], 0.0, 1e-5);
    EXPECT_NEAR(vInt[1], 0.0, 1e-5);
    EXPECT_NEAR(vInt[2], 1.0, 1e-5);

    // Extrinsic: first X0 then Z0, R = Rz(90).Rx(90) -> ex goes to ey
    Vector3D vExt = qExt.rotate(Vector3D(1, 0, 0));
    EXPECT_NEAR(vExt[0], 0.0, 1e-5);
    EXPECT_NEAR(vExt[1], 1.0, 1e-5);
    EXPECT_NEAR(vExt[2], 0.0, 1e-5);

    // Extrinsic XYZ == intrinsic ZYX with reversed angles
    EXPECT_TRUE(compareQuat(qExt, fromEuler(90, 0, 90, Quat::ZYX, true, false), 1e-6));
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