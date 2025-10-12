#include "matrix.hpp"
#include <gtest/gtest.h>

using namespace yaqle;

class MatrixTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
    }
};

// -------------------------
// Matrix Tests
// -------------------------

TEST(MatrixTest, DefaultConstructor)
{
    Matrix<3, 3> m;
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            EXPECT_FLOAT_EQ(m(i, j), 0.0f);
        }
    }
}

TEST(MatrixTest, InitFromETLArray)
{
    etl::array<float, 9> arr = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    Matrix<3, 3> m(arr);
    EXPECT_FLOAT_EQ(m(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 5.0f);
    EXPECT_FLOAT_EQ(m(2, 2), 9.0f);
}

TEST(MatrixTest, InitFromInitializerList)
{
    Matrix<2, 2> m = {{1, 2}, {3, 4}};
    EXPECT_FLOAT_EQ(m(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(m(1, 0), 3.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 4.0f);
}

TEST(MatrixTest, IdentityMatrix)
{
    Matrix<3, 3> I = Matrix<3, 3>::IdentityMatrix();
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            if (i == j)
                EXPECT_FLOAT_EQ(I(i, j), 1.0f);
            else
                EXPECT_FLOAT_EQ(I(i, j), 0.0f);
        }
    }
}

TEST(MatrixTest, ElementAccess)
{
    Matrix<2, 2> m;
    m(0, 0) = 10.0f;
    m(1, 1) = 20.0f;
    EXPECT_FLOAT_EQ(m(0, 0), 10.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 20.0f);
}

TEST(MatrixTest, Addition)
{
    Matrix<2, 2> a = {{1, 2}, {3, 4}};
    Matrix<2, 2> b = {{5, 6}, {7, 8}};
    Matrix<2, 2> c = a + b;
    EXPECT_FLOAT_EQ(c(0, 0), 6.0f);
    EXPECT_FLOAT_EQ(c(0, 1), 8.0f);
    EXPECT_FLOAT_EQ(c(1, 0), 10.0f);
    EXPECT_FLOAT_EQ(c(1, 1), 12.0f);
}

TEST(MatrixTest, Subtraction)
{
    Matrix<2, 2> a = {{5, 6}, {7, 8}};
    Matrix<2, 2> b = {{1, 2}, {3, 4}};
    Matrix<2, 2> c = a - b;
    EXPECT_FLOAT_EQ(c(0, 0), 4.0f);
    EXPECT_FLOAT_EQ(c(0, 1), 4.0f);
    EXPECT_FLOAT_EQ(c(1, 0), 4.0f);
    EXPECT_FLOAT_EQ(c(1, 1), 4.0f);
}

TEST(MatrixTest, ScalarMultiplication)
{
    Matrix<2, 2> a = {{1, 2}, {3, 4}};
    Matrix<2, 2> b = a * 2.0f;
    EXPECT_FLOAT_EQ(b(0, 0), 2.0f);
    EXPECT_FLOAT_EQ(b(1, 1), 8.0f);
}

TEST(MatrixTest, MatrixMultiplication)
{
    Matrix<2, 3> a = {{1, 2, 3}, {4, 5, 6}};
    Matrix<3, 2> b = {{7, 8}, {9, 10}, {11, 12}};
    Matrix<2, 2> c = a * b;
    EXPECT_FLOAT_EQ(c(0, 0), 58.0f);  // 1*7 + 2*9 + 3*11
    EXPECT_FLOAT_EQ(c(0, 1), 64.0f);  // 1*8 + 2*10 + 3*12
    EXPECT_FLOAT_EQ(c(1, 0), 139.0f); // 4*7 + 5*9 + 6*11
    EXPECT_FLOAT_EQ(c(1, 1), 154.0f); // 4*8 + 5*10 + 6*12
}

TEST(MatrixTest, Transpose)
{
    Matrix<2, 3> a = {{1, 2, 3}, {4, 5, 6}};
    Matrix<3, 2> t = a.transpose();
    EXPECT_FLOAT_EQ(t(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(t(0, 1), 4.0f);
    EXPECT_FLOAT_EQ(t(1, 0), 2.0f);
    EXPECT_FLOAT_EQ(t(1, 1), 5.0f);
    EXPECT_FLOAT_EQ(t(2, 0), 3.0f);
    EXPECT_FLOAT_EQ(t(2, 1), 6.0f);
}
