#ifndef YAQLE_MATRIX_HPP
#define YAQLE_MATRIX_HPP

// #include <cstddef>
#include <etl/array.h>
#include <etl/vector.h>
#include <initializer_list>

#ifdef YAQLE_USE_COUT
#include <iostream>
#endif

namespace yaqle
{

template <size_t rows, size_t cols> class Matrix
{
  public:
    using container_type = etl::array<float, rows * cols>;

    Matrix() : m_data{}
    {
        m_data.fill(0.0f);
    }

    Matrix(const etl::array<float, rows * cols> &arr)
    {
        for (size_t i = 0; i < rows * cols; ++i)
            m_data[i] = arr[i];
    }

    // Allow {{...}, {...}, {...}} initialization
    Matrix(std::initializer_list<std::initializer_list<float>> init)
    {
        size_t i = 0;
        for (auto rowIt = init.begin(); rowIt != init.end() && i < rows; ++rowIt, ++i)
        {
            size_t j = 0;
            for (auto colIt = rowIt->begin(); colIt != rowIt->end() && j < cols; ++colIt, ++j)
            {
                (*this)(i, j) = *colIt;
            }
        }
        // Fill remaining entries with zeros (optional)
        for (size_t k = i * cols; k < rows * cols; ++k)
        {
            m_data[k] = 0.0f;
        }
    }

    static Matrix IdentityMatrix();

    float &operator()(size_t row, size_t col)
    {
        return m_data[row * cols + col];
    }

    const float &operator()(size_t row, size_t col) const
    {
        return m_data[row * cols + col];
    }

    // Addition
    Matrix operator+(const Matrix &other) const
    {
        Matrix result;
        for (size_t i = 0; i < rows * cols; ++i)
            result.m_data[i] = m_data[i] + other.m_data[i];
        return result;
    }

    // Subtraction
    Matrix operator-(const Matrix &other) const
    {
        Matrix result;
        for (size_t i = 0; i < rows * cols; ++i)
            result.m_data[i] = m_data[i] - other.m_data[i];
        return result;
    }

    // Scalar multiplication
    Matrix operator*(const float &scalar) const
    {
        Matrix result;
        for (size_t i = 0; i < rows * cols; ++i)
            result.m_data[i] = m_data[i] * scalar;
        return result;
    }

    // Matrix multiplication
    template <size_t other_cols> Matrix<rows, other_cols> operator*(const Matrix<cols, other_cols> &other) const
    {
        Matrix<rows, other_cols> result;
        for (size_t i = 0; i < rows; ++i)
        {
            for (size_t j = 0; j < other_cols; ++j)
            {
                float sum = 0.0f;
                for (size_t k = 0; k < cols; ++k)
                {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

#ifdef YAQLE_USE_COUT
    friend std::ostream &operator<<(std::ostream &out, const Matrix &matrix)
    {
        out << "[\n";
        for (size_t i = 0; i < rows; ++i)
        {
            out << " [ ";
            for (size_t j = 0; j < cols; ++j)
            {
                out << matrix(i, j);
                if (j < cols - 1)
                    out << ", ";
            }
            out << " ]\n";
        }
        out << "]\n";
        return out;
    }
#endif

    // Transpose
    Matrix<cols, rows> transpose() const
    {
        Matrix<cols, rows> result;
        for (size_t i = 0; i < rows; ++i)
            for (size_t j = 0; j < cols; ++j)
                result(j, i) = (*this)(i, j);
        return result;
    }

  private:
    container_type m_data;
};

template <size_t rows, size_t cols> Matrix<rows, cols> Matrix<rows, cols>::IdentityMatrix()
{
    ETL_ASSERT(rows == cols, "Identity matrix must be square.");
    Matrix<rows, cols> identity;
    for (size_t i = 0; i < rows; ++i)
    {
        identity(i, i) = 1.0f;
    }
    return identity;
}
} // namespace yaqle

#endif // YAQLE_MATRIX_HPP