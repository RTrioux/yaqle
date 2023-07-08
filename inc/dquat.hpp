#ifndef DQUAT_HPP
#define DQUAT_HPP
#include "quat.hpp"

namespace yaqle
{

class DQuat
{
  public:
    DQuat();
    DQuat(Quat hRe, Quat hIm);
    DQuat(float arr[8]);
    DQuat(float hRe[4], float hIm[4]);
    DQuat(etl::array<float, 4> hRe, etl::array<float, 4> hIm);
    DQuat(float w0, float x0, float y0, float z0, float w1, float x1, float y1, float z1); // Usefull for gtest

    virtual ~DQuat()
    {
    }

    /* Test */
    bool isEqual(DQuat const &) const;
    bool isNull() const;
    bool isNormalized() const;

    /* Operators */
    DQuat operator*(DQuat const &) const;
    DQuat operator+(DQuat const &) const;
    DQuat operator-(DQuat const &) const;
    DQuat operator-() const;
    bool operator==(DQuat const &) const;
    bool operator!=(DQuat const &) const;
    float &operator[](size_t index);
    float operator[](size_t index) const;
    DQuat operator/(DQuat const &) const;
#ifdef YAQLE_USE_COUT
    friend std::ostream &operator<<(std::ostream &, DQuat const &);
#endif

    /** Algebra **/
    Quat hRe() const;        // Return Hyper real part
    Quat hIm() const;        // Return Hyper Im part
    DQuat norm() const;      // Yes norms are dual numbers
    DQuat norm2() const;
    DQuat conj() const;      // Quaternion conjugate    (A+e.B) => (A* + e.B*)
    DQuat dconj() const;     // Dual conjugate          (A+e.B) => (A - e.B)
    DQuat mconj() const;     // Mixed conjugate         (A+e.B) => (A* - e.B*)
    DQuat inverse() const;
    DQuat normalize() const; // Return normalized dual
    Vector3D pos() const;

    /** Display **/
#ifdef YAQLE_USE_COUT
    void print() const;
    void writeToFile(std::ofstream &) const;
#endif

  private:
    float m_arr[8];
    Quat m_hRe; // Hyper real part
    Quat m_hIm; // Hyper Imaginary part
};

/** "Static" functions **/

inline bool isEqual(DQuat const &dq1, DQuat const &dq2)
{
    return dq1.isEqual(dq2);
}
inline bool isNull(DQuat const &dq)
{
    return dq.isNull();
}
inline DQuat inverse(DQuat const &dq)
{
    return dq.inverse();
}
inline DQuat conj(DQuat const &dq)
{
    return dq.conj();
}
inline DQuat dconj(DQuat const &dq)
{
    return dq.dconj();
}
inline DQuat mconj(DQuat const &dq)
{
    return dq.mconj();
}
inline Quat hIm(DQuat const &dq)
{
    return dq.hIm();
}
inline Quat hRe(DQuat const &dq)
{
    return dq.hRe();
}
// inline float norm(DQuat const &dq) { return dq.norm(); }
inline DQuat norm2(DQuat const &dq)
{
    return dq.norm2();
}
// inline Quat normalize(Quat const &q) { return q.normalize(); }
// inline float getAngle(Quat const &q) { return q.getAngle(); }

/** Tools **/

DQuat rigidTransform(Quat Qr, Vector3D t);
DQuat rigidTransform(Quat Qr, float x, float y, float z);

/** Interpolation **/
DQuat slerp(const DQuat &dq0, const DQuat &dq1, float t);

/** Operators **/

template <typename T> DQuat operator/(DQuat const &dq, T const &scalar)
{
    if (etl::is_arithmetic<T>::value)
    {
        float arr[8];
        for (size_t i = 0; i < 8; i++)
        {
            arr[i] = dq[i] / scalar;
        }
        return DQuat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}

template <typename T> DQuat operator/(T const &scalar, DQuat const &dq)
{

    if (etl::is_arithmetic<T>::value)
    {
        float arr[8];
        DQuat dq_inv = dq.inverse();
        for (size_t i = 0; i < 8; i++)
        {
            arr[i] = scalar * dq_inv[i];
        }
        return DQuat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}

template <typename T> DQuat operator*(T const &scalar, DQuat const &dq)
{
    if (etl::is_arithmetic<T>::value)
    {
        float arr[8] = {0};
        for (size_t i = 0; i < 8; i++)
        {
            arr[i] = scalar * dq[i];
        }
        return DQuat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}

template <typename T> DQuat operator*(DQuat const &dq, T const &scalar)
{
    return scalar * dq;
}

} // namespace yaqle

#endif