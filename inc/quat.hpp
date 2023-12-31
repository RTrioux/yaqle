#ifndef QUAT_HPP
#define QUAT_HPP
#include "vector3d.hpp"
#include <cmath>
#include <etl/array.h>
#include <etl/map.h>

#include <etl/static_assert.h>
#include <etl/string.h>
#include <fstream>

#ifdef YAQLE_USE_COUT
#include <ostream>
#endif

namespace yaqle
{

class Quat
{
  public:
    Quat(float q0 = 0, float q1 = 0, float q2 = 0, float q3 = 0);
    Quat(float arr[4]);
    Quat(float q0, Vector3D im);
    Quat(etl::array<float, 4> arr);
    virtual ~Quat()
    {
    }

    /** Rotation sequences:
     *  In intrinsic convention (default), it rotates around a new base intermediate base every time
     *  XYZ means first rotate around X0 then Y1 then Z2.
     *  where B0 is the world (static base), B1 the first relative base and B2 the final base.
     *
     * In extrinsic convention, the rotation is always done with respect to the static base B0.
     * /!\ The rotation order is inversed /!\
     * XYZ means first rotation around Z0, then around Y0 and finaly around X0.
     **/
    enum Sequence
    {
        XYX,
        XYZ,
        XZX,
        XZY,
        YXY,
        YXZ,
        YZX,
        YZY,
        ZXY,
        ZXZ,
        ZYX,
        ZYZ
    };

    /** Tests **/
    bool isEqual(Quat const &) const;
    bool isNull() const;

    /** Operators **/
    Quat operator+(Quat const &) const;
    Quat operator-(Quat const &) const;
    Quat operator-() const;
    Quat operator*(Quat const &) const;
    Quat operator/(Quat const &) const;
    Quat &operator+=(Quat const &);
    Quat &operator-=(Quat const &);
    Quat &operator*=(Quat const &);
    Quat &operator/=(Quat const &);
    bool operator==(Quat const &) const;
    bool operator!=(Quat const &) const;
    float &operator[](size_t index);
    float operator[](size_t index) const;
#ifdef YAQLE_USE_COUT
    friend std::ostream &operator<<(std::ostream &, Quat const &);
#endif

    /** Algebra **/
    Quat inverse() const;
    Quat conj() const;
    Vector3D im() const;
    float re() const;

    // Norms
    float norm() const;
    float norm2() const; // Norm squared

    Quat normalize() const;

    /** Rotations **/
    // If the quaternion is unitary then it describes a 3D rotation
    // getAngle return the angle of this 3D rotation.
    float getAngle() const;

    Vector3D rotate(Vector3D const &) const;
    Vector3D rotate(float[3]) const;

    /** Conversions **/
    // Quat fromEuler(float ypr[3]);
    Vector3D toEuler(Sequence seq = ZYX, bool degree = false,
                     bool isExtrinsic = false) const; // YPR as default sequence

    /** Display **/
#ifdef YAQLE_USE_COUT
    void print() const;
    void writeToFile(std::ofstream &) const;
#endif

  private:
    float m_arr[4];
    Vector3D m_im;
};

/** "Static" functions **/

inline bool isEqual(Quat const &q1, Quat const &q2)
{
    return q1.isEqual(q2);
}
inline bool isNull(Quat const &q)
{
    return q.isNull();
}
inline Quat inverse(Quat const &q)
{
    return q.inverse();
}
inline Quat conj(Quat const &q)
{
    return q.conj();
}
inline Vector3D im(Quat const &q)
{
    return q.im();
}
inline float re(Quat const &q)
{
    return q.re();
}
inline float norm(Quat const &q)
{
    return q.norm();
}
inline float norm2(Quat const &q)
{
    return q.norm2();
}
inline Quat normalize(Quat const &q)
{
    return q.normalize();
}
inline float getAngle(Quat const &q)
{
    return q.getAngle();
}

inline Vector3D rotate(Quat const &q, Vector3D const &v)
{
    return q.rotate(v);
}
inline Vector3D rotate(Quat const &q, float arr[3])
{
    return q.rotate(arr);
}

/* Conversion */
inline Vector3D toEuler(Quat const &q, Quat::Sequence seq = Quat::ZYX, bool degree = false, bool isExtrinsic = false)
{
    return q.toEuler(seq, degree, isExtrinsic);
}

inline float innerProd(Quat const &q1, Quat const &q2)
{
    float sum = 0.0f;
    for (size_t i = 0; i < 4; i++)
    {
        sum += q1[i] * q2[i];
    }
    return sum;
}

/** Tools **/
Quat getRotation(Vector3D const &v1, Vector3D const &v2);

// Unit quaternions
Quat unitQuat(float angle, Vector3D im, bool degree = false);
Quat unitQuat(float angle, float x, float y, float z, bool degree = false);

Quat fromEuler(etl::array<float, 3> euler, Quat::Sequence seq = Quat::ZYX, bool degree = false,
               bool isExtrinsic = false);
Quat fromEuler(float euler[3], Quat::Sequence seq = Quat::ZYX, bool degree = false, bool isExtrinsic = false);
Quat fromEuler(float alpha, float beta, float gamma, Quat::Sequence seq = Quat::ZYX, bool degree = false,
               bool isExtrinsic = false);
Quat fromEuler(Vector3D euler, Quat::Sequence seq = Quat::ZYX, bool degree = false, bool isExtrinsic = false);

Quat slerp(const Quat &q1, const Quat &q2, float t);
Quat lerp(const Quat &q1, const Quat &q2, float t);

template <typename T> Quat operator/(Quat const &q, T const &scalar)
{
    ETL_STATIC_ASSERT(etl::is_arithmetic<T>::value, "Not a scalar");
    // TODO: Add assertion on non null scalar

    float arr[4];
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = q[i] / scalar;
    }
    return Quat(arr);
}

template <typename T> Quat operator/(T const &scalar, Quat const &q)
{

    ETL_STATIC_ASSERT(etl::is_arithmetic<T>::value, "Not a scalar");
    // TODO: Add assertion on non null scalar

    float arr[4];
    Quat q_inv = q.inverse();
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = scalar * q_inv[i];
    }
    return Quat(arr);
}

template <typename T> Quat operator*(Quat const &q, T const &scalar)
{
    ETL_STATIC_ASSERT(etl::is_arithmetic<T>::value, "Not a scalar");
    // TODO: Add assertion on non null scalar
    float arr[4];
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = scalar * q[i];
    }
    return Quat(arr);
}

template <typename T> Quat operator*(T const &scalar, Quat const &q)
{
    return q * scalar;
}

} // namespace yaqle

#endif