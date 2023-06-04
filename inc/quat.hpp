#ifndef QUAT_HPP
#define QUAT_HPP
#include <iostream>
#include <array>
#include <string>
#include <map>
#include <cmath>
#include "vector3d.hpp"

namespace yaql
{

class Quat
{
    public:
    Quat(double q0=0, double q1=0, double q2=0, double q3=0);
    Quat(double arr[4]);
    Quat(double q0,Vector3D im);
    Quat(std::array<double,4> arr);
    virtual ~Quat(){}

    /** Rotation sequences:
     *  In intrinsic convention (default), it rotates around a new base intermediate base every time
     *  XYZ means first rotate around X0 then Y1 then Z2.
     *  where B0 is the world (static base), B1 the first relative base and B2 the final base.
     * 
     * In extrinsic convention, the rotation is always done with respect to the static base B0.
     * /!\ The rotation order is inversed /!\
     * XYZ means first rotation around Z0, then around Y0 and finaly around X0. 
    **/
    enum Sequence {XYX, XYZ, XZX, XZY, YXY, YXZ, YZX, YZY, ZXY, ZXZ, ZYX, ZYZ};

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
    double & operator[](size_t index);
    double operator[](size_t index)const;
    friend std::ostream & operator<<(std::ostream &, Quat const &);

    
    /** Algebra **/
    Quat inverse() const;
    Quat conj() const;
    Vector3D im() const;
    double re() const;

    // Norms
    double norm() const;
    double norm2() const; // Norm squared

    Quat normalize() const;

    /** Rotations **/
    // If the quaternion is unitary then it describes a 3D rotation
    // getAngle return the angle of this 3D rotation.
    double getAngle() const;

    Vector3D rotate(Vector3D const &) const;
    Vector3D rotate(double [3]) const;

    /** Conversions **/
    //Quat fromEuler(double ypr[3]);
    Vector3D toEuler(Sequence seq = ZYX, bool degree = false, bool isExtrinsic = false) const; // YPR as default sequence

    /** Display **/
    void print() const;
    
    private:
    double m_arr[4];
    Vector3D m_im;
};

/** "Static" functions **/

inline bool isEqual(Quat const & q1, Quat const & q2){ return q1.isEqual(q2); }
inline bool isNull(Quat const & q){ return q.isNull(); }
inline Quat inverse(Quat const & q){ return q.inverse(); }
inline Quat conj(Quat const & q){ return q.conj(); }
inline Vector3D im(Quat const & q){ return q.im(); }
inline double re(Quat const & q){ return q.re(); }
inline double norm(Quat const & q){ return q.norm(); }
inline double norm2(Quat const & q){ return q.norm2(); }
inline Quat normalize(Quat const & q){ return q.normalize(); }
inline double getAngle(Quat const & q){ return q.getAngle(); }


inline Vector3D rotate(Quat const & q, Vector3D const & v){ return q.rotate(v); }
inline Vector3D rotate(Quat const & q, double arr[3]){ return q.rotate(arr); }

/* Conversion */
inline Vector3D toEuler(Quat const & q, Quat::Sequence seq = Quat::ZYX, bool degree = false, bool isExtrinsic = false)
{ return q.toEuler(seq, degree, isExtrinsic); }


/** Tools **/
Quat getRotation(Vector3D const & v1, Vector3D const & v2);

// Unit quaternions
Quat unitQuat(double angle, Vector3D im, bool degree = false);
Quat unitQuat(double angle, double x, double y, double z, bool degree = false);

Quat fromEuler(std::array<double,3> euler, Quat::Sequence seq = Quat::ZYX, bool degree = false, bool isExtrinsic = false);
Quat fromEuler(double euler[3], Quat::Sequence seq = Quat::ZYX, bool degree = false, bool isExtrinsic = false);
Quat fromEuler(double alpha, double beta, double gamma, Quat::Sequence seq = Quat::ZYX, bool degree = false, bool isExtrinsic = false);
Quat fromEuler(Vector3D euler, Quat::Sequence seq = Quat::ZYX, bool degree = false, bool isExtrinsic = false);



template<typename T> Quat operator/ (Quat const & q, T const & scalar)
{

    if(std::is_arithmetic<T>::value)
    {
        double arr[4];
        for (size_t i = 0; i < 4; i++)
        {
            arr[i] = q[i]/scalar;
        }
        return Quat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}

template<typename T> Quat operator/ (T const & scalar, Quat const & q)
{

    if(std::is_arithmetic<T>::value)
    {
        double arr[4];
        Quat q_inv = q.inverse();
        for (size_t i = 0; i < 4; i++)
        {
            arr[i] = scalar * q_inv[i];
        }
        return Quat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}



template<typename T> Quat operator* (Quat const & q, T const & scalar)
{
    if(std::is_arithmetic<T>::value)
    {
        double arr[4];
        for (size_t i = 0; i < 4; i++)
        {
            arr[i] = scalar * q[i];
        }
        return Quat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}

template<typename T> Quat operator* (T const & scalar, Quat const & q)
{
    return q*scalar;
}

}

#endif