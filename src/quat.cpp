#include "quat.hpp"
#include <cmath>
#include <etl/array.h>
#include <etl/map.h>
#include <etl/string.h>
#include <iostream>

using namespace etl;

// TODO: Find a better way to handle errors

namespace yaqle
{

Quat::Quat(float q0, float q1, float q2, float q3)
{
    m_arr[0] = q0;
    m_arr[1] = q1;
    m_arr[2] = q2;
    m_arr[3] = q3;

    m_im = Vector3D(q1, q2, q3);
}

Quat::Quat(float arr[4])
{
    for (size_t i = 0; i < 4; i++)
    {
        m_arr[i] = arr[i];
    }
    m_im = Vector3D(arr[1], arr[2], arr[3]);
}

Quat::Quat(float q0, Vector3D im)
{
    m_arr[0] = q0;
    for (size_t i = 0; i < 3; i++)
    {
        m_arr[i + 1] = im[i];
    }
    m_im = im;
}

Quat::Quat(array<float, 4> arr)
{
    for (size_t i = 0; i < 4; i++)
    {
        m_arr[i] = arr[i];
    }
    m_im = Vector3D(arr[1], arr[2], arr[3]);
}

/** Tests **/

bool Quat::isEqual(Quat const &q) const
{
    for (size_t i = 0; i < 4; i++)
    {
        if (m_arr[i] != q[i])
        {
            return false;
        }
    }
    return true;
}

bool Quat::isNull() const
{
    for (size_t i = 0; i < 4; i++)
    {
        if (m_arr[i] != 0)
        {
            return false;
        }
    }
    return true;
}

/** Operators **/

Quat Quat::operator+(Quat const &q) const
{
    float arr[4];
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = m_arr[i] + q[i];
    }
    return Quat(arr);
}

Quat Quat::operator-(Quat const &q) const
{
    float arr[4];
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = m_arr[i] - q[i];
    }
    return Quat(arr);
}

Quat Quat::operator-() const
{
    return Quat(-m_arr[0], -m_arr[1], -m_arr[2], -m_arr[3]);
}

Quat Quat::operator*(Quat const &q) const
{
    float q0 = m_arr[0] * q[0] - innerProd(m_im, q.m_im);
    Vector3D im = m_arr[0] * q.m_im + q[0] * m_im + crossProd(m_im, q.m_im);
    return Quat(q0, im);
}

Quat Quat::operator/(Quat const &q) const
{
    return (*this) * q.inverse();
}

Quat &Quat::operator+=(Quat const &q)
{
    *this = *this + q;
    return *this;
}

Quat &Quat::operator-=(Quat const &q)
{
    *this = *this - q;
    return *this;
}

Quat &Quat::operator*=(Quat const &q)
{
    *this = *this * q;
    return *this;
}

Quat &Quat::operator/=(Quat const &q)
{
    *this = *this / q;
    return *this;
}

bool Quat::operator!=(Quat const &q) const
{
    return !this->isEqual(q);
}

bool Quat::operator==(Quat const &q) const
{
    return this->isEqual(q);
}

float &Quat::operator[](size_t index)
{
    return m_arr[index];
}

float Quat::operator[](size_t index) const
{
    return m_arr[index];
}

#ifdef YAQLE_USE_COUT
// friend operator
std::ostream &operator<<(std::ostream &out, Quat const &q)
{
    out << "(";
    for (size_t i = 0; i < 4; i++)
    {
        out << q.m_arr[i];
        if (i < 3)
        {
            out << ",";
        }
    }
    out << ")";
    return out;
}
#endif

/** Algebra **/

Quat Quat::inverse() const
{
    return 1.0 / norm2() * conj();
}

Quat Quat::conj() const
{
    float arr[4] = {m_arr[0]};
    for (size_t i = 1; i < 4; i++)
    {
        arr[i] = -m_arr[i];
    }
    return Quat(arr);
}

Vector3D Quat::im() const
{
    return m_im;
}

float Quat::re() const
{
    return m_arr[0];
}

// Norms

float Quat::norm2() const
{
    float norm2 = 0;
    for (size_t i = 0; i < 4; i++)
    {
        norm2 += (*this)[i] * (*this)[i];
    }
    return norm2;
}

float Quat::norm() const
{
    return sqrtf(norm2());
}

Quat Quat::normalize() const
{
    return (*this) / norm();
}

/** Rotations **/

float Quat::getAngle() const
{
    return 2 * acosf(m_arr[0]);
}

Vector3D Quat::rotate(Vector3D const &vec) const
{
    Quat imQuat(0, vec[0], vec[1], vec[2]);
    imQuat = (*this) * imQuat * (*this).inverse();
    return Vector3D(imQuat[1], imQuat[2], imQuat[3]);
}

Vector3D Quat::rotate(float arr[3]) const
{
    return this->rotate(Vector3D(arr[0], arr[1], arr[2]));
}

/** Conversions **/

Vector3D Quat::toEuler(Sequence seq, bool degree, bool isExtrinsic) const
{
    /**
     * Compute the coefficients (a,b,c,...i) of the rotation matrix (R_q) associated to an
     * arbitrary unitary quaternion (q0,q1,q2,q3).
     * Compute the rotation matrix R_xxx for Euler angles (alpha, beta, gamma).
     * Solve alpha, beta, gamma such that R_xxx = R_q for the selected sequence xxx.
     *
     * The 12 equations are solved offline thanks to wxmaxima and hardcoded here.
     *
     * IMPORTANT: The intrinsic convention is used to compute the rotation matrices
     *            (See: https://www.wikiwand.com/en/Davenport_chained_rotations#)
     * IMPORTANT: This function assume that the quaternion is already normalized.
     *            Therefore, we don't force the normalization in order to gain performance.
     **/

    static map<Sequence, string<4>, 12> seq2str = {{XYX, "XYX"}, {XYZ, "XYZ"}, {XZX, "XZX"}, {XZY, "XZY"},
                                                   {YXY, "YXY"}, {YXZ, "YXZ"}, {YZX, "YZX"}, {YZY, "YZY"},
                                                   {ZXY, "ZXY"}, {ZXZ, "ZXZ"}, {ZYX, "ZYX"}, {ZYZ, "ZYZ"}};
    string<4> strSeq = seq2str[seq];

    Quat Q = *this;
    // Check if the quaternion is a valid rotation
    if (norm2() > 1)
    {
        if (-1 <= Q[0] && Q[0] <= 1)
        {
            Q = unitQuat(2 * acosf(Q[0]), Q.im());
        }
        else
        {
            throw std::domain_error("Q[0] must range in [-1,1]");
        }
    }

    float q0 = Q[0], q1 = Q[1], q2 = Q[2], q3 = Q[3];
    float a = 1 - 2 * (q2 * q2 + q3 * q3), b = 2 * (q1 * q2 - q3 * q0), c = 2 * (q1 * q3 + q2 * q0),
          d = 2 * (q1 * q2 + q3 * q0), e = 1 - 2 * (q1 * q1 + q3 * q3), f = 2 * (q2 * q3 - q1 * q0),
          g = 2 * (q1 * q3 - q2 * q0), h = 2 * (q2 * q3 + q1 * q0), i = 1 - 2 * (q1 * q1 + q2 * q2);
    Vector3D euler;

    switch (seq)
    {
    case XYX:
        euler[0] = atan2f(d, -g);
        euler[1] = acosf(a);
        euler[2] = atan2f(b, c);
        break;
    case XYZ:
        euler[0] = atan2f(-f, i);
        euler[1] = asinf(c);
        euler[2] = atan2f(-b, a);
        break;
    case XZX:
        euler[0] = atan2f(g, d);
        euler[1] = acosf(a);
        euler[2] = atan2f(c, -b);
        break;
    case XZY:
        euler[0] = atan2f(h, e);
        euler[1] = -asinf(b);
        euler[2] = atan2f(c, a);
        break;
    case YXY:
        euler[0] = atan2f(b, h);
        euler[1] = acosf(e);
        euler[2] = atan2f(d, -f);
        break;
    case YXZ:
        euler[0] = atan2f(c, i);
        euler[1] = -asinf(f);
        euler[2] = atan2f(d, e);
        break;
    case YZX:
        euler[0] = atan2f(-g, a);
        euler[1] = asinf(d);
        euler[2] = atan2f(-f, e);
        break;
    case YZY:
        euler[0] = atan2f(h, -b);
        euler[1] = acosf(e);
        euler[2] = atan2f(f, d);
        break;
    case ZXY:
        euler[0] = atan2f(-b, e);
        euler[1] = asinf(h);
        euler[2] = atan2f(-g, i);
        break;
    case ZXZ:
        euler[0] = atan2f(c, -f);
        euler[1] = acosf(i);
        euler[2] = atan2f(g, h);
        break;
    case ZYX:
        euler[0] = atan2f(d, a);
        euler[1] = -asinf(g);
        euler[2] = atan2f(h, i);
        break;
    case ZYZ:
        euler[0] = atan2f(f, c);
        euler[1] = acosf(i);
        euler[2] = atan2f(h, -g);
        break;
    default:
        throw std::invalid_argument("Not a correct sequence");
        break;
    }

    /* Gimbal lock check */
    float eps = 1e-7;

    if (abs(euler[1]) < eps || abs(euler[1] - M_PI) < eps)
    {
        // Ensure that the third angle is 0 after swapping 0 & 2
        if (isExtrinsic)
        {
            if (strSeq[0] != 'Y')
            {
                euler[2] = acosf(e);
            }
            else
            {
                euler[2] = acosf(a);
            }
            euler[0] = 0;
        }
        else
        {
            if (strSeq[0] != 'Y')
            {
                euler[0] = acosf(e);
            }
            else
            {
                euler[0] = acosf(a);
            }
            euler[2] = 0;
        }
        std::cout << "WARNING: Gimbal locked: Third angle has been set to 0" << std::endl;
    }

    if (isExtrinsic)
    {
        float temp = euler[0];
        euler[0] = euler[2];
        euler[2] = temp;
    }

    if (degree)
    {
        for (size_t i = 0; i < 3; i++)
        {
            euler[i] *= 180.0 / M_PI;
        }
    }
    return euler;
}

/** Display **/

#ifdef YAQLE_USE_COUT
void Quat::print() const
{
    std::cout << "(";
    for (size_t i = 0; i < 4; i++)
    {
        std::cout << m_arr[i];
        if (i < 3)
        {
            std::cout << ",";
        }
    }
    std::cout << ")" << std::endl;
}

void Quat::writeToFile(std::ofstream &file) const
{
    file << m_arr[0] << ',' << m_arr[1] << ',' << m_arr[2] << ',' << m_arr[3] << std::endl;
}

#endif

/** Tools **/

Quat getRotation(Vector3D const &v1, Vector3D const &v2)
{
    float theta = acosf(innerProd(v1, v2) / (norm(v1) * norm(v2)));
    Vector3D im = crossProd(v1, v2);
    return unitQuat(theta, im);
}

// Unit quaternions
Quat unitQuat(float angle, Vector3D im, bool degree)
{
    if (angle == 0)
    {
        return Quat(1, 0, 0, 0);
    }
    else
    {
        if (degree)
        {
            angle *= M_PI / 180.0f;
        }

        float q0 = cosf(angle / 2);
        // Change direction in case of negative angle
        if (angle < 0)
        {
            im = -im;
        }
        float lambda = sqrtf((1 - q0 * q0) / im.norm2());
        return Quat(q0, lambda * im);
    }
}

Quat unitQuat(float angle, float x, float y, float z, bool degree)
{
    angle = degree == true ? angle * M_PI / 180.0f : angle;
    return unitQuat(angle, Vector3D(x, y, z));
}

Quat fromEuler(etl::array<float, 3> euler, Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    /**
     * https://handwiki.org/wiki/Rotation_formalisms_in_three_dimensions
     **/
    static map<Quat::Sequence, string<4>, 12> seq2str = {{Quat::XYX, "XYX"}, {Quat::XYZ, "XYZ"}, {Quat::XZX, "XZX"},
                                                         {Quat::XZY, "XZY"}, {Quat::YXY, "YXY"}, {Quat::YXZ, "YXZ"},
                                                         {Quat::YZX, "YZX"}, {Quat::YZY, "YZY"}, {Quat::ZXY, "ZXY"},
                                                         {Quat::ZXZ, "ZXZ"}, {Quat::ZYX, "ZYX"}, {Quat::ZYZ, "ZYZ"}};

    string<4> strSeq = seq2str[seq];

    if (degree)
    {
        for (size_t i = 0; i < 3; i++)
        {
            euler[i] *= M_PI / 180.0;
        }
    }

    if (isExtrinsic)
    {
        float temp = euler[0];
        euler[0] = euler[2];
        euler[2] = temp;
    }

    Quat Q[3];
    Quat Qresult = Quat(1, 0, 0, 0);
    int i = 0;
    for (char c : strSeq)
    {
        switch (c)
        {
        case 'X':
            Q[i] = Quat(cosf(euler[i] / 2), sinf(euler[i] / 2), 0, 0);
            break;
        case 'Y':
            Q[i] = Quat(cosf(euler[i] / 2), 0, sinf(euler[i] / 2), 0);
            break;
        case 'Z':
            Q[i] = Quat(cosf(euler[i] / 2), 0, 0, sinf(euler[i] / 2));
            break;
        }
        /*
        if(isExtrinsic)
        {
            Qresult = Q[i] * Qresult;
        }
        else
        {
            Qresult = Qresult * Q[i];
        }
        */
        Qresult = Qresult * Q[i];
        i++;
    }

    return Qresult;
}

Quat fromEuler(float euler[3], Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    etl::array<float, 3> arr = {euler[0], euler[1], euler[2]};
    return fromEuler(arr, seq, degree, isExtrinsic);
}

Quat fromEuler(float alpha, float beta, float gamma, Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    etl::array<float, 3> arr = {alpha, beta, gamma};
    return fromEuler(arr, seq, degree, isExtrinsic);
}

Quat fromEuler(Vector3D euler, Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    etl::array<float, 3> arr = {euler[0], euler[1], euler[2]};
    return fromEuler(arr, seq, degree, isExtrinsic);
}

Quat slerp(const Quat &q1, const Quat &q2, float t)
{
    // Calculate the dot product of the two quaternions
    // float dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    float dot = innerProd(q1, q2);

    // Ensure the dot product is within [-1, 1] to avoid numerical issues
    // dot = fmaxf(fminf(dot, 1.0f), -1.0f);

    // Determine the direction of the interpolation
    float direction = (dot < 0) ? -1.0f : 1.0f;

    // Calculate the angle between the quaternions
    float angle = acosf(direction * dot);

    // Calculate the interpolation coefficients
    float sinInverse = 1.0f / sinf(angle);
    float coeff1 = sinf((1.0f - t) * angle) * sinInverse;
    float coeff2 = sinf(t * angle) * sinInverse * direction;

    // Perform the spherical linear interpolation
    /*
    float r[4];
    r[0] = coeff1 * q1[0] + coeff2 * q2[0];
    r[1] = coeff1 * q1[1] + coeff2 * q2[1];
    r[2] = coeff1 * q1[2] + coeff2 * q2[2];
    r[3] = coeff1 * q1[3] + coeff2 * q2[3];

    return Quat(r[0], r[1], r[2], r[3]);
    */
    Quat q = coeff1 * q1 + coeff2 * q2;
    return q;
}

Quat lerp(const Quat &q1, const Quat &q2, float t)
{
    Quat q = (1 - t) * q1 + q2 * t;
    return q.normalize();
}

} // namespace yaqle