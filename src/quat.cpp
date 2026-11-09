#include "quat.hpp"
#include <cmath>
#include <etl/array.h>
#include <etl/map.h>
#include <etl/string.h>
#include <iostream>

using namespace etl;

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
        if (fabs(m_arr[i]) > YAQLE_EPS)
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
    float q0 = m_arr[0] * q[0] - dot(m_im, q.m_im);
    Vector3D im = m_arr[0] * q.m_im + q[0] * m_im + cross(m_im, q.m_im);
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
    // TODO: Check that norm2 is not null
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
    // TODO: Check norm is not null
    return (*this) / norm();
}

/** Rotations **/

float Quat::angle() const
{
    // Check that q0 domain [-1, 1]
    return 2 * acosf(m_arr[0]);
}

Vector3D Quat::rotate(Vector3D const &vec) const
{
    YAQLE_QUAT_MAYBE_NORMALIZE(q);
    Quat imQuat(0, vec[0], vec[1], vec[2]);
    imQuat = q * imQuat * q.inverse();
    return Vector3D(imQuat[1], imQuat[2], imQuat[3]);
}

Vector3D Quat::rotate(float arr[3]) const
{
    YAQLE_QUAT_MAYBE_NORMALIZE(q);
    return q.rotate(Vector3D(arr[0], arr[1], arr[2]));
}

/** Conversions **/

namespace
{
// Axes of each Quat::Sequence, indexed by the enum value
const char *const SEQ_AXES[12] = {"XYX", "XYZ", "XZX", "XZY", "YXY", "YXZ", "YZX", "YZY", "ZXY", "ZXZ", "ZYX", "ZYZ"};

// Sequence with reversed axes (e.g. XYZ -> ZYX), indexed by the enum value
const Quat::Sequence SEQ_REVERSED[12] = {Quat::XYX, Quat::ZYX, Quat::XZX, Quat::YZX, Quat::YXY, Quat::ZXY,
                                         Quat::XZY, Quat::YZY, Quat::YXZ, Quat::ZXZ, Quat::XYZ, Quat::ZYZ};

float clampUnit(float x)
{
    return x > 1.0f ? 1.0f : (x < -1.0f ? -1.0f : x);
}
} // namespace

Vector3D Quat::toEuler(Sequence seq, bool degree, bool isExtrinsic) const
{
    /**
     * Compute the coefficients (a,b,c,...i) of the rotation matrix (R_q) associated to an
     * arbitrary unitary quaternion (q0,q1,q2,q3).
     * Compute the rotation matrix R_xxx for Euler angles (alpha, beta, gamma).
     * Solve alpha, beta, gamma such that R_xxx = R_q for the selected sequence xxx.
     *
     * The 12 equations are solved offline thanks to wxmaxima and hardcoded here.
     * Solution can also be found here :
     * https://ntrs.nasa.gov/api/citations/19770024290/downloads/19770024290.pdf
     *
     * The equations are solved in the intrinsic convention (R = R_1.R_2.R_3).
     * (See: https://www.wikiwand.com/en/Davenport_chained_rotations#)
     * An extrinsic sequence ABC with angles (a,b,c) is the intrinsic sequence CBA with angles (c,b,a)
     * so for extrinsic we solve the reversed sequence and then reverse the angles.
     **/

    if (isExtrinsic)
    {
        seq = SEQ_REVERSED[seq];
    }
    const char *strSeq = SEQ_AXES[seq];

    YAQLE_QUAT_MAYBE_NORMALIZE(Q);

    // Compute R_q coefficients the equivalent rotation matrix of the quaternion
    // q.v.q* for an arbitrary unitary quaternion and arbitrary vector v yields:

    float q0 = Q[0], q1 = Q[1], q2 = Q[2], q3 = Q[3];
    float a = 1 - 2 * (q2 * q2 + q3 * q3), b = 2 * (q1 * q2 - q3 * q0), c = 2 * (q1 * q3 + q2 * q0),
          d = 2 * (q1 * q2 + q3 * q0), e = 1 - 2 * (q1 * q1 + q3 * q3), f = 2 * (q2 * q3 - q1 * q0),
          g = 2 * (q1 * q3 - q2 * q0), h = 2 * (q2 * q3 + q1 * q0), i = 1 - 2 * (q1 * q1 + q2 * q2);
    const float R[3][3] = {{a, b, c}, {d, e, f}, {g, h, i}};
    Vector3D euler;
    // Matrix coefficient giving the middle angle (sin(beta) for Tait-Bryan, cos(beta) for proper Euler)
    float mid = 0;

    // By identification from R_q and R_xxx
    // You can easily find for each combinaison a solution for each angle
    // R_xxx = R_1.R_2.R_3 (This intrinsic convention)
    switch (seq)
    {
    case XYX:
        mid = clampUnit(a);
        euler[0] = atan2f(d, -g);
        euler[1] = acosf(mid);
        euler[2] = atan2f(b, c);
        break;
    case XYZ:
        mid = clampUnit(c);
        euler[0] = atan2f(-f, i);
        euler[1] = asinf(mid);
        euler[2] = atan2f(-b, a);
        break;
    case XZX:
        mid = clampUnit(a);
        euler[0] = atan2f(g, d);
        euler[1] = acosf(mid);
        euler[2] = atan2f(c, -b);
        break;
    case XZY:
        mid = clampUnit(-b);
        euler[0] = atan2f(h, e);
        euler[1] = asinf(mid);
        euler[2] = atan2f(c, a);
        break;
    case YXY:
        mid = clampUnit(e);
        euler[0] = atan2f(b, h);
        euler[1] = acosf(mid);
        euler[2] = atan2f(d, -f);
        break;
    case YXZ:
        mid = clampUnit(-f);
        euler[0] = atan2f(c, i);
        euler[1] = asinf(mid);
        euler[2] = atan2f(d, e);
        break;
    case YZX:
        mid = clampUnit(d);
        euler[0] = atan2f(-g, a);
        euler[1] = asinf(mid);
        euler[2] = atan2f(-f, e);
        break;
    case YZY:
        mid = clampUnit(e);
        euler[0] = atan2f(h, -b);
        euler[1] = acosf(mid);
        euler[2] = atan2f(f, d);
        break;
    case ZXY:
        mid = clampUnit(h);
        euler[0] = atan2f(-b, e);
        euler[1] = asinf(mid);
        euler[2] = atan2f(-g, i);
        break;
    case ZXZ:
        mid = clampUnit(i);
        euler[0] = atan2f(c, -f);
        euler[1] = acosf(mid);
        euler[2] = atan2f(g, h);
        break;
    case ZYX:
        mid = clampUnit(-g);
        euler[0] = atan2f(d, a);
        euler[1] = asinf(mid);
        euler[2] = atan2f(h, i);
        break;
    case ZYZ:
        mid = clampUnit(i);
        euler[0] = atan2f(f, c);
        euler[1] = acosf(mid);
        euler[2] = atan2f(h, -g);
        break;
    default:
        euler = Vector3D(0, 0, 0);
        break;
    }

    /* Gimbal lock check */

    // Tait-Bryan (e.g. XYZ): locked when |sin(beta)| = 1, proper Euler (e.g. XYX): locked when |cos(beta)| = 1
    if (fabsf(mid) > 1.0f - YAQLE_EPS)
    {
        // Only alpha + gamma (or alpha - gamma) is observable: the last returned angle is set to 0.
        // For extrinsic the returned angles are reversed so the last returned angle is the intrinsic alpha.
        const int ax1 = strSeq[0] - 'X', ax2 = strSeq[1] - 'X', ax3 = strSeq[2] - 'X';
        if (isExtrinsic)
        {
            // alpha = 0 so R = R_2(beta).R_3(gamma) and R^T.e_ax2 = R_3(-gamma).e_ax2
            const int k = 3 - ax2 - ax3;
            const float sign = ((ax2 - ax3 + 3) % 3 == 1) ? 1.0f : -1.0f;
            euler[0] = 0;
            euler[2] = atan2f(-sign * R[ax2][k], R[ax2][ax2]);
        }
        else
        {
            // gamma = 0 so R = R_1(alpha).R_2(beta) and R.e_ax2 = R_1(alpha).e_ax2
            const int k = 3 - ax1 - ax2;
            const float sign = ((ax2 - ax1 + 3) % 3 == 1) ? 1.0f : -1.0f;
            euler[0] = atan2f(sign * R[k][ax2], R[ax2][ax2]);
            euler[2] = 0;
        }
#ifdef YAQLE_USE_COUT
        std::cout << "WARNING: Gimbal locked: Third angle has been set to 0" << std::endl;
#endif
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

Matrix<3, 3> Quat::toRotationMatrix(Sequence, bool) const
{
    // The rotation matrix only depends on the rotation itself, not on the Euler sequence or convention
    YAQLE_QUAT_MAYBE_NORMALIZE(Q);

    float q0 = Q[0], q1 = Q[1], q2 = Q[2], q3 = Q[3];
    return Matrix<3, 3>{{1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q3 * q0), 2 * (q1 * q3 + q2 * q0)},
                        {2 * (q1 * q2 + q3 * q0), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q1 * q0)},
                        {2 * (q1 * q3 - q2 * q0), 2 * (q2 * q3 + q1 * q0), 1 - 2 * (q1 * q1 + q2 * q2)}};
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

void Quat::writeToFile(const char *id, std::ofstream &file) const
{
    // id is the name of the quaternion
    // :q: to specify it's a quaternion
    file << id << ":q:" << m_arr[0] << ',' << m_arr[1] << ',' << m_arr[2] << ',' << m_arr[3] << std::endl;
}

#endif

/** Tools **/

Quat getRotation(Vector3D const &v1, Vector3D const &v2)
{
    /**
     * Compute the quaternion representing the rotation from vector v1 to vector v2.
     * Both vectors must be non-null.
     * The rotation axis is given by the cross product of v1 and v2.
     * The rotation angle is given by the angle between v1 and v2.
     * The resulting quaternion is a unit quaternion.
     */
    float theta = acosf(dot(v1, v2) / (norm(v1) * norm(v2))); // Angle between the two vectors
    Vector3D im = cross(v1, v2);                              // Use cross product to get the rotation axis
    return fromAxisAngle(theta, im);
}

// Unit quaternions
Quat fromAxisAngle(float angle, Vector3D im, bool degree)
{
    // TODO: Check that im is not null
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

Quat fromAxisAngle(float angle, float x, float y, float z, bool degree)
{
    angle = degree == true ? angle * M_PI / 180.0f : angle;
    return fromAxisAngle(angle, Vector3D(x, y, z));
}

Quat uQ(float angle, Vector3D im, bool degree)
{
    return fromAxisAngle(angle, im, degree);
}

Quat uQ(float angle, float x, float y, float z, bool degree)
{
    return fromAxisAngle(angle, x, y, z, degree);
}

Quat fromEuler(etl::array<float, 3> euler, Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    /**
     * https://handwiki.org/wiki/Rotation_formalisms_in_three_dimensions
     * Intrinsic: Q = Q_1(alpha).Q_2(beta).Q_3(gamma)
     * Extrinsic sequence ABC with angles (a,b,c) is the intrinsic sequence CBA with angles (c,b,a)
     **/
    if (isExtrinsic)
    {
        seq = SEQ_REVERSED[seq];
        float temp = euler[0];
        euler[0] = euler[2];
        euler[2] = temp;
    }
    const char *strSeq = SEQ_AXES[seq];

    if (degree)
    {
        for (size_t i = 0; i < 3; i++)
        {
            euler[i] *= M_PI / 180.0;
        }
    }

    Quat Qresult = Quat(1, 0, 0, 0);
    for (int n = 0; n < 3; n++)
    {
        const float half = euler[n] / 2;
        switch (strSeq[n])
        {
        case 'X':
            Qresult = Qresult * Quat(cosf(half), sinf(half), 0, 0);
            break;
        case 'Y':
            Qresult = Qresult * Quat(cosf(half), 0, sinf(half), 0);
            break;
        case 'Z':
            Qresult = Qresult * Quat(cosf(half), 0, 0, sinf(half));
            break;
        }
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
    YAQLE_QUAT_ASSERT_NORMALIZED(q1);
    YAQLE_QUAT_ASSERT_NORMALIZED(q2);
    // Calculate the dot product of the two quaternions assumed normalized
    // float dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    float dot_q1_q2 = dot(q1, q2);

    // Ensure the dot product is within [-1, 1] to avoid numerical issues
    // dot = fmaxf(fminf(dot, 1.0f), -1.0f);

    // Determine the direction of the interpolation
    float direction = (dot_q1_q2 < 0) ? -1.0f : 1.0f;

    // Calculate the angle between the quaternions
    float angle = acosf(direction * dot_q1_q2);

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