#include "dquat.hpp"
#include "quat.hpp"

namespace yaqle
{
DQuat::DQuat()
{
    m_hRe[0] = 0;
    m_hRe[1] = 0;
    m_hRe[2] = 0;
    m_hRe[3] = 0;

    m_hIm[0] = 0;
    m_hIm[1] = 0;
    m_hIm[2] = 0;
    m_hIm[3] = 0;
    memset(m_arr, 0, 8 * sizeof(float));
}

DQuat::DQuat(Quat hRe, Quat hIm)
{
    m_hRe = hRe;
    m_hIm = hIm;
    m_arr[0] = m_hRe[0];
    m_arr[1] = m_hRe[1];
    m_arr[2] = m_hRe[2];
    m_arr[3] = m_hRe[3];

    m_arr[4] = m_hIm[0];
    m_arr[5] = m_hIm[1];
    m_arr[6] = m_hIm[2];
    m_arr[7] = m_hIm[3];
}

DQuat::DQuat(float arr[8])
{
    m_hRe = Quat(arr[0], arr[1], arr[2], arr[3]);
    m_hIm = Quat(arr[4], arr[5], arr[6], arr[7]);
    for (size_t i = 0; i < 8; i++)
    {
        m_arr[i] = arr[i];
    }
}

DQuat::DQuat(float hRe[4], float hIm[4]) : DQuat(Quat(hRe), Quat(hIm))
{
}

DQuat::DQuat(etl::array<float, 4> hRe, etl::array<float, 4> hIm) : DQuat(Quat(hRe), Quat(hIm))
{
}

DQuat::DQuat(float w0, float x0, float y0, float z0, float w1, float x1, float y1, float z1)
    : DQuat(Quat(w0, x0, y0, z0), Quat(w1, x1, y1, z1))
{
}

/** Tests */
bool DQuat::isEqual(const DQuat &dq) const
{
    for (size_t i = 0; i < 8; i++)
    {
        if (m_arr[i] != dq[i])
        {
            return false;
        }
    }
    return true;
}

bool DQuat::isNull() const
{
    for (size_t i = 0; i < 8; i++)
    {
        if (m_arr[i] != 0)
        {
            return false;
        }
    }
    return true;
}

bool DQuat::isNormalized() const
{
    static const float eps = 1e-6;
    DQuat dq = (*this) * this->conj();
    bool cond1 = (dq.hRe().norm() - 1.0f) < eps ? true : false;

    if (!cond1)
        return false;

    bool cond2 = true;
    for (size_t i = 0; i < 4; i++)
    {
        if (dq.hIm()[i] > eps)
            return false;
    }
    return true;
}

DQuat slerp(const DQuat &dq0, const DQuat &dq1, float t)
{
    Quat Q0r = dq0.hRe();
    Quat Q1r = dq1.hRe();
    Quat Q0d = dq0.hIm();
    Quat Q1d = dq1.hIm();

    // Get initial position
    Vector3D t1 = dq0.pos();
    Vector3D t2 = dq1.pos();

    // Interpolate the translation
    Vector3D tVec = (1 - t) * t1 + t * t2;
    Quat tQuat = Quat(0, tVec);
    // Interpolate the rotation
    Quat qr = slerp(Q0r, Q1r, t);
    // Insert the position in the dual quaternion
    Quat qd = 0.5f * tQuat * qr;

    return DQuat(qr, qd).normalize();
}

/** Operators **/
DQuat DQuat::operator*(const DQuat &dq) const
{
    // (A + e.B)*(C + e.D) = A.C + e.(B.C + A.D)
    const Quat A = m_hRe;
    const Quat B = m_hIm;
    const Quat C = dq.hRe();
    const Quat D = dq.hIm();
    return DQuat(A * C, B * C + A * D);
}

DQuat DQuat::operator+(const DQuat &dq) const
{
    // (A + e.B) + (C + e.D) = A + C + e.(B + D)
    const Quat A = m_hRe;
    const Quat B = m_hIm;
    const Quat C = dq.hRe();
    const Quat D = dq.hIm();
    return DQuat(A + C, B + D);
}

DQuat DQuat::operator-(const DQuat &dq) const
{
    // (A + e.B) + (C + e.D) = A - C + e.(B - D)
    const Quat A = m_hRe;
    const Quat B = m_hIm;
    const Quat C = dq.hRe();
    const Quat D = dq.hIm();
    return DQuat(A - C, B - D);
}

DQuat DQuat::operator-() const
{
    float arr[8];
    for (size_t i = 0; i < 8; i++)
    {
        arr[i] = -m_arr[i];
    }
    return DQuat(arr);
}

bool DQuat::operator==(Quat const &dq) const
{
    return !((*this) != dq);
}

bool DQuat::operator!=(Quat const &dq) const
{
    for (size_t i = 0; i < 8; i++)
    {
        if (m_arr[i] != dq[i])
        {
            return true;
        }
    }
    return false;
}

float &DQuat::operator[](size_t index)
{
    return m_arr[index];
}

float DQuat::operator[](size_t index) const
{
    return m_arr[index];
}

DQuat DQuat::operator/(DQuat const &dq) const
{
    // dq1 / dq2 = (Q1_re + e.Q1_im) /(Q2_re + e.Q2_im) = (Q1_re * Q2_re) / Q2_re² + e.(Q2_re * Q1_im - Q1_re * Q2_im) /
    // Q2_re²
    Quat Q1_re = m_hRe;
    Quat Q1_im = m_hIm;
    Quat Q2_re = dq.hRe();
    Quat Q2_im = dq.hIm();
    return DQuat((Q1_re * Q2_re) / (Q2_re * Q2_re), (Q2_re * Q1_im - Q1_re * Q2_im) / (Q2_re * Q2_re));
}

#ifdef YAQLE_USE_COUT
// friend operator
std::ostream &operator<<(std::ostream &out, DQuat const &dq)
{
    out << "[(";
    for (size_t i = 0; i < 4; i++)
    {
        out << dq.m_arr[i];
        if (i < 3)
        {
            out << ",";
        }
    }
    out << "),";

    out << "(";
    for (size_t i = 4; i < 8; i++)
    {
        out << dq.m_arr[i];
        if (i < 7)
        {
            out << ",";
        }
    }
    out << ")]";

    return out;
}
#endif

/** Algebra **/

Quat DQuat::hRe() const
{
    return m_hRe;
}

Quat DQuat::hIm() const
{
    return m_hIm;
}

DQuat DQuat::norm() const
{
    Quat A = this->hRe();
    Quat B = this->hIm();

    float AB = innerProd(A, B);

    float A_norm = A.norm();
    Quat C = A_norm;
    Quat D = AB / A_norm;
    return DQuat(C, D);
}

DQuat DQuat::norm2() const
{
    // Q = A + e.B
    // Norm^2 = Q.Q* = C + e.D = |A|² + e.(2.A.B)
    // To gain performances, we can simplify the computation
    Quat A = this->hRe(); // Implicit construction
    Quat B = this->hIm();
    float AB = innerProd(A, B);
    Quat D = 2 * AB;
    Quat C = A.norm2();
    return DQuat(C, D);
    // return (*this) * (this->conj());
}

/**
 * @brief  Return the conjugate with respect to quaternion
 * (A + e.B) => (A* + e.B*)
 *
 * @return DQuat
 */
DQuat DQuat::conj() const
{
    return DQuat(m_hRe.conj(), m_hIm.conj());
}

/**
 * @brief Return the dual conjugate
 * (A + e.B) => (A - e.B)
 *
 * @return DQuat
 */
DQuat DQuat::dconj() const
{
    return DQuat(m_hRe, -m_hIm);
}

/**
 * @brief Return a mixed form of conjugate
 * (A + e.B) => (A* - e.B*)
 *
 * @return DQuat
 */
DQuat DQuat::mconj() const
{
    return DQuat(m_hRe.conj(), -m_hIm.conj());
}

/** Tools **/
DQuat rigidTransform(Quat Qr, Vector3D pos)
{
    Quat t(0, pos);
    return DQuat(Qr, 0.5f * t * Qr).normalize();
}

DQuat rigidTransform(Quat Qr, float x, float y, float z)
{
    Quat t(0, x, y, z);
    return DQuat(Qr, 0.5f * t * Qr).normalize();
}

DQuat DQuat::inverse() const
{
    Quat Q_re = m_hRe / m_hRe.norm2();
    Quat Q_im = -m_hRe * m_hIm / m_hRe.norm2();
    return DQuat(Q_re, Q_im);
}

DQuat DQuat::normalize() const // Return normalized dual
{
    // TODO: Check that A is not null
    Quat A = this->hRe();
    Quat B = this->hIm();

    float A_norm = A.norm();
    float A_norm2 = A_norm * A_norm;

    float AB = innerProd(A, B);

    Quat C = A / A_norm;
    Quat D = B - AB * A / A_norm2;
    return DQuat(C, D);
}

Vector3D DQuat::pos() const
{
    Quat q = 2 * hIm() * (hRe().conj());
    return Vector3D(q[1], q[2], q[3]);
}

#ifdef YAQLE_USE_COUT
void DQuat::print() const
{
    std::cout << "(" << std::endl;
    for (size_t i = 0; i < 4; i++)
    {
        std::cout << m_arr[i];
        if (i < 3)
        {
            std::cout << ", ";
        }
    }
    std::cout << ")"
              << "\t";

    std::cout << "(" << std::endl;
    for (size_t i = 4; i < 8; i++)
    {
        std::cout << m_arr[i];
        if (i < 7)
        {
            std::cout << ", ";
        }
    }
    std::cout << ")" << std::endl;
}

void DQuat::writeToFile(std::ofstream &file) const
{
    for (size_t i = 0; i < 8; i++)
    {
        char separator = ',';
        if (i < 7)
        {
            file << m_arr[i] << separator;
        }
        else
        {
            file << m_arr[i];
        }
    }
    file << std::endl;
}
#endif

} // namespace yaqle
