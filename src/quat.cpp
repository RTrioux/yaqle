#include <iostream>
#include <cmath>
#include <array>
#include "quat.hpp"
using namespace std;
namespace yaql
{

Quat::Quat(double q0,double q1, double q2, double q3)
{
    m_arr[0] = q0;
    m_arr[1] = q1;
    m_arr[2] = q2;
    m_arr[3] = q3;

    m_im = Vector3D(q1,q2,q3);
}

Quat::Quat(double arr[4])
{
    for (size_t i = 0; i < 4; i++)
    {
        m_arr[i] = arr[i];
    }
    m_im = Vector3D(arr[1],arr[2],arr[3]);
}


Quat::Quat(double q0, Vector3D im)
{
    m_arr[0] = q0;
    for (size_t i = 0; i < 3; i++)
    {
        m_arr[i+1] = im[i];
    }
    m_im = im;
}

Quat::Quat(array<double,4> arr)
{
    for (size_t i = 0; i < 4; i++)
    {
        m_arr[i] = arr[i];
    }
    m_im = Vector3D(arr[1],arr[2],arr[3]);
}


/** Tests **/

bool Quat::isEqual(Quat const &q) const
{
    for (size_t i = 0; i < 4; i++)
    {
        if(m_arr[i] != q[i])
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
        if(m_arr[i] != 0)
        {
            return false;
        }
    }
    return true;
}


/** Operators **/

Quat Quat::operator+(Quat const & q) const
{
    double arr[4];
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = m_arr[i] + q[i];
    }
    return Quat(arr);
}

Quat Quat::operator-(Quat const & q) const
{
    double arr[4];
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = m_arr[i] - q[i];
    }
    return Quat(arr);
}

Quat Quat::operator-() const
{
    return Quat(-m_arr[0],-m_arr[1],-m_arr[2],-m_arr[3]);
}

Quat Quat::operator*(Quat const & q) const
{
    double q0 = m_arr[0] * q[0] - innerProd(m_im,q.m_im);
    Vector3D im = m_arr[0] * q.m_im + q[0] * m_im + crossProd(m_im,q.m_im);
    return Quat(q0, im);
}

Quat Quat::operator/(Quat const & q) const
{
    return (*this) * q.inverse();
}

Quat & Quat::operator+=(Quat const &q)
{
    *this = *this + q;
    return *this;
}

Quat & Quat::operator-=(Quat const & q)
{
    *this = *this - q;
    return *this;
}

Quat & Quat::operator*=(Quat const & q)
{
    *this = *this * q; 
    return *this;
}

Quat & Quat::operator/=(Quat const & q)
{
    *this = *this / q;
    return *this;
}

bool Quat::operator!=(Quat const & q) const
{
    return !this->isEqual(q);
}

bool Quat::operator==(Quat const & q) const
{
    return this->isEqual(q);

}

double & Quat::operator[](size_t index)
{
    return m_arr[index];
}

double Quat::operator[](size_t index) const 
{
    return m_arr[index];
}

// friend operator
ostream & operator << (ostream &out, Quat const &q)
{
    out << "(";
    for (size_t i = 0; i < 4; i++)
    {
        out<<q.m_arr[i];
        if(i<3)
        {
            out<<",";
        }
    }
    out<<")";
    return out;
}


/** Algebra **/

Quat Quat::inverse() const
{
    return 1.0/norm2()*conj();
}


Quat Quat::conj() const 
{
    double arr[4]={m_arr[0]};
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


double Quat::re() const
{
    return m_arr[0];
}


// Norms

double Quat::norm2() const
{
    double norm2=0;
    for (size_t i = 0; i < 4; i++)
    {
        norm2+= (*this)[i] * (*this)[i];
    }
    return norm2;
}


double Quat::norm() const
{
    return sqrt(norm2());
}


Quat Quat::normalize() const
{
    return (*this) / norm();
}



/** Rotations **/

double Quat::getAngle() const
{
    return 2*acos(m_arr[0]);
}



Vector3D Quat::rotate(Vector3D const & vec) const
{
    Quat imQuat(0,vec[0],vec[1],vec[2]);
    imQuat = (*this) * imQuat * (*this).inverse();
    return Vector3D(imQuat[1],imQuat[2],imQuat[3]);
}


Vector3D Quat::rotate(double arr[3]) const
{
    return this->rotate(Vector3D(arr[0],arr[1],arr[2]));
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

    static map<Sequence, string> seq2str = {{XYX,"XYX"},{XYZ,"XYZ"},{XZX,"XZX"},{XZY,"XZY"},
                                            {YXY,"YXY"},{YXZ,"YXZ"},{YZX,"YZX"},{YZY,"YZY"},
                                            {ZXY,"ZXY"},{ZXZ,"ZXZ"},{ZYX,"ZYX"},{ZYZ,"ZYZ"}};
    string strSeq = seq2str[seq];

    Quat Q = *this;
    // Check if the quaternion is a valid rotation
    if(norm2() > 1)
    {
        if(-1 <= Q[0] && Q[0] <= 1)
        {
            Q = unitQuat(2*acos(Q[0]),Q.im());
        }
        else
        {
            throw domain_error("This quaternion is not a valid rotation (q0 must satisifies -1 < q0 < 1)");
        }
    }

    double q0 = Q[0],
           q1 = Q[1],
           q2 = Q[2],
           q3 = Q[3];
    double a = 1 - 2*(q2*q2 + q3*q3),
           b =     2*(q1*q2 - q3*q0),
           c =     2*(q1*q3 + q2*q0),
           d =     2*(q1*q2 + q3*q0),
           e = 1 - 2*(q1*q1 + q3*q3),
           f =     2*(q2*q3 - q1*q0),
           g =     2*(q1*q3 - q2*q0),
           h =     2*(q2*q3 + q1*q0),
           i = 1 - 2*(q1*q1 + q2*q2);
    Vector3D euler;

    switch (seq)
    {
    case XYX:
        euler[0] = atan2(d,-g);
        euler[1] = acos(a);
        euler[2] = atan2(b,c);
    break;
    case XYZ:
        euler[0] = atan2(-f,i);
        euler[1] = asin(c);
        euler[2] = atan2(-b,a);
    break;
    case XZX:
        euler[0] = atan2(g,d);
        euler[1] = acos(a);
        euler[2] = atan2(c,-b);
    break;
    case XZY:
        euler[0] = atan2(h,e);
        euler[1] = -asin(b);
        euler[2] = atan2(c,a);
    break;
    case YXY:
        euler[0] = atan2(b,h);
        euler[1] = acos(e);
        euler[2] = atan2(d,-f);
    break;
    case YXZ:
        euler[0] = atan2(c,i);
        euler[1] = -asin(f);
        euler[2] = atan2(d,e);
    break;
    case YZX:
        euler[0] = atan2(-g,a);
        euler[1] = asin(d);
        euler[2] = atan2(-f,e);
    break;
    case YZY:
        euler[0] = atan2(h,-b);
        euler[1] = acos(e);
        euler[2] = atan2(f,d);
    break;
    case ZXY:
        euler[0] = atan2(-b,e);
        euler[1] = asin(h);
        euler[2] = atan2(-g,i);
    break;
    case ZXZ:
        euler[0] = atan2(c,-f);
        euler[1] = acos(i);
        euler[2] = atan2(g,h);
    break;
    case ZYX:
        euler[0] = atan2(d,a);
        euler[1] = -asin(g);
        euler[2] = atan2(h,i);
    break;
    case ZYZ:
        euler[0] = atan2(f,c);
        euler[1] = acos(i);
        euler[2] = atan2(h,-g);
    break;
    default:
        throw invalid_argument("Not a correct sequence");
    break;
    }


    /* Gimbal lock check */
    double eps = 1e-7;

    if (abs(euler[1]) < eps || abs(euler[1] - M_PI) < eps)
    {
        // Ensure that the third angle is 0 after swapping 0 & 2
        if(isExtrinsic)
        {
            if(strSeq[0] != 'Y')
            {
                euler[2] = acos(e);
            }
            else
            {
                euler[2] = acos(a);
            }
            euler[0] = 0;
        }
        else
        {
            if(strSeq[0] != 'Y')
            {
                euler[0] = acos(e);
            }
            else
            {
                euler[0] = acos(a);
            }
            euler[2] = 0;
        }
        cout<<"WARNING: Gimbal locked: Third angle has been set to 0" << endl;
    }

    if (isExtrinsic)
    {
        double temp = euler[0];
        euler[0] = euler[2];
        euler[2] = temp; 
    }

    if(degree)
    {
        for (size_t i = 0; i < 3; i++)
        {
            euler[i]*=180.0/M_PI;
        }
    }
    return euler;
}


/** Display **/

void Quat::print() const
{
    cout << "(";
    for (size_t i = 0; i < 4; i++)
    {
        cout<<m_arr[i];
        if(i<3)
        {
            cout<<",";
        }
    }
    cout<<")"<<endl;
}

/** Tools **/

Quat getRotation(Vector3D const & v1, Vector3D const & v2)
{
    double theta = acos( innerProd(v1, v2) / (norm(v1) * norm(v2)) );
    Vector3D im = crossProd(v1, v2);
    return unitQuat(theta, im);
}

// Unit quaternions
Quat unitQuat(double angle, Vector3D im, bool degree)
{
    if(angle == 0)
    {
        return Quat(1,0,0,0);
    }
    else
    {
        if(degree)
        {
            angle *= 180.0 / M_PI;
        }

        double q0 = cos(angle/2);
        // Change direction in case of negative angle
        if(angle<0)
        {
            im = -im;
        }
        double lambda = sqrt((1 - q0*q0)/im.norm2());
        return Quat(q0,lambda*im);
    }
}

Quat unitQuat(double angle, double x, double y, double z, bool degree)
{
    return unitQuat(angle, Vector3D(x,y,z));
}


Quat fromEuler(std::array<double,3> euler, Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    /** 
     * https://handwiki.org/wiki/Rotation_formalisms_in_three_dimensions
    **/
    static std::map<Quat::Sequence, std::string> seq2str = {{Quat::XYX,"XYX"},{Quat::XYZ,"XYZ"},{Quat::XZX,"XZX"},{Quat::XZY,"XZY"},
                                                       {Quat::YXY,"YXY"},{Quat::YXZ,"YXZ"},{Quat::YZX,"YZX"},{Quat::YZY,"YZY"},
                                                       {Quat::ZXY,"ZXY"},{Quat::ZXZ,"ZXZ"},{Quat::ZYX,"ZYX"},{Quat::ZYZ,"ZYZ"}};

    std::string strSeq = seq2str[seq];

    if(degree)
    {
        for (size_t i = 0; i < 3; i++)
        {
            euler[i] *= M_PI/180.0;
        }
    }

    if(isExtrinsic)
    {
        double temp = euler[0];
        euler[0] = euler[2];
        euler[2] = temp;
    }

    Quat Q[3];
    Quat Qresult = Quat(1,0,0,0);
    int i=0;
    for(char c: strSeq)
    {
        switch (c)
        {
        case 'X':
            Q[i] = Quat(cos(euler[i]/2),sin(euler[i]/2),0,0);
        break;
        case 'Y':
            Q[i] = Quat(cos(euler[i]/2),0,sin(euler[i]/2),0);
        break;
        case 'Z':
            Q[i] = Quat(cos(euler[i]/2),0,0,sin(euler[i]/2));
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

Quat fromEuler(double euler[3], Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    std::array<double,3> arr = {euler[0],euler[1],euler[2]};
    return fromEuler(arr,seq, degree, isExtrinsic);
}

Quat fromEuler(double alpha, double beta, double gamma, Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    std::array<double,3> arr = {alpha, beta, gamma};
    return fromEuler(arr, seq, degree, isExtrinsic);
}

Quat fromEuler(Vector3D euler, Quat::Sequence seq, bool degree, bool isExtrinsic)
{
    std::array<double,3> arr = {euler[0],euler[1],euler[2]};
    return fromEuler(arr, seq, degree, isExtrinsic);
}




}