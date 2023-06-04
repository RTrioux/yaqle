#include <math.h>
#include "vector3d.hpp"

using namespace std;

namespace yaql
{

Vector3D::Vector3D(array<double,3> arr)
{
    for (size_t i = 0; i < 3; i++)
    {
        m_arr[i] = arr[i];
    }
}

Vector3D::Vector3D(double x,double y,double z)
{
    m_arr[0]=x;
    m_arr[1]=y;
    m_arr[2]=z;
}


Vector3D::Vector3D(double arr[3])
{
    for (size_t i = 0; i < 3; i++)
    {
        m_arr[i]=arr[i];
    }
}

/** Tests **/

bool Vector3D::isNull() const
{
    for (size_t i = 0; i < 3; i++)
    {
        if(m_arr[i] != 0)
        {
            return false;
        }
    }
    return true;
}


bool Vector3D::isEqual(Vector3D const & vect) const
{
    for (size_t i = 0; i < 3; i++)
    {
        if(this->m_arr[i] != vect[i])
        {
            return false;
        }
    }
    return true;
}



/** Operators **/
Vector3D Vector3D::operator+(Vector3D const & vect) const
{
    double arr[3];
    for (size_t i = 0; i < 3; i++)
    {
        arr[i] = m_arr[i] + vect.m_arr[i];
    }
    return Vector3D(arr);
}

Vector3D Vector3D::operator-(Vector3D const &vect) const
{
    double arr[3];
    for (size_t i = 0; i < 3; i++)
    {
        arr[i] = m_arr[i] - vect.m_arr[i];
    }
    return Vector3D(arr);
}

Vector3D Vector3D::operator-() const
{
    double arr[3];
    for (size_t i = 0; i < 3; i++)
    {
        arr[i] = -m_arr[i];
    }
    return Vector3D(arr);
}


Vector3D & Vector3D::operator+=(Vector3D const & A)
{
    *this = *this + A;
    return *this;
}

Vector3D & Vector3D::operator-=(Vector3D const & A)
{
    *this = *this - A;
    return *this;
}

double & Vector3D::operator[](size_t index)
{
    return m_arr[index];
}

const double & Vector3D::operator[](size_t index) const
{
    return m_arr[index];
}

bool Vector3D::operator==(Vector3D const & vec) const
{
    return this->isEqual(vec);
}

bool Vector3D::operator!=(Vector3D const & vec) const
{
    return !(this->isEqual(vec));
}

ostream & operator<< (ostream & out, Vector3D const &vec)
{
    out << "(";
    for (size_t i = 0; i < 3; i++)
    {
        out << vec.m_arr[i];
        if(i<2)
        {
            out << ",";
        }
    }
    out<<")";
    return out;
}

/** Algebra **/
Vector3D Vector3D::crossProd(Vector3D const & B) const
{
    double arr[3];
    for (size_t i = 0; i < 3; i++)
    {
        arr[(i+2)%3] = this->m_arr[i] * B.m_arr[(i+1)%3] - this->m_arr[(i+1)%3] * B.m_arr[i];
    }
    return Vector3D(arr);
}

double Vector3D::innerProd(Vector3D const & B) const
{
    double innerProd=0;
    for (size_t i = 0; i < 3; i++)
    {
        innerProd += this->m_arr[i] * B.m_arr[i];
    }
    return innerProd;
}

double Vector3D::getAngle(Vector3D const & B) const
{
    return acos(this->innerProd(B)/(norm() * B.norm()));
}

Vector3D Vector3D::normalize() const
{
    return (*this)/norm();
}

double Vector3D::norm() const
{
    return sqrt(norm2());
}

double Vector3D::norm2() const
{
    return this->innerProd(*this);
}


/** Display **/

void Vector3D::print() const
{
    cout << "(";
    for (size_t i = 0; i < 3; i++)
    {
        cout<<m_arr[i];
        if(i<2)
        {
            cout<<",";
        }
    }
    cout<<")"<<endl;
}

}