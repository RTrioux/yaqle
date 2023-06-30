#ifndef VECTOR3D_HPP
#define VECTOR3D_HPP
#include <iostream>
#include <etl/array.h>
namespace yaqle
{
    class Vector3D
    {
    public:
        Vector3D(etl::array<float, 3> arr);
        Vector3D(float x = 0, float y = 0, float z = 0);
        Vector3D(float arr[3]);
        virtual ~Vector3D() {}

        /** Tests **/
        bool isNull() const;
        bool isEqual(Vector3D const &) const;

        /** Operators **/
        Vector3D operator+(Vector3D const &) const;
        Vector3D operator-(Vector3D const &) const;
        Vector3D operator-() const; // Negative vector
        Vector3D &operator+=(Vector3D const &);
        Vector3D &operator-=(Vector3D const &);
        bool operator!=(Vector3D const &) const;
        bool operator==(Vector3D const &) const;
        float &operator[](size_t index);
        const float &operator[](size_t index) const;
        friend std::ostream &operator<<(std::ostream &, Vector3D const &);

        /** Algebra **/
        Vector3D crossProd(Vector3D const &) const;
        float innerProd(Vector3D const &) const;
        float getAngle(Vector3D const &) const;

        // Calculate angle between two vectors in radian [-pi ; pi]
        Vector3D normalize() const;
        float norm() const;
        float norm2() const;

        /** Display **/
        void print() const;

    private:
        float m_arr[3];
    };

    /** "Static" methods **/
    inline bool isEqual(Vector3D const &v1, Vector3D const &v2) { return v1.isEqual(v2); }
    inline bool isNull(Vector3D const &v) { return v.isNull(); }
    inline Vector3D crossProd(Vector3D const &v1, Vector3D const &v2) { return v1.crossProd(v2); }
    inline float innerProd(Vector3D const &v1, Vector3D const &v2) { return v1.innerProd(v2); }
    inline float getAngle(Vector3D const &v1, Vector3D const &v2) { return v1.getAngle(v2); }
    inline Vector3D normalize(Vector3D const &v) { return v.normalize(); }
    inline float norm(Vector3D const &v) { return v.norm(); }
    inline float norm2(Vector3D const &v) { return v.norm2(); }

    template <typename T>
    Vector3D operator/(Vector3D const &vect, T const &scalar)
    {
        if (etl::is_arithmetic<T>::value)
        {
            if (scalar == 0)
            {
                throw std::domain_error("Divide by 0");
            }
            float arr[3];
            for (size_t i = 0; i < 3; i++)
            {
                arr[i] = vect[i] / scalar;
            }
            return Vector3D(arr);
        }
        else
        {
            throw std::logic_error("Not a scalar");
        }
    }

    template <typename T>
    Vector3D operator*(T const &scalar, Vector3D const &vect)
    {
        if (etl::is_arithmetic<T>::value)
        {
            float arr[3];
            for (size_t i = 0; i < 3; i++)
            {
                arr[i] = scalar * vect[i];
            }
            return Vector3D(arr);
        }
        else
        {
            throw std::logic_error("Not a scalar");
        }
    }

    template <typename T>
    Vector3D operator*(Vector3D const &vect, T const &scalar)
    {
        return scalar * vect;
    }

}

#endif