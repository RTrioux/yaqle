#include <iostream>
#include <math.h>
#include "quat.hpp"
using namespace std;
using namespace yaql;

int main()
{
    /** Create a quaternion **/
    /* (double q0, double q1 , double q2,double q3) i.e (w, x, y, z) */ 
    Quat Q0 = Quat(1, 2, 3, 4); // 1 + 2.i + 3.j + 4.k 

    /* (double arr[4]) */
    double arr[4] = {1, 2, 3, 4};
    Quat Q1 = Quat(arr);

    /* (double real, Vector3D im) */
    Quat Q2 = Quat(1, Vector3D(2, 3, 4));

    /** Create a unit quaternion describing a rotation `angle` along a given axis **/
    /* (double angle, double x, double y, double z, bool degree = false) */
    Quat Uq1 = unitQuat(M_PI_4, 1, 0, 0); // Rotation of Pi/4 rad about X axis 
    Quat Uq2 = unitQuat(45, 1, 0, 0, true); // Rotation of 45° about X axis 

    /* (double real, Vector3D im, bool degree = false) */
    Quat Uq3 = unitQuat(M_PI_4, Vector3D(1, 0, 0));
    Quat Uq4 = unitQuat(45, Vector3D(1, 0, 0), true);

    /** Quaternion arithmetic operations **/
    Q0 = Quat(2,0,0,0);
    Q1 = Quat(0,1,0,0);
    cout << Q0 * Q1 << endl; // >> (0, 2, 0, 0)

    Q0 = Quat(0,1,0,0);
    Q1 = Quat(0,1,2,3);
    cout << Q0 * Q1 << endl; // >> (-1, 0, -3, 2)

    Q0 = Quat(1,2,3,4);
    Q1 = Quat(0,1,0,0);
    cout << Q0 / Q0 << endl; // = Q0 * conj(Q0) / ||Q0||² >> (1, 0, 0, 0)
    cout << Q0 / Q1 << endl; // = Q0 * conj(Q1) / ||Q1||² >> (2, -1 ,-4 , 3)

    Q0 = Quat(1,2,3,4);
    Q1 = Quat(1,1,1,1);
    cout << Q0 + Q1 << endl; // >> (2, 3, 4, 5)
    cout << Q0 - Q1 << endl; // >> (0, 1, 2, 3)

    /** Complex numbers functions **/
    Q0 = Quat(1,2,3,4);
    cout << Q0.norm2() << endl; // = norm² = 1² + 2² + 3² + 4² >> 30
    cout << Q0.norm() << endl; // >> 5.47
    cout << Q0.normalize() << endl; // (0.182574,0.365148,0.547723,0.730297)
    cout << Q0.conj() << endl; // >> (1, -2, -3, -4)
    cout << Q0.re() << endl; // >> 1
    cout << Q0.im() << endl; // >> (2, 3, 4)

    // Static methods
    cout << norm2(Q0) << endl;
    cout << norm(Q0) << endl;
    // ...

    /** Spatial rotations **/
    /* Euler <-> Quaternion conversions */
    Q0 = unitQuat(M_PI_4, 1,1,0); // >> (cos(Pi/4/2), 0.27, 0.27, 0)
    // Default settings (ZYX/radian/intrinsic)
    Vector3D ypr = Q0.toEuler();
    cout << ypr << endl; // >> (0.169918, 0.523599, 0.61548)
    cout << fromEuler(ypr) << endl; // >> (cos(Pi/4/2), 0.27, 0.27, 0)
    
    // Optional arguments
    Quat::Sequence seq = Quat::ZXZ;
    bool isDegree = true;
    bool isExtrinsic = true;
    Vector3D euler = Q0.toEuler(seq,isDegree,isExtrinsic);
    cout << euler << endl; // >> (-45, 45, 45)
    cout << fromEuler(euler, seq, isDegree, isExtrinsic) << endl; // >> (cos(Pi/4/2), 0.27, 0.27, 0)
    
    /* 3D Rotation */
    Vector3D ex(1,0,0),ey(0,1,0),ez(0,0,1);
    Q0 = unitQuat(M_PI_2,0,1,0);
    cout << Q0.rotate(ex) << endl; // >> (~0, 0, -1)
    cout << Q0.rotate(ey) << endl; // >> (0, 1, 0)
    cout << Q0.rotate(ez) << endl; // >> (1, 0, ~0)

    Q0 = unitQuat(-M_PI_2,0,1,0);
    cout << Q0.rotate(ex) << endl; // >> (~0, 0, 1)
    cout << Q0.rotate(ey) << endl; // >> (0, 1, 0)
    cout << Q0.rotate(ez) << endl; // >> (-1, 0, ~0)

    // Get the equivalent rotation from vector v1 to vector v2
    Vector3D v1(0.5,1,100), v2(2,1,-50);
    Q0 = getRotation(v1, v2);
    cout << Q0 << "\t" << Q0.rotate(v1) * norm(v2) / norm(v1) << "\t" << (1/Q0).rotate(v2) * norm(v1)/norm(v2) << endl;


    /*
    bool extrinsic = true;
    Quat q1 = Quat::unitQuat(0.3,70,-20,33);
    Quat::Sequence seq = Quat::XYX;

    Vector3D euler = Vector3D(q1.toEuler(seq,false,extrinsic));
    Quat qFromEuler = Quat::fromEuler(euler,seq,false,extrinsic);
    Vector3D toEuler = Vector3D(qFromEuler.toEuler(seq,false,extrinsic)); 

    cout<<q1<<endl;
    cout<<euler<<endl;
    cout<<qFromEuler<<endl;
    cout<<toEuler<<endl;
    */

    return 0;
}