#include <iostream>
#include <math.h>
#include "quat.hpp"
#include "dquat.hpp"
#include <string>

#include <fstream> // Write to file

using namespace std;
using namespace yaqle;

int main()
{
    {
        /** Create a quaternion **/
        /* (float q0, float q1 , float q2,float q3) i.e (w, x, y, z) */
        cout << "# DEFINITIONS" << endl;
        Quat Q0 = Quat(1, 2, 3, 4); // 1 + 2.i + 3.j + 4.k

        /* (float arr[4]) */
        float arr[4] = {1, 2, 3, 4};
        Quat Q1 = Quat(arr);

        /* (float real, Vector3D im) */
        Quat Q2 = Quat(1, Vector3D(2, 3, 4));

        /* Implicit declaration */
        // This call the constructor Quat(float q0,float q1,float q2,float q3)
        Q0 = (Quat)15; // Note that the cast 'Quat' is not necessary
        cout << Q0 << endl;

        /** Create a unit quaternion describing a rotation `angle` along a given axis **/
        /* (float angle, float x, float y, float z, bool degree = false) */
        Quat Uq1 = unitQuat(M_PI_4, 1, 0, 0);   // Rotation of Pi/4 rad about X axis
        Quat Uq2 = unitQuat(45, 1, 0, 0, true); // Rotation of 45° about X axis

        /* (float real, Vector3D im, bool degree = false) */
        Quat Uq3 = unitQuat(M_PI_4, Vector3D(1, 0, 0));
        Quat Uq4 = unitQuat(45, Vector3D(1, 0, 0), true);

        cout << "# Arithmetics" << endl;
        cout << "## Products" << endl;
        /** Quaternion arithmetic operations **/
        Q0 = Quat(2, 0, 0, 0);
        Q1 = Quat(0, 1, 0, 0);
        cout << Q0 * Q1 << endl; // >> (0, 2, 0, 0)

        Q0 = Quat(0, 1, 0, 0);
        Q1 = Quat(0, 1, 2, 3);
        cout << Q0 * Q1 << endl; // >> (-1, 0, -3, 2)

        cout << "## Division" << endl;
        Q0 = Quat(1, 2, 3, 4);
        Q1 = Quat(0, 1, 0, 0);
        cout << Q0 / Q0 << endl; // = Q0 * conj(Q0) / ||Q0||² >> (1, 0, 0, 0)
        cout << Q0 / Q1 << endl; // = Q0 * conj(Q1) / ||Q1||² >> (2, -1 ,-4 , 3)

        cout << "## Sums" << endl;
        Q0 = Quat(1, 2, 3, 4);
        Q1 = Quat(1, 1, 1, 1);
        cout << Q0 + Q1 << endl; // >> (2, 3, 4, 5)
        cout << Q0 - Q1 << endl; // >> (0, 1, 2, 3)

        cout << "## Complex functions" << endl;
        /** Complex numbers functions **/
        Q0 = Quat(1, 2, 3, 4);
        cout << Q0.norm2() << endl;     // = norm² = 1² + 2² + 3² + 4² >> 30
        cout << Q0.norm() << endl;      // >> 5.47
        cout << Q0.normalize() << endl; // (0.182574,0.365148,0.547723,0.730297)
        cout << Q0.conj() << endl;      // >> (1, -2, -3, -4)
        cout << Q0.re() << endl;        // >> 1
        cout << Q0.im() << endl;        // >> (2, 3, 4)

        // Static methods
        cout << norm2(Q0) << endl;
        cout << norm(Q0) << endl;
        // ...

        cout << "# Spatial rotation" << endl;
        /** Spatial rotations **/
        /* Euler <-> Quaternion conversions */
        Q0 = unitQuat(M_PI_4, 1, 1, 0); // >> (cos(Pi/4/2), 0.27, 0.27, 0)
        // Default settings (ZYX/radian/intrinsic)
        Vector3D ypr = Q0.toEuler();
        cout << ypr << endl;            // >> (0.169918, 0.523599, 0.61548)
        cout << fromEuler(ypr) << endl; // >> (cos(Pi/4/2), 0.27, 0.27, 0)

        // Optional arguments
        Quat::Sequence seq = Quat::ZXZ;
        bool isDegree = true;
        bool isExtrinsic = true;
        Vector3D euler = Q0.toEuler(seq, isDegree, isExtrinsic);
        cout << euler << endl;                                        // >> (-45, 45, 45)
        cout << fromEuler(euler, seq, isDegree, isExtrinsic) << endl; // >> (cos(Pi/4/2), 0.27, 0.27, 0)

        /* 3D Rotation */
        Vector3D ex(1, 0, 0), ey(0, 1, 0), ez(0, 0, 1);
        Q0 = unitQuat(M_PI_2, 0, 1, 0);
        cout << Q0.rotate(ex) << endl; // >> (~0, 0, -1)
        cout << Q0.rotate(ey) << endl; // >> (0, 1, 0)
        cout << Q0.rotate(ez) << endl; // >> (1, 0, ~0)

        Q0 = unitQuat(-M_PI_2, 0, 1, 0);
        cout << Q0.rotate(ex) << endl; // >> (~0, 0, 1)
        cout << Q0.rotate(ey) << endl; // >> (0, 1, 0)
        cout << Q0.rotate(ez) << endl; // >> (-1, 0, ~0)

        // Get the equivalent rotation from vector v1 to vector v2
        Vector3D v1(0.5, 1, 100), v2(2, 1, -50);
        Q0 = getRotation(v1, v2);
        cout << Q0 << "\t" << Q0.rotate(v1) * norm(v2) / norm(v1) << "\t" << (1 / Q0).rotate(v2) * norm(v1) / norm(v2) << endl;

        cout << "Propriété" << endl;
        Q0 = Quat(4, 10, -7, 5);
        Q1 = Quat(-3, 17, 21, 42);

        cout << Q0 * Q1.conj() << "\t" << Q1 * Q0.conj() << endl;
        cout << Q0 * Q0.conj() << "\t" << Q0.norm2() << endl;
    }
    cout << endl;
    {
        cout << "DUAL QUATERNIONS" << endl;
        // Simple arithmetic
        Quat Q0 = Quat(1, 2, 3, 4);
        Quat Q1 = Quat(2, 3, 4, 5);

        Quat Q2 = Quat(2, 3, 5, 7);
        Quat Q3 = Quat(2, 4, 6, 8);

        DQuat dq1 = DQuat(Q0, Q1);
        DQuat dq2 = DQuat(Q2, Q3);

        cout << dq1 + dq2 << endl;
        cout << dq1 - dq2 << endl;
        cout << dq1 * dq2 << endl;
        cout << dq1 / dq1 << endl;
        cout << dq2 / dq2 << endl;

        cout << "Conjugate" << endl;
        cout << dq2 * dq2.conj() << endl;

        // 3D transformations
        Q0 = unitQuat(0, {0, 0, 1});
        Q1 = Quat(0, 1, 0, 0);

        Q2 = unitQuat(M_PI, {0, 0, 1});
        Q3 = Quat(0, 0, 1, 0);

        dq1 = DQuat(Q0, Q1);
        dq2 = DQuat(Q2, Q3);
        cout << dq1 << "\t" << dq2 << endl;
        cout << dq1 * dq2 << endl
             << endl;

        cout << "Normalization" << endl;
        dq1 = DQuat(Quat(-10, 4, 6, 7), Quat(4, 3, 12, -15));
        dq2 = DQuat(Quat(4, 2, 1, -17), Quat(1, -4, -9, 21));
        DQuat dq1N = dq1.normalize();
        cout << "dq1N"
             << "\t\t" << dq1N << endl;
        cout << "dq1N.dq1N*=1"
             << "\t" << dq1N * dq1N.conj() << endl;

        cout << "Norm properties |Q1.Q2| = |Q1|*|Q2|" << endl;
        cout << (dq1 * dq2).norm() << endl;
        cout << dq1.norm() * dq2.norm() << endl
             << endl;
    }

    {
        Quat Q0, t0, Q1, t1;
        DQuat dq0, dq1;
        cout << "Transformation" << endl;
        Q0 = unitQuat(M_PI_4, {0, 0, 1});
        t0 = Quat(0, 0, 1, 0);
        dq0 = DQuat(Q0, 0.5 * t0 * Q0);
        cout << dq0 << endl;
        dq0 = dq0.normalize();

        Q1 = unitQuat(M_PI_4, {0, 0, 1});
        t1 = Quat(0, -1, 0, 0);
        dq1 = DQuat(Q1, 0.5 * t1 * Q1);
        cout << dq1 << endl;
        dq1 = dq1.normalize();
        cout << dq0 << "\t" << dq1 << endl;

        // Results
        DQuat dq3 = (dq1 * dq0).normalize();
        cout << dq3 << endl;

        cout << dq3.hRe() << endl;
        cout << dq3.pos() << endl;
    }

    // Generation Animation
    {
        // Quaternion animation
        ofstream file("simple_slerp.txt");
        Quat Q0 = Quat(1, 0, 0, 0);
        Quat Q1 = unitQuat(M_PI_2, 0, 0, 1);

        Quat D0 = Quat(0, 0, 0, 0);
        Quat D1 = Quat(0, 0, 3, 0);

        DQuat DQ0(Q0, 0.5f * D0 * Q0), DQ1(Q1, 0.5f * D1 * Q1);

        for (float t = 0; t < 1; t += 0.01)
        {
            Quat q = slerp(Q0, Q1, t);
            q.writeToFile(file);
        }

        file = ofstream("simple_lerp.txt");
        for (float t = 0; t < 1; t += 0.01)
        {
            Quat q = lerp(Q0, Q1, t);
            // file << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << endl;
            q.writeToFile(file);
        }
        DQuat DQ2, DQ3;

        DQ0 = rigidTransform(Quat(1, 0, 0, 0), 0, 0, 0);
        DQ1 = rigidTransform(unitQuat(M_PI_2, 0, 0, 1), 1, 0, 0);
        DQ2 = rigidTransform(unitQuat(M_PI_2, 0, 0, 1), 1, 0, 0);
        DQ3 = rigidTransform(unitQuat(M_PI_2, 0, 0, 1), 1, 0, 0);

        file = ofstream("dual_slerp.txt");
        for (float t = 0; t < 1; t += 0.01)
        {
            DQuat dq = slerp(DQ0, DQ2 * DQ1, t);
            dq.writeToFile(file);
        }
    }

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