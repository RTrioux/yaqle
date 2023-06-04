# Yet Another Quaternion Library for Embedded

This library is meant to be a simple way to handle quaternion in C++ and use 
quaternion to deal with 3D rotations. This library can handle all Euler angle sequences (Tait-Bryan & Proper Euler angles)
with intrinsic or extrinsic convention (See [Davenport chained rotations](https://www.wikiwand.com/en/Davenport_chained_rotations#/Conversion_between_intrinsic_and_extrinsic_rotations)).
Functions are hardcoded as much as possible on purpose in order to improve performances for real time applications (Not ready for µC yet). 


yaql also provide a `Vector3D` class with overloaded arithmetic operators that you may use to apply rotation on.

## Create a quaternion
One can simply instantiate quaternions.

```cpp
    /** Create a quaternion **/
    /* (double q0, double q1 , double q2,double q3) i.e (w, x, y, z) */ 
    Quat Q0 = Quat(1, 2, 3, 4); // 1 + 2.i + 3.j + 4.k 

    /* (double arr[4]) */
    double arr[4] = {1, 2, 3, 4};
    Quat Q1 = Quat(arr);

    /* (double real, Vector3D im) */
    Quat Q2 = Quat(1, Vector3D(2, 3, 4));
```

One can create unitary quaternions from angle/axis argument from `unitQuat`.
This function yield a normalized quaternion with respect to the specified angle, hence q0 is always equal to cos(angle/2),
then the imaginary part is adjusted such that the norm is unitary. (angle = 0 => Q = (1,0,0,0)).

```cpp
    /** Create a unit quaternion describing a rotation `angle` along a given axis **/
    /* (double angle, double x, double y, double z, bool degree = false) */
    Quat Uq1 = unitQuat(M_PI_4, 1, 0, 0); // Rotation of Pi/4 rad about X axis 
    Quat Uq2 = unitQuat(45, 1, 0, 0, true); // Rotation of 45° about X axis 

    /* (double real, Vector3D im, bool degree = false) */
    Quat Uq3 = unitQuat(M_PI_4, Vector3D(1, 0, 0));
    Quat Uq4 = unitQuat(45, Vector3D(1, 0, 0), true);
```

See **Euler <=> Quaternion conversion** section for creating quaternion from Euler angles
## Arithmetic operation on quaternions
Most arithmetics operators have been overloaded.
`>>` works with std::cout, you can then print result directly in the standard output. 

```cpp
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
```

Some other classic functions are implemented to handle complex numbers such as `norm`, `re`, `im` ...
Note that the initial object is **never** modified by the function itself. These functions always return 
a result without modifying anything.
Note that most of these functions have a static definition you can call directly from the class (Quat::Foo(Quat const & q) vs q.foo())

```cpp
    /** Complex numbers functions **/
    Q0 = Quat(1,2,3,4);
    cout << Q0.norm2() << endl; // = norm² = 1² + 2² + 3² + 4² >> 30
    cout << Q0.norm() << endl; // >> 5.47
    cout << Q0.normalize() << endl; // (0.182574,0.365148,0.547723,0.730297)
    cout << Q0.conj() << endl; // >> (1, -2, -3, -4)
    cout << Q0.re() << endl; // >> 1
    cout << Q0.im() << endl; // >> (2, 3, 4)

    // "Static" methods
    cout << norm2(Q0) << endl;
    cout << norm(Q0) << endl;
    // ...
```

## Spatial rotations
You can convert back and forth quaternions to Euler.
Keep in mind that a set of Euler angle **does not** make any sense unless you specify the rotation sequence and convention used (intrinsic/extrinsic).
The default settings (and the most commonly used) is `ZYX`(yaw / pitch / roll) intrinsic expressed in radian.
The sequence can be selected through the `Sequence` enum in `Quat` class.

```cpp
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
    cout << Q0.rotateVector(ex) << endl; // >> (~0, 0, -1)
    cout << Q0.rotateVector(ey) << endl; // >> (0, 1, 0)
    cout << Q0.rotateVector(ez) << endl; // >> (1, 0, ~0)

    Q0 = unitQuat(-M_PI_2,0,1,0);
    cout << Q0.rotateVector(ex) << endl; // >> (~0, 0, 1)
    cout << Q0.rotateVector(ey) << endl; // >> (0, 1, 0)
    cout << Q0.rotateVector(ez) << endl; // >> (-1, 0, ~0)
```
# TODOs

- Make ETL version of this library to be able to use it from a microcontroler (Planned on STM32)
- Add conversion to/from rotation matrix
- Create a class 'Rotation', that wrap the whole thing, so that one can set its preferred settings during object instantiation (units/sequences/convention...) 






