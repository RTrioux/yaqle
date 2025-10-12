#include "dquat.hpp"
#include "matrix.hpp"
#include "quat.hpp"
#include <iostream>
#include <math.h>
#include <string>

#include <fstream> // Write to file

using namespace std;
using namespace yaqle;

int main()
{

    // Generation Animation
    {
        // Quaternion animation
        ofstream file("simple_slerp.txt");
        Quat Q0 = Quat(1, 0, 0, 0);
        Quat Q1 = fromAxisAngle(M_PI_4, 1, 1, 0);

        // Compare LERP and SLERP
        file = ofstream("simple_lerp_vs_slerp.txt");
        Q0.writeToFile("Q_init", file);
        Q1.writeToFile("Q_final", file);
        for (float t = 0; t < 1; t += 0.01)
        {
            Quat q_lerp = lerp(Q0, Q1, t);
            Quat q_slerp = slerp(Q0, Q1, t);
            q_lerp.writeToFile("q_lerp", file);
            q_slerp.writeToFile("q_slerp", file);
        }

        Quat D0 = Quat(0, 0, 0, 0);
        Quat D1 = Quat(0, 0, 3, 0);
        DQuat DQ0(Q0, 0.5f * D0 * Q0), DQ1(Q1, 0.5f * D1 * Q1);

        DQuat DQ2, DQ3;

        DQ2 = rigidTransform(fromAxisAngle(M_PI_2 / 2, 0, 0, 1), 0, 1, 0);
        DQ3 = rigidTransform(fromAxisAngle(M_PI_2 / 2, 0, 0, 1), 1, 0, 0);

        DQ0 = rigidTransform(Quat(1, 0, 0, 0), 0, 0, 0);
        DQ1 = rigidTransform(fromAxisAngle(M_PI_2, 0, 0, 1), 1, 1, 0);

        file = ofstream("dual_slerp.txt");
        for (float t = 0; t < 1; t += 0.01)
        {
            DQuat dq = slerp(DQ0, DQ2 * DQ3, t);
            dq.writeToFile("dq", file);
        }
    }

    return 0;
}