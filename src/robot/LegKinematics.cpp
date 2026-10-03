#include "robot/LegKinematics.h"

#include <cmath>


constexpr double PI = 3.14159265358979323846;



Mat3 identity()
{
    return {{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }};
}

Vec3 add(const Vec3& a, const Vec3& b)
{
    return {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}

Vec3 multiply(const Mat3& R, const Vec3& v)
{
    return {
        R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
        R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
        R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z
    };
}

Mat3 multiply(const Mat3& A, const Mat3& B)
{
    Mat3 C{};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                C.m[i][j] += A.m[i][k] * B.m[k][j];
            }
        }
    }
    return C;
}


Transform multiply(const Transform& A, const Transform& B)
{
    Transform C;

    C.R = multiply(A.R, B.R);
    C.p = add(A.p, multiply(A.R, B.p));

    return C;
}

Transform makeTransform(const Mat3& R, const Vec3& p)
{
    return {R, p};
}


Transform rotY(double q)
{
    double c = std::cos(q);
    double s = std::sin(q);

    Mat3 R{{
        { c, 0, s},
        { 0, 1, 0},
        {-s, 0, c}
    }};
    return makeTransform(R, {0, 0, 0});
}


Transform rotZ(double q)
{
    double c = std::cos(q);
    double s = std::sin(q);

    Mat3 R{{
        {c, -s, 0},
        {s,  c, 0},
        {0,  0, 1}
    }};

    return makeTransform(R, {0, 0, 0});
}

Transform translate(const Vec3& p)
{
    return makeTransform(identity(), p);
}


Transform forwardKinematics(
    double q1,
    double q2,
    double q3,
    const LegGeometry& g)
{
    Transform T_BH1 =
        multiply(
            g.T_BH1_zero,
            rotY(q1)
        );

    Transform T_H1H2 =
        multiply(
            g.T_H1H2_zero,
            rotY(q2)
        );

    Transform T_H2K =
        multiply(
            g.T_H2K_zero,
            rotY(-q3)
        );

    Transform T_KF =
        g.T_KF_zero;


    Transform T_BH2 =
        multiply(T_BH1, T_H1H2);

    Transform T_BK =
        multiply(T_BH2, T_H2K);

    Transform T_BF =
        multiply(T_BK, T_KF);

    return T_BF;
}
