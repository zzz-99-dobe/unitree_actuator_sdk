#pragma once


struct Vec3
{
    double x;
    double y;
    double z;
};


struct Mat3
{
    double m[3][3];
};


struct Transform
{
    Mat3 R;
    Vec3 p;
};


struct LegGeometry
{
    Transform T_BH1_zero;
    Transform T_H1H2_zero;
    Transform T_H2K_zero;
    Transform T_KF_zero;
};


// 对外提供的FK接口

Mat3 identity();

Transform makeTransform(
    const Mat3& R,
    const Vec3& p
);

Transform rotZ(double q);

Transform forwardKinematics(
    double q1,
    double q2,
    double q3,
    const LegGeometry& geometry
);
