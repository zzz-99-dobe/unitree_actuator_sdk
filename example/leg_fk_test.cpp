#include <iostream>

#include "robot/LegKinematics.h"


constexpr double PI = 3.14159265358979323846;


int main()
{
    LegGeometry leg;


    // Body -> H1
    // Body位置暂时无效，所以Body原点暂时与H1原点重合
    // 坐标系方向关系保留
    leg.T_BH1_zero =
        makeTransform(
            rotZ(PI / 2).R,
            {0.0, 0.0, 0.0}
        );


    // H1 -> H2
    leg.T_H1H2_zero =
        makeTransform(
            rotZ(-PI / 2).R,
            {-0.085703, -0.0646, 0.0}
        );


    // H2 -> Knee
    leg.T_H2K_zero =
        makeTransform(
            identity(),
            {-0.24204, 0.0006, -0.059331}
        );


    // Knee -> Wheel center
    leg.T_KF_zero =
        makeTransform(
            identity(),
            {0.24892, 0.0, -0.023226}
        );


    // CAD零位
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;


    // 调用运动学模块
    Transform T_BF =
        forwardKinematics(q1, q2, q3, leg);


    // 测试结果
    std::cout
        << "foot = ["
        << T_BF.p.x << ", "
        << T_BF.p.y << ", "
        << T_BF.p.z << "] m\n";


    return 0;
}
