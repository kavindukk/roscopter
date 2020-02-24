#include <iostream>
#include <eigen3/Eigen/Eigen>
#include<math.h>

#define pi 3.1415926535
#define s(theta) sin(theta)
#define c(theta) cos(theta)
#define t(theta) tan(theta)


Eigen::Matrix<double, 4,4> coeff;
    double Ct = 0.00028;
    double dCt = 0.000012;
    double Cq = 0.000012;
    coeff << Ct,Ct, Ct, Ct,
            0, dCt, 0, -dCt,
            -dCt, 0, dCt, 0,
            -Cq, Cq, -Cq, Cq;

Eigen::Matrix<double, 3,3> R;
R<< (c(psi)*c(theta)-s(phi)*s(psi)*s(theta)), -c(psi)*c(theta), (c(psi)*s(theta)+c(theta)*s(phi)*s(psi)),
        (c(theta)*s(psi)+c(psi)*s(phi)*s(theta)), c(phi)*c(psi), (s(psi)*s(theta) - c(psi)*c(theta)*s(psi)),
        (-c(phi)*s(theta)), s(phi), c(phi)*c(theta);


R_inv = R.inverse()

Eigen:: Matrix<double, 3,1> F_i;
F << actual_forces.Fx, -actual_forces_.Fy, -actual_forces_.Fz;
