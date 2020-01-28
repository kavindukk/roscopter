#include <iostream>
#include <eigen3/Eigen/Eigen>
#include<math.h>

#define pi 3.1415926535
#define s(theta) sin(theta)
#define c(theta) cos(theta)
#define t(theta) tan(theta)

Eigen::Matrix3d rotation(float phi, float theta, float psi, std::vector<double> v){

  
  Eigen::Matrix<double, 3,3> R;
  R<< (c(psi)*c(theta)-s(phi)*s(psi)*s(theta)), -c(psi)*c(theta), (c(psi)*s(theta)+c(theta)*s(phi)*s(psi)),
  (c(theta)*s(psi)+c(psi)*s(phi)*s(theta)), c(phi)*c(psi), (s(psi)*s(theta) - c(psi)*c(theta)*s(psi)),
  (-c(phi)*s(theta)), s(phi), c(phi)*c(theta);

  Eigen::Matrix<double, 3,1> V;
  V<< v[0], v[1], v[2];

  Eigen:: MatrixXd p;

  p = R * V;

  std::cout<< R <<std::endl;
}

void ForcenTorquecalc( std::vector<double> v){
  
  // std::vector<double> v = { 250, 250, 250, 250};

  Eigen::Matrix<double, 4,4> coeff;
  double Ct, dCt, Cq = 9.0e-5, 1.2e-5, 1.2e-5;
  coeff<<Ct,Ct, Ct, Ct,
  0, dCt, 0, -dCt,
  -dCt, 0, dCt, 0,
  -Cq, Cq, -Cq, Cq;

  Eigen::Matrix<double, 4,1> speeds;

  // speeds<< v[0] , V[1], v[2], v[3];
  speeds<< 250,250,250, 250;

  Eigen::MatrixXd results = coeff * speeds;

  std::cout<<results<<std::endl;

}

int main()
{
  double phi = pi/6;
  double theta = pi/4;
  double psi = pi/3;
  std::vector<double> v = {1, 1, 1};

  rotation(phi, theta, psi, v);
  
}