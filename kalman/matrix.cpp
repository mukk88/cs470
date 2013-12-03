#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

int main(){
  MatrixXd m(2,2);
  MatrixXd m2(2,2);
  m << 1,2,3,4;
  m2 << 3,4,5,6;

  std::cout << m*m2 << std::endl;
}