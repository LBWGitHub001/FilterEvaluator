//std
#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>
#include <vector>
//project
#include "FilterManager.h"



int main(int argc, char **argv){
    Eigen::Matrix<double,3,1> a;
    Eigen::Matrix<double,1,1> b;
    a << 1,2,3;
    b << 4;
    std::cout << a << std::endl;
    a.block(0,0,a.rows()-1,a.cols()) = a.block(1,0,a.rows()-1,a.cols());
    std::cout << a << std::endl;
    a.block(a.rows()-1,0,1,1) = b;
    std::cout << a << std::endl;

   return 0;
}
