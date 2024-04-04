#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
//using namespace std;
//for test
int test(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << j << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    return 0;
}

int main(){
    // 45度
    double angle=45.0f/180.0f*M_PI;
    
    // 初始点
    Eigen::Vector2d P(2,1);//vector

    Eigen::Rotation2Dd rotation(angle);

    //旋转45度(默认逆时针)
    Eigen::Vector2d rotated=rotation*P;

    // 输出旋转后的点
    std::cout << "Rotated point: [" << rotated.x() << ", " << rotated.y() << "]" << std::endl;

    // 扩展 rotated 为 3x1 的矩阵，最后一行为 [x, y, 1]
    Eigen::Vector3d extended_rotated(rotated.x(), rotated.y(), 1);

    // 输出扩展后的 rotated
    std::cout << "Extended rotated:\n" << extended_rotated << std::endl;

    // 定义矩阵 M1
    Eigen::Matrix3d M1;
    M1 << 1, 0, 1,
          0, 1, 2,
          0, 0, 1;

    // 输出矩阵 M1
    std::cout << "Matrix M1:\n" << M1 << std::endl;

    // 平移等于左乘某个矩阵
    Eigen::MatrixXd  M=M1*extended_rotated;
    std::cout<<"MatrixXd:\n"<<M<<std::endl;

    return 0;
}