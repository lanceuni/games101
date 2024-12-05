#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

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
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    Eigen::Matrix3f matrix_add_result = i + j;
    std::cout << "Matrix add i + j: \n" << matrix_add_result << std::endl;
    Eigen::Matrix3f matrix_scalar_multiply_result = i * 2.0f;
    std::cout << "Matrix scalar multiply i * 2.0: \n" << matrix_scalar_multiply_result << std::endl;
    Eigen::Matrix3f matrix_multiply_result = i * j;
    std::cout << "Matrix multiply i * j: \n" << matrix_multiply_result << std::endl;
    Eigen::Vector3f matrix_vector_multiply_result = i * v;
    std::cout << "Matrix multiply vector i * v: \n" << matrix_vector_multiply_result<< std::endl;
    //vector cross product
    Eigen::Vector3f av(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f bv(4.0f, 5.0f, 6.0f);
    Eigen::Vector3f cross_product = av.cross(bv);
    std::cout << "Cross product of v and w: \n" << cross_product << std::endl;
    // 给定一个点 P =(2,1), 将该点绕原点先逆时针旋转 45◦ ，再平移 (1,2), 计算出变换后点的坐标（要求用齐次坐标进行计算）。
    Eigen::Vector3f P(2.0f, 1.0f, 1.0f);
    float angle = 45.0f * M_PI / 180.0f; // 将角度转化为弧度
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix << std::cos(angle), -std::sin(angle), 0.0f,
                       std::sin(angle),  std::cos(angle), 0.0f,
                       0.0f,            0.0f,            1.0f;
    Eigen::Vector3f rotated_P = rotation_matrix * P; // 先旋转
    Eigen::Vector3f translation(1.0f, 2.0f, 0.0f); // 平移向量
    Eigen::Vector3f transformed_P = rotated_P + translation; // 执行平移
    std::cout << "Transformed point: \n" << transformed_P << std::endl;
    return 0;
}   
