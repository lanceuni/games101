#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry> // 引入Eigen的几何模块

constexpr double MY_PI = 3.1415926;
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    // 将角度转换为弧度
    float rad = angle * MY_PI / 180.0;

    // 归一化旋转轴
    axis.normalize();

    // 计算罗德里格斯公式的各个部分
    Eigen::Matrix3f K;
    K << 0, -axis[2], axis[1],
         axis[2], 0, -axis[0],
         -axis[1], axis[0], 0;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R = I + sin(rad) * K + (1 - cos(rad)) * (K * K);
    Eigen::Matrix3f R1 = cos(rad) * I+(1-cos(rad))*(axis*axis.transpose())+sin(rad)*K;

    // 将3x3旋转矩阵扩展到4x4矩阵
    rotation.block<3, 3>(0, 0) = R;

    return rotation;
}

// 该函数用于获取视图矩阵，视图矩阵用于将场景中的物体从世界坐标系转换到视图坐标系。
// 输入参数 eye_pos 是一个三维向量，表示观察者的位置。
// 函数首先创建一个单位矩阵 view，然后构造一个平移矩阵 translate，
// 该矩阵将物体沿着负的观察者位置方向平移，从而实现视图变换。
// 最后，返回平移后的视图矩阵。
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rad = rotation_angle * MY_PI / 180.0;
    model << cos(rad), -sin(rad), 0, 0,
             sin(rad),  cos(rad), 0, 0,
             0,         0,        1, 0,
             0,         0,        0, 1;

    // // 添加缩放因子
    // float scale_factor = 0.5; // 根据需要调整缩放因子
    // model(0, 0) *= scale_factor;
    // model(1, 1) *= scale_factor;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    eye_fov *=2;//放缩系数。屏幕越界会段错误异常退出用2缩小。
    //转换为正交投影矩阵
    projection << zNear, 0, 0, 0,
                    0, zNear, 0, 0,
                    0, 0, zNear + zFar, -zNear * zFar,
                    0, 0, 1, 0;
    float t = (-zNear) * tan(eye_fov * MY_PI / 180.0/2);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t; 
    //转换到正则正方体
    Eigen::Matrix4f Mortho,Mtrans,Mscale;
    Mtrans << 1, 0, 0, -(r+l)/2,
               0, 1, 0, -(t+b)/2,
               0, 0, 1, -(zNear+zFar)/2,
               0, 0, 0, 1;
    Mscale << 2/(r-l), 0, 0, 0,
               0, 2/(t-b), 0, 0,
               0, 0, 2/(zNear-zFar), 0,
               0, 0, 0, 1;
    Mortho = Mscale * Mtrans;
    return Mortho * projection;
}
// int main(int argc, const char** argv)
// {
//     float angle = 0;
//     bool command_line = false;
//     std::string filename = "output.png";

//     if (argc >= 3) {
//         command_line = true;
//         angle = std::stof(argv[2]); // -r by default
//         if (argc == 4) {
//             filename = std::string(argv[3]);
//         }
//         else
//             return 0;
//     }

//     rst::rasterizer r(700, 700);

//     Eigen::Vector3f eye_pos = {0, 0, 5};

//     std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

//     std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

//     auto pos_id = r.load_positions(pos);
//     auto ind_id = r.load_indices(ind);

//     int key = 0;
//     int frame_count = 0;

//     if (command_line) {
//         r.clear(rst::Buffers::Color | rst::Buffers::Depth);

//         r.set_model(get_model_matrix(angle));
//         r.set_view(get_view_matrix(eye_pos));
//         r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

//         r.draw(pos_id, ind_id, rst::Primitive::Triangle);
//         cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
//         image.convertTo(image, CV_8UC3, 1.0f);

//         cv::imwrite(filename, image);

//         return 0;
//     }

//     while (key != 27) {
//         r.clear(rst::Buffers::Color | rst::Buffers::Depth);

//         r.set_model(get_model_matrix(angle));
//         r.set_view(get_view_matrix(eye_pos));
//         r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

//         r.draw(pos_id, ind_id, rst::Primitive::Triangle);

//         cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
//         image.convertTo(image, CV_8UC3, 1.0f);
//         cv::imshow("image", image);
//         key = cv::waitKey(10);

//         std::cout << "frame count: " << frame_count++ << '\n';

//         if (key == 'a') {
//             angle += 10;
//         }
//         else if (key == 'd') {
//             angle -= 10;
//         }
//     }

//     return 0;
// }

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    
    float rangle = 0;
    Eigen::Vector3f axis;
    int mod = 0;

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    
    std::cin >> axis.x() >> axis.y() >> axis.z();

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        Eigen::Matrix4f m = get_rotation(axis, rangle)*get_model_matrix(angle);
        r.set_model(m);
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        if(key == 'r')
        {mod ^= 1;}
        else if (key == 'a') {
            if(mod == 0)
                angle += 10;
            else
                rangle += 10;
        }
        else if (key == 'd') {
            if(mod == 0)
                angle -= 10;
            else
                rangle -= 10;
        }
    }

    return 0;
}
