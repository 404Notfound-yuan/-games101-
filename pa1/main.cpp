#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

//构建绕z轴的旋转矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    model<<
        cos(rotation_angle/180*MY_PI),-sin(rotation_angle/180*MY_PI),0,0,
        sin(rotation_angle/180*MY_PI),cos(rotation_angle/180*MY_PI),0,0,
        0,0,1,0,
        0,0,0,1;

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

//返回透视投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    //eye_fov为视角范围
    //aspect ratio为宽高比
    //zNear为近裁剪面距离_near
    //zFar为远裁剪面距离_far
    projection<<
        -1/(tan(eye_fov/2*MY_PI/180)*aspect_ratio),0,0,0,
        0,-1/(tan(eye_fov/2*MY_PI/180)),0,0,
        0,0,(zNear+zFar)/(zNear-zFar),-2*zNear*zFar/(zNear-zFar),
        0,0,1,0;

    //还有一种表示方式：https://blog.csdn.net/Motarookie/article/details/121638314
    return projection;
}


//得到绕任意过原点的轴的旋转变换矩阵
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    //平移——旋转——平移
    Eigen::Matrix4f translate;
    translate<<
        1,0,0,-axis[0],
        0,1,0,-axis[1],
        0,0,1,-axis[2],
        0,0,0,1;

    Eigen::Matrix4f rotate;
    rotate<<
        cos(angle/180*MY_PI),-sin(angle/180*MY_PI),0,0,
        sin(angle/180*MY_PI),cos(angle/180*MY_PI),0,0,
        0,0,1,0,
        0,0,0,1;

    Eigen::Matrix4f translate_back;
    translate_back<<
        1,0,0,axis[0],
        0,1,0,axis[1],
        0,0,1,axis[2],
        0,0,0,1;

    rotation=translate*rotate*translate_back;

    return rotation;
}

/*
//使用罗德里格斯旋转公式进行计算
//参考网上做法，实际运行报错，主要是提醒相应的公式
Eigen::Matrix4f get_rotate_matrix(Vector3f v, float angle)
{
    Eigen::Matrix3f E = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R,t,p;
    angle=angle/180.0*M_PI;
	t<<0,-v[2],v[1],
       v[2],0,-v[0],
       -v[1],v[0],0;
    R=cos(angle)*E+(1-cos(angle))*v*v.transpose()+sin(angle)*t;
    return R;
}
*/
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
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

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Vector3f(1,0,0),angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Vector3f(1,0,0),angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
