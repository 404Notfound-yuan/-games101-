//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {

        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

//    Eigen::Vector3f getColorBilinear(float u, float v)
//     {
//         // 将输入坐标限制在 [0, 1] 范围内
//         if (u < 0) u = 0;
//         if (u > 1) u = 1;
//         if (v < 0) v = 0;
//         if (v > 1) v = 1;

//         // 计算纹理图像中对应的浮点坐标
//         auto u_img = u * width;
//         auto v_img = (1 - v) * height;

//         // 获取四个相邻像素的颜色值
//         Eigen::Vector3f color1 = getColor(u, v);
//         Eigen::Vector3f color2 = getColor(u + 1.0f / width, v);
//         Eigen::Vector3f color3 = getColor(u, v + 1.0f / height);
//         Eigen::Vector3f color4 = getColor(u + 1.0f / width, v + 1.0f / height);

//         // 计算双线性插值结果
//         float s = u_img - std::floor(u_img);
//         float t = v_img - std::floor(v_img);
//         Eigen::Vector3f color_top = color1 + (color2 - color1) * s;
//         Eigen::Vector3f color_bottom = color3 + (color4 - color3) * s;
//         Eigen::Vector3f interpolated_color = color_top + (color_bottom - color_top) * t;

//         return interpolated_color;
//     }

    Eigen::Vector3f getColorBilinear(float u, float v){
        float w1 = int(u * width), h1 = int(v * height);
        float w2 = w1 + 1, h2 = h1;
        float w3 = w1, h3 = h1 + 1;
        float w4 = w1 + 1, h4 = h1 + 1;

        Eigen::Vector3f color1, color2, color3, color4, color5, color6, color;
        color1 = getColor(w1 / width, h1 / height);
        color2 = getColor(w2 / width, h2 / height);
        color3 = getColor(w3 / width, h3 / height);
        color4 = getColor(w4 / width, h4 / height);
        color5 = color1 + (color2 - color1) * (u * width - w1);
        color6 = color3 + (color4 - color3) * (u * width - w1);
        color = color5 + (color6 - color5) * (v * height - h1);
        return color;
    }


};
#endif //RASTERIZER_TEXTURE_H
