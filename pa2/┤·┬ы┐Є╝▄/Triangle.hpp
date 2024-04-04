//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <eigen3/Eigen/Eigen>


using namespace Eigen;
class Triangle{

public:
    Vector3f v[3];              // 三角形的原始坐标，按逆时针顺序为 v0、v1、v2
    Vector3f color[3];          // 每个顶点的颜色
    Vector2f tex_coords[3];     // 每个顶点的纹理坐标 u 和 v
    Vector3f normal[3];         // 每个顶点的法向量

    // 纹理指针
    // Texture *tex;

    // 三角形类的构造函数
    Triangle();

    // 设置第 ind 个顶点的坐标
    void setVertex(int ind, Vector3f ver);

    // 设置第 ind 个顶点的法向量
    void setNormal(int ind, Vector3f n);

    // 设置第 ind 个顶点的颜色
    void setColor(int ind, float r, float g, float b);

    // 获取三角形的颜色（只返回第一个顶点的颜色）
    Vector3f getColor() const { return color[0] * 255; }

    // 设置第 ind 个顶点的纹理坐标
    void setTexCoord(int ind, float s, float t);

    // 将三角形的顶点数据转换为 Vector4f 数组（每个顶点增加一个齐次坐标）
    std::array<Vector4f, 3> toVector4() const;
};






#endif //RASTERIZER_TRIANGLE_H
