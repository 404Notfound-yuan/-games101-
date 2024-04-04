// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


//测试点是否在三角形内部
//差值都大于0, 则点在三角形内部
//AB、BC、CA的分别与AP、BP、CP的叉积
//https://blog.csdn.net/Motarookie/article/details/121649418


//这个需要修改为浮点数
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
   
    Eigen::Vector2f p;
    p << x, y;

    Eigen::Vector3f p3(p[0], p[1], 0.0f);//多一个0.0，是因为cross需要三维

    Eigen::Vector3f v0(_v[0][0], _v[0][1], 0.0f);
    Eigen::Vector3f v1(_v[1][0], _v[1][1], 0.0f);
    Eigen::Vector3f v2(_v[2][0], _v[2][1], 0.0f);

    Eigen::Vector3f AB = v1 - v0;
    Eigen::Vector3f BC = v2 - v1;
    Eigen::Vector3f CA = v0 - v2;

    Eigen::Vector3f AP = p3 - v0;
    Eigen::Vector3f BP = p3 - v1;
    Eigen::Vector3f CP = p3 - v2;

    float cross_ABP = AB.cross(AP)[2];
    float cross_BCP = BC.cross(BP)[2];
    float cross_CAP = CA.cross(CP)[2];

    if (cross_ABP >= 0 && cross_BCP >= 0 && cross_CAP >= 0)
    {
        return true;
    }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//三角形栅格化______基础要求
//Screen space rasterization
// void rst::rasterizer::rasterize_triangle(const Triangle& t) {
//     auto v = t.toVector4();
    
//     // TODO : Find out the bounding box of current triangle.
//     // iterate through the pixel and find if the current pixel is inside the triangle

//     // If so, use the following code to get the interpolated z value.
//     //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
//     //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//     //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//     //z_interpolated *= w_reciprocal;

//     // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

//     // TODO：找出当前三角形的边界框。
//     int min_x = std::floor(std::min(std::min(v[0][0], v[1][0]), v[2][0]));
//     int max_x = std::ceil(std::max(std::max(v[0][0], v[1][0]), v[2][0]));

//     int min_y = std::floor(std::min(std::min(v[0][1], v[1][1]), v[2][1]));
//     int max_y = std::ceil(std::max(std::max(v[0][1], v[1][1]), v[2][1]));
//     // 遍历像素并查找当前像素是否在三角形内部。
//    for(int i=min_x; i<max_x; i++){
//        for(int j=min_y; j<max_y; j++){
//              if(insideTriangle(i+0.5, j+0.5, t.v)){
//                 // 如果是，则使用以下代码获取插值的 z 值。
//                 // auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
//                 // float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//                 // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//                 // z_interpolated *= w_reciprocal;
//                 auto [alpha, beta, gamma] = computeBarycentric2D((float) i + 0.5f, (float)j + 0.5f, t.v);
//                 float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//                 float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//                 z_interpolated *= w_reciprocal;
//                 // 设置当前像素（使用 set_pixel 函数）为三角形的颜色（使用 getColor 函数）如果它应该被绘制。
                
//                 //z-buffer——根据z值来判断颜色
//                 if (z_interpolated < depth_buf[get_index(i, j)]) {
//                     // 如果当前像素的插值深度值小于深度缓冲中的值，则执行以下操作：
//                     // 创建一个三维向量，表示当前像素的坐标和插值深度值
//                     Vector3f point(i, j, z_interpolated);
//                     // 设置当前像素的颜色为三角形的颜色，并将像素颜色写入到画布上
//                     set_pixel(point, t.getColor());
//                     // 更新深度缓冲中该像素的深度值为插值深度值，以便进行后续深度测试
//                     depth_buf[get_index(i, j)] = z_interpolated;
//                 }
//             }
//          }
//    }
  

//     // TODO：如果应该绘制，将当前像素（使用 set_pixel 函数）设置为三角形的颜色（使用 getColor 函数）。

// }


// 三角形栅格化______进阶要求
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    int min_x = std::floor(std::min(std::min(v[0][0], v[1][0]), v[2][0]));
    int max_x = std::ceil(std::max(std::max(v[0][0], v[1][0]), v[2][0]));

    int min_y = std::floor(std::min(std::min(v[0][1], v[1][1]), v[2][1]));
    int max_y = std::ceil(std::max(std::max(v[0][1], v[1][1]), v[2][1]));

    float k1[4]={0.25,0.25,0.75,0.75};
    float k2[4]={0.25,0.75,0.25,0.75};
    // 遍历像素并查找当前像素是否在三角形内部。
    
    float count=0;
    float mindep=1000000;

    float eid=0;
    for(int i=min_x; i<max_x; i++){
       for(int j=min_y; j<max_y; j++){
            count=0;eid = get_index(i,j)*4;mindep=1000000;
            for(int k=0; k<4; k++){
                if(insideTriangle(float(i+k1[k]), float(j+k2[k]), t.v)){
                    auto [alpha, beta, gamma] = computeBarycentric2D(float(i+k1[k]),float( j+k2[k]), t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    //一个像素分成四个部分
                    if (z_interpolated < depth_sample[eid + k]){
                        depth_sample[eid + k] = z_interpolated;
                        frame_sample[eid + k] = t.getColor() / 4;
                    }
                    mindep = std::min(depth_sample[eid + k], mindep);
                }
            }
           
            //这里还需要修改—计算覆盖率(n/4)x100%

            //像素颜色=三角形颜色*覆盖率
           
            Vector3f color = frame_sample[eid] + frame_sample[eid + 1] + frame_sample[eid + 2] + frame_sample[eid + 3];
            set_pixel(Vector3f(i, j, mindep).cast<float>(), color);
            depth_buf[get_index(i, j)] = std::min(depth_buf[get_index(i, j)], mindep);
         }
   }
}


void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
   
        //黑边问题
        std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        //黑边问题
        std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    //黑边问题
    frame_sample.resize(w * h * 4);
    depth_sample.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on