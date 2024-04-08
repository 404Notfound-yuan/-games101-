#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() <4 ) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    //实现相应的算法

    //如果控制点只有一个，直接返回
    if(control_points.size() == 1){
        return control_points[0];
    }

    std::vector<cv::Point2f> new_control_points ;
    for(int i = 0; i < control_points.size() - 1; i++){
        //计算新的控制点
        cv::Point2f new_point = (1 - t) * control_points[i] + t * control_points[i + 1];
        new_control_points.push_back(new_point);
    }
    return recursive_bezier(new_control_points, t);
    //return cv::Point2f();
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // Call naive_bezier to draw red curve
    naive_bezier(control_points, window);

    // Create a temporary Mat to store the green bezier curve
    cv::Mat green_curve(window.size(), window.type(), cv::Scalar(0, 0, 0));

    // Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's recursive Bezier algorithm
    for(double t = 0.0; t <= 1.0; t += 0.001){
        cv::Point2f point = recursive_bezier(control_points, t);
        green_curve.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(0, 255, 0); // 设置颜色为绿色

        // 对周围的九个邻近像素进行绘制
        auto x = point.x;
        auto y = point.y;
        auto centerx = x + 0.5; // 中心点坐标
        auto centery = y + 0.5;
        for(int i = -1; i <= 1; i++){
            for(int j = -1; j <= 1; j++){
                // 计算中心点与邻近点的距离
                double distance = std::sqrt(std::pow(x - (centerx + i), 2) + std::pow(y - (centery + j), 2));

                // 需要进行归一化
                distance = distance * std::sqrt(2) / 3;

                // 根据归一化后的距离计算颜色
                double color = 255 * (1 - distance);

                // 如果颜色大于原来的颜色
                if(color > green_curve.at<cv::Vec3b>(y + j, x + i)[1]){
                    green_curve.at<cv::Vec3b>(y + j, x + i) = cv::Vec3b(0, color, 0);
                }
            }
        }
    }


    // Add the green curve to the window
    cv::add(window, green_curve, window);
}
int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() ==4) 
        {
             naive_bezier(control_points, window);
             //bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
