//
// Created by lbw on 24-11-15.
//

#ifndef FILTER_ROBOT_H
#define FILTER_ROBOT_H
//std
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>
#include <condition_variable>
//project
#include "Armor.h"
#include "Mutexs.h"



constexpr int width = 640;
constexpr int height = 360;

constexpr double r = 30.0;  //机器人半径
constexpr double b = 20.0;   //装甲板半宽度
constexpr double h = 15.0;  //装甲板高度
constexpr double f = 200.0;   //相机焦距
constexpr double visual_bound = 0.8;//可视边界
const double d = sqrt(r * r + b * b); //点到装甲班顶点的距离
const double a = atan(b / r);     //装甲板顶点与机器人半径的夹角

class Robot {
public:
    Robot(double x=0.0,double y=0.0, double yaw=0.0, double vx=0.0, double vy=0.0, double vyaw=0.0);

    ~Robot();

    void update(double dt);

    void setVx(double val);
    void setVy(double val);
    void setVyaw(double val);
    void setX(double val);
    void setY(double val);
    void setYaw(double val);

    Result* getVisibleArmors();

private:
    //机器人数据
    std::vector<std::shared_ptr<Armor>> _armors;
    double _yaw;
    double _v_yaw;
    double _x;
    double _y;
    double _v_x;
    double _v_y;

    //进程锁
    //std::mutex armors_mutex;
    //std::condition_variable armors_cv;

    //函数
    std::shared_ptr<std::vector<cv::Point2f *>>
        parsePoints(std::shared_ptr<std::vector<cv::Point2f *>>& realPoints, double yaw);

    //可视化
    //std::vector<cv::Point2f*> _realPoints;
    //std::vector<cv::Point2f*> _picPoints;
    void show(Color color);//可视化主程序
    //根据距离计算出图像上的位置
    std::shared_ptr<std::vector<cv::Point2d *>>
        calLocGraph(std::shared_ptr<std::vector<cv::Point2f *>> realPoints, std::shared_ptr<std::vector<cv::Point2d *>> picPoints);
};


#endif //FILTER_ROBOT_H
