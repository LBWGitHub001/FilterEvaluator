//
// Created by lbw on 24-11-15.
//

#ifndef FILTER_ARMOR_H
#define FILTER_ARMOR_H
#include <vector>
#include <opencv2/opencv.hpp>

struct point{
    point(double x, double y):x(x),y(y){}
    double x;
    double y;
};

typedef struct Armor{
    std::shared_ptr<std::vector<cv::Point2f*>> realPoints;
    std::shared_ptr<std::vector<cv::Point2d*>> picPoints;
    double yaw;
    bool visual;
}Armor;

typedef struct Result{
    Result()=default;
    Result(cv::Point2f armorCenter, double yaw, cv::Point2f robotCenter):armorCenter(armorCenter),yaw(yaw),robotCenter(robotCenter){}
    cv::Point2f armorCenter;
    double yaw;
    cv::Point2f robotCenter;
}Result;

enum class Color{
    RED,
    BLUE,
};




#endif //FILTER_ARMOR_H
