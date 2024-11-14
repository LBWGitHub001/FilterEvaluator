//
// Created by lbw on 24-11-14.
//

#ifndef NEW_FILTER_FILTERMANAGER_H
#define NEW_FILTER_FILTERMANAGER_H

//std
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
//project
#include "Kalman.h"
#include "MLS.h"

class FilterManager :
    public rclcpp::Node{
public:
    FilterManager();
    ~FilterManager();


private:



};


#endif //NEW_FILTER_FILTERMANAGER_H
