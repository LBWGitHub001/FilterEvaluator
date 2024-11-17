//
// Created by lbw on 24-11-14.
//

#ifndef NEW_FILTER_FILTERMANAGER_H
#define NEW_FILTER_FILTERMANAGER_H

//std
#include <functional>
#include <vector>
#include <opencv2/opencv.hpp>
//project
#include "Robot.h"
#include "Kalman.h"
#include "MLS.h"
#include "config.h"

//定义滤波器的基类
class FilterManager {
public:
    FilterManager() = default;

    ~FilterManager() = default;

    virtual void init() = 0;

    virtual std::shared_ptr<Eigen::MatrixXd> update(const Eigen::VectorXd &data) = 0;

    virtual void *getFilter() = 0;


private:


};


#endif //NEW_FILTER_FILTERMANAGER_H
