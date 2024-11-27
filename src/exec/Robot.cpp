//
// Created by lbw on 24-11-15.
//

#include "Robot.h"
Robot::Robot(double x, double y, double yaw, double vx, double vy, double vyaw) :
        _yaw(yaw), _v_yaw(vyaw), _x(x), _y(y), _v_x(vx), _v_y(vy) {
    cv::namedWindow("view");
    for (int i = 0; i < 4; i++) {
        double yaw = i * M_PI / 2;
        std::shared_ptr<Armor> armor = std::make_shared<Armor>();
        armor->realPoints = std::make_shared<std::vector<cv::Point2f *>>();
        armor->picPoints = std::make_shared<std::vector<cv::Point2d *>>();
        parsePoints(armor->realPoints, yaw);
        calLocGraph(armor->realPoints, armor->picPoints);
        armor->yaw = yaw;
        armor->visual = cos(yaw) >= visual_bound;
        _armors.push_back(armor);
    }

}

Robot::~Robot() {
    _armors.clear();
    cv::destroyAllWindows();
}

std::shared_ptr<std::vector<cv::Point2f *>>
Robot::parsePoints(std::shared_ptr<std::vector<cv::Point2f *>> &realPoints, double yaw) {
    float x1 = _x - d * cos(yaw - a);
    float x2 = _x - d * cos(yaw + a);
    float y1 = _y + d * sin(yaw - a);
    float y2 = _y + d * sin(yaw + a);
    //std::shared_ptr<std::vector<cv::Point2f *>> realPoints = std::make_shared<std::vector<cv::Point2f *>>();
    realPoints->clear();
    realPoints->push_back(new cv::Point2f(x1, y1));
    realPoints->push_back(new cv::Point2f(x1, y1));
    realPoints->push_back(new cv::Point2f(x2, y2));
    realPoints->push_back(new cv::Point2f(x2, y2));
    realPoints->push_back(new cv::Point2f((x1 + x2) / 2, (y1 + y2) / 2));
    return realPoints;
}

std::shared_ptr<std::vector<cv::Point2d *>>
Robot::calLocGraph(std::shared_ptr<std::vector<cv::Point2f *>> realPoints,
                   std::shared_ptr<std::vector<cv::Point2d *>> picPoints) {
    picPoints->clear();
    int i = 0;
    for (auto &point: *realPoints) {
        float x = point->x;
        float y = point->y;
        int xx = std::round(f / x * y);
        int yy = 0;
        if (i == 4) {
            yy = 0;
        } else if (i % 2 == 0) {
            yy = std::round(f / x * h / 2);
        } else {
            yy = std::round(-f / x * h / 2);
        }
        picPoints->push_back(new cv::Point2d(xx + width, yy + height));
        i++;
    }
    return picPoints;

}

void Robot::show(Color color) {
    //std::cout << "Update Robot" << std::endl;
    cv::Mat img = cv::Mat::zeros(height * 2, width * 2, CV_8UC3);
    cv::Scalar colorType;
    if (color == Color::RED) {
        colorType = cv::Scalar(0, 0, 255);
    } else {
        colorType = cv::Scalar(255, 0, 0);
    }
    for (auto armor: _armors) {
        if (armor->visual) {
            cv::line(img, *(*armor->picPoints)[0], *(*armor->picPoints)[1], colorType, 2);
            cv::line(img, *(*armor->picPoints)[1], *(*armor->picPoints)[3], colorType, 2);
            cv::line(img, *(*armor->picPoints)[3], *(*armor->picPoints)[2], colorType, 2);
            cv::line(img, *(*armor->picPoints)[2], *(*armor->picPoints)[0], colorType, 2);
            auto center = *(*armor->picPoints)[4];
            //cv::line(img, cv::Point2d(center.x - 20, center.y), cv::Point2d(center.x + 20, center.y), colorType, 2);
            cv::line(img, cv::Point2d(center.x, center.y - 20), cv::Point2d(center.x, center.y + 20), colorType, 2);
        }
    }
    auto data = getVisibleArmors();
    if (data != nullptr) {
        std::string a = "ArmorCenter:(" + std::to_string(data->armorCenter.x) + "," \
            + std::to_string(data->armorCenter.y) + ")";
        std::string b = "RobotCenter:(" + std::to_string(data->robotCenter.x) + "," \
                + std::to_string(data->robotCenter.y)+ ")";
        std::string c = "Yaw:" + std::to_string(data->yaw);
        cv::putText(img, a, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
        cv::putText(img, b, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
        cv::putText(img, c, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
    }
    else{
        std::string a = "ArmorCenter:(XXXXXXXX,XXXXXXXX)";
        std::string b = "RobotCenter:(XXXXXXXX,XXXXXXXX)";
        std::string c = "Yaw:XXXXXXXX";
        cv::putText(img, a, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
        cv::putText(img, b, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
        cv::putText(img, c, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);

    }

    std::string vx = "Vx:" + std::to_string(_v_x);
    std::string vy = "Vy:" + std::to_string(_v_y);
    std::string vyaw = "Vyaw:" + std::to_string(_v_yaw);
    cv::putText(img, vx, cv::Point(1300, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
    cv::putText(img, vy, cv::Point(1300, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
    cv::putText(img, vyaw, cv::Point(1300, 130), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);

    delete data;
    cv::imshow("view", img);
}

void Robot::update(double dt) {
    _x += _v_x * dt;
    _y += _v_y * dt;
    _yaw += _v_yaw * dt;
    for (int i = 0; i < 4; i++) {
        double yaw = _yaw + i * M_PI / 2;
        auto armor = _armors[i];
        parsePoints(armor->realPoints, yaw);
        calLocGraph(armor->realPoints, armor->picPoints);
        armor->yaw = yaw;
        armor->visual = cos(yaw) >= visual_bound;
    }
    show(Color::RED);
}

Result *Robot::getVisibleArmors() {
    cv::Point2f armorCenter;
    int i = 0;
    double yaw = _yaw;
    for (auto &armor: _armors) {
        if (armor->visual) {
            armorCenter = *(*armor->realPoints)[4];
            yaw = _yaw + i * M_PI / 2;
            break;
        }
        i++;
    }
    if (i == 4) {
        return nullptr;
    }

    return new Result(armorCenter, yaw, cv::Point2f(_x, _y));
}

void Robot::setVx(double val) {
    _v_x = val;

}

void Robot::setVy(double val) {
    _v_y = val;

}

void Robot::setVyaw(double val) {
    _v_yaw = val;

}

void Robot::setX(double val) {
    _x = val;

}

void Robot::setY(double val) {
    _y = val;

}

void Robot::setYaw(double val) {
    _yaw = val;

}

void Robot::addVx(double val) {
    _v_x += val;
}

void Robot::addVy(double val) {
_v_y += val;
}

void Robot::addVyaw(double val) {
    _v_yaw += val;
}

void Robot::addX(double val) {
    _x += val;
}

void Robot::addY(double val) {
    _y += val;
}

void Robot::addYaw(double val) {
    _yaw += val;
}








