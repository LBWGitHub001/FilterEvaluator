//
// Created by lbw on 24-11-17.
//

#ifndef FILTER_DRAWGRAPH_H
#define FILTER_DRAWGRAPH_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <map>
#include <functional>

constexpr int GraphHeight = 1000;
constexpr int GraphWidth = 2000;

class GraphInfo {
public:
    static constexpr int STEP = 10;
    GraphInfo();
    ~GraphInfo() = default;

    void addData(double val);
    void update();
    std::function<int(double, double)> getPointVal;
    std::function<int(double, double)> getRealVal;

    void setName(std::string name);
    void setColor(cv::Scalar color);
    cv::Scalar& getColor();

    void showImg();
private:
    cv::Mat img;
    std::vector<std::shared_ptr<cv::Point2f>> points;
    int maxIndex;
    int minVal;
    int maxVal;
    double unit;

    cv::Scalar color;
    std::string name;
};

class DrawGraph {
public:
    DrawGraph();

    ~DrawGraph();

    std::shared_ptr<GraphInfo> createScreen(const std::string name);

    void destoryScreen(const std::string name);

    void addPoint(const std::string name,double val);

    void showAll();

private:
    //图像和管理系统
    std::map<std::string, std::shared_ptr<GraphInfo>> windowQuarry;

    static void clear(cv::Mat &img);
};


#endif //FILTER_DRAWGRAPH_H
