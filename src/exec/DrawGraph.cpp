//
// Created by lbw on 24-11-17.
//

#include "DrawGraph.h"

DrawGraph::DrawGraph() {

}

DrawGraph::~DrawGraph() {}

void DrawGraph::clear(cv::Mat &img) {
    img = cv::Mat::zeros(GraphHeight,GraphWidth, CV_8UC3);
}

std::shared_ptr<GraphInfo> DrawGraph::createScreen(const std::string name) {
    cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::resizeWindow(name, GraphWidth, GraphHeight);
    auto windowPtr = std::make_shared<GraphInfo>();
    windowPtr->setName(name);
    windowPtr->setColor(cv::Scalar(rand()%256,rand()%256,rand()%256));
    windowQuarry[name] = windowPtr;
    return windowPtr;
}

void DrawGraph::destoryScreen(const std::string name) {
    if(windowQuarry.find(name) != windowQuarry.end()) {
        windowQuarry.erase(name);
    }
}

void DrawGraph::addPoint(const std::string name, double val) {
    auto window = windowQuarry[name];
    if(window == nullptr){
        window = createScreen(name);
    }
    window->addData(val);

    window->update();

}

void DrawGraph::showAll() {
    for(auto &window : windowQuarry){
        window.second->showImg();
    }

}

GraphInfo::GraphInfo() : maxIndex(0),maxVal(10),minVal(0),unit(10.0/300) {
    getPointVal = [this](double val,double unit){
        return (val - minVal)/unit;
    };

    getRealVal = [this](double val,double unit){
        return val*unit + minVal;
    };

    img = cv::Mat::zeros(GraphHeight,GraphWidth, CV_8UC3);
}

void GraphInfo::addData(double val) {
    if(val <= maxVal && val >= minVal){
        val = getPointVal(val,unit);
        maxIndex += STEP;
        points.push_back(std::make_shared<cv::Point2f>(maxIndex,val));
        if(maxIndex >= GraphWidth){
            maxIndex = 0;
            points.clear();
            img = cv::Mat::zeros(GraphHeight,GraphWidth,CV_8UC3);
        }
    }
    else{
        if(val > maxVal){
            maxVal = val+1;
        }
        if(val < minVal){
            minVal = val-1;
        }
        auto newUnit = static_cast<double>((maxVal-minVal))/GraphHeight;
        img = cv::Mat::zeros(GraphHeight,GraphWidth,CV_8UC3);
        for(auto iter = points.begin();iter != points.end();++iter){
            (*iter)->y = getRealVal((*iter)->y,unit);
            (*iter)->y = getPointVal((*iter)->y,newUnit);
            if(iter != points.begin()){
                cv::line(img,**(iter-1), **iter, color, 2, 8);
            }
        }


        unit = newUnit;
    }

}

void GraphInfo::update() {
    if(points.size() > 1){
        cv::line(img, *points[points.size()-2], *points[points.size()-1], color, 2, 8, 0);
    }

}

void GraphInfo::setName(std::string name) {
    this->name = name;
}

void GraphInfo::showImg() {
    cv::imshow(name, img);
    cv::waitKey(0);
}

cv::Scalar &GraphInfo::getColor() {
    return color;
}

void GraphInfo::setColor(cv::Scalar color) {
    this->color = color;
}
