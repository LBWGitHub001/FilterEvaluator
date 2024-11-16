//std
#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>
#include <vector>
#include <thread>
#include <ncurses.h>
//project
#include "FilterManager.h"

//std::mutex armors_mutex;
//std::condition_variable armors_cv;
std::atomic<bool> armors_is_busy;

void visualize(Robot &rbt) {
    std::cout << "DataGenerator Thread Start!" << std::endl;
    while (true) {
        if (!armors_is_busy) {
            armors_is_busy = true;
            rbt.update(0.03);
            armors_is_busy = false;
        }

        cv::waitKey(1);
    }
}

void filter_(Robot &rbt) {
    std::cout << "Filter Thread Start!" << std::endl;
    while (true) {
        if (!armors_is_busy) {
            armors_is_busy = true;
            Result *data;
            data = rbt.getVisibleArmors();
            if (data != nullptr) {

            }

            armors_is_busy = false;
            cv::waitKey(1);
        }

    }

}

int main(int argc, char **argv) {
    initscr(); // 初始化ncurses
    cbreak();  // 设置cbreak模式，这样按键会立即被处理
    noecho();  // 不回显按键

    Robot rbt(50, 0, 0, 0, 0, 0.1);
    std::thread visual_thread(visualize, std::ref(rbt));
    std::thread filter_thread(filter_, std::ref(rbt));

    std::cout << "asd" << std::endl;
    if (visual_thread.joinable()) {
        visual_thread.join();
    }
    if (filter_thread.joinable()) {
        filter_thread.join();
    }
    return 0;
}

/*
    Eigen::Matrix<double,3,1> a;
    Eigen::Matrix<double,1,1> b;
    a << 1,2,3;
    b << 4;
    std::cout << a << std::endl;
    a.block(0,0,a.rows()-1,a.cols()) = a.block(1,0,a.rows()-1,a.cols());
    std::cout << a << std::endl;
    a.block(a.rows()-1,0,1,1) = b;
    std::cout << a << std::endl;
 *
 * */