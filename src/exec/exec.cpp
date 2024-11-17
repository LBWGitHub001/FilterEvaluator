//std
#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>
#include <vector>
#include <thread>
#include <termio.h>
#include <chrono>
//project
#include "KF_filter.h"


using tran_Kalman = KalmanFilter<CA_Tran_Kalman, CZ_Tran_Kalman>;
using rota_Kalman = KalmanFilter<CA_Rota_Kalman, CZ_Rota_Kalman>;
//std::mutex armors_mutex;
//std::condition_variable armors_cv;
std::atomic<bool> armors_is_busy;

int scanKeyboard();

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
    KF_filter kf_rotation("Rotation");
    KF_filter kf_translation("Translation");
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

void Keyboard(Robot &rbt) {
    std::cout << "Keyboard Thread Start!" << std::endl;
    int mode = 0;//0是改变速度，1是改变位置
    int T = 10;
    while (true) {
        if (scanKeyboard() == 'q') {
            mode = !mode;
            continue;
        }
        if (mode == 0) {//改变速度
            if (scanKeyboard() == 'w') {
                armors_is_busy = true;
                rbt.addVx(1);
                armors_is_busy = false;
                continue;
            }
            if (scanKeyboard() == 'a') {
                armors_is_busy = true;
                rbt.addVy(-1);
                armors_is_busy = false;
                continue;
            }
            if (scanKeyboard() == 's') {
                armors_is_busy = true;
                rbt.addVx(-1);
                armors_is_busy = false;
                continue;
            }
            if (scanKeyboard() == 'd') {
                armors_is_busy = true;
                rbt.addVy(1);
                armors_is_busy = false;
                continue;
            }
            if (scanKeyboard() == 'e') {
                armors_is_busy = true;
                rbt.addVyaw(1);
                armors_is_busy = false;
                continue;
            }
            if (scanKeyboard() == 'r') {
                armors_is_busy = true;
                rbt.addVyaw(-1);
                armors_is_busy = false;
                continue;
            }
        } else if (mode == 1) {//改变位置

        }


        std::this_thread::sleep_for(std::chrono::milliseconds(T));
    }

}

void test();

int main(int argc, char **argv) {
    //test();
    Robot rbt(50, 0, 0, 0, 0, 0.1);
    std::thread visual_thread(visualize, std::ref(rbt));
    std::thread filter_thread(filter_, std::ref(rbt));
    std::thread keyboard_thread(Keyboard, std::ref(rbt));

    if (keyboard_thread.joinable()) {
        keyboard_thread.join();
    }
    if (visual_thread.joinable()) {
        visual_thread.join();
    }
    if (filter_thread.joinable()) {
        filter_thread.join();
    }
    return 0;
}


int scanKeyboard() {
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    in = getchar();

    tcsetattr(0, TCSANOW, &stored_settings);
    return in;
}


void test() {
    while (1) {
        printf(":%d", scanKeyboard());
    }
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