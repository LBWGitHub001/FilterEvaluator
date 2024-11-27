//std
#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>
#include <vector>
#include <thread>
#include <termio.h>
#include <chrono>
#include <random>
//project
#include "DrawGraph.h"
#include "KF_filter.h"
#include "config.h"
#include "filter.cpp"


using tran_Kalman = KalmanFilter<CA_Tran_Kalman, CZ_Tran_Kalman>;
using rota_Kalman = KalmanFilter<CA_Rota_Kalman, CZ_Rota_Kalman>;
//std::mutex armors_mutex;
//std::condition_variable armors_cv;
std::atomic<bool> armors_is_busy;

double generateGaussianNoise(double mean, double stddev);
int scanKeyboard();

template<class T>
void genNoise(T& data, NoiseType noise_type, double noise_level){
    if(noise_type == NoiseType::Gaussian){
        try{
            for(auto &val : data){
                val += generateGaussianNoise(0, noise_level);
                if(rand()%100 < 1){
                    val *= 100;
                }
            }
        }
        catch (...){
            std::cerr << "[Error]The variables type must be iterable" << std::endl;
        }
    }

}

void visualize(Robot &rbt) {
    std::cout << "DataGenerator Thread Start!" << std::endl;
    while (true) {
        if (!armors_is_busy) {
            armors_is_busy = true;
            rbt.update(0.03);
            armors_is_busy = false;
        }
        else{
            std::cout << "armors_is_busy" << std::endl;
        }

        cv::waitKey(1);
    }
}

void filter_(Robot &rbt) {
    std::cout << "Filter Thread Start!" << std::endl;
    KF_filter kf_rotation(FilterType::Rotation);
    KF_filter kf_translation(FilterType::Translation);

    DrawGraph graph_drawer;
    while (true) {
        if (!armors_is_busy) {
            armors_is_busy = true;
            Result *data;
            data = rbt.getVisibleArmors();
            if (data != nullptr) {
                //添加噪声
                Result noised_data;
                noised_data.armorCenter.x = data->armorCenter.x + generateGaussianNoise(0, 1);
                noised_data.armorCenter.y = data->armorCenter.y + generateGaussianNoise(0, 1);
                noised_data.yaw = data->yaw + generateGaussianNoise(0, 0.5);

                //旋转运动滤波
                Eigen::Matrix<double, 1, 1> measurement_rota;
                measurement_rota << noised_data.armorCenter.x;
                auto AfterMeasurement = kf_rotation.update(measurement_rota);
                noised_data.yaw = (*AfterMeasurement)(0);
                std::cout << "AfterMeasurement: " << (*AfterMeasurement)(0) << std::endl;

                graph_drawer.addPoint("Real yaw", data->yaw);
                graph_drawer.addPoint("Mearsure yaw", noised_data.yaw);

            }

            armors_is_busy = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            graph_drawer.showAll();
        }

    }

    graph_drawer.destoryScreen("test");

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

void test(bool);

int main(int argc, char **argv) {
    test(false);
    srand(time(nullptr));
    Robot rbt(200, 0, 0, 0, 0, 0.1);
    std::thread visual_thread(visualize, std::ref(rbt));
    std::thread filter_thread(filter_, std::ref(rbt));
    //std::thread keyboard_thread(Keyboard, std::ref(rbt));

    //if (keyboard_thread.joinable()) {
    //    keyboard_thread.join();
    //}
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

double generateGaussianNoise(double mean, double stddev) {
    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());

    // 创建正态分布
    std::normal_distribution<> d(mean, stddev);

    // 返回生成的正态分布随机数
    return d(gen);
}

void test(bool isStop = true) {
    std::vector<double> testvec{1,2,3,4,5,6,7};
    genNoise<std::vector<double>>(testvec,NoiseType::Gaussian,1);
    for (auto &i:testvec) {
        std::cout << i << std::endl;
    }
    KF_filter kf_rotation(FilterType::Rotation);
    while(isStop);
}


