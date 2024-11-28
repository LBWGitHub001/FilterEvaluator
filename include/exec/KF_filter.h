//
// Created by lbw on 24-11-17.
//

#ifndef FILTER_KF_FILTER_H
#define FILTER_KF_FILTER_H
#include "FilterManager.h"
constexpr int CA_Tran_Kalman = 6;
constexpr int CZ_Tran_Kalman = 2;
constexpr int CA_Rota_Kalman = 3;
constexpr int CZ_Rota_Kalman = 1;

using Rotation = KalmanFilter<CA_Rota_Kalman,CZ_Rota_Kalman>;
using Translation = KalmanFilter<CA_Tran_Kalman,CZ_Tran_Kalman>;

class KF_filter :
    public FilterManager{
public:
    KF_filter(const FilterType filterType);
    ~KF_filter();

    void init(Eigen::MatrixXd &state) override;
    std::shared_ptr<Eigen::MatrixXd> update(const Eigen::VectorXd &data) override;
    void* getFilter() override;

    void setQ(double s2qx, double s2qy, double s2qyaw, double r_x, double r_y, double r_yaw);


private:
    //滤波器
    void* kf;
    //基础参数
    FilterType filterType;
    int StateSize;
    int MeasurementSize;

    //系统参数
    time_t _timestamp;
    double _dt;
    double s2qx;
    double s2qy;
    double s2qyaw;
    double r_x_;
    double r_y_;
    double r_yaw_;

    //滤波器参数
    std::function<Eigen::MatrixXd(void)> A;
    Eigen::MatrixXd H;
    Eigen::MatrixXd P;
    //std::function<Eigen::MatrixXd(void)> Q;
    //std::function<Eigen::MatrixXd(const MatrixZ1 &z)> R;

    //功能函数
    time_t get_timestamp();
};


#endif //FILTER_KF_FILTER_H
