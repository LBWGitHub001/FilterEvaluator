//
// Created by lbw on 24-11-17.
//

#include "KF_filter.h"

KF_filter::KF_filter(const FilterType filterType) : FilterManager(), filterType(filterType) {
    _timestamp = get_timestamp();
    if (filterType == FilterType::Translation) {
        StateSize = CA_Tran_Kalman;
        MeasurementSize = CZ_Tran_Kalman;
        H.resize(MeasurementSize, StateSize);
        H << 1, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0;
        A = [this]() {
            Eigen::MatrixXd A_(StateSize, StateSize);
            A_ << 1, _dt, 0.5 * _dt * _dt, 0, 0, 0,
                    0, 1, _dt, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 1, _dt, 0.5 * _dt * _dt,
                    0, 0, 0, 0, 1, _dt,
                    0, 0, 0, 0, 0, 1;
            return A_;
        };
        auto Q = [this]() {
            Eigen::MatrixXd U(MeasurementSize, MeasurementSize);
            Eigen::MatrixXd G(StateSize, MeasurementSize);
            U << s2qx, 0,
                    0, s2qy;
            G << pow(_dt, 2) / 2, 0,
                    0, _dt,
                    pow(_dt, 2) / 2, 0,
                    0, _dt,
                    pow(_dt, 2) / 2, 0,
                    0, _dt;
            return G * U * G.transpose();
        };
        auto R = [this](const Eigen::Matrix<double, CZ_Tran_Kalman, 1> &z) {
            Eigen::MatrixXd R_(MeasurementSize, MeasurementSize);
            R_ << r_x_ * std::abs(z[0]), 0,
                    0, r_y_ * std::abs(z[1]);
            return R_;
        };
        P = Eigen::MatrixXd::Identity(StateSize, StateSize);

        kf = new Translation(A(), H, P, Q, R);
    } else if (filterType == FilterType::Rotation) {
        StateSize = CA_Rota_Kalman;
        MeasurementSize = CZ_Rota_Kalman;
        H.resize(MeasurementSize, StateSize);
        H << 1, 0, 0;
        A = [this]() {
            Eigen::MatrixXd A_(StateSize, StateSize);
            A_ << 1, _dt, 0.5 * _dt * _dt,
                    0, 1, _dt,
                    0, 0, 1;
            return A_;
        };
        auto Q = [this]() {
            Eigen::MatrixXd U(MeasurementSize, MeasurementSize);
            Eigen::MatrixXd G(StateSize, MeasurementSize);
            U << s2qyaw;
            G << _dt, _dt, _dt;
            Eigen::MatrixXd ans(MeasurementSize,MeasurementSize);

            ans = G * U * G.transpose();
            /*std::cout << "ans:" << std::endl;
            std::cout << ans(0) << " " << ans(1) << " " << ans(2) << std::endl;
            std::cout << ans(3) << " " << ans(4) << " " << ans(5) << std::endl;
            std::cout << ans(6) << " " << ans(7) << " " << ans(8) << std::endl;*/
            return ans;
        };
        /*auto a = Q();
        std::cout << "A:" << std::endl;
        std::cout << a(0) << " " << a(1) << " " << a(2) << std::endl;
        std::cout << a(3) << " " << a(4) << " " << a(5) << std::endl;
        std::cout << a(6) << " " << a(7) << " " << a(8) << std::endl;*/
        auto R = [this](const Eigen::Matrix<double, CZ_Rota_Kalman, 1> &z) {
            Eigen::MatrixXd R_(MeasurementSize, MeasurementSize);
            R_ << r_yaw_ * std::abs(z[0]);
            return R_;
        };
        P = Eigen::MatrixXd::Identity(StateSize, StateSize);

        kf = new Rotation(A(), H, P, Q, R);
    }

    s2qx = 0.1;
    s2qy = 0.1;
    s2qyaw = 0.1;
    r_x_ = 0.1;
    r_y_ = 0.1;
    r_yaw_ = 0.1;

    _timestamp = get_timestamp();
}

void KF_filter::init() {

}


std::shared_ptr<Eigen::MatrixXd> KF_filter::update(const Eigen::VectorXd &data) {
    auto timestamp_now = get_timestamp();
    _dt = timestamp_now - _timestamp;
    std::shared_ptr<Eigen::MatrixXd> state;
    if (filterType == FilterType::Rotation) {
        auto filter = static_cast<Rotation *>(kf);

        filter->predict();
        state = std::make_shared<Eigen::MatrixXd>(filter->update(data));
    } else if (filterType == FilterType::Translation){
        auto filter = static_cast<Translation *>(kf);
        filter->predict();
        state = std::make_shared<Eigen::MatrixXd>(filter->update(data));
    }

    _timestamp = get_timestamp();
    return state;
}


time_t KF_filter::get_timestamp() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

void *KF_filter::getFilter() {
    assert(kf == nullptr);
    return kf;
}

KF_filter::~KF_filter() {

}






