//
// Created by lbw on 24-11-14.
//

#ifndef NEW_FILTER_KALMAN_H
#define NEW_FILTER_KALMAN_H
#include <cmath>
#include <Eigen/Dense>
#include <functional>

template<int N_X, int N_Z>
class KalmanFilter {
public:
    KalmanFilter() = default;

    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
    using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
    using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;
    using Matrix11 = Eigen::Matrix<double, 1, 1>;

    using UpdateQFunc = std::function<MatrixXX()>;
    using UpdateRFunc = std::function<MatrixZZ(const MatrixZ1 &z)>;

    explicit KalmanFilter(const MatrixXX &A,
                          const MatrixZX &H,
                          const MatrixXX &P,
                          const UpdateQFunc &u_q,
                          const UpdateRFunc &u_r) noexcept
            : A(A), H(H), P(P), update_Q(u_q), update_R(u_r) {

    }

    // Set the initial state
    void setState(const MatrixX1 &x0) noexcept { x = x0; }

    void Qset(MatrixXX Q) noexcept { this->Q = Q; }

    void Rset(MatrixZZ R) noexcept { this->R = R; }

    void Aset(const MatrixXX A) noexcept { this->A = A; }

    void Hset(const MatrixZX H) noexcept { this->H = H; }

    void Pset(const MatrixXX P) noexcept { this->P = P; }

    // Compute a predicted state
    MatrixX1 predict() noexcept {
        x = A * x;
        Q = update_Q();
        P = A * P * A.transpose() + Q;
        return x;
    }

    // Update the estimated state based on measurement
    MatrixX1 update(const MatrixZ1 &z) noexcept {
        MatrixXZ K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
        x = x + K * (z - H * x);
        MatrixXX Ind;Ind.setIdentity();
        P = (Ind - K * H) * P;
        R = update_R(z);
        return x;
    }

private:
    //温馨提示,匀加速和匀速模型的维数不一样哦

    MatrixXX A;              //模型
    MatrixZX H;    //测量模型

    MatrixXX P;              //预测变量的协方差

    MatrixXX Q;              //过程噪声协方差
    MatrixZZ R;    //观测噪声协方差


    MatrixX1 x;              //系统参数

    //噪声计算函数
    UpdateRFunc update_R;
    UpdateQFunc update_Q;
};


#endif //NEW_FILTER_KALMAN_H
