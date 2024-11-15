//
// Created by lbw on 24-11-14.
//
/**
 * MLS自适应滤波器
 * */
#ifndef NEW_FILTER_LMS_H
#define NEW_FILTER_LMS_H

#include <cmath>
#include <Eigen/Dense>
#include <functional>

template<int L>
class LMS {
    using MatL1 = Eigen::Matrix<double, L, 1>;
    using MatLL = Eigen::Matrix<double, L, L>;
    using Mat11 = Eigen::Matrix<double, 1, 1>;

public:
    LMS() = default;

    ~LMS() = default;

    explicit LMS(const MatL1 &x, const MatL1 &d, const MatLL &W, double mu) : x(x), d(d), W(W), mu(mu) {

    }

    void setInput(const MatL1 &x) { this->x = x; }

    void setDesired(const MatL1 &d) { this->d = d; }

    void setWeight(const MatLL &W) { this->W = W; }

    MatL1& predict() {//预测输出
        y = W.transpose() * x;
        return y;
    }

    MatL1& update() {//更新滤波器系数
        e = d - y;
        W += mu * e * x.transpose();
        return y;
    }

    MatL1& update(const MatL1 d){//更新期望信号
        this->d = d;
        return update();
    }

    MatL1& infer(const Mat11& x_new){//更新输入信号
        //x移位并添加新元素
        x.block(0,0,x.rows()-1,x.cols()) = x.block(1,0,x.rows()-1,x.cols());
        x.block(x.rows()-1,0,1,1) = x;
        update();
        return predict();
    }

private:
    MatL1 x;    //输入信号
    MatL1 y;    //输出信号
    MatL1 d;    //期望信号
    MatL1 e;    //误差信号
    MatLL W;    //滤波器系数
    double mu;  //步长

};


#endif //NEW_FILTER_LMS_H
