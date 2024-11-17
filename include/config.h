//
// Created by lbw on 24-11-17.
//

#ifndef FILTER_CONFIG_H
#define FILTER_CONFIG_H
#include <vector>
enum class NoiseType {
    Gaussian,
    SaltAndPepper,
    Poisson
};

enum class FilterType {
    Rotation,
    Translation,
};
#endif //FILTER_CONFIG_H
