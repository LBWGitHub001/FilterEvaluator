//
// Created by lbw on 24-11-16.
//

#ifndef FILTER_MUTUXS_H
#define FILTER_MUTUXS_H
#include <mutex>
#include <condition_variable>
#include <atomic>

extern std::mutex armors_mutex;
extern std::condition_variable armors_cv;
extern std::atomic<bool> armors_is_busy;
#endif //FILTER_MUTUXS_H
