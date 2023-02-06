//
// Created by 欧阳亚坤 on 2021/9/15.
//
#pragma once
#include <chrono>

namespace liom_local_planner {

using namespace std::chrono;

inline double GetCurrentTimestamp() {
  return ((double) duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count() / 1000);
}


}