/**
 * @file tic_toc.hpp
 * @author NLC (qhuangag@connect.ust.hk)
 * @brief Adopted from VINS
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>

constexpr int US_TO_MS = 1000;

/**
 * @brief Time measurment tool
 *
 */
class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end - start;
    return elapsedSeconds.count() * US_TO_MS;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
