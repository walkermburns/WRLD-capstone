#pragma once
#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

class SerialImuReader {
public:
  struct Sample {
    std::chrono::steady_clock::time_point t;
    Eigen::Quaternionf q; // (w,x,y,z) in Eigen storage
  };

  SerialImuReader(const std::string& port, int baud);
  ~SerialImuReader();

  void set_use_conjugate(bool v) { use_conjugate_ = v; }

  bool start();
  void stop();

  size_t size() const;

  // Interpolated quaternion at time t (steady_clock domain)
  std::optional<Eigen::Quaternionf> quat_at(const std::chrono::steady_clock::time_point& t_query) const;

  // NEW: get the last two samples (for omega prediction)
  bool latest_two(Sample& s1, Sample& s2) const;

private:
  void thread_fn();

  bool parse_line_to_quat(const std::string& line, Eigen::Quaternionf& q_out) const;

private:
  std::string port_;
  int baud_ = 115200;
  bool use_conjugate_ = false;

  std::atomic<bool> stop_{false};
  std::thread th_;

  mutable std::mutex mtx_;
  std::deque<Sample> buf_;
  size_t max_len_ = 600; // set in start()
};