#pragma once
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

struct ImuSample {
  std::chrono::steady_clock::time_point t;
  Eigen::Quaternionf q; // (w,x,y,z)
};

class SerialImuReader {
public:
  SerialImuReader(std::string device, int baud);
  ~SerialImuReader();

  bool start();
  void stop();

  // Query quaternion at time using SLERP; nullopt if out of range.
  std::optional<Eigen::Quaternionf> quat_at(std::chrono::steady_clock::time_point t_query) const;

  // Latest sample
  std::optional<ImuSample> latest() const;

  size_t size() const;

  void set_use_conjugate(bool v) { use_conjugate_.store(v); }

private:
  void thread_fn();
  bool open_port();
  void close_port();
  bool configure_port();
  bool parse_line(const std::string& line, Eigen::Quaternionf& q_out) const;

  std::string device_;
  int baud_;
  int fd_ = -1;

  mutable std::mutex mtx_;
  std::vector<ImuSample> buf_;
  size_t max_samples_ = 600; // ~6s at 100 Hz

  std::atomic<bool> running_{false};
  std::atomic<bool> use_conjugate_{false};

  struct ThreadState;
  ThreadState* thread_state_ = nullptr;
};