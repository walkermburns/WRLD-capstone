#include "serial_imu.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <vector>

static bool set_serial_attribs(int fd, int baud) {
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) return false;

  cfmakeraw(&tty);

  speed_t speed = B115200;
  if (baud == 57600) speed = B57600;
  if (baud == 9600)  speed = B9600;

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1; // 100 ms

  if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
  return true;
}

SerialImuReader::SerialImuReader(const std::string& port, int baud)
    : port_(port), baud_(baud) {}

SerialImuReader::~SerialImuReader() {
  stop();
}

bool SerialImuReader::start() {
  stop_ = false;

  // Size your buffer for ~6 seconds at 100 Hz by default; you can adjust later.
  // (You can also expose this as a setter.)
  max_len_ = 600;

  th_ = std::thread([this]() { thread_fn(); });
  return true;
}

void SerialImuReader::stop() {
  stop_ = true;
  if (th_.joinable()) th_.join();
}

size_t SerialImuReader::size() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return buf_.size();
}

bool SerialImuReader::latest_two(Sample& s1, Sample& s2) const {
  std::lock_guard<std::mutex> lk(mtx_);
  if (buf_.size() < 2) return false;
  s1 = buf_[buf_.size() - 2];
  s2 = buf_[buf_.size() - 1];
  return true;
}

std::optional<Eigen::Quaternionf> SerialImuReader::quat_at(
    const std::chrono::steady_clock::time_point& t_query) const {

  std::lock_guard<std::mutex> lk(mtx_);
  if (buf_.size() < 2) return std::nullopt;

  if (t_query < buf_.front().t || t_query > buf_.back().t) return std::nullopt;

  // Find first idx with t >= query
  size_t hi = 0;
  while (hi < buf_.size() && buf_[hi].t < t_query) hi++;

  if (hi == 0) return buf_.front().q;
  if (hi >= buf_.size()) return buf_.back().q;

  const auto& a = buf_[hi - 1];
  const auto& b = buf_[hi];

  const double ta = std::chrono::duration<double>(a.t.time_since_epoch()).count();
  const double tb = std::chrono::duration<double>(b.t.time_since_epoch()).count();
  const double tq = std::chrono::duration<double>(t_query.time_since_epoch()).count();

  const double denom = std::max(tb - ta, 1e-9);
  const double u = std::clamp((tq - ta) / denom, 0.0, 1.0);

  Eigen::Quaternionf qa = a.q;
  Eigen::Quaternionf qb = b.q;
  qa.normalize();
  qb.normalize();

  Eigen::Quaternionf q = qa.slerp(float(u), qb);
  q.normalize();
  return q;
}

bool SerialImuReader::parse_line_to_quat(const std::string& line, Eigen::Quaternionf& q_out) const {
  // Expect: roll_deg,pitch_deg,yaw_deg,w,x,y,z
  // We use ONLY w,x,y,z (float). Arduino prints in that order.
  std::vector<float> vals;
  vals.reserve(7);

  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    try {
      vals.push_back(std::stof(item));
    } catch (...) {
      return false;
    }
  }
  if (vals.size() < 7) return false;

  float w = vals[3];
  float x = vals[4];
  float y = vals[5];
  float z = vals[6];

  if (use_conjugate_) {
    x = -x; y = -y; z = -z;
  }

  Eigen::Quaternionf q(w, x, y, z);
  q.normalize();
  q_out = q;
  return true;
}

void SerialImuReader::thread_fn() {
  int fd = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    std::cerr << "[imu] Failed to open " << port_ << "\n";
    return;
  }
  if (!set_serial_attribs(fd, baud_)) {
    std::cerr << "[imu] Failed to configure serial\n";
    close(fd);
    return;
  }

  std::string line;
  line.reserve(256);

  char buf[256];

  while (!stop_) {
    int n = read(fd, buf, sizeof(buf));
    if (n <= 0) continue;

    for (int i = 0; i < n; i++) {
      char c = buf[i];
      if (c == '\n') {
        Eigen::Quaternionf q;
        if (parse_line_to_quat(line, q)) {
          Sample s;
          s.t = std::chrono::steady_clock::now();
          s.q = q;

          std::lock_guard<std::mutex> lk(mtx_);
          buf_.push_back(s);
          if (buf_.size() > max_len_) buf_.pop_front();
        }
        line.clear();
      } else if (c != '\r') {
        line.push_back(c);
      }
    }
  }

  close(fd);
}