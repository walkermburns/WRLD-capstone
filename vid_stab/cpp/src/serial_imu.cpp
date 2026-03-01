#include "serial_imu.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

struct SerialImuReader::ThreadState {
  std::thread th;
};

static speed_t baud_to_termios(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

SerialImuReader::SerialImuReader(std::string device, int baud)
    : device_(std::move(device)), baud_(baud) {}

SerialImuReader::~SerialImuReader() { stop(); }

bool SerialImuReader::open_port() {
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "open(" << device_ << ") failed: " << std::strerror(errno) << "\n";
    return false;
  }
  return true;
}

void SerialImuReader::close_port() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialImuReader::configure_port() {
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "tcgetattr failed: " << std::strerror(errno) << "\n";
    return false;
  }

  cfmakeraw(&tty);

  speed_t spd = baud_to_termios(baud_);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1; // 0.1s

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "tcsetattr failed: " << std::strerror(errno) << "\n";
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

bool SerialImuReader::start() {
  if (running_.load()) return true;
  if (!open_port()) return false;
  if (!configure_port()) {
    close_port();
    return false;
  }

  running_.store(true);
  thread_state_ = new ThreadState();
  thread_state_->th = std::thread([this]() { thread_fn(); });
  return true;
}

void SerialImuReader::stop() {
  if (!running_.load()) return;
  running_.store(false);
  if (thread_state_) {
    if (thread_state_->th.joinable()) thread_state_->th.join();
    delete thread_state_;
    thread_state_ = nullptr;
  }
  close_port();
}

size_t SerialImuReader::size() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return buf_.size();
}

std::optional<ImuSample> SerialImuReader::latest() const {
  std::lock_guard<std::mutex> lk(mtx_);
  if (buf_.empty()) return std::nullopt;
  return buf_.back();
}

// CSV: roll,pitch,yaw,w,x,y,z
bool SerialImuReader::parse_line(const std::string& line, Eigen::Quaternionf& q_out) const {
  float vals[7];
  int idx = 0;
  std::string token;
  std::stringstream ss(line);
  while (std::getline(ss, token, ',')) {
    if (idx >= 7) break;
    try {
      vals[idx] = std::stof(token);
    } catch (...) {
      return false;
    }
    idx++;
  }
  if (idx < 7) return false;

  float w = vals[3], x = vals[4], y = vals[5], z = vals[6];

  if (use_conjugate_.load()) {
    x = -x; y = -y; z = -z;
  }

  q_out = Eigen::Quaternionf(w, x, y, z);
  q_out.normalize();
  return true;
}

void SerialImuReader::thread_fn() {
  std::string accum;
  accum.reserve(4096);

  while (running_.load()) {
    char buf[512];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      std::cerr << "read error: " << std::strerror(errno) << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    accum.append(buf, buf + n);

    size_t pos = 0;
    while (true) {
      size_t nl = accum.find('\n', pos);
      if (nl == std::string::npos) {
        accum.erase(0, pos);
        break;
      }

      std::string line = accum.substr(pos, nl - pos);
      pos = nl + 1;
      if (!line.empty() && line.back() == '\r') line.pop_back();

      Eigen::Quaternionf q;
      if (!parse_line(line, q)) continue;

      ImuSample s;
      s.t = std::chrono::steady_clock::now();
      s.q = q;

      {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_.push_back(s);
        if (buf_.size() > max_samples_) {
          buf_.erase(buf_.begin(), buf_.begin() + (buf_.size() - max_samples_));
        }
      }
    }
  }
}

std::optional<Eigen::Quaternionf> SerialImuReader::quat_at(std::chrono::steady_clock::time_point t_query) const {
  std::lock_guard<std::mutex> lk(mtx_);
  if (buf_.size() < 2) return std::nullopt;

  if (t_query < buf_.front().t || t_query > buf_.back().t) return std::nullopt;

  size_t hi = 0;
  while (hi < buf_.size() && buf_[hi].t < t_query) hi++;
  if (hi == 0) return buf_[0].q;
  if (hi >= buf_.size()) return buf_.back().q;

  const auto& a = buf_[hi - 1];
  const auto& b = buf_[hi];

  const double dt = std::chrono::duration<double>(b.t - a.t).count();
  if (dt <= 1e-9) return b.q;

  const double tq = std::chrono::duration<double>(t_query - a.t).count();
  float u = static_cast<float>(tq / dt);
  if (u < 0.f) u = 0.f;
  if (u > 1.f) u = 1.f;

  Eigen::Quaternionf q = a.q.slerp(u, b.q);
  q.normalize();
  return q;
}