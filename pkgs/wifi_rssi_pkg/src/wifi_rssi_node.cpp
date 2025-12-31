#include <chrono>
#include <string>
#include <array>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <limits>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "wifi_interface/msg/wifi_rssi.hpp"

using namespace std::chrono_literals;

static constexpr const char* SERIAL_PORT = "/dev/serial0";
static constexpr int SERIAL_BAUD = 115200;
static constexpr const char* TOPIC_WIFI_RAW = "/wifi/raw";
static constexpr const char* DRIVER_PATH = "/dev/rssi_driver_table_test";

static constexpr int TIMER_PERIOD_MS = 20;
static constexpr size_t READ_CHUNK = 4096;
static constexpr size_t MAX_BUFFER_BYTES = 300000;

static speed_t to_speed_t(int baud)
{
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

class WifiRssiNode : public rclcpp::Node
{
public:
  WifiRssiNode()
  : Node("wifi_rssi_node"), serial_fd_(-1), drv_fd_(-1)
  {
    pub_ = create_publisher<wifi_interface::msg::WifiRssi>(TOPIC_WIFI_RAW, 10);

    open_serial();
    open_driver();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(TIMER_PERIOD_MS),
      std::bind(&WifiRssiNode::tick, this)
    );

    RCLCPP_INFO(get_logger(), "wifi_rssi_node started");
  }

  ~WifiRssiNode() override
  {
    if (serial_fd_ >= 0) ::close(serial_fd_);
    if (drv_fd_ >= 0) ::close(drv_fd_);
  }

private:
  void open_serial()
  {
    if (serial_fd_ >= 0) return;

    serial_fd_ = ::open(SERIAL_PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "serial open failed: %s", std::strerror(errno));
      return;
    }

    struct termios tio{};
    tcgetattr(serial_fd_, &tio);
    cfmakeraw(&tio);

    speed_t spd = to_speed_t(SERIAL_BAUD);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    tcsetattr(serial_fd_, TCSANOW, &tio);
  }

  void open_driver()
  {
    drv_fd_ = ::open(DRIVER_PATH, O_WRONLY | O_NONBLOCK);
  }

  void tick()
  {
    std::array<char, READ_CHUNK> buf{};
    ssize_t n = ::read(serial_fd_, buf.data(), buf.size() - 1);

    if (n <= 0) return;

    buf[n] = '\0';
    rx_acc_ += buf.data();

    if (rx_acc_.size() > MAX_BUFFER_BYTES) {
      rx_acc_.erase(0, rx_acc_.size() / 2);
    }

    publish_complete_frames();
  }

  void publish_complete_frames()
  {
    while (true) {
      size_t b = rx_acc_.find("BEGIN\n");
      size_t e = rx_acc_.find("END\n");

      if (b == std::string::npos || e == std::string::npos || e < b)
        return;

      size_t end_pos = e + 4;
      std::string frame = rx_acc_.substr(b + 6, e - (b + 6));
      rx_acc_.erase(0, end_pos);

      parse_and_publish(frame);
    }
  }

  void parse_and_publish(const std::string& frame)
  {
    std::istringstream iss(frame);
    std::string line;

    int best_rssi = std::numeric_limits<int>::min();
    std::string best_ssid = "UNKNOWN";

    while (std::getline(iss, line)) {
      if (line.empty()) continue;

      // 기대 포맷: SSID:xxx,RSSI:-45
      size_t ssid_pos = line.find("SSID:");
      size_t rssi_pos = line.find(",RSSI:");

      if (ssid_pos == std::string::npos || rssi_pos == std::string::npos)
        continue;

      std::string ssid = line.substr(
        ssid_pos + 5,
        rssi_pos - (ssid_pos + 5)
      );

      int rssi = std::stoi(
        line.substr(rssi_pos + 6)
      );

      if (rssi > best_rssi) {
        best_rssi = rssi;
        best_ssid = ssid;
      }
    }

    wifi_interface::msg::WifiRssi msg;
    msg.header.stamp = now();
    msg.header.frame_id = "wifi";
    msg.ssid = best_ssid;
    msg.rssi = best_rssi;

    pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "WiFi strongest: %s (%d dBm)",
      best_ssid.c_str(), best_rssi
    );
  }

private:
  int serial_fd_;
  int drv_fd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<wifi_interface::msg::WifiRssi>::SharedPtr pub_;
  std::string rx_acc_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiRssiNode>());
  rclcpp::shutdown();
  return 0;
}
