#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "wifi_interface/msg/wifi_rssi.hpp"
#include "wifi_interface/msg/wifi_fused.hpp"

class WifiFusionNode : public rclcpp::Node
{
public:
  WifiFusionNode()
  : Node("fusion_node")
  {
    sub_wifi_ = create_subscription<wifi_interface::msg::WifiRssi>(
      "/wifi/raw",
      10,
      std::bind(&WifiFusionNode::on_wifi, this, std::placeholders::_1)
    );

    sub_pose_ = create_subscription<geometry_msgs::msg::Point>(
      "/grid_pose",
      10,
      std::bind(&WifiFusionNode::on_pose, this, std::placeholders::_1)
    );

    pub_fused_ = create_publisher<wifi_interface::msg::WifiFused>(
      "/wifi/fused",
      10
    );

    RCLCPP_INFO(get_logger(), "fusion_node started");
  }

private:
  void on_wifi(const wifi_interface::msg::WifiRssi::SharedPtr msg)
  {
    last_wifi_ = *msg;
  }

  void on_pose(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (!last_wifi_.has_value())
      return;

    double x = msg->x;
    double y = msg->y;

    // 위치가 안 바뀌면 skip (중복 방지)
    if (last_pos_.has_value() &&
        last_pos_->first == x &&
        last_pos_->second == y)
    {
      return;
    }

    last_pos_ = {x, y};

    wifi_interface::msg::WifiFused fused;
    fused.header.stamp = now();
    fused.header.frame_id = "map";   // amcl 기준
    fused.x = x;
    fused.y = y;
    fused.ssid = last_wifi_->ssid;
    fused.rssi = last_wifi_->rssi;

    pub_fused_->publish(fused);

    RCLCPP_INFO(
      get_logger(),
      "FUSED (x=%.2f, y=%.2f) SSID=%s RSSI=%d",
      x, y,
      fused.ssid.c_str(),
      fused.rssi
    );
  }

  // members
  rclcpp::Subscription<wifi_interface::msg::WifiRssi>::SharedPtr sub_wifi_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_pose_;
  rclcpp::Publisher<wifi_interface::msg::WifiFused>::SharedPtr pub_fused_;

  std::optional<wifi_interface::msg::WifiRssi> last_wifi_;
  std::optional<std::pair<double, double>> last_pos_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiFusionNode>());
  rclcpp::shutdown();
  return 0;
}
