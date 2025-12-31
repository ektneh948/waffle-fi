#include <rclcpp/rclcpp.hpp>
#include <sqlite3.h>
#include <string>
#include <filesystem>

#include "wifi_interface/msg/wifi_fused.hpp"

class WifiDbNode : public rclcpp::Node
{
public:
  WifiDbNode() : Node("wifi_db_node")
  {
    device_info_ = "turtlebot3";
    db_path_ = "/home/ros/turtlebot3_ws/data/my_wifi.db";

    // ✅ DB 디렉토리 보장
    std::filesystem::create_directories(
      std::filesystem::path(db_path_).parent_path()
    );

    if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK) {
      RCLCPP_FATAL(get_logger(), "Cannot open DB: %s", sqlite3_errmsg(db_));
      rclcpp::shutdown();
      return;
    }

    createTable();

    sub_ = create_subscription<wifi_interface::msg::WifiFused>(
      "/wifi/fused",
      10,
      std::bind(&WifiDbNode::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "wifi_db_node started, DB=%s", db_path_.c_str());
  }

  ~WifiDbNode()
  {
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

private:
  void createTable()
  {
    const char *sql =
      "CREATE TABLE IF NOT EXISTS wifi_data ("
      "id INTEGER PRIMARY KEY AUTOINCREMENT,"
      "device_info TEXT NOT NULL,"
      "ssid TEXT NOT NULL,"
      "grid_id_x REAL NOT NULL,"
      "grid_id_y REAL NOT NULL,"
      "rssi_value REAL NOT NULL,"
      "timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ', 'now'))"
      ");";

    char *err_msg = nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
      RCLCPP_ERROR(get_logger(),
        "Create table error: %s", err_msg);
      sqlite3_free(err_msg);
    }
  }

  void callback(const wifi_interface::msg::WifiFused::SharedPtr msg)
  {
    // ✅ 의미 없는 데이터 차단 (선택적이지만 강력 추천)
    if (msg->ssid.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip DB insert: empty SSID (rssi=%.1f)", msg->rssi
      );
      return;
    }

    const char *sql =
      "INSERT INTO wifi_data "
      "(device_info, ssid, grid_id_x, grid_id_y, rssi_value) "
      "VALUES (?, ?, ?, ?, ?);";

    sqlite3_stmt *stmt = nullptr;

    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      RCLCPP_ERROR(get_logger(),
        "Prepare failed: %s", sqlite3_errmsg(db_));
      return;
    }

    sqlite3_bind_text(
      stmt, 1, device_info_.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(
      stmt, 2, msg->ssid.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 3, msg->x);
    sqlite3_bind_double(stmt, 4, msg->y);
    sqlite3_bind_double(stmt, 5, msg->rssi);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      RCLCPP_ERROR(get_logger(),
        "Insert failed: %s", sqlite3_errmsg(db_));
    } else {
      // ✅ 실제 저장 로그 (디버깅용)
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "DB insert: ssid=%s x=%.2f y=%.2f rssi=%.1f",
        msg->ssid.c_str(), msg->x, msg->y, msg->rssi
      );
    }

    sqlite3_finalize(stmt);
  }

private:
  sqlite3 *db_{nullptr};
  std::string db_path_;
  std::string device_info_;
  rclcpp::Subscription<wifi_interface::msg::WifiFused>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiDbNode>());
  rclcpp::shutdown();
  return 0;
}
