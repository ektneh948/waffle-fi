#include <rclcpp/rclcpp.hpp>
#include <sqlite3.h>
#include <string>
#include <filesystem>
#include <chrono>
#include <sstream>

#include "wifi_interface/msg/wifi_fused.hpp"
#include "wifi_interface/srv/get_heatmap.hpp"
#include "std_srvs/srv/trigger.hpp"

class WifiDbNode : public rclcpp::Node
{
public:
  WifiDbNode() : Node("wifi_db_node")
  {
    device_info_ = "turtlebot3";
    db_path_ = "/home/ros/turtlebot3_ws/data/my_wifi.db";

    std::filesystem::create_directories(std::filesystem::path(db_path_).parent_path());

    if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK) {
      RCLCPP_FATAL(get_logger(), "Cannot open DB: %s", sqlite3_errmsg(db_));
      rclcpp::shutdown();
      return;
    }

    createOrMigrateSchema();

    // /wifi/fused subscribe (DB insert)
    sub_ = create_subscription<wifi_interface::msg::WifiFused>(
      "/wifi/fused", 10,
      std::bind(&WifiDbNode::onFused, this, std::placeholders::_1)
    );

    // 서비스: 세션 시작/종료
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/db/start_session",
      std::bind(&WifiDbNode::onStartSession, this, std::placeholders::_1, std::placeholders::_2)
    );

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/db/stop_session",
      std::bind(&WifiDbNode::onStopSession, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 서비스: 히트맵 조회
    heat_srv_ = create_service<wifi_interface::srv::GetHeatmap>(
      "/db/get_heatmap",
      std::bind(&WifiDbNode::onGetHeatmap, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(get_logger(), "wifi_db_node started, DB=%s", db_path_.c_str());
    RCLCPP_INFO(get_logger(), "Services: /db/start_session, /db/stop_session, /db/get_heatmap");
  }

  ~WifiDbNode()
  {
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

private:
  // =========================
  // Schema / Migration
  // =========================
  void execSql(const char* sql)
  {
    char* err=nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &err) != SQLITE_OK) {
      RCLCPP_ERROR(get_logger(), "SQL error: %s", err ? err : "(null)");
      sqlite3_free(err);
    }
  }

  bool columnExists(const std::string& table, const std::string& col)
  {
    std::string q = "PRAGMA table_info(" + table + ");";
    sqlite3_stmt* stmt=nullptr;
    if (sqlite3_prepare_v2(db_, q.c_str(), -1, &stmt, nullptr) != SQLITE_OK) return false;

    bool ok=false;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* name = sqlite3_column_text(stmt, 1);
      if (name && col == reinterpret_cast<const char*>(name)) { ok=true; break; }
    }
    sqlite3_finalize(stmt);
    return ok;
  }

  void createOrMigrateSchema()
  {
    // 기존 테이블(없으면 생성)
    const char *create_wifi =
      "CREATE TABLE IF NOT EXISTS wifi_data ("
      "id INTEGER PRIMARY KEY AUTOINCREMENT,"
      "device_info TEXT NOT NULL,"
      "ssid TEXT NOT NULL,"
      "grid_id_x REAL NOT NULL,"
      "grid_id_y REAL NOT NULL,"
      "rssi_value REAL NOT NULL,"
      "timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))"
      ");";
    execSql(create_wifi);

    // sessions 테이블 생성
    const char* create_sessions =
      "CREATE TABLE IF NOT EXISTS sessions ("
      "session_id TEXT PRIMARY KEY,"
      "device_info TEXT NOT NULL,"
      "started_at TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),"
      "ended_at TEXT"
      ");";
    execSql(create_sessions);

    // ✅ session_id 컬럼 추가(마이그레이션)
    if (!columnExists("wifi_data", "session_id")) {
      RCLCPP_WARN(get_logger(), "Migrating DB: add wifi_data.session_id ...");
      execSql("ALTER TABLE wifi_data ADD COLUMN session_id TEXT;");
      // 기존 데이터는 session_id가 NULL일 수 있으니 기본값 채우기(legacy)
      execSql("UPDATE wifi_data SET session_id='legacy' WHERE session_id IS NULL;");
    }

    // 인덱스(조회 성능)
    execSql("CREATE INDEX IF NOT EXISTS idx_wifi_session ON wifi_data(session_id);");
    execSql("CREATE INDEX IF NOT EXISTS idx_wifi_ssid ON wifi_data(ssid);");
    execSql("CREATE INDEX IF NOT EXISTS idx_wifi_session_ssid ON wifi_data(session_id, ssid);");
  }

  // =========================
  // Session Control
  // =========================
  static std::string nowCompact()
  {
    // YYYYMMDD_HHMMSS (UTC 기준이 아닌 로컬로 만들어도 무방; 여기선 단순)
    auto t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
      tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buf);
  }

  std::string makeSessionId()
  {
    return device_info_ + "_" + nowCompact();
  }

  void onStartSession(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // 새 세션 발급
    const std::string sid = makeSessionId();
    current_session_id_ = sid;
    logging_enabled_ = true;

    // sessions insert
    const char* sql = "INSERT INTO sessions(session_id, device_info) VALUES(?, ?);";
    sqlite3_stmt* stmt=nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      res->success = false;
      res->message = std::string("start_session failed: ") + sqlite3_errmsg(db_);
      return;
    }
    sqlite3_bind_text(stmt, 1, sid.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, device_info_.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      res->success = false;
      res->message = std::string("start_session insert failed: ") + sqlite3_errmsg(db_);
    } else {
      res->success = true;
      res->message = sid; // ✅ Qt가 이 값을 session_id로 사용
      RCLCPP_INFO(get_logger(), "Session started: %s", sid.c_str());
    }
    sqlite3_finalize(stmt);
  }

  void onStopSession(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (current_session_id_.empty()) {
      res->success = false;
      res->message = "No active session";
      return;
    }

    // ended_at 업데이트
    const char* sql = "UPDATE sessions SET ended_at=strftime('%Y-%m-%dT%H:%M:%SZ','now') "
                      "WHERE session_id=?;";
    sqlite3_stmt* stmt=nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      res->success = false;
      res->message = std::string("stop_session failed: ") + sqlite3_errmsg(db_);
      return;
    }
    sqlite3_bind_text(stmt, 1, current_session_id_.c_str(), -1, SQLITE_TRANSIENT);

    sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    RCLCPP_INFO(get_logger(), "Session stopped: %s", current_session_id_.c_str());
    res->success = true;
    res->message = current_session_id_;

    current_session_id_.clear();
    logging_enabled_ = false;
  }

  // =========================
  // /wifi/fused -> DB insert
  // =========================
  void onFused(const wifi_interface::msg::WifiFused::SharedPtr msg)
  {
    if (!msg) return;

    // session이 시작되지 않았으면 저장 안 함 (2번(세션) 요구사항의 핵심)
    if (!logging_enabled_ || current_session_id_.empty()) return;

    if (msg->ssid.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skip DB insert: empty SSID (rssi=%.1f)", msg->rssi);
      return;
    }

    const char *sql =
      "INSERT INTO wifi_data "
      "(device_info, ssid, grid_id_x, grid_id_y, rssi_value, session_id) "
      "VALUES (?, ?, ?, ?, ?, ?);";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      RCLCPP_ERROR(get_logger(), "Prepare failed: %s", sqlite3_errmsg(db_));
      return;
    }

    sqlite3_bind_text(stmt, 1, device_info_.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, msg->ssid.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 3, msg->x);
    sqlite3_bind_double(stmt, 4, msg->y);
    sqlite3_bind_double(stmt, 5, msg->rssi);
    sqlite3_bind_text(stmt, 6, current_session_id_.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      RCLCPP_ERROR(get_logger(), "Insert failed: %s", sqlite3_errmsg(db_));
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "DB insert[%s]: ssid=%s x=%.2f y=%.2f rssi=%.1f",
        current_session_id_.c_str(), msg->ssid.c_str(), msg->x, msg->y, msg->rssi);
    }
    sqlite3_finalize(stmt);
  }

  // =========================
  // /db/get_heatmap service
  // =========================
  void onGetHeatmap(
    const std::shared_ptr<wifi_interface::srv::GetHeatmap::Request> req,
    std::shared_ptr<wifi_interface::srv::GetHeatmap::Response> res)
  {
    res->ok = false;
    res->message.clear();
    res->xs.clear(); res->ys.clear(); res->rssis.clear();
    res->ssids.clear(); res->stamps.clear();

    std::string sql =
      "SELECT ssid, grid_id_x, grid_id_y, rssi_value, timestamp "
      "FROM wifi_data WHERE 1=1 ";

    const bool use_session = (!req->session_id.empty());
    const bool use_ssid    = (!req->ssid.empty() && req->ssid != "ALL");
    const bool use_thr     = req->thr_enable;

    if (use_session) sql += " AND session_id = ? ";
    if (use_ssid)    sql += " AND ssid = ? ";
    if (use_thr)     sql += " AND rssi_value >= ? ";

    sql += " ORDER BY id ASC LIMIT ? OFFSET ?;";

    sqlite3_stmt* stmt=nullptr;
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
      res->message = std::string("Prepare failed: ") + sqlite3_errmsg(db_);
      return;
    }

    int idx=1;
    if (use_session) sqlite3_bind_text(stmt, idx++, req->session_id.c_str(), -1, SQLITE_TRANSIENT);
    if (use_ssid)    sqlite3_bind_text(stmt, idx++, req->ssid.c_str(), -1, SQLITE_TRANSIENT);
    if (use_thr)     sqlite3_bind_int (stmt, idx++, req->thr_rssi);

    sqlite3_bind_int(stmt, idx++, (int)req->limit);
    sqlite3_bind_int(stmt, idx++, (int)req->offset);

    int count=0;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* ssid = sqlite3_column_text(stmt, 0);
      const double x = sqlite3_column_double(stmt, 1);
      const double y = sqlite3_column_double(stmt, 2);
      const int rssi = (int)std::lround(sqlite3_column_double(stmt, 3));
      const unsigned char* ts = sqlite3_column_text(stmt, 4);

      res->ssids.push_back(ssid ? (const char*)ssid : "");
      res->xs.push_back(x);
      res->ys.push_back(y);
      res->rssis.push_back(rssi);
      res->stamps.push_back(ts ? (const char*)ts : "");
      ++count;
    }
    sqlite3_finalize(stmt);

    res->ok = true;
    res->message = "rows=" + std::to_string(count);
  }

private:
  sqlite3 *db_{nullptr};
  std::string db_path_;
  std::string device_info_;

  // session state
  std::string current_session_id_;
  bool logging_enabled_{false};

  rclcpp::Subscription<wifi_interface::msg::WifiFused>::SharedPtr sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<wifi_interface::srv::GetHeatmap>::SharedPtr heat_srv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiDbNode>());
  rclcpp::shutdown();
  return 0;
}

