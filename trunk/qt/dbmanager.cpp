#include "dbmanager.h"
#include <QSqlQuery>
#include <QSqlError>
#include <QVariant>
#include <QDateTime>
#include <QDebug>
#include <QUuid>

bool DbManager::open(const QString& path) {
    if (QSqlDatabase::contains("wifi_qt_conn"))
        db_ = QSqlDatabase::database("wifi_qt_conn");
    else
        db_ = QSqlDatabase::addDatabase("QSQLITE", "wifi_qt_conn");

    db_.setDatabaseName(path);
    if (!db_.open()) {
        qDebug() << "DB open failed:" << db_.lastError().text();
        return false;
    }
    return true;
}

bool DbManager::initSchema() {
    QSqlQuery q(db_);

    if (!q.exec(
            "CREATE TABLE IF NOT EXISTS sessions ("
            "  session_id TEXT PRIMARY KEY,"
            "  created_at TEXT NOT NULL"
            ");"
            )) {
        qDebug() << "schema sessions failed:" << q.lastError().text();
        return false;
    }

    if (!q.exec(
            "CREATE TABLE IF NOT EXISTS samples ("
            "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "  session_id TEXT NOT NULL,"
            "  x REAL NOT NULL,"
            "  y REAL NOT NULL,"
            "  ssid TEXT NOT NULL,"
            "  rssi REAL NOT NULL,"
            "  timestamp TEXT NOT NULL"
            ");"
            )) {
        qDebug() << "schema samples failed:" << q.lastError().text();
        return false;
    }

    q.exec("CREATE INDEX IF NOT EXISTS idx_samples_session ON samples(session_id);");
    return true;
}

QVector<QPair<QString, QString>> DbManager::listSessions() {
    QVector<QPair<QString, QString>> out;
    QSqlQuery q(db_);
    if (!q.exec("SELECT session_id, created_at FROM sessions ORDER BY created_at DESC")) {
        qDebug() << "listSessions failed:" << q.lastError().text();
        return out;
    }
    while (q.next()) out.push_back({ q.value(0).toString(), q.value(1).toString() });
    return out;
}

QVector<SampleRow> DbManager::loadSamples(const QString& sessionId,
                                          const QString& ssid,
                                          bool thrEnable, int thrRssi) {
    QVector<SampleRow> out;
    if (sessionId.isEmpty()) return out;

    QString sql =
        "SELECT x, y, ssid, rssi, timestamp "
        "FROM samples WHERE session_id = ?";

    if (!ssid.isEmpty() && ssid != "ALL") sql += " AND ssid = ?";
    if (thrEnable) sql += " AND rssi >= ?";

    sql += " ORDER BY id ASC";

    QSqlQuery q(db_);
    q.prepare(sql);
    q.addBindValue(sessionId);
    if (!ssid.isEmpty() && ssid != "ALL") q.addBindValue(ssid);
    if (thrEnable) q.addBindValue(thrRssi);

    if (!q.exec()) {
        qDebug() << "loadSamples failed:" << q.lastError().text();
        return out;
    }

    while (q.next()) {
        SampleRow s;
        s.x = q.value(0).toDouble();
        s.y = q.value(1).toDouble();
        s.ssid = q.value(2).toString();
        s.rssi = q.value(3).toFloat();
        s.ts = q.value(4).toString();
        out.push_back(s);
    }
    return out;
}

bool DbManager::deleteSession(const QString& sessionId) {
    if (sessionId.isEmpty()) return false;

    QSqlQuery q(db_);
    q.prepare("DELETE FROM samples WHERE session_id = ?");
    q.addBindValue(sessionId);
    if (!q.exec()) {
        qDebug() << "delete samples failed:" << q.lastError().text();
        return false;
    }

    q.prepare("DELETE FROM sessions WHERE session_id = ?");
    q.addBindValue(sessionId);
    if (!q.exec()) {
        qDebug() << "delete session failed:" << q.lastError().text();
        return false;
    }
    return true;
}

QString DbManager::beginSession() {
    const QString sid = QUuid::createUuid().toString(QUuid::WithoutBraces);
    const QString now = QDateTime::currentDateTime().toString(Qt::ISODate);
    QSqlQuery q(db_);
    q.prepare("INSERT INTO sessions(session_id, created_at) VALUES(?, ?)");
    q.addBindValue(sid); q.addBindValue(now);
    if (!q.exec()) return {};
    return sid;
}


bool DbManager::endSession(const QString&) { return true; }
bool DbManager::insertSample(const QString& sessionId, const SampleRow& s)
{
    if (sessionId.isEmpty()) return false;

    QSqlQuery q(db_);
    q.prepare("INSERT INTO samples(session_id, x, y, ssid, rssi, timestamp) "
              "VALUES(?, ?, ?, ?, ?, ?)");
    q.addBindValue(sessionId);
    q.addBindValue(s.x);
    q.addBindValue(s.y);
    q.addBindValue(s.ssid.isEmpty() ? "ALL" : s.ssid);
    q.addBindValue(s.rssi);
    q.addBindValue(s.ts);

    if (!q.exec()) {
        qDebug() << "insertSample failed:" << q.lastError().text();
        return false;
    }
    return true;
}

