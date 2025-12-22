#pragma once
#include <QString>
#include <QVector>
#include <QPair>
#include <QSqlDatabase>

    struct SampleRow {
    double x = 0.0;
    double y = 0.0;
    QString ssid = "ALL";
    float rssi = -80.f;
    QString ts;
};

class DbManager {
public:
    bool open(const QString& path);
    bool initSchema();

    QVector<QPair<QString, QString>> listSessions(); // (session_id, created_at)
    QVector<SampleRow> loadSamples(const QString& sessionId,
                                   const QString& ssid,
                                   bool thrEnable, int thrRssi);

    bool deleteSession(const QString& sessionId);

    // Step4에서 사용할 API(지금은 선언만 있어도 됨)
    QString beginSession();
    bool endSession(const QString& id);
    bool insertSample(const QString& sessionId, const SampleRow& s);

private:
    QSqlDatabase db_;
};
