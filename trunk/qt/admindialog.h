#pragma once
#include <QDialog>
#include <QSqlDatabase>

class QComboBox;
class QTableView;
class QSqlTableModel;
class QPushButton;

class AdminDialog : public QDialog
{
    Q_OBJECT
public:
    explicit AdminDialog(QSqlDatabase db, QWidget* parent = nullptr);
    ~AdminDialog() override;

private slots:
    void onReload();
    void onTableChanged(const QString& name);

private:
    void loadTableList();
    void bindTable(const QString& tableName);

    QSqlDatabase db_;
    QComboBox* tableCombo_ = nullptr;
    QTableView* view_ = nullptr;
    QSqlTableModel* model_ = nullptr;
    QPushButton* btnReload_ = nullptr;
};
