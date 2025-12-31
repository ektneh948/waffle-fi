#include "admindialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QTableView>
#include <QPushButton>
#include <QLabel>
#include <QSqlTableModel>
#include <QSqlError>
#include <QDebug>

AdminDialog::AdminDialog(QSqlDatabase db, QWidget* parent)
    : QDialog(parent), db_(db)
{
    setWindowTitle("DB Admin (Raw Table View)");
    resize(980, 640);

    auto* root = new QVBoxLayout(this);

    auto* top = new QHBoxLayout();
    top->addWidget(new QLabel("Table:", this));

    tableCombo_ = new QComboBox(this);
    top->addWidget(tableCombo_, 1);

    btnReload_ = new QPushButton("Reload", this);
    top->addWidget(btnReload_);
    root->addLayout(top);

    view_ = new QTableView(this);
    view_->setSelectionBehavior(QAbstractItemView::SelectRows);
    view_->setSelectionMode(QAbstractItemView::SingleSelection);
    view_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    view_->setSortingEnabled(true);                 // raw 보기지만 정렬은 유용
    view_->setAlternatingRowColors(true);
    root->addWidget(view_, 1);

    connect(btnReload_, &QPushButton::clicked, this, &AdminDialog::onReload);
    connect(tableCombo_, &QComboBox::currentTextChanged, this, &AdminDialog::onTableChanged);

    loadTableList();
    if (tableCombo_->count() > 0) {
        bindTable(tableCombo_->currentText());
    }
}

AdminDialog::~AdminDialog()
{
    delete model_;
    model_ = nullptr;
}

void AdminDialog::loadTableList()
{
    tableCombo_->clear();

    if (!db_.isValid()) {
        qDebug() << "AdminDialog: invalid db handle";
        return;
    }
    if (!db_.isOpen()) {
        qDebug() << "AdminDialog: db is not open";
        return;
    }

    // SQLite 기준: 테이블 목록
    QStringList tables = db_.tables();

    // sqlite 내부 테이블 숨기고 싶으면:
    tables.removeIf([](const QString& t){ return t.startsWith("sqlite_", Qt::CaseInsensitive); });

    tableCombo_->addItems(tables);
}

void AdminDialog::bindTable(const QString& tableName)
{
    if (tableName.isEmpty()) return;
    if (!db_.isValid() || !db_.isOpen()) return;

    delete model_;
    model_ = new QSqlTableModel(this, db_);
    model_->setTable(tableName);
    model_->setEditStrategy(QSqlTableModel::OnManualSubmit);
    model_->select();

    if (model_->lastError().isValid()) {
        qDebug() << "AdminDialog select error:" << model_->lastError().text();
    }

    view_->setModel(model_);
    view_->resizeColumnsToContents();
}

void AdminDialog::onReload()
{
    loadTableList();
    if (tableCombo_->count() > 0) {
        bindTable(tableCombo_->currentText());
    }
}

void AdminDialog::onTableChanged(const QString& name)
{
    bindTable(name);
}
