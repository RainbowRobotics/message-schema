#ifndef LOGGER_H
#define LOGGER_H

#include "global_defines.h"

#include <string>
#include <iostream>

#include <QDir>
#include <QObject>
#include <QString>
#include <QTime>
#include <QTextEdit>

class LOGGER: public QObject
{
    Q_OBJECT
public:
    explicit LOGGER(QObject *parent = nullptr);
    ~LOGGER();

    std::recursive_mutex mtx;

    void init();
    inline void set_log_file_name(QString filename) { log_file_name = filename + ".html"; }

    void write_log(QString user_log, QString color_code="white", bool time = true, bool hide = false);
    void write_log_list(std::vector<QString> user_log_list, QString color_code="white", bool time = true, bool hide = false);

    QString log_path = "";
    QString pre_user_log = "";

private:

    void check_system_time(QString& _log_file_name);
    void write_log_file(QString log);
    void log_with_html(QString msg, QString color_code, bool time=true, bool hide=false);
    QString log_with_time(QString msg);

    QString log_file_name = "";
    std::string log_ = "";
    std::vector<std::string> log_list_;

    tbb::concurrent_queue<LOG_INFO> log_que;

    std::atomic<bool> log_flag = {false};
    std::thread *log_thread = NULL;
    void log_loop();

Q_SIGNALS:
    void signal_write_log(QString user_log, QString color_code);
};

#endif // LOGGER_H
