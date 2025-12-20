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

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// Module logging macros
//#define LOG_MODULE_NAME(name) static const char* MODULE_NAME = name

#define log_info(...)       spdlog::info("[{}] {}",     MODULE_NAME, fmt::format(__VA_ARGS__))
#define log_warn(...)       spdlog::warn("[{}] {}",     MODULE_NAME, fmt::format(__VA_ARGS__))
#define log_error(...)      spdlog::error("[{}] {}",    MODULE_NAME, fmt::format(__VA_ARGS__))
#define log_debug(...)      spdlog::debug("[{}] {}",    MODULE_NAME, fmt::format(__VA_ARGS__))
#define log_critical(...)   spdlog::critical("[{}] {}", MODULE_NAME, fmt::format(__VA_ARGS__))

class LOGGER: public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(LOGGER)
public:
    // make singleton
    static LOGGER* instance(QObject* parent = nullptr);

public:
    explicit LOGGER(QObject *parent = nullptr);
    ~LOGGER();

    void init();

    void set_log_path(QString path);

    void write_log(QString user_log, QString color_code="white", bool time = true, bool hide = false);
    void write_log_list(std::vector<QString> user_log_list, QString color_code="white", bool time = true, bool hide = false);

    // spdlog
    void write_log_to_txt(const QString& msg);

    void write_temperature_log_to_txt(const QString& msg);

private:
    std::mutex mtx;

    void set_log_file_name(QString filename);
    QString log_path = "";
    QString pre_user_log = "";

    void check_system_time(QString& _log_file_name);
    void write_log_file(QString log);
    void log_with_html(QString msg, QString color_code, bool time=true, bool hide=false);
    QString log_with_time(QString msg);

    QString log_file_name = "";

    std::string log_ = "";
    std::vector<std::string> log_list_;

    tbb::concurrent_queue<LOG_INFO> log_que;

    std::atomic<bool> log_flag = {false};
    std::unique_ptr<std::thread> log_thread;
    void log_loop();

    // spdlog
    std::shared_ptr<spdlog::logger> spd_logger = nullptr;
    std::shared_ptr<spdlog::logger> spd_temperature_logger = nullptr;


Q_SIGNALS:

//    void signal_write_temperature_log(const QString& msg);
    void signal_write_log(QString user_log, QString color_code);
};

#endif // LOGGER_H
