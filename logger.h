#pragma once
#include <QString>
#include <QTime>
#include <QTextEdit>
#include <string>

class LOGGER
{
public:
    LOGGER();
    ~LOGGER();

    void init();
    inline void SetLogFileName(QString filename) { log_file_name = filename + ".html"; }

    void PrintLog(QString user_log, QString color_code="white", bool time = true, bool hide = false);
    void PrintLogList(std::vector<QString> user_log_list, QString color_code="white", bool time = true, bool hide = false);

    QString log_path;

private:

    void logWithHtml(QString msg, QString color_code, bool time=true, bool hide=false);
    QString logWithTime(QString msg);
    void writeLogFile(QString log);

    QString log_file_name;
    std::string log_;
    std::vector<std::string> log_list_;
};


