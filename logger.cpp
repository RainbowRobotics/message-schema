#include "logger.h"

LOGGER::LOGGER(QObject *parent)
    : QObject{parent}
{
}

LOGGER::~LOGGER()
{
    if(log_thread != NULL)
    {
        log_flag = false;
        log_thread->join();
        log_thread = NULL;
    }
}

void LOGGER::init()
{
    mtx.lock();
    {
        // make log folder
        QDir log_folder_path(log_path);
        if(!log_folder_path.exists())
        {
            log_folder_path.mkdir(log_path);
        }

        // make fault log folder (signalhander ---> look main.cpp)
        QString fault_log_path = log_path + "/fault_log/";
        QDir fault_log_folder_path(fault_log_path);
        if(!fault_log_folder_path.exists())
        {
            fault_log_folder_path.mkdir(fault_log_path);
        }

        QString date_and_time = QDateTime::currentDateTime().toString("yyyy-MM-dd");
        log_file_name = log_path + date_and_time + "-log-list.html";

        // make css file
        QDir css_path(log_path + "log_styles.css");
        if(!css_path.exists())
        {
            FILE* pFile;
            std::string str = log_path.toStdString() + "log_styles.css";
            pFile = fopen(str.c_str(), "w");

            fprintf(pFile, "*{margin:0px}");
            fprintf(pFile,"p{font-family: 'Spoqa Han Sans', snan-serif;}");
            fclose(pFile);
        }

        // style option when make first log file
        QDir log_file_path(log_file_name);
        if(!log_file_path.exists())
        {
            FILE* pFile;
            pFile = fopen(log_file_name.toStdString().c_str(), "a");

            fprintf(pFile, "<link rel=\"stylesheet\" href=\"log_styles.css\"><link>");
            fprintf(pFile, "<body bgcolor=\"#252831\"></body>");

            fclose(pFile);
        }
    }
    mtx.unlock();

    if(log_thread == NULL)
    {
        log_flag = true;
        log_thread = new std::thread(&LOGGER::log_loop, this);
    }
}

void LOGGER::log_loop()
{
    while(log_flag)
    {
        LOG_INFO log_info;
        if(log_que.try_pop(log_info))
        {
            QString user_log = log_info.user_log;
            QString color_code = log_info.color_code;
            bool is_time = log_info.is_time;
            bool is_hide = log_info.is_hide;

            if(pre_user_log != user_log)
            {
                log_with_html(user_log, color_code, is_time, is_hide);
            }
            pre_user_log = user_log;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void LOGGER::write_log(QString user_log, QString color_code, bool time, bool hide)
{
    LOG_INFO log_info;
    log_info.user_log = user_log;
    log_info.color_code = color_code;
    log_info.is_time = time;
    log_info.is_hide = hide;
    log_que.push(log_info);
}

void LOGGER::write_log_list(std::vector<QString> user_log_list, QString color_code, bool time, bool hide)
{
    for (auto ele : user_log_list)
    {
        LOG_INFO log_info;
        log_info.user_log = ele;
        log_info.color_code = color_code;
        log_info.is_time = time;
        log_info.is_hide = hide;
        log_que.push(log_info);
    }
}

void LOGGER::log_with_html(QString msg, QString color_code, bool time, bool hide)
{
    QString log_for_write;
    if(time == true)
    {
        log_for_write = log_with_time(msg);
    }
    else
    {
        log_for_write = msg;
    }

    QString style_font = "<font color=" + color_code + ">" + log_for_write + "</font>";
    write_log_file("<p>" + style_font + "</p>");

    if(hide == false)
    {
        printf("%s\n", log_for_write.toStdString().c_str());
    }

    Q_EMIT signal_write_log(log_for_write, color_code);
}

void LOGGER::write_log_file(QString log)
{
    mtx.lock();
    {
        check_system_time(log_file_name);

        FILE* pFile;
        pFile = fopen(log_file_name.toStdString().c_str(), "a");
        fprintf(pFile, log.toStdString().c_str());
        fclose(pFile);
    }
    mtx.unlock();
}

QString LOGGER::log_with_time(QString msg)
{
    QString time = QTime::currentTime().toString("hh:mm:ss");;
    QString log = "[" + time + "] " + msg;

    return log;
}

void LOGGER::check_system_time(QString& _log_file_name)
{
    QString date_and_time = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    QString res = log_path + date_and_time + "-log-list.html";
    if(res != _log_file_name)
    {
        _log_file_name = res;

        QDir log_file_path(_log_file_name);
        if (!log_file_path.exists())
        {
            FILE* pFile = fopen(_log_file_name.toStdString().c_str(), "a");
            fprintf(pFile, "<link rel=\"stylesheet\" href=\"log_styles.css\"><link>");
            fprintf(pFile, "<body bgcolor=\"#252831\"></body>");
            fclose(pFile);
        }
    }
}
