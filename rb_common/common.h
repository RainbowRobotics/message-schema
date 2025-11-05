#ifndef COMMON_H
#define COMMON_H
//------------------------------------------------------------------------
//system
//------------------------------------------------------------------------
#include <pthread.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
//------------------------------------------------------------------------
//utils
//------------------------------------------------------------------------
#include <rbdl/rbdl.h>
#include <eigen3/Eigen/Dense>
//------------------------------------------------------------------------
#include "message.h"

using namespace std::chrono_literals;

//------------------------------------------------------------------------
//functions
//------------------------------------------------------------------------
enum class LogLevel { Info, Warning, Error, User, Debug, Msg};
#define LOG_INFO(msg)               rb_common::log_push(LogLevel::Info, msg, P_NAME)
#define LOG_WARNING(msg)            rb_common::log_push(LogLevel::Warning, msg, P_NAME)
#define LOG_ERROR(msg)              rb_common::log_push(LogLevel::Error, msg, P_NAME)
#define MESSAGE(m_level, m_code)    rb_common::shot_message(m_level, m_code, P_NAME);

namespace rb_common {
    int     thread_create(void* (*t_handler)(void *), int t_cpu_no, std::string t_name, pthread_t &thread_nrt, void* arg);
    
    void    log_initialize(const std::string& baseName, const std::string& logDir);
    void    log_push(LogLevel level, const std::string& message, const std::string& who_issue);

    int     shot_message(int message_level, int message_code, std::string str_pname);

}
#endif // COMMON_H
