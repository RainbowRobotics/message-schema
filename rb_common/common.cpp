

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "utils/spdlog/sinks/basic_file_sink.h"
#include "utils/spdlog/spdlog.h"

#include "common.h"
#include "dof.h"

// #include "utils/magic_enum/magic_enum.hpp"
#include "rb_ipc/ipc.h"

#define P_NAME "common"

namespace rb_common {
namespace {
std::string g_baseName;
std::string g_logDir;
std::string g_currentDateStr;
std::shared_ptr<spdlog::logger> g_logger;
std::mutex g_mutex;

std::string log_CurrentDate() {
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};

  localtime_r(&t, &tm);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y_%m_%d");
  return oss.str();
}

std::string log_CurrentTime() {
  using namespace std::chrono;
  auto now = system_clock::now();
  auto itt = system_clock::to_time_t(now);
  auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::tm tm{};
  localtime_r(&itt, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}

void log_CleanOlds() {
  std::vector<std::filesystem::directory_entry> logFiles;
  for (auto& entry : std::filesystem::directory_iterator(g_logDir)) {
    if (entry.is_regular_file()) {
      auto filename = entry.path().filename().string();
      if (filename.find(g_baseName + "_") == 0 && filename.substr(filename.size() - 4) == ".txt") {
        logFiles.push_back(entry);
      }
    }
  }

  std::sort(logFiles.begin(), logFiles.end(), [](const auto& a, const auto& b) {
    return std::filesystem::last_write_time(a) > std::filesystem::last_write_time(b);  // 최신순
  });

  const size_t maxFiles = 15;
  for (size_t i = maxFiles; i < logFiles.size(); ++i) {
    std::filesystem::remove(logFiles[i]);
  }
}

void log_UpdateFile() {
  std::string todayStr = log_CurrentDate();
  if (todayStr != g_currentDateStr) {
    g_currentDateStr = todayStr;
    std::string filename = g_baseName + "_" + todayStr + ".txt";
    std::string fullPath = std::filesystem::path(g_logDir) / filename;

    spdlog::drop_all();  // clear previous loggers
    g_logger = spdlog::basic_logger_mt("daily_logger", fullPath);
    g_logger->set_pattern("%v");  // Use custom formatting via our code
    g_logger->flush_on(spdlog::level::info);

    log_CleanOlds();
  }
}

std::string getMessageName(int code) {
  std::string ret_str = "N/D";
  if (code >= MSG_JOINT_NOT_DETECTED_0) {
    int anchor_val = (code / 50) * 50;
    int anchor_offset = code - anchor_val;
    for (const auto& m : MESSAGE_LIST_TABLE) {
      if (m.code < MSG_JOINT_NOT_DETECTED_0) {
        continue;
      } else if (m.code == anchor_val) {
        ret_str = std::string(m.name);

        size_t pos = ret_str.find_last_of('_');
        if (pos != std::string::npos) {
          ret_str.replace(pos + 1, ret_str.length() - (pos + 1), std::to_string(anchor_offset));
        }
        break;
      }
    }
  } else {
    for (const auto& m : MESSAGE_LIST_TABLE) {
      if (m.code == code) {
        ret_str = std::string(m.name);
        break;
      } else if (m.code >= MSG_JOINT_NOT_DETECTED_0) {
        break;
      }
    }
  }

  // for (auto m : MESSAGE_LIST_TABLE) {
  //     if(code >= MSG_JOINT_NOT_DETECTED_0){
  //          int anchor_val = (code / 50) * 50;
  //     }else{
  //         if (m.code == code){
  //             ret_str = std::string(m.name);
  //             break;
  //         }
  //     }
  // }
  return ret_str;
}
}  // namespace

int thread_create(void* (*t_handler)(void*), int t_cpu_no, std::string t_name, pthread_t& thread_nrt, void* arg) {
  pthread_attr_t attr;
  cpu_set_t cpuset;
  int err;
  int ret;

  // initialized with default attributes
  err = pthread_attr_init(&attr);
  if (err != 0) {
    return err;
  }
  // set cpu ID
  if (t_cpu_no >= 0) {
    CPU_ZERO(&cpuset);
    CPU_SET(t_cpu_no, &cpuset);
    err = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
    if (err != 0) {
      return err;
    }
  }
  // create a RT thread
  ret = pthread_create(&thread_nrt, &attr, t_handler, arg);
  if (ret != 0) {
    err = pthread_attr_destroy(&attr);
    return err;
  }

  err = pthread_setname_np(thread_nrt, t_name.c_str());
  if (err != 0) {
    return err;
  }

  return ret;
}

void log_initialize(const std::string& baseName, const std::string& logDir) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_baseName = baseName;
  g_logDir = logDir;
  std::filesystem::create_directories(g_logDir);  // Create if not exist
  log_UpdateFile();
}

void log_push(LogLevel level, const std::string& message, const std::string& who_issue) {
  std::string level_str = "X";
  std::string level_color = COUT_NORMAL;
  switch (level) {
    case LogLevel::Info:
      level_str = "I";
      level_color = COUT_GREEN;
      break;
    case LogLevel::Debug:
      level_str = "D";
      break;
    case LogLevel::Warning:
      level_str = "W";
      level_color = COUT_YELLOW;
      break;
    case LogLevel::Error:
      level_str = "E";
      level_color = COUT_RED;
      break;
    case LogLevel::User:
      level_str = "U";
      break;
    case LogLevel::Msg:
      level_str = "M";
      level_color = COUT_MAGENTA;
      break;
    default:
      break;
  }

  std::lock_guard<std::mutex> lock(g_mutex);
  log_UpdateFile();

  std::string current_time_stamp = log_CurrentTime();

  std::ostringstream oss;
  oss << std::left << std::setw(2) << level_str << std::setw(15) << current_time_stamp << std::setw(15)
      << ("[" + who_issue + "]") << message;

  rb_ipc::PUB_LOG_ST pub_msg;
  pub_msg.level = (int)level;
  pub_msg.timestamp = current_time_stamp;
  pub_msg.logcontents = message;
  rb_ipc::Publish_Log(pub_msg);

  std::string formatted = oss.str();

  std::cout << level_color << formatted << COUT_NORMAL << std::endl;
  if (g_logger) {
    g_logger->info(formatted);
  }
}

int shot_message(int message_level, int message_code, std::string str_pname) {
  std::string raw_str = getMessageName(message_code);
  std::string happend_str = "MSG:" + std::to_string(message_code) + "/" + raw_str;

  if (message_level == MSG_LEVEL_WARN) {
    rb_common::log_push(LogLevel::Warning, happend_str, str_pname);
  } else if (message_level == MSG_LEVEL_ERRR) {
    rb_common::log_push(LogLevel::Error, happend_str, str_pname);
  } else {
    rb_common::log_push(LogLevel::Msg, happend_str, str_pname);
  }

  rb_ipc::PUB_MESSAGE_ST t_msg;
  t_msg.type = message_level;
  t_msg.message_code = message_code;
  t_msg.sub_message = raw_str;
  rb_ipc::Publish_Message(t_msg);

  return message_code;
}
}  // namespace rb_common