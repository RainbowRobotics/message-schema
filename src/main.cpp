#include "global_defines.h"
#include "mainwindow.h"
#include <QApplication>

// for critical logging
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <csignal>
#include <execinfo.h>
#include <unistd.h>
#include <limits.h>

std::string getExecutableDir()
{
    char path[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
    if(count == -1)
    {
        return "";
    }
    path[count] = '\0';

    std::string fullPath(path);
    size_t pos = fullPath.find_last_of('/');
    return (std::string::npos == pos) ? "" : fullPath.substr(0, pos);
}

std::string getCurrentDate()
{
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm localTime = *std::localtime(&time);

    std::ostringstream oss;
    oss << std::put_time(&localTime, "%Y-%m-%d-%H-%M-%S");
    return oss.str();
}

void signalHandler(int signal)
{
    if(signal == SIGINT)
    {
        printf("[MAIN] SIGINT received\n");
        QCoreApplication::quit();
        return;
    }

    std::string date_and_time = getCurrentDate();
    std::string full_path = getExecutableDir() + "/snlog/fault_log/" + date_and_time + "-fault_log.txt";

    std::ofstream logFile(full_path, std::ios::app);
    logFile << "Signal (" << signal << ") received." << std::endl;

    void *array[10];
    size_t size = backtrace(array, 10);
    char **messages = backtrace_symbols(array, size);

    for (size_t i = 0; i < size; ++i)
    {
        logFile << messages[i] << std::endl;
    }

    free(messages);
    logFile.close();
    std::cerr << "Signal (" << signal << ") received." << std::endl;
    std::_Exit(signal);
}

int main(int argc, char *argv[])
{
    // Disable buffering for stdout (apply globally)
    setbuf(stdout, NULL);

    // critical error catcher
    std::signal(SIGINT, signalHandler);
    std::signal(SIGSEGV, signalHandler);
    std::signal(SIGABRT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGKILL, signalHandler);
    std::signal(SIGPIPE, signalHandler);

    // set for high dpi
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    // init qt app
    qRegisterMetaType<Eigen::Matrix4d>();

    qRegisterMetaType<DATA_MOVE>();
    qRegisterMetaType<DATA_LOCALIZATION>();
    qRegisterMetaType<DATA_LOAD>();
    qRegisterMetaType<DATA_RANDOMSEQ>();
    qRegisterMetaType<DATA_MAPPING>();
    qRegisterMetaType<DATA_DOCK>();
    qRegisterMetaType<DATA_VIEW_LIDAR>();
    qRegisterMetaType<DATA_VIEW_PATH>();
    qRegisterMetaType<DATA_LED>();
    qRegisterMetaType<DATA_MOTOR>();
    qRegisterMetaType<DATA_PATH>();
    qRegisterMetaType<DATA_VOBS>();
    qRegisterMetaType<DATA_SOFTWARE>();
    qRegisterMetaType<DATA_FOOT>();
    qRegisterMetaType<DATA_FIELD>();

    QApplication a(argc, argv);

//    // Shared memory 객체 생성
//       QSharedMemory sharedMemory;
//       sharedMemory.setKey("MyUniqueAppKey");

//       // 이미 실행 중인 인스턴스가 있는지 확인
//       if (sharedMemory.attach())
//       {
//           // 이미 실행 중이면 종료
//   //        QMessageBox::warning(nullptr, "경고", "이미 프로그램이 실행 중입니다.");
//           return 0;
//       }
//       else
//       {
//           // 첫 번째 인스턴스라면 공유 메모리 할당
//           if (!sharedMemory.create(1))
//           {
//   //            QMessageBox::critical(nullptr, "에러", "공유 메모리를 생성할 수 없습니다.");
//               return 0;
//           }
//       }

    // set VTK default option
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    vtkObject::GlobalWarningDisplayOff();

    // show ui
    MainWindow w;
    w.show();
    return a.exec();
}
