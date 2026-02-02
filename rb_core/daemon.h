#ifndef DAEMON_H
#define DAEMON_H

#include <unistd.h>

namespace rb_daemon {
    bool initialize(std::string domain, int th_cpu_rt, int th_cpu_lanread);
}

#endif // DAEMON_H
