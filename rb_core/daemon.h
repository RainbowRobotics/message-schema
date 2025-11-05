#ifndef DAEMON_H
#define DAEMON_H

#include <unistd.h>

namespace rb_daemon {
    bool initialize(std::string domain);
}

#endif // DAEMON_H
