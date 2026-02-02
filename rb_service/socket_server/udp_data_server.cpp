#define P_NAME  "UDP_DATA_SERVER"

#include <arpa/inet.h>
#include <netinet/tcp.h> // TCP_KEEPALIVE 옵션용

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <regex>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <chrono>

#include "common.h"
#include "data_server.h"
#include "shareddata.h"
#include "system.h"







#define MAX_CLIENTS 5
#define MAX_EVENTS 64
#define EPOLL_TIMEOUT_MS 100 // epoll_wait timeout (ms)

namespace rb_udp_data_server {
    namespace {
        ;
    }
    bool initialize(std::string domain, int th_cpu, int port_no){
        // running.store(true);
        // listen_port = port_no;
        // if (rb_common::thread_create(thread_dataserver, th_cpu, ("RB_" + domain + "_SDS"), server_thread, NULL) != 0) {
        //     running.store(false);
        //     return false;
        // }
        return true;
    }
    
} // namespace rb_udp_data_server