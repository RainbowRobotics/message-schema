#ifndef SOCKET_DATA_SERVER_H
#define SOCKET_DATA_SERVER_H

namespace rb_socket_data_server {
    bool initialize(std::string domain, int th_cpu, int port_no);
    void shutdown();
    void broadcast(const std::string& message);
}

#endif // SOCKET_DATA_SERVER_H
