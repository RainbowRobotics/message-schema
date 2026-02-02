#ifndef SOCKET_COMMAND_SERVER_H
#define SOCKET_COMMAND_SERVER_H

namespace rb_socket_command_server {
    bool initialize(std::string domain, int th_cpu, int port_no);
    void shutdown();
    void broadcast(const std::string& message);

    void ascii_script_execution(std::string& t_script);
}

#endif // SOCKET_COMMAND_SERVER_H
