#define P_NAME  "DATA_SERVER"

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










#define MAX_CLIENTS 5
#define MAX_EVENTS 64
#define EPOLL_TIMEOUT_MS 100 // epoll_wait timeout (ms)

namespace rb_socket_data_server {
    namespace {
        std::atomic<bool> running(false);
        int listen_port = 0;
        int event_fd = -1;
        pthread_t server_thread;

        struct MyDataStruct {
            int id;
            float temperature;
            float pressure;
            char status[8];
        };
        static MyDataStruct g_data = {1, 25.6f, 1013.2f, "OK"};

        std::string serialize_data(const MyDataStruct &d) {
            char buf[128];
            snprintf(buf, sizeof(buf), "DATA[id=%d,temp=%.2f,press=%.2f,status=%s]",
                    d.id, d.temperature, d.pressure, d.status);
            return std::string(buf);
        }

        struct ClientInfo {
            int fd;
            std::string ip;
            std::string buffer; // partial recv buffer

            bool auto_send = false;
            double send_hz = 0.0;
            std::chrono::steady_clock::time_point next_send_time;
        };
        std::unordered_map<int, ClientInfo> clients;

        std::mutex clients_mutex;
        
        // =============================
        // 🔹 Non-blocking 설정
        // =============================
        static int set_nonblocking(int fd) {
            int flags = fcntl(fd, F_GETFL, 0);
            if (flags == -1) return -1;
            return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        }

        // =============================
        // 🔹 TCP KeepAlive 설정 추가
        // =============================
        void enable_tcp_keepalive(int sockfd) {
            int optval;
            socklen_t optlen = sizeof(optval);

            // 기본 keepalive 활성화
            optval = 1;
            if (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) == -1) {
                perror("setsockopt(SO_KEEPALIVE)");
            }

#ifdef TCP_KEEPIDLE
            // 10초 동안 트래픽 없으면 keepalive 시작
            optval = 10;
            if (setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPIDLE, &optval, optlen) == -1) {
                perror("setsockopt(TCP_KEEPIDLE)");
            }
#endif

#ifdef TCP_KEEPINTVL
            // 실패한 keepalive 재시도 간격 5초
            optval = 5;
            if (setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPINTVL, &optval, optlen) == -1) {
                perror("setsockopt(TCP_KEEPINTVL)");
            }
#endif

#ifdef TCP_KEEPCNT
            // 최대 3번 시도 후 연결 끊음
            optval = 3;
            if (setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPCNT, &optval, optlen) == -1) {
                perror("setsockopt(TCP_KEEPCNT)");
            }
#endif
        }

        // =============================
        // 🔹 클라이언트 종료 처리
        // =============================
        void close_client(int epfd, int fd) {
            std::cout << "[SDS] Closing client fd: " << fd << std::endl;
            epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr);
            close(fd);
            std::lock_guard<std::mutex> lk(clients_mutex);
            clients.erase(fd);
        }

        // =============================
        // 🔹 명령 처리
        // =============================
        static void parse_command(const std::string &input, std::string &func_name, std::vector<std::string> &args) {
            func_name.clear();
            args.clear();

            std::string cmd = input;
            cmd.erase(std::remove_if(cmd.begin(), cmd.end(), ::isspace), cmd.end());

            auto open = cmd.find('(');
            auto close = cmd.find(')');
            if (open == std::string::npos || close == std::string::npos || close <= open) {
                func_name = cmd;
                return;
            }

            func_name = cmd.substr(0, open);
            std::string arg_str = cmd.substr(open + 1, close - open - 1);

            std::stringstream ss(arg_str);
            std::string arg;
            while (std::getline(ss, arg, ',')) {
                args.push_back(arg);
            }
        }

        // 🔹 디스패처: 함수명 → 실제 동작
        static int execute_command(int client_fd, const std::string &func, const std::vector<std::string> &args) {
            if (func == "reqdata") {
                // std::lock_guard<std::mutex> lk(clients_mutex);
                auto it = clients.find(client_fd);
                if (it == clients.end())
                    return MSG_OK; // 클라이언트 정보 없음
                auto &c = it->second;

                if (args.empty()) {
                    // 1회 송신 + 자동 전송 해제
                    c.auto_send = false;
                    std::string payload = serialize_data(g_data) + "\n";
                    send(client_fd, payload.c_str(), payload.size(), MSG_NOSIGNAL);
                    std::cout << "[SDS] Sent single data to " << c.ip << std::endl;
                    return MSG_OK;
                } else {
                    try {
                        double hz = std::stod(args[0]);
                        if (hz <= 0)
                            return MSG_NOT_VALID_COMMAND_FORMAT;
                        c.auto_send = true;
                        c.send_hz = hz;
                        c.next_send_time = std::chrono::steady_clock::now();
                        std::cout << "[SDS] " << c.ip << " enabled auto-send (" << hz << "Hz)" << std::endl;
                        return MSG_OK;
                    } catch (...) {
                        return MSG_NOT_VALID_COMMAND_FORMAT;
                    }
                }
            }

            std::cerr << "[SDS] Unknown command: " << func << std::endl;
            return MSG_NOT_VALID_COMMAND_FORMAT;
        }
        void parse_and_execute(const std::string &cmd, int client_fd) {
            std::string func;
            std::vector<std::string> args;
            parse_command(cmd, func, args);
            int ret = execute_command(client_fd, func, args);

            std::string response = "return[SDS][" + std::to_string(ret) + "]\n";

            send(client_fd, response.c_str(), response.size(), 0);
        }

        // =============================
        // 🔹 클라이언트 데이터 처리
        // =============================
        void handle_client_data(int epfd, int fd) {
            char buf[512];
            int n = read(fd, buf, sizeof(buf));
            if (n <= 0) {
                if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
                    std::cout << "[SDS] Client disconnected (FD=" << fd << ")" << std::endl;
                    close_client(epfd, fd);
                }
                return;
            }

            std::lock_guard<std::mutex> lk(clients_mutex);
            auto &info = clients[fd];
            info.buffer.append(buf, n);

            size_t start = 0;
            int paren_level = 0;

            for (size_t i = 0; i < info.buffer.size(); ++i) {
                char c = info.buffer[i];

                // 공백, 탭 무시
                if (c == ' ' || c == '\t') continue;

                // 괄호 레벨 계산
                if (c == '(') paren_level++;
                else if (c == ')') paren_level--;

                // 명령 종료 조건: 괄호가 닫히거나 종료 문자 발견
                bool end_of_command = false;
                if (paren_level == 0) {
                    if (c == ')' || c == '\n' || c == '\r' || c == '\0') {
                        end_of_command = true;
                    }
                }

                if (end_of_command) {
                    // 명령어 추출
                    std::string cmd = info.buffer.substr(start, i - start + 1);

                    // 앞뒤 공백, 개행, 널 제거
                    size_t first = cmd.find_first_not_of(" \r\n\t\0");
                    size_t last = cmd.find_last_not_of(" \r\n\t\0");
                    if (first != std::string::npos && last != std::string::npos)
                        cmd = cmd.substr(first, last - first + 1);
                    else
                        cmd.clear(); // 전부 공백이면 빈 문자열

                    if (!cmd.empty()) {
                        std::cout << "[SDS] Executing raw: " << cmd << std::endl;
                        parse_and_execute(cmd, fd);
                    }

                    start = i + 1; // 다음 명령 시작 위치
                }
            }

            // 처리된 부분은 버퍼에서 제거
            if (start > 0) {
                info.buffer.erase(0, start);
            }
        }

        // =============================
        // 🔹 서버 스레드 본체
        // =============================
        void *thread_dataserver(void *) {
            int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (listen_fd < 0) {
                perror("socket");
                return nullptr;
            }

            int opt = 1;
            setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(listen_port);

            if (bind(listen_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
                perror("bind");
                close(listen_fd);
                return nullptr;
            }

            if (listen(listen_fd, MAX_CLIENTS) < 0) {
                perror("listen");
                close(listen_fd);
                return nullptr;
            }

            set_nonblocking(listen_fd);
            event_fd = eventfd(0, EFD_NONBLOCK);

            int epfd = epoll_create1(0);
            if (epfd == -1) {
                perror("epoll_create1");
                close(listen_fd);
                return nullptr;
            }

            // epoll 등록
            {
                epoll_event ev{};
                ev.events = EPOLLIN;
                ev.data.fd = listen_fd;
                epoll_ctl(epfd, EPOLL_CTL_ADD, listen_fd, &ev);
            }
            {
                epoll_event ev{};
                ev.events = EPOLLIN;
                ev.data.fd = event_fd;
                epoll_ctl(epfd, EPOLL_CTL_ADD, event_fd, &ev);
            }

            std::vector<epoll_event> events(MAX_EVENTS);

            std::cout << "[SDS] TCP Server started on port " << listen_port << std::endl;

            // ----------------------------
            // 메인 루프
            // ----------------------------
            while (running.load()) {
                // ----------------------------
                // ⏱ 다음 자동 전송 시각 계산
                // ----------------------------
                auto now = std::chrono::steady_clock::now();
                int timeout_ms = EPOLL_TIMEOUT_MS;

                {
                    std::lock_guard<std::mutex> lk(clients_mutex);
                    bool has_pending_send = false;
                    auto soonest = now + std::chrono::milliseconds(EPOLL_TIMEOUT_MS);

                    for (auto &p : clients) {
                        auto &c = p.second;
                        if (c.auto_send) {
                            has_pending_send = true;
                            if (c.next_send_time < soonest)
                                soonest = c.next_send_time;
                        }
                    }

                    if (has_pending_send) {
                        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                            soonest - now).count();
                        timeout_ms = std::clamp((int)diff, 0, EPOLL_TIMEOUT_MS);
                    }
                }

                // ----------------------------
                // ⚙ epoll 대기 (남은 시간만큼)
                // ----------------------------
                int nfds = epoll_wait(epfd, events.data(), MAX_EVENTS, timeout_ms);
                now = std::chrono::steady_clock::now();

                // ----------------------------
                // ✅ 자동 전송 처리
                // ----------------------------
                {
                    std::lock_guard<std::mutex> lk(clients_mutex);
                    for (auto &p : clients) {
                        auto &c = p.second;
                        if (c.auto_send && now >= c.next_send_time) {
                            std::string payload = serialize_data(g_data) + "\n";
                            send(c.fd, payload.c_str(), payload.size(), MSG_NOSIGNAL);

                            // 다음 전송 시각 계산 (Hz 기반)
                            auto interval = std::chrono::microseconds((int)(1'000'000.0 / c.send_hz));
                            c.next_send_time = now + interval;
                        }
                    }
                }

                // ----------------------------
                // 🔔 이벤트 처리
                // ----------------------------
                if (nfds == -1) {
                    if (errno == EINTR) continue;
                    perror("epoll_wait");
                    break;
                }

                for (int i = 0; i < nfds; ++i) {
                    int fd = events[i].data.fd;
                    uint32_t evt = events[i].events;

                    if (fd == event_fd) {
                        uint64_t v;
                        read(event_fd, &v, sizeof(v));
                        running.store(false);
                        break;
                    }

                    if (fd == listen_fd) {
                        while (true) {
                            sockaddr_in caddr{};
                            socklen_t clen = sizeof(caddr);
                            int cfd = accept(listen_fd, (sockaddr *)&caddr, &clen);
                            if (cfd == -1) {
                                if (errno == EAGAIN || errno == EWOULDBLOCK)
                                    break;
                                perror("accept");
                                break;
                            }

                            set_nonblocking(cfd);
                            enable_tcp_keepalive(cfd);

                            char ipbuf[INET_ADDRSTRLEN];
                            inet_ntop(AF_INET, &caddr.sin_addr, ipbuf, sizeof(ipbuf));

                            epoll_event cev{};
                            cev.events = EPOLLIN | EPOLLRDHUP;
                            cev.data.fd = cfd;
                            epoll_ctl(epfd, EPOLL_CTL_ADD, cfd, &cev);

                            {
                                std::lock_guard<std::mutex> lk(clients_mutex);
                                clients.emplace(cfd, ClientInfo{cfd, ipbuf, ""});
                            }

                            std::cout << "[SDS] Client connected: " << ipbuf << " FD=" << cfd << std::endl;
                        }
                        continue;
                    }

                    if (evt & (EPOLLERR | EPOLLRDHUP | EPOLLHUP)) {
                        close_client(epfd, fd);
                        continue;
                    }

                    if (evt & EPOLLIN) {
                        handle_client_data(epfd, fd);
                    }
                }
            }
            // ----------------------------
            // 종료 처리
            // ----------------------------
            std::cout << "[SDS] Shutting down..." << std::endl;
            {
                std::lock_guard<std::mutex> lk(clients_mutex);
                for (auto &p : clients) {
                    epoll_ctl(epfd, EPOLL_CTL_DEL, p.first, nullptr);
                    close(p.first);
                }
                clients.clear();
            }

            close(event_fd);
            close(listen_fd);
            close(epfd);

            std::cout << "[SDS] Server stopped." << std::endl;
            return nullptr;
        } // thread_dataserver
    }
    bool initialize(std::string domain, int th_cpu, int port_no){
        running.store(true);
        listen_port = port_no;
        if (rb_common::thread_create(thread_dataserver, th_cpu, ("RB_" + domain + "_SDS"), server_thread, NULL) != 0) {
            running.store(false);
            return false;
        }
        return true;
    }
    void shutdown(){
        running.store(false);
        if (event_fd != -1) {
            uint64_t v = 1;
            write(event_fd, &v, sizeof(v));
        }
        pthread_join(server_thread, nullptr);
    }
    void broadcast(const std::string& message){
        std::lock_guard<std::mutex> lk(clients_mutex);

        if (clients.empty()) {
            std::cout << "[SDS] Broadcast skipped (no clients)" << std::endl;
            return;
        }

        std::string msg = message;
        if (msg.back() != '\n') msg += "\n";  // 클라이언트에서 읽기 편하게 개행 추가

        for (auto it = clients.begin(); it != clients.end();) {
            int fd = it->first;
            ssize_t sent = send(fd, msg.c_str(), msg.size(), MSG_NOSIGNAL);
            if (sent == -1) {
                if (errno == EPIPE || errno == ECONNRESET) {
                    std::cerr << "[SDS] Removing dead client FD=" << fd << std::endl;
                    close(fd);
                    it = clients.erase(it);
                    continue;
                }
            }
            ++it;
        }

        std::cout << "[SDS] Broadcasted to " << clients.size() << " clients: " << message << std::endl;
    }
} // namespace rb_socket_data_server