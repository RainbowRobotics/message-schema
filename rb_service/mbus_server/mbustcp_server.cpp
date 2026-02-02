#define P_NAME  "MBUSTCP"

#include <arpa/inet.h>
#include <netinet/tcp.h> // TCP_KEEPALIVE 옵션용

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <modbus/modbus.h>

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <chrono>

#include "common.h"
#include "mbustcp_server.h"

#define MAX_CLIENTS 5
#define MAX_EVENTS 64
#define EPOLL_TIMEOUT_MS 100 // epoll_wait timeout (ms). 100ms 권장 (10ms -> CPU 증가)
                              // 필요시 5~10ms로 줄이세요.

namespace rb_mbus_server {
    namespace {
        pthread_t           hThread_mbustcpserver;
        std::atomic<bool>   running(false);

        int                 sPORT_NO;

        // Protect libmodbus ctx usage because libmodbus APIs assume some internal state.
        std::mutex          modbus_ctx_mutex;

        // Protect mb_mapping updates (register writes from other threads)
        std::mutex          mb_mutex;

        // Clients map: key = fd
        struct ClientInfo {
            int fd;
            std::string ip;
            std::chrono::steady_clock::time_point last_activity;
        };
        std::unordered_map<int, ClientInfo> clients;

        int event_fd = -1; // eventfd for wakeup/shutdown

        // helper: set non-blocking
        static int set_nonblocking(int fd) {
            int flags = fcntl(fd, F_GETFL, 0);
            if (flags == -1) return -1;
            return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        }

        void enable_tcp_keepalive(int sockfd) {
            int optval;
            socklen_t optlen = sizeof(optval);

            // 기본 keepalive 활성화
            optval = 1;
            if (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) == -1) {
                perror("setsockopt(SO_KEEPALIVE)");
            }

            // 10초 동안 트래픽 없으면 keepalive 패킷 전송 시작
#ifdef TCP_KEEPIDLE
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

        // Remove client: epfd cleanup + close fd + clients map erase
        void close_client_and_cleanup(int epfd, int fd) {
            // del from epoll
            if (epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr) == -1) {
                // log but ignore
                // perror("epoll_ctl(DEL)");
            }
            // close socket
            close(fd);
            // erase from clients map
            auto it = clients.find(fd);
            if (it != clients.end()) {
                clients.erase(it);
            }
        }

        void *thread_mbustcpserver(void *) {
            // Create libmodbus TCP context (single context protected by mutex)
            modbus_t *ctx = modbus_new_tcp("0.0.0.0", sPORT_NO);
            if (!ctx) {
                std::cerr << "Failed to create Modbus context\n";
                return nullptr;
            }

            modbus_mapping_t *mb_mapping = modbus_mapping_new(MBUS_BITS_NUM, MBUS_INPUT_BITS_NUM, MBUS_REGS_NUM, MBUS_INPUT_REGS_NUM);
            if (!mb_mapping) {
                std::cerr << "Failed to create Modbus mapping: " << modbus_strerror(errno) << "\n";
                modbus_free(ctx);
                return nullptr;
            }

            // Create listening socket via libmodbus (uses ctx)
            int server_socket = modbus_tcp_listen(ctx, MAX_CLIENTS);
            if (server_socket == -1) {
                std::cerr << "Unable to listen: " << modbus_strerror(errno) << "\n";
                modbus_mapping_free(mb_mapping);
                modbus_free(ctx);
                return nullptr;
            }

            // Try to set server socket non-blocking (not fatal)
            if (set_nonblocking(server_socket) == -1) {
                perror("set_nonblocking(server_socket)");
            }

            // create eventfd for graceful shutdown
            event_fd = eventfd(0, EFD_NONBLOCK);
            if (event_fd == -1) {
                perror("eventfd");
                close(server_socket);
                modbus_mapping_free(mb_mapping);
                modbus_free(ctx);
                return nullptr;
            }

            // create epoll instance
            int epfd = epoll_create1(0);
            if (epfd == -1) {
                perror("epoll_create1");
                close(event_fd);
                close(server_socket);
                modbus_mapping_free(mb_mapping);
                modbus_free(ctx);
                return nullptr;
            }

            // register server_socket (listen) on epoll
            {
                struct epoll_event ev{};
                ev.events = EPOLLIN; // accept events
                ev.data.fd = server_socket;
                if (epoll_ctl(epfd, EPOLL_CTL_ADD, server_socket, &ev) == -1) {
                    perror("epoll_ctl ADD server_socket");
                    close(epfd);
                    close(event_fd);
                    close(server_socket);
                    modbus_mapping_free(mb_mapping);
                    modbus_free(ctx);
                    return nullptr;
                }
            }

            // register eventfd (for shutdown)
            {
                struct epoll_event ev{};
                ev.events = EPOLLIN;
                ev.data.fd = event_fd;
                if (epoll_ctl(epfd, EPOLL_CTL_ADD, event_fd, &ev) == -1) {
                    perror("epoll_ctl ADD eventfd");
                    close(epfd);
                    close(event_fd);
                    close(server_socket);
                    modbus_mapping_free(mb_mapping);
                    modbus_free(ctx);
                    return nullptr;
                }
            }

            std::vector<struct epoll_event> events(MAX_EVENTS);

            // std::cout << "🟢 Modbus TCP Server started" << std::endl;

            while (running.load()) {
                int nfds = epoll_wait(epfd, events.data(), MAX_EVENTS, EPOLL_TIMEOUT_MS);
                if (nfds == -1) {
                    if (errno == EINTR) continue;
                    perror("epoll_wait");
                    break;
                }

                if (nfds == 0) {
                    // timeout: no events -> you can do maintenance if needed
                    // e.g., cleanup stale clients, health checks, etc.
                    // std::cout<<"epoll timeout"<<std::endl;
                }

                for (int i = 0; i < nfds; ++i) {
                    int fd = events[i].data.fd;
                    uint32_t evt = events[i].events;

                    // eventfd -> shutdown signal
                    if (fd == event_fd) {
                        uint64_t val;
                        // read to clear eventfd
                        if (read(event_fd, &val, sizeof(val)) > 0) {
                            std::cout << "Received shutdown event.\n";
                            // break outer loop by setting running = false
                            running.store(false);
                            break;
                        }
                        continue;
                    }

                    // new connection on server socket
                    if (fd == server_socket) {
                        // accept loop (non-blocking)
                        while (true) {
                            struct sockaddr_in client_addr;
                            socklen_t addrlen = sizeof(client_addr);
                            int cfd = accept(server_socket, (struct sockaddr*)&client_addr, &addrlen);
                            if (cfd == -1) {
                                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                    // no more pending connections
                                    break;
                                } else {
                                    perror("accept");
                                    break;
                                }
                            }

                            // Enforce max clients
                            if ((int)clients.size() >= MAX_CLIENTS) {
                                char ipbuf[INET_ADDRSTRLEN];
                                inet_ntop(AF_INET, &client_addr.sin_addr, ipbuf, sizeof(ipbuf));
                                std::cout << "⚠️  Max clients reached. Refusing connection from " << ipbuf << std::endl;
                                close(cfd);
                                continue;
                            }

                            // Set non-blocking & keepalive
                            if (set_nonblocking(cfd) == -1) {
                                perror("set_nonblocking(client)");
                                // continue anyway
                            }
                            enable_tcp_keepalive(cfd);

                            // get IP
                            char ipbuf[INET_ADDRSTRLEN];
                            inet_ntop(AF_INET, &client_addr.sin_addr, ipbuf, sizeof(ipbuf));

                            // register with epoll
                            struct epoll_event cev{};
                            cev.events = EPOLLIN | EPOLLRDHUP; // read + remote hangup
                            cev.data.fd = cfd;
                            if (epoll_ctl(epfd, EPOLL_CTL_ADD, cfd, &cev) == -1) {
                                perror("epoll_ctl ADD client");
                                close(cfd);
                                continue;
                            }

                            clients.emplace(cfd, ClientInfo{cfd, std::string(ipbuf), std::chrono::steady_clock::now()});
                            std::cout << "🟢 New client connected from " << ipbuf << " (FD=" << cfd << ") | Total clients: " << clients.size() << std::endl;
                        } // accept loop
                        continue; // next event
                    } // server_socket handling done

                    // client socket event
                    auto cit = clients.find(fd);
                    if (cit == clients.end()) {
                        // unknown fd, close defensively
                        close_client_and_cleanup(epfd, fd);
                        continue;
                    }

                    // handle error/hangup
                    if (evt & (EPOLLHUP | EPOLLERR | EPOLLRDHUP)) {
                        std::cout << "🔴 Client (FD=" << fd << ", IP=" << cit->second.ip << ") disconnected (HUP/ERR)." << std::endl;
                        close_client_and_cleanup(epfd, fd);
                        std::cout << "🧾 Current clients: " << clients.size() << std::endl;
                        continue;
                    }

                    if (evt & EPOLLIN) {
                        // Update last activity time
                        cit->second.last_activity = std::chrono::steady_clock::now();

                        // Use libmodbus to receive and reply
                        // Protect modbus context with mutex to avoid mixing sockets
                        {
                            std::lock_guard<std::mutex> lk(modbus_ctx_mutex);
                            modbus_set_socket(ctx, fd);
                            uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
                            int rc = modbus_receive(ctx, query);
                            if (rc > 0) {
                                // Optionally protect mapping while replying (modbus_reply will read mapping)
                                // Here modbus_reply reads mapping; if other threads may write mapping, ensure mb_mutex protects writes.
                                modbus_reply(ctx, query, rc, mb_mapping);
                            } else if (rc == -1) {
                                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                    // no data right now (non-blocking) — ignore
                                    continue;
                                } else {
                                    // other errors: client likely disconnected
                                    std::cerr << "modbus_receive error on FD=" << fd << ": " << modbus_strerror(errno) << std::endl;
                                    close_client_and_cleanup(epfd, fd);
                                    std::cout << "🧾 Current clients: " << clients.size() << std::endl;
                                    continue;
                                }
                            }
                        } // unlock modbus ctx
                    }
                } // for nfds

                // perform periodic tasks (e.g., update registers)
                {
                    std::lock_guard<std::mutex> lk(mb_mutex);
                    // ---------------------
                    // bits (Coils)
                    // ---------------------
                    // 주소 영역: 0xxxx (00001–09999)
                    // 읽기: FC01
                    // 쓰기: FC05/15
                    // ---------------------
                    // input_bits (Discrete Inputs)
                    // ---------------------
                    // 주소 영역: 1xxxx (10001–19999)
                    // 읽기: FC02
                    // ---------------------
                    //  registers (Holding Registers)
                    // ---------------------
                    // 주소 영역: 4xxxx (40001–49999)
                    // 읽기: FC03
                    // 쓰기: FC06/16
                    // ---------------------
                    // input_registers (Input Registers)
                    // ---------------------
                    // 주소 영역: 3xxxx (30001–39999)
                    // 읽기: FC04
                    // ---------------------
                    // auto shm_ptr = rb_shareddata::get();
                    // for(int i = 0; i < MBUS_INPUT_REGS_NUM; ++i){
                    //     int tar_addr = MBUS_MAP_INPUT_REGS[i].shm_index;
                    //     float tar_scaler = MBUS_MAP_INPUT_REGS[i].scaler;

                    //     int shm_type = SHARED_DATA_TYPE[tar_addr];
                    //     if(shm_type == 0){
                    //         mb_mapping->tab_input_registers[i] = static_cast<uint16_t>(shm_ptr->idata[tar_addr]);
                    //     }else{
                    //         mb_mapping->tab_input_registers[i] = static_cast<uint16_t>(shm_ptr->fdata[tar_addr] / tar_scaler);
                    //     }
                    // }
                }
            } // while running

            // cleanup on exit
            std::cout << "🛑 Shutting down server...\n";

            // close all client sockets
            for (auto &p : clients) {
                int fd = p.first;
                // remove from epoll and close
                if (epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr) == -1) {
                    // ignore
                }
                close(fd);
            }
            clients.clear();

            // remove eventfd & server socket
            if (event_fd != -1) {
                if (epoll_ctl(epfd, EPOLL_CTL_DEL, event_fd, nullptr) == -1) { /*ignore*/ }
                close(event_fd);
                event_fd = -1;
            }
            if (server_socket != -1) {
                if (epoll_ctl(epfd, EPOLL_CTL_DEL, server_socket, nullptr) == -1) { /*ignore*/ }
                close(server_socket);
            }

            close(epfd);

            // free modbus mapping/context
            modbus_mapping_free(mb_mapping);
            modbus_free(ctx);

            std::cout << "Server stopped cleanly.\n";
            return nullptr;
        } // thread function
    } // anonymous namespace

    bool initialize(std::string domain, int th_cpu, int port_no){
        // start server thread
        running.store(true);
        sPORT_NO = port_no;
        if(rb_common::thread_create(thread_mbustcpserver, th_cpu, ("RB_" + domain + "_MBUS"), hThread_mbustcpserver, NULL) != 0){
            running.store(false);
            return false;
        }
        return true;
    }

    void shutdown() {
        // trigger eventfd to wake epoll_wait and ask thread to stop
        running.store(false);
        if (event_fd != -1) {
            uint64_t v = 1;
            ssize_t s = write(event_fd, &v, sizeof(v));
            if (s != sizeof(v)) {
                // write may fail if eventfd closed; ignore
            }
        }
    }
} // namespace rb_mbus_server
