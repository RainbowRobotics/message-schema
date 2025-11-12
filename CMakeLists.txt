cmake_minimum_required(VERSION 3.10)
project(MyRBDLTest)
set(CMAKE_CXX_STANDARD 20)
# 릴리즈 버전
set(CMAKE_BUILD_TYPE Release)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") # KJ 만약에 ``apt install libgrpc++-dev grpc-proto``로 설치한 경우에 필요합니다.

# set(Flatbuffers_DIR "/usr/local/lib/cmake/flatbuffers") # KJ
find_package(Flatbuffers REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)
find_package(zenohc REQUIRED) # KJ ZENOH_C_PATH 환경설정 대신 find_package 사용
# libmodbus
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# Xenomai config
# KJ Xenomai 설정 프로그램(xeno-config)을 찾기 위한 코드 추가
find_program(XENO_CONFIG
    NAMES xeno-config
    PATHS
        /usr/xenomai/bin
        /usr/bin
)
if (NOT XENO_CONFIG)
    message(FATAL_ERROR "xeno-config not found. Please install Xenomai or specify XENO_CONFIG manually.")
else()
    message(STATUS "Found xeno-config: ${XENO_CONFIG}")
endif()

execute_process(
    COMMAND ${XENO_CONFIG} --alchemy --cflags
    OUTPUT_VARIABLE XENO_CFLAGS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND ${XENO_CONFIG} --alchemy --ldflags
    OUTPUT_VARIABLE XENO_LDFLAGS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Xenomai CFLAGS: ${XENO_CFLAGS}")
message(STATUS "Xenomai LDFLAGS: ${XENO_LDFLAGS}")

separate_arguments(XENO_CFLAGS)
separate_arguments(XENO_LDFLAGS)

add_subdirectory(rb_session)

# include 경로
include_directories(
    .
    rb_common
    rb_common/utils
    rb_core
    rb_hardware
    rb_ipc
    rb_motion
    rb_motion/types
    rb_service
    rb_service/joystick
    /usr/xenomai/include
    /usr/xenomai/include/alchemy
    /usr/xenomai/include/cobalt
    /usr/include/eigen3
    /usr/local/include

    # ${ZENOH_C_PATH}/src
    # ${ZENOH_C_PATH}/include

    ${PROTOBUF_INCLUDE_DIRS}
    ${GRPC_INCLUDE_DIRS}

    # ${LIBMODBUS_INCLUDE_DIRS}
)

# 라이브러리 경로
link_directories(/usr/local/lib
    # ${LIBMODBUS_LIBRARY_DIRS}
)

# 실행 파일 + 모든 소스 추가
add_executable(rb_program
    main.cpp
    rb_common/common.cpp
    rb_common/rmath.cpp
    rb_common/rquadprog.cpp
    rb_common/rmoduleinfo.cpp
    rb_common/sqlite_function.cpp
    rb_common/utils/QuadProg++.cc
    rb_common/utils/Array.cc
    
    rb_core/daemon.cpp
    rb_core/system.cpp
    rb_core/configcall.cpp

    rb_ipc/ipc.cpp
    
    rb_hardware/lan2can.cpp
    rb_hardware/scb_v1.cpp
    rb_hardware/side_io.cpp

    rb_hardware/motor.cpp
    rb_hardware/toolflange.cpp
    rb_hardware/ledlight.cpp

    rb_motion/motion.cpp
    rb_motion/types/move_j.cpp
    rb_motion/types/move_l.cpp
    rb_motion/types/move_jb.cpp
    rb_motion/types/move_lb.cpp
    rb_motion/types/move_xb.cpp
    rb_motion/types/move_cir.cpp
    rb_motion/types/move_speed_j.cpp
    rb_motion/types/move_speed_l.cpp

    rb_service/shareddata.cpp
    rb_service/joystick/joystick.cpp
    rb_service/mbus_server/mbustcp_server.cpp
    rb_service/socket_server/command_server.cpp
    rb_service/socket_server/data_server.cpp

    rb_motion/rrbdl.cpp
)

# 플래그 적용
target_compile_options(rb_program PRIVATE ${XENO_CFLAGS}
    # -Werror   # 경고를 에러로 처리
    # -Wall
    # -Wextra
    -Wuninitialized
    # un-used function 경고는 안뜨게
    # -Wno-unused-function
)

target_link_libraries(rb_program
    PRIVATE
    ${XENO_LDFLAGS}
    rbdl
    pthread
    nlohmann_json::nlohmann_json
    ${LIBMODBUS_LIBRARIES}
    protobuf::libprotobuf
    gRPC::grpc++
    zenohc::shared

    rb_session
    #${ZENOH_C_PATH}/target/release/libzenohc.a  # 정적 링크 시
    # 또는 그냥 zenohc  # 동적 링크 시 라이브러리가 /usr/local/lib 등에 있음
)

# 빌드 후 자동 strip
find_program(STRIP_EXECUTABLE strip)
if(STRIP_EXECUTABLE)
    add_custom_command(TARGET rb_program POST_BUILD
        COMMAND ${STRIP_EXECUTABLE} $<TARGET_FILE:rb_program>
        COMMENT "Stripping binary to reduce size"
    )
endif()