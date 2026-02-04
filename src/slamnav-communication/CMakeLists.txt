cmake_minimum_required(VERSION 3.16)
project(slamnav-communication LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Qt ----
set(CMAKE_AUTOMOC ON)
set(QT_REQUIRED_COMPONENTS Core WebSockets Gamepad)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})

# ---- Dependencies ----
# socket.io
find_library(SIOCLIENT_LIB
    NAMES sioclient libsioclient
    PATHS /usr/local/lib /usr/lib
)
find_library(SIOCLIENT_TLS_LIB
    NAMES sioclient_tls libsioclient_tls
    PATHS /usr/local/lib /usr/lib
)

# zenoh
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

# flatbuffers
set(FLATBUFFERS_INCLUDE_DIR /tmp/flatbuffers/include)
set(FLATBUFFERS_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc/flatbuffer/generated)

# ---- Sources ----
file(GLOB_RECURSE COMMUNICATION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE COMMUNICATION_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Types target (INTERFACE) ----
add_library(slamnav-communication-types INTERFACE)
target_include_directories(slamnav-communication-types INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(slamnav-communication-types INTERFACE
        slamnav-navigation-types
)

# ---- Main library ----
add_library(${PROJECT_NAME} SHARED ${COMMUNICATION_SOURCES} ${COMMUNICATION_HEADERS}
        inc/slamnav_communication_types.h)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/src/ui/inc
    ${CMAKE_SOURCE_DIR}/src/slamnav-debug/inc
    ${FLATBUFFERS_INCLUDE_DIR}
    ${FLATBUFFERS_GENERATED_DIR}
)

# ---- link ----
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        slamnav-common
        slamnav-robot
        slamnav-sensors
        slamnav-slam
        slamnav-navigation
        Qt${QT_VERSION_MAJOR}::WebSockets
        zenohcxx::zenohc
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        ${SIOCLIENT_LIB}
        ${SIOCLIENT_TLS_LIB}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_communication")
