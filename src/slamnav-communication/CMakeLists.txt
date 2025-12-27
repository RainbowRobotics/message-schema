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

# ---- Sources ----
file(GLOB_RECURSE COMMUNICATION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE COMMUNICATION_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${COMMUNICATION_SOURCES} ${COMMUNICATION_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/src/ui/inc
    ${CMAKE_SOURCE_DIR}/src/slamnav-debug/inc
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC
    slamnav-common
    slamnav-robot
    slamnav-sensors
    slamnav-slam
    slamnav-navigation

    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::WebSockets
    Qt${QT_VERSION_MAJOR}::Gamepad

    ${SIOCLIENT_LIB}
    ${SIOCLIENT_TLS_LIB}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_communication")
