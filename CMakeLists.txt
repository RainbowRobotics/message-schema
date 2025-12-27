cmake_minimum_required(VERSION 3.16)
project(SLAMNAV2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)
set(SRC_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ---- Qt (moc/uic/rcc) ----
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ---- Common compile options/defs ----
add_compile_options(-Werror=return-type)
add_compile_definitions(QT_NO_SIGNALS_SLOTS_KEYWORDS QT_NO_KEYWORDS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Qt ----
set(QT_REQUIRED_COMPONENTS Core Gui Widgets WebSockets Gamepad)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})

# ---- Dependencies ----
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL GLX)
find_package(VTK 9.1 REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(fmt REQUIRED)
find_package(GTSAM CONFIG REQUIRED)
find_package(PDAL CONFIG REQUIRED)

# octomap
find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED octomap)

# socket.io
find_library(SIOCLIENT_LIB
    NAMES sioclient libsioclient
    PATHS /usr/local/lib /usr/lib
)
find_library(SIOCLIENT_TLS_LIB
    NAMES sioclient_tls libsioclient_tls
    PATHS /usr/local/lib /usr/lib
)

# ---- subdir: common library ----
add_subdirectory(${SRC_DIR}/slamnav-common)
add_subdirectory(${SRC_DIR}/slamnav-robot)
add_subdirectory(${SRC_DIR}/slamnav-sensors)

# ---- SLAMNAV2 executable ----
file(GLOB_RECURSE PROJECT_SOURCES ${SRC_DIR}/*.cpp)
file(GLOB_RECURSE PROJECT_HEADERS ${SRC_DIR}/*.h ${SRC_DIR}/*.hpp)
file(GLOB PROJECT_UI ${SRC_DIR}/*.ui)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS} ${PROJECT_UI})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PRIVATE
    ${SRC_DIR}
    ${SRC_DIR}/comm
    ${SRC_DIR}/qr_sensor

    # external SDK headers
    ${OCTOMAP_INCLUDE_DIRS}
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PRIVATE
    slamnav-common
    slamnav-robot
    slamnav-sensors

    # Qt
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::WebSockets
    Qt${QT_VERSION_MAJOR}::Gamepad

    OpenMP::OpenMP_CXX

    # deps
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    TBB::tbb
    Boost::filesystem
    Boost::system
    Boost::thread
    Threads::Threads
    fmt::fmt
    gtsam

    pdal_base
    pdal_util

    ${OCTOMAP_LIBRARIES}

    ${SIOCLIENT_LIB}
    ${SIOCLIENT_TLS_LIB}
)

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
