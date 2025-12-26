cmake_minimum_required(VERSION 3.16)
project(SLAMNAV2 LANGUAGES C CXX)

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

# RS DRIVER (RoboSense)
find_package(rs_driver REQUIRED)

# Orbbec SDK
set(ORBBEC_SDK_DIR      "${LIBS_DIR}/OrbbecSDK/SDK")
set(ORBBEC_INCLUDE_DIR  "${ORBBEC_SDK_DIR}/include")
set(ORBBEC_LIB_DIR      "${ORBBEC_SDK_DIR}/lib")
find_library(ORBBEC_SDK_LIB
    NAMES OrbbecSDK libOrbbecSDK
    PATHS ${ORBBEC_LIB_DIR}
    NO_DEFAULT_PATH
)
if(NOT ORBBEC_SDK_LIB)
    message(FATAL_ERROR "OrbbecSDK library not found in: ${ORBBEC_LIB_DIR}")
endif()
add_library(Orbbec::SDK SHARED IMPORTED GLOBAL)
set_target_properties(Orbbec::SDK PROPERTIES
    IMPORTED_LOCATION "${ORBBEC_SDK_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${ORBBEC_INCLUDE_DIR}"
)

# RPLIDAR SDK
set(RPLIDAR_SDK_DIR      "${LIBS_DIR}/rplidar_sdk")
set(RPLIDAR_INCLUDE_DIR  "${RPLIDAR_SDK_DIR}/sdk/include")
set(RPLIDAR_LIB          "${RPLIDAR_SDK_DIR}/output/Linux/Release/libsl_lidar_sdk.a")

# CSPC bottom lidar SDK (Coin)
set(CSPC_LIDAR_SDK_DIR      "${LIBS_DIR}/CSPC_LIDAR_SDK_V7.3")
set(CSPC_LIDAR_INCLUDE_DIR  "${CSPC_LIDAR_SDK_DIR}/include")
set(CSPC_LIDAR_LIB          "${CSPC_LIDAR_SDK_DIR}/build/libcspclidar_driver.a")

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
    ${SRC_DIR}/cam
    ${SRC_DIR}/comm
    ${SRC_DIR}/lidar
    ${SRC_DIR}/qr_sensor

    # external SDK headers
    ${ORBBEC_INCLUDE_DIR}
    ${RPLIDAR_INCLUDE_DIR}
    ${CSPC_LIDAR_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIRS}
    ${rs_driver_INCLUDE_DIRS}
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PRIVATE
    slamnav-common

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

    Orbbec::SDK
    ${RPLIDAR_LIB}
    ${CSPC_LIDAR_LIB}

    ${rs_driver_LIBRARIES}
    sick_safetyscanners_base
    livox_lidar_sdk_shared

    ${SIOCLIENT_LIB}
    ${SIOCLIENT_TLS_LIB}
)

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
