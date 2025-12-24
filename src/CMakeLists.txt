cmake_minimum_required(VERSION 3.16)

project(SLAMNAV2 LANGUAGES C CXX)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_compile_options(-Werror=return-type)
add_compile_definitions(QT_NO_SIGNALS_SLOTS_KEYWORDS QT_NO_KEYWORDS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(HOME_DIR $ENV{HOME})

# ---- Qt ----
set(QT_REQUIRED_COMPONENTS Core Gui Widgets WebSockets Gamepad)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})

# ---- Threads ----
find_package(Threads REQUIRED)

# ---- OpenMP ----
find_package(OpenMP)

# ---- Eigen ----
find_package(Eigen3 REQUIRED)

# ---- OpenCV ----
find_package(OpenCV REQUIRED)

# ---- VTK ----
find_package(VTK 9.1 REQUIRED)

# ---- PCL ----
find_package(PCL 1.12 REQUIRED)

# ---- TBB ----
find_package(TBB REQUIRED)

# ---- Boost(system/thread) ----
find_package(Boost QUIET COMPONENTS system thread)

# ---- GTSAM ----
find_package(GTSAM QUIET)

# ---- RoboSense rs_driver (Airy) ----
list(APPEND CMAKE_PREFIX_PATH "/usr/local")
find_package(rs_driver QUIET)
set(RS_DRIVER_INCLUDE_DIR "/usr/local/rs_driver/include")

# Orbbec SDK
set(ORBBEC_SDK_DIR      "${HOME_DIR}/OrbbecSDK/SDK")
set(ORBBEC_INCLUDE_DIR  "${ORBBEC_SDK_DIR}/include")
set(ORBBEC_LIB_DIR      "${ORBBEC_SDK_DIR}/lib")

# RPLIDAR SDK
set(RPLIDAR_DIR         "${HOME_DIR}/rplidar_sdk")
set(RPLIDAR_INCLUDE_DIR "${RPLIDAR_DIR}/sdk/include")
set(RPLIDAR_LIB_DIR     "${RPLIDAR_DIR}/output/Linux/Release")

# fmt
find_package(fmt REQUIRED)

# CSPC bottom lidar SDK
set(CSPC_LIDAR_SDK_DIR   "${HOME_DIR}/CSPC_LIDAR_SDK_V7.3")
set(CSPC_LIDAR_BUILD_DIR "${CSPC_LIDAR_SDK_DIR}/build")

# -----------------------------
# slamnav-common (subproject)
# 경로는 실제 폴더 위치에 맞춰서 수정.
# 예) 최상위/src/slamnav-common 라면 아래 그대로.
# -----------------------------
add_subdirectory(slamnav-common)

# -----------------------------
# Main executable sources
# (common으로 옮긴 cpp들은 여기서 제거!)
# -----------------------------
set(PROJECT_SOURCES
    comm/comm_coop.cpp
    comm/comm_fms.cpp
    comm/comm_msa.cpp
    autocontrol.cpp
    cam.cpp
    cam/ORBBEC/ORBBEC.cpp
    dockcontrol.cpp
    ekf.cpp
    ekf_3d.cpp
    lidar/RP/rp_lidar.cpp
    lidar/SICK/sick.cpp
    lidar/LAKI/LakiBeamHTTP.cpp
    lidar/LAKI/LakiBeamUDP.cpp
    lidar/LAKI/laki.cpp
    lidar_2d.cpp
    localization.cpp
    main.cpp
    mainwindow.cpp
    mapping.cpp
    imu_filter.cpp
    pgo.cpp
    qr_sensor.cpp
    qr_sensor/GLS611/GLS611.cpp
    unimap.cpp
    mobile.cpp
    lidar_3d.cpp
    lidar/LIVOX/livox.cpp
    lidar/AIRY/airy.cpp
    obsmap.cpp
    sim.cpp
    policy.cpp
    safety.cpp
    task.cpp
)

set(PROJECT_HEADERS
    comm/comm_coop.h
    comm/comm_fms.h
    comm/comm_msa.h
    autocontrol.h
    cam.h
    cam/ORBBEC/ORBBEC.h
    comm_data.h
    dockcontrol.h
    ekf.h
    ekf_3d.h
    lidar/RP/rp_lidar.h
    lidar/SICK/sick.h
    lidar/LAKI/LakiBeamHTTP.h
    lidar/LAKI/LakiBeamUDP.h
    lidar/LAKI/laki.h
    lidar_2d.h
    localization.h
    mainwindow.h
    mapping.h
    imu_filter.h
    pgo.h
    nanoflann.hpp
    qr_sensor.h
    qr_sensor/GLS611/GLS611.h
    tinycolormap.hpp
    unimap.h
    mobile.h
    lidar_3d.h
    lidar/LIVOX/livox.h
    lidar/AIRY/airy.h
    obsmap.h
    sim.h
    policy.h
    safety.h
    task.h
)

set(PROJECT_UI
    mainwindow.ui
)

add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
    ${PROJECT_HEADERS}
    ${PROJECT_UI}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src

    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}

    ${ORBBEC_INCLUDE_DIR}
    ${RPLIDAR_INCLUDE_DIR}

    /usr/include/eigen3
    /usr/local/include/sophus

    /usr/include/rapidjson
    /usr/include/boost
    /usr/include/boost/beast

    ${CMAKE_SOURCE_DIR}/src/blidar
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ${PCL_DEFINITIONS}
)

target_link_directories(${PROJECT_NAME} PRIVATE
    ${PCL_LIBRARY_DIRS}
    ${ORBBEC_LIB_DIR}
    ${RPLIDAR_LIB_DIR}
    /usr/local/lib
    /usr/lib
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    # --- common ---
    slamnav_common

    # Qt
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::WebSockets
    Qt${QT_VERSION_MAJOR}::Gamepad

    # deps
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    TBB::tbb
    Threads::Threads
    fmt::fmt
)

# OpenMP
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

# Boost
if(Boost_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE Boost::system Boost::thread)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE boost_system boost_thread)
endif()

# Orbbec / RPLIDAR (필수라면)
target_link_libraries(${PROJECT_NAME} PRIVATE OrbbecSDK sl_lidar_sdk)

# GTSAM
if(TARGET GTSAM::gtsam)
    target_link_libraries(${PROJECT_NAME} PRIVATE GTSAM::gtsam)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE gtsam metis-gtsam)
endif()

# Airy rs_driver (필요할 때만)
if(EXISTS "${RS_DRIVER_INCLUDE_DIR}/rs_driver/api/lidar_driver.hpp")
    target_include_directories(${PROJECT_NAME} PRIVATE ${RS_DRIVER_INCLUDE_DIR})
else()
    message(STATUS "rs_driver header not found; build will continue without rs_driver include")
endif()

if(TARGET rs_driver::rs_driver)
    target_link_libraries(${PROJECT_NAME} PRIVATE rs_driver::rs_driver)
elseif(TARGET rs_driver)
    target_link_libraries(${PROJECT_NAME} PRIVATE rs_driver)
endif()

# libpcap (필요하면 fatal 유지)
find_library(PCAP_LIB NAMES pcap HINTS /usr/lib /usr/local/lib)
if(PCAP_LIB)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PCAP_LIB})
else()
    message(FATAL_ERROR "libpcap not found. Install: sudo apt-get install libpcap-dev")
endif()

# Octomap / PDAL (optional)
find_library(OCTOMAP_LIB   NAMES octomap  PATHS /usr/lib /usr/local/lib)
find_library(OCTOMATH_LIB  NAMES octomath PATHS /usr/lib /usr/local/lib)
if(OCTOMAP_LIB AND OCTOMATH_LIB)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OCTOMAP_LIB} ${OCTOMATH_LIB})
endif()

find_library(PDALCPP_LIB NAMES pdalcpp PATHS /usr/lib /usr/local/lib)
if(PDALCPP_LIB)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PDALCPP_LIB})
endif()

# Socket.IO / SICK / Livox / jsoncpp (optional)
set(_HINT_LIB_DIRS
    /usr/lib
    /usr/local/lib
    ${ORBBEC_LIB_DIR}
    ${RPLIDAR_LIB_DIR}
    ${HOME_DIR}/.local/lib
)

set(_CANDIDATE_LIBS
    sioclient
    sioclient_tls
    sick_safetyscanners_base
    livox_lidar_sdk_static
    jsoncpp
)

foreach(lib IN LISTS _CANDIDATE_LIBS)
    find_library(${lib}_FOUND NAMES ${lib} HINTS ${_HINT_LIB_DIRS})
    if(${lib}_FOUND)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${${lib}_FOUND})
    endif()
endforeach()

target_link_libraries(${PROJECT_NAME} PRIVATE pthread dl)

# CSPC optional
if(EXISTS "${CSPC_LIDAR_SDK_DIR}/include" AND EXISTS "${CSPC_LIDAR_SDK_DIR}/src")
    target_include_directories(${PROJECT_NAME} PRIVATE
        "${CSPC_LIDAR_SDK_DIR}/include"
        "${CSPC_LIDAR_SDK_DIR}/src"
    )
endif()

find_library(CSPC_LIDAR_DRIVER_LIB
    NAMES cspclidar_driver
    HINTS "${CSPC_LIDAR_BUILD_DIR}"
          "${CSPC_LIDAR_SDK_DIR}/lib"
          /usr/local/lib /usr/lib
)
if(CSPC_LIDAR_DRIVER_LIB)
    target_link_libraries(${PROJECT_NAME} PRIVATE "${CSPC_LIDAR_DRIVER_LIB}")
endif()

# RPATH
set(_RPATH_DIRS
    /usr/local/lib
    /usr/lib
    ${ORBBEC_LIB_DIR}
    ${RPLIDAR_LIB_DIR}
    ${HOME_DIR}/.local/lib
    ${CSPC_LIDAR_BUILD_DIR}
    /usr/local/rs_driver/lib
)

list(JOIN _RPATH_DIRS ":" _RPATH_JOINED)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${_RPATH_JOINED}"
    INSTALL_RPATH "${_RPATH_JOINED}"
    BUILD_WITH_INSTALL_RPATH ON
    SKIP_BUILD_RPATH OFF
)

vtk_module_autoinit(
    TARGETS ${PROJECT_NAME}
    MODULES ${VTK_LIBRARIES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../"
)
