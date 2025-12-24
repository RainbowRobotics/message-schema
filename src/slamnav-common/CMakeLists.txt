cmake_minimum_required(VERSION 3.16)

project(slamnav-common LANGUAGES C CXX)

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

# -----------------------
# Qt (global_defines.h에 QApplication 등 포함됨)
# -----------------------
set(QT_REQUIRED_COMPONENTS Core Gui Widgets Concurrent)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})

# -----------------------
# Threads / OpenMP
# -----------------------
find_package(Threads REQUIRED)
find_package(OpenMP)

# -----------------------
# Eigen / OpenCV / PCL / TBB
# -----------------------
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(TBB REQUIRED)

# -----------------------
# VTK
# -----------------------
find_package(VTK 9.1 REQUIRED)

# MPI는 VTK가 요구할 수 있는데, common 단독/전체 모두에서 안전하게:
# - C/CXX 둘 다 enable 되어 있으니 C도 가능
# - 다만 MPI가 없을 수도 있으니 QUIET
find_package(MPI QUIET COMPONENTS C CXX)

# -----------------------
# spdlog / fmt
# -----------------------
find_package(spdlog QUIET)
find_package(fmt REQUIRED)

# -----------------------
# Sophus (보통 헤더 only)
# -----------------------
find_package(Sophus QUIET)

# -----------------------
# Library: slamnav_common
# -----------------------
add_library(slamnav_common STATIC)

target_sources(slamnav_common PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cv_to_qt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/error_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/my_utils.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/inc/config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/cv_to_qt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/error_manager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/logger.h
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/my_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/global_defines.h
)

# -----------------------
# Include dirs (일단 최대한 넓게)
# -----------------------
target_include_directories(slamnav_common
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
        ${EIGEN3_INCLUDE_DIRS}

        ${CMAKE_CURRENT_LIST_DIR}/..
        ${CMAKE_CURRENT_LIST_DIR}/../src
        ${CMAKE_CURRENT_LIST_DIR}/../thirdparty
        ${CMAKE_CURRENT_LIST_DIR}/../external
        ${CMAKE_CURRENT_LIST_DIR}/../include

        /usr/include/eigen3
        /usr/local/include
        /usr/local/include/sophus
        /usr/include

        /usr/include/rapidjson
        /usr/include/boost
        /usr/include/boost/beast
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
)

# Sophus target 있으면 사용
if(TARGET Sophus::Sophus)
    target_link_libraries(slamnav_common PUBLIC Sophus::Sophus)
else()
    target_include_directories(slamnav_common PUBLIC /usr/local/include/sophus)
endif()

# PCL defines
target_compile_definitions(slamnav_common PRIVATE ${PCL_DEFINITIONS})

# link directories
target_link_directories(slamnav_common PRIVATE
    ${PCL_LIBRARY_DIRS}
    /usr/local/lib
    /usr/lib
)

# -----------------------
# Link libs (가능한 건 다 링크)
# -----------------------
target_link_libraries(slamnav_common
    PUBLIC
        Eigen3::Eigen
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Concurrent
    PRIVATE
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
        TBB::tbb
        Threads::Threads
        fmt::fmt
)

# spdlog (없어도 되게)
if(TARGET spdlog::spdlog)
    target_link_libraries(slamnav_common PRIVATE spdlog::spdlog)
else()
    if(EXISTS "${HOME_DIR}/spdlog/include")
        target_include_directories(slamnav_common PRIVATE "${HOME_DIR}/spdlog/include")
    endif()
endif()

# OpenMP
if(OpenMP_CXX_FOUND)
    target_link_libraries(slamnav_common PRIVATE OpenMP::OpenMP_CXX)
endif()

# MPI (있으면 링크)
if(MPI_C_FOUND)
    target_link_libraries(slamnav_common PRIVATE MPI::MPI_C)
endif()
if(MPI_CXX_FOUND)
    target_link_libraries(slamnav_common PRIVATE MPI::MPI_CXX)
endif()

# Octomap (없으면 warning만)
find_library(OCTOMAP_LIB   NAMES octomap  PATHS /usr/lib /usr/local/lib)
find_library(OCTOMATH_LIB  NAMES octomath PATHS /usr/lib /usr/local/lib)
if(OCTOMAP_LIB AND OCTOMATH_LIB)
    target_link_libraries(slamnav_common PRIVATE ${OCTOMAP_LIB} ${OCTOMATH_LIB})
else()
    message(WARNING "OctoMap not found; skipping")
endif()

# PDAL (없으면 warning만)
find_library(PDALCPP_LIB NAMES pdalcpp PATHS /usr/lib /usr/local/lib)
if(PDALCPP_LIB)
    target_link_libraries(slamnav_common PRIVATE ${PDALCPP_LIB})
else()
    message(WARNING "PDAL (pdalcpp) not found; skipping")
endif()

# pthread / dl
target_link_libraries(slamnav_common PRIVATE pthread dl)

# -----------------------
# VTK autoinit
# -----------------------
vtk_module_autoinit(
    TARGETS slamnav_common
    MODULES ${VTK_LIBRARIES}
)

# -----------------------
# RPATH
# -----------------------
set(_RPATH_DIRS
    /usr/local/lib
    /usr/lib
    ${HOME_DIR}/.local/lib
)

list(JOIN _RPATH_DIRS ":" _RPATH_JOINED)

set_target_properties(slamnav_common PROPERTIES
    BUILD_RPATH "${_RPATH_JOINED}"
    INSTALL_RPATH "${_RPATH_JOINED}"
    BUILD_WITH_INSTALL_RPATH ON
    SKIP_BUILD_RPATH OFF
)
