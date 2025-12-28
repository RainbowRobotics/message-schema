cmake_minimum_required(VERSION 3.16)
project(slamnav-slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Dependencies ----
find_package(PDAL CONFIG REQUIRED)
find_package(GTSAM CONFIG REQUIRED)

# octomap
find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED octomap)

# ---- Sources ----
file(GLOB_RECURSE SLAM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE SLAM_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Types target (INTERFACE) ----
add_library(slamnav-slam-types INTERFACE)
target_include_directories(slamnav-slam-types INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(slamnav-slam-types INTERFACE
        slamnav-sensors-types
        slamnav-map-types
)

# ---- Main library ----
add_library(${PROJECT_NAME} SHARED ${SLAM_SOURCES} ${SLAM_HEADERS}
        inc/slamnav_slam_types.h)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${OCTOMAP_INCLUDE_DIRS}
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC
    slamnav-common
    slamnav-robot
    slamnav-sensors
        slamnav-map

    gtsam

    pdal_base
    pdal_util

    ${OCTOMAP_LIBRARIES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_slam")
