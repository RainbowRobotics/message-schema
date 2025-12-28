cmake_minimum_required(VERSION 3.16)
project(slamnav-navigation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----
find_package(GTSAM CONFIG REQUIRED)

# ---- Sources ----
file(GLOB_RECURSE NAVIGATION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE NAVIGATION_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Types target (INTERFACE) ----
add_library(slamnav-navigation-types INTERFACE)
target_include_directories(slamnav-navigation-types INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(slamnav-navigation-types INTERFACE
        slamnav-slam-types
        slamnav-sensors-types
)

# ---- Main library ----
add_library(${PROJECT_NAME} SHARED
        ${NAVIGATION_SOURCES}
        ${NAVIGATION_HEADERS}
        inc/slamnav_navigation_types.h
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        slamnav-navigation-types

        slamnav-common
        slamnav-robot
        slamnav-sensors
        slamnav-slam
        gtsam
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_navigation")
