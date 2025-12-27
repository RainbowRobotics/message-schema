cmake_minimum_required(VERSION 3.16)
project(slamnav_sensors_livox LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Dependencies ----


# ---- Sources ----
file(GLOB_RECURSE LIVOX_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE LIVOX_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${LIVOX_SOURCES} ${LIVOX_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC
    slamnav-common
    slamnav-robot
    livox_lidar_sdk_shared
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_sensors_livox")
