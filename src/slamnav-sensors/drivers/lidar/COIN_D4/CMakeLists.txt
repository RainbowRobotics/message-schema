cmake_minimum_required(VERSION 3.16)
project(slamnav_sensors_coin_d4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Dependencies (CSPC bottom lidar SDK) ----
set(CSPC_LIDAR_SDK_DIR      "${LIBS_DIR}/CSPC_LIDAR_SDK_V7.3")
set(CSPC_LIDAR_INCLUDE_DIR  "${CSPC_LIDAR_SDK_DIR}/include")
set(CSPC_LIDAR_LIB          "${CSPC_LIDAR_SDK_DIR}/build/libcspclidar_driver.a")

# ---- Sources ----
file(GLOB_RECURSE COIN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE COIN_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} STATIC ${COIN_SOURCES} ${COIN_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CSPC_LIDAR_INCLUDE_DIR}
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC
    slamnav-common
    ${CSPC_LIDAR_LIB}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_sensors_coin_d4")
