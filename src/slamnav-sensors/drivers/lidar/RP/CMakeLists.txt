cmake_minimum_required(VERSION 3.16)
project(slamnav_sensors_rp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Dependencies (RPLIDAR SDK) ----
set(RPLIDAR_SDK_DIR      "${LIBS_DIR}/rplidar_sdk")
set(RPLIDAR_INCLUDE_DIR  "${RPLIDAR_SDK_DIR}/sdk/include")
set(RPLIDAR_LIB          "${RPLIDAR_SDK_DIR}/output/Linux/Release/libsl_lidar_sdk.a")

# ---- Sources ----
file(GLOB_RECURSE RP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE RP_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${RP_SOURCES} ${RP_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
    ${RPLIDAR_INCLUDE_DIR}
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC
    slamnav-common
    slamnav-robot
    ${RPLIDAR_LIB}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_sensors_rp")
