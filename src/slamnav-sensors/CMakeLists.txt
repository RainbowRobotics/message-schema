cmake_minimum_required(VERSION 3.16)
project(slamnav-sensors LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(slamnav-sensors-types INTERFACE)
target_include_directories(slamnav-sensors-types INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(slamnav-sensors-types INTERFACE
        slamnav-robot-types
)

# ---- Dependencies ----
find_package(OpenCV REQUIRED)

# ---- add drivers ----
add_subdirectory(drivers/cam/ORBBEC)
add_subdirectory(drivers/lidar/LIVOX)
add_subdirectory(drivers/lidar/AIRY)
add_subdirectory(drivers/lidar/SICK)
add_subdirectory(drivers/lidar/RP)
add_subdirectory(drivers/lidar/LAKI)
add_subdirectory(drivers/lidar/COIN_D4)
add_subdirectory(drivers/qr_sensor/GLS611)

# ---- Sources ----
file(GLOB_RECURSE SENSORS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE SENSORS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${SENSORS_SOURCES} ${SENSORS_HEADERS}
        inc/slamnav_sensor_types.h
        inc/aruco.h)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

# ---- link ----
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${OpenCV_LIBS}
        slamnav_sensors_rp
        slamnav_sensors_airy
        slamnav_sensors_sick
        slamnav_sensors_orbbec
        slamnav_sensors_livox
    PRIVATE
        slamnav_sensors_laki
        slamnav_sensors_coin_d4
        slamnav_sensors_gls611
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_sensors")
