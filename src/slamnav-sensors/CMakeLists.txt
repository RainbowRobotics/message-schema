cmake_minimum_required(VERSION 3.16)
project(slamnav-sensors LANGUAGES CXX)

# ---- add drivers ----
add_subdirectory(drivers/cam/ORBBEC)

# ---- Sources ----
file(GLOB_RECURSE SENSORS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE SENSORS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${SENSORS_SOURCES} ${SENSORS_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC
    slamnav_sensors_orbbec
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_sensors")
