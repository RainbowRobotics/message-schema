cmake_minimum_required(VERSION 3.16)
project(slamnav-robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Sources ----
file(GLOB_RECURSE ROBOT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE ROBOT_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${ROBOT_SOURCES} ${ROBOT_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

# ---- includes ----
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC
        slamnav-common
        Qt${QT_VERSION_MAJOR}::Gamepad
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_robot")
