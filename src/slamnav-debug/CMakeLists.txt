cmake_minimum_required(VERSION 3.16)
project(slamnav-debug LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Dependencies ----

# ---- Sources ----
file(GLOB_RECURSE DEBUG_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE DEBUG_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Types target (INTERFACE) ----
add_library(slamnav-debug-types INTERFACE)
target_include_directories(slamnav-debug-types INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(slamnav-debug-types INTERFACE
        slamnav-navigation-types
)

# ---- Main library ----
add_library(${PROJECT_NAME} SHARED ${DEBUG_SOURCES} ${DEBUG_HEADERS}
        inc/slamnav_debug_types.h)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

# ---- PCH ----
target_precompile_headers(${PROJECT_NAME} REUSE_FROM slamnav-common)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# ---- link ----
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        slamnav-common
        slamnav-navigation
    PRIVATE
        slamnav-robot
        slamnav-sensors
        slamnav-slam
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_debug")
