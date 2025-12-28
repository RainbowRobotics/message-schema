cmake_minimum_required(VERSION 3.16)
project(ui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Qt ----
set(QT_REQUIRED_COMPONENTS Core Gui Widgets WebSockets Gamepad)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})

# ---- Dependencies ----
find_package(VTK 9.1 REQUIRED)

# ---- Sources ----
file(GLOB_RECURSE UI_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE UI_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${UI_SOURCES} ${UI_HEADERS}
        inc/slamnav_ui_types.h)

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
    slamnav-sensors
    slamnav-slam
    slamnav-navigation
    slamnav-communication
    slamnav-debug
    ${VTK_LIBRARIES}

    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::WebSockets

)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "ui")
