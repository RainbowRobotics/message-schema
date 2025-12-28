cmake_minimum_required(VERSION 3.16)
project(SLAMNAV2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)
set(SRC_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ---- Qt (moc/uic/rcc) ----
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ---- Common compile options/defs ----
add_compile_options(-Werror=return-type)
add_compile_definitions(QT_NO_SIGNALS_SLOTS_KEYWORDS QT_NO_KEYWORDS)

# ---- Qt ----
set(QT_REQUIRED_COMPONENTS Core Gui Widgets WebSockets Gamepad)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})

# ---- Dependencies ----

# ---- subdir: common library ----
add_subdirectory(${SRC_DIR}/slamnav-common)
add_subdirectory(${SRC_DIR}/slamnav-robot)
add_subdirectory(${SRC_DIR}/slamnav-sensors)
add_subdirectory(${SRC_DIR}/slamnav-slam)
add_subdirectory(${SRC_DIR}/slamnav-navigation)
add_subdirectory(${SRC_DIR}/slamnav-communication)
add_subdirectory(${SRC_DIR}/slamnav-debug)
add_subdirectory(${SRC_DIR}/slamnav-map)
add_subdirectory(${SRC_DIR}/ui)

# ---- SLAMNAV2 executable ----
add_executable(${PROJECT_NAME}
        ${SRC_DIR}/ui/src/main.cpp
)
#file(GLOB_RECURSE PROJECT_SOURCES ${SRC_DIR}/*.cpp)
#file(GLOB_RECURSE PROJECT_HEADERS ${SRC_DIR}/*.h ${SRC_DIR}/*.hpp)
#file(GLOB PROJECT_UI ${SRC_DIR}/*.ui)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

# ---- includes ----
target_include_directories(${PROJECT_NAME} PRIVATE
    ${SRC_DIR}
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PRIVATE
    slamnav-common
    slamnav-robot
    slamnav-sensors
    slamnav-slam
    slamnav-navigation
    slamnav-communication
    slamnav-debug
        slamnav-map
    ui
)

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
