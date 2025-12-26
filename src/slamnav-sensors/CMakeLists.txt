cmake_minimum_required(VERSION 3.16)
project(slamnav-sensors LANGUAGES C CXX)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Common compile options/defs ----
add_compile_options(-Werror=return-type)
add_compile_definitions(QT_NO_SIGNALS_SLOTS_KEYWORDS QT_NO_KEYWORDS)

# ---- Qt ----
set(QT_REQUIRED_COMPONENTS Core Gui Widgets)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})


# ---- Sources ----
file(GLOB_RECURSE SENSORS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/*/*/src/*.cpp
)
file(GLOB_RECURSE SENSORS_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/*/*/inc/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/*/*/inc/*.hpp
)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${SENSORS_SOURCES} ${SENSORS_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

# include dirs
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/cam/ORBBEC/inc
        ${LIBS_DIR}/OrbbecSDK/SDK/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC slamnav-common slamnav-robot)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_sensors")
