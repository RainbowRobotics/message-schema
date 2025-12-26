cmake_minimum_required(VERSION 3.16)
project(slamnav_sensors_orbbec LANGUAGES CXX)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Dependencies ----
# Orbbec SDK
set(ORBBEC_SDK_DIR      "${LIBS_DIR}/OrbbecSDK/SDK")
set(ORBBEC_INCLUDE_DIR  "${ORBBEC_SDK_DIR}/include")
set(ORBBEC_LIB_DIR      "${ORBBEC_SDK_DIR}/lib")
find_library(ORBBEC_SDK_LIB
    NAMES OrbbecSDK libOrbbecSDK
    PATHS ${ORBBEC_LIB_DIR}
    NO_DEFAULT_PATH
)
if(NOT ORBBEC_SDK_LIB)
    message(FATAL_ERROR "OrbbecSDK library not found in: ${ORBBEC_LIB_DIR}")
endif()
add_library(Orbbec::SDK SHARED IMPORTED GLOBAL)
set_target_properties(Orbbec::SDK PROPERTIES
    IMPORTED_LOCATION "${ORBBEC_SDK_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${ORBBEC_INCLUDE_DIR}"
)

# ---- Sources ----
file(GLOB_RECURSE ORBBEC_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE ORBBEC_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${ORBBEC_SOURCES} ${ORBBEC_HEADERS})

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
    Orbbec::SDK
)

set_target_properties(slamnav_sensors_orbbec PROPERTIES BUILD_RPATH "${ORBBEC_LIB_DIR}")
