cmake_minimum_required(VERSION 3.16)
project(slamnav-common LANGUAGES C CXX)

# ---- paths ----
set(LIBS_DIR $ENV{HOME}/libs)

# ---- Qt (moc/uic/rcc) ----
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ---- Common compile options/defs ----
add_compile_options(-Werror=return-type)
add_compile_definitions(QT_NO_SIGNALS_SLOTS_KEYWORDS QT_NO_KEYWORDS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Qt ----
set(QT_REQUIRED_COMPONENTS Core Gui Widgets)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})

# ---- Dependencies ----
find_package(Threads REQUIRED)
find_package(OpenMP)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VTK 9.1 REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(fmt REQUIRED)
find_package(GTSAM CONFIG REQUIRED)

# ---- Sources ----
file(GLOB_RECURSE COMMON_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE COMMON_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# ---- Target ----
add_library(${PROJECT_NAME} SHARED ${COMMON_SOURCES} ${COMMON_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

# spdlog header-only
target_include_directories(${PROJECT_NAME} PUBLIC "${LIBS_DIR}/spdlog/include")
# target_compile_definitions(${PROJECT_NAME} PUBLIC SPDLOG_HEADER_ONLY)

# ---- includes ----
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ---- link ----
target_link_libraries(${PROJECT_NAME} PUBLIC
    # Qt
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets

    # deps
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    TBB::tbb
    Boost::filesystem
    Boost::system
    Boost::thread
    Threads::Threads
    fmt::fmt

    gtsam
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "slamnav_common")
