QT       += core gui websockets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG += resources_big

# Optimize
QMAKE_CXXFLAGS_RELEASE += -O2
QMAKE_CXXFLAGS += -Werror=return-type

# OpenMP
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
DEFINES += QT_NO_SIGNALS_SLOTS_KEYWORDS
DEFINES += QT_NO_KEYWORDS

TARGET = SLAMNAV2
TEMPLATE = app

DESTDIR = ../

# set home dir
HOME = $$system(echo $HOME)
message($$HOME)

SOURCES += \
    comm/comm_coop.cpp \
    comm/comm_fms.cpp \
    comm/comm_rrs.cpp \
    autocontrol.cpp \
    cam.cpp \
    cam/ORBBEC/ORBBEC.cpp \
    dockcontrol.cpp \
    lidar/RP/rp_lidar.cpp \
    lidar/SICK/sick.cpp \
    lidar_2d.cpp \
    localization.cpp \
    main.cpp \
    mainwindow.cpp \
    config.cpp \
    mapping.cpp \
    my_utils.cpp \
    imu_filter.cpp \
    pgo.cpp \
    cv_to_qt.cpp \
    logger.cpp \
    qr_sensor.cpp \
    qr_sensor/GLS611/GLS611.cpp \
    unimap.cpp \
    mobile.cpp \
    lidar_3d.cpp \
    lidar/LIVOX/livox.cpp \
    obsmap.cpp \
    sim.cpp \
    policy.cpp


HEADERS += \
    comm/comm_coop.h \
    comm/comm_fms.h \
    comm/comm_rrs.h \
    autocontrol.h \
    cam.h \
    cam/ORBBEC/ORBBEC.h \
    comm_data.h \
    dockcontrol.h \
    lidar/RP/rp_lidar.h \
    lidar/SICK/sick.h \
    lidar_2d.h \
    localization.h \
    mainwindow.h \
    config.h \
    global_defines.h \
    mapping.h \
    my_utils.h \
    imu_filter.h \
    pgo.h \
    cv_to_qt.h \
    nanoflann.hpp \
    qr_sensor.h \
    qr_sensor/GLS611/GLS611.h \
    tinycolormap.hpp \
    logger.h \
    unimap.h \
    mobile.h \
    lidar_3d.h \
    lidar/LIVOX/livox.h \
    obsmap.h \
    sim.h \
    policy.h


FORMS += \
    mainwindow.ui

# OpenCV library all
INCLUDEPATH += /usr/include/opencv4/
LIBS += -L/usr/lib/x86_64-linux-gnu/
LIBS += -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgcodecs \
        -lopencv_imgproc \
        -lopencv_calib3d \
        -lopencv_features2d \
        -lopencv_flann \
        -lopencv_objdetect \
        -lopencv_photo \
        -lopencv_video \
        -lopencv_videoio \
        -lboost_system \
        -lopencv_ximgproc \
        -lopencv_aruco

# VTK
VTK_VERSION = 9.1
INCLUDEPATH += /usr/include/vtk-$${VTK_VERSION}
LIBS += -L/usr/local/lib/
LIBS += -lvtkCommonColor-$${VTK_VERSION} \
        -lvtkCommonExecutionModel-$${VTK_VERSION} \
        -lvtkCommonCore-$${VTK_VERSION} \
        -lvtkCommonDataModel-$${VTK_VERSION} \  # Para PCL
        -lvtkCommonMath-$${VTK_VERSION} \       # Para PCL
        -lvtkFiltersCore-$${VTK_VERSION} \
        -lvtkFiltersSources-$${VTK_VERSION} \
        -lvtkFiltersGeometry-$${VTK_VERSION} \
        -lvtkInfovisCore-$${VTK_VERSION} \
        -lvtkInteractionStyle-$${VTK_VERSION} \
        -lvtkRenderingContextOpenGL2-$${VTK_VERSION} \
        -lvtkRenderingCore-$${VTK_VERSION} \
        -lvtkRenderingFreeType-$${VTK_VERSION} \
        -lvtkRenderingGL2PSOpenGL2-$${VTK_VERSION} \
        -lvtkRenderingOpenGL2-$${VTK_VERSION} \
        -lvtkViewsQt-$${VTK_VERSION} \
        -lvtkGUISupportQt-$${VTK_VERSION} \
        -lvtkRenderingQt-$${VTK_VERSION} \
        -lvtkRenderingLOD-$${VTK_VERSION} \
        -ljsoncpp


# PCL
INCLUDEPATH += /usr/include/pcl-1.12
LIBS += -L/usr/lib/aarch64-linux-gnu/
LIBS += -lpcl_common \
        -lpcl_features \
        -lpcl_filters \
        -lpcl_io \
        -lpcl_io_ply \
        -lpcl_kdtree \
        -lpcl_keypoints \
        -lpcl_ml \
        -lpcl_octree \
        -lpcl_outofcore \
        -lpcl_people \
        -lpcl_recognition \
        -lpcl_registration \
        -lpcl_sample_consensus \
        -lpcl_search \
        -lpcl_segmentation \
        -lpcl_stereo \
        -lpcl_surface \
        -lpcl_tracking \
        -lpcl_visualization

# Eigen and Sophus library
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/local/include/sophus

# TBB
LIBS += -L/usr/lib/x86_64-linux-gnu/
LIBS += -ltbb

# Socket.io
INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib/
LIBS += -lsioclient \
        -lsioclient_tls

# GTSAM
INCLUDEPATH += /usr/local/include/gtsam/
LIBS += -L/usr/local/lib/
LIBS += -lgtsam \
        -lmetis-gtsam

# Octomap
INCLUDEPATH += /usr/local/include/octomap
LIBS += -L/usr/local/lib/
LIBS += -loctomath \
        -loctomap

# Lakibeam lidar
INCLUDEPATH += /usr/include/rapidjson/
INCLUDEPATH += /usr/include/boost/
INCLUDEPATH += /usr/include/boost/beast/
LIBS += -L/usr/lib/x86_64-linux-gnu/
LIBS += -lboost_system \
        -lboost_thread

# sick sdk
INCLUDEPATH += /usr/local/include/sick_safetyscanners_base/
LIBS += -L/usr/local/lib/
LIBS += -lsick_safetyscanners_base

# Gemini2L sdk
INCLUDEPATH += $$HOME/OrbbecSDK/SDK/include/
LIBS += -L$$HOME/OrbbecSDK/SDK/lib/
LIBS += -lOrbbecSDK

# rplidar sdk
INCLUDEPATH += $$HOME/rplidar_sdk/sdk/include/
LIBS += -L$$HOME/rplidar_sdk/output/Linux/Release/
LIBS += -lsl_lidar_sdk

# bottom lidar
INCLUDEPATH += $$PWD/blidar/

# PDAL
INCLUDEPATH += /usr/include/pdal
LIBS += -L/usr/lib/
LIBS += -lpdalcpp

# livox
INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib/
LIBS += -llivox_lidar_sdk_static

# spdlog
INCLUDEPATH += $$HOME/spdlog/include/

