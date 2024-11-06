QT       += core gui websockets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG += resources_big

QMAKE_CXXFLAGS_RELEASE += -O3

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
DEFINES += QT_NO_SIGNALS_SLOTS_KEYWORDS
DEFINES += QT_NO_KEYWORDS

# select mobile platform type
#DEFINES += USE_SRV
#DEFINES += USE_AMR_400
DEFINES += USE_AMR_400_LAKI
#DEFINES += USE_AMR_400_PROTO
#DEFINES += USE_AMR_KAI

# auto copy config files
#copy_config.commands = cp -r $$PWD/config $$OUT_PWD/
#copy_config.target = copy_config
#QMAKE_EXTRA_TARGETS += copy_config
#PRE_TARGETDEPS += copy_config
#.PHONY: copy_config

# set home dir
HOME = $$system(echo $HOME)
message($$HOME)

SOURCES += \
    LakiBeamHTTP.cpp \
    LakiBeamUDP.cpp \
    autocontrol.cpp \
    blidar/calibration.cpp \
    blidar/lidar_data_processing.cpp \
    blidar/lidar_information.cpp \
    blidar/mtime.cpp \
    blidar/node_lidar.cpp \
    blidar/point_cloud_optimize.cpp \
    blidar/serial_port.cpp \
    blidar/timer.cpp \
    cam.cpp \
    code_reader.cpp \
    comm_fms.cpp \
    comm_ms.cpp \
    comm_ui.cpp \
    dockingcontrol.cpp \
    config.cpp \
    cv_to_qt.cpp \
    lidar_2d.cpp \
    lidar_bottom.cpp \
    logger.cpp \
    main.cpp \
    mainwindow.cpp \
    mobile.cpp \
    task.cpp \
    obsmap.cpp \
    pgo.cpp \
    sim.cpp \
    slam_2d.cpp \
    unimap.cpp \
    utils.cpp

HEADERS += \
    LakiBeamHTTP.h \
    LakiBeamUDP.h \
    autocontrol.h \
    blidar/calibration.h \
    blidar/lidar_data_processing.h \
    blidar/lidar_information.h \
    blidar/locker.h \
    blidar/msg_recept.h \
    blidar/mtime.h \
    blidar/node_lidar.h \
    blidar/point_cloud_optimize.h \
    blidar/serial_port.h \
    blidar/timer.h \
    cam.h \
    code_reader.h \
    comm_fms.h \
    comm_ms.h \
    comm_ui.h \
    dockingcontrol.h \
    config.h \
    cv_to_qt.h \
    global_defines.h \
    lidar_2d.h \
    lidar_bottom.h \
    logger.h \
    mainwindow.h \
    mobile.h \
    task.h \
    nanoflann.hpp \
    obsmap.h \
    pgo.h \
    sim.h \
    slam_2d.h \
    tinycolormap.hpp \
    unimap.h \
    utils.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

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
        -lopencv_ximgproc

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

# OMPL
INCLUDEPATH += /usr/local/include/ompl-1.6/
LIBS += -L/usr/local/lib/
LIBS += -lompl

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
INCLUDEPATH += $$HOME/OrbbecSDK/include/
LIBS += -L$$HOME/OrbbecSDK/lib/linux_x64/
LIBS += -lOrbbecSDK

# rplidar sdk
INCLUDEPATH += $$HOME/rplidar_sdk/sdk/include/
LIBS += -L$$HOME/rplidar_sdk/output/Linux/Release/
LIBS += -lsl_lidar_sdk

# bottom lidar
INCLUDEPATH += $$PWD/blidar/
