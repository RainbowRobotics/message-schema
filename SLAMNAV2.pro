QT       += core gui websockets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
DEFINES += QT_NO_SIGNALS_SLOTS_KEYWORDS
DEFINES += QT_NO_KEYWORDS

SOURCES += \
    LakiBeamHTTP.cpp \
    LakiBeamUDP.cpp \
    complementary_filter.cpp \
    config.cpp \
    cv_to_qt.cpp \
    lidar_2d.cpp \
    logger.cpp \
    main.cpp \
    mainwindow.cpp \
    mobile.cpp \
    pgo.cpp \
    slam_2d.cpp \
    unimap.cpp \
    utils.cpp

HEADERS += \
    LakiBeamHTTP.h \
    LakiBeamUDP.h \
    complementary_filter.h \
    config.h \
    cv_to_qt.h \
    global_defines.h \
    lidar_2d.h \
    logger.h \
    mainwindow.h \
    mobile.h \
    nanoflann.hpp \
    pgo.h \
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

# GTSAM
INCLUDEPATH += /usr/local/include/gtsam/
LIBS += -L/usr/local/lib/
LIBS += -lgtsam \
        -lmetis-gtsam

# Lakibeam lidar
INCLUDEPATH += /usr/include/rapidjson/
INCLUDEPATH += /usr/include/boost/
INCLUDEPATH += /usr/include/boost/beast/
LIBS += -L/usr/lib/x86_64-linux-gnu/
LIBS += -lboost_system \
        -lboost_thread
