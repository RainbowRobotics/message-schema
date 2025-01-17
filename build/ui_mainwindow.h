/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKOpenGLNativeWidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QTabWidget *main_tab;
    QWidget *page_slamnav;
    QVBoxLayout *verticalLayout_7;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QComboBox *cb_ViewType;
    QComboBox *cb_ViewHeight;
    QPushButton *bt_ViewReset;
    QPushButton *bt_SetTopView;
    QSpacerItem *horizontalSpacer;
    QLabel *lb_RobotId;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_3;
    QCheckBox *ckb_PlotLive;
    QCheckBox *ckb_PlotNodes;
    QCheckBox *ckb_PlotEdges;
    QCheckBox *ckb_PlotNames;
    QCheckBox *ckb_PlotEnable;
    QCheckBox *ckb_Panning;
    QVTKOpenGLNativeWidget *qvtkWidget;
    QLabel *lb_Screen1;
    QLabel *lb_Screen2;
    QLabel *lb_Screen3;
    QLabel *lb_Screen4;
    QLabel *lb_Screen5;
    QPushButton *bt_ViewLeft;
    QPushButton *bt_ViewRight;
    QPushButton *bt_ViewUp;
    QPushButton *bt_ViewDown;
    QPushButton *bt_ViewZoomIn;
    QPushButton *bt_ViewZoomOut;
    QLabel *lb_NodeId;
    QLabel *lb_RobotVel;
    QLabel *lb_PickPose;
    QLabel *lb_LogInfo;
    QWidget *page_annot;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer_2;
    QCheckBox *ckb_PlotNodes2;
    QCheckBox *ckb_PlotEdges2;
    QCheckBox *ckb_PlotNames2;
    QCheckBox *ckb_Panning2;
    QVTKOpenGLNativeWidget *qvtkWidget2;
    QPushButton *bt_ViewLeft2;
    QPushButton *bt_ViewRight2;
    QPushButton *bt_ViewUp2;
    QPushButton *bt_ViewZoomIn2;
    QPushButton *bt_ViewZoomOut2;
    QPushButton *bt_ViewDown2;
    QLabel *lb_Measure;
    QLabel *lb_PickPose2;
    QTabWidget *menu_tab;
    QWidget *tab_S;
    QVBoxLayout *verticalLayout_6;
    QVBoxLayout *verticalLayout_2;
    QTabWidget *tabWidget;
    QWidget *tab;
    QPushButton *bt_MapBuild;
    QPushButton *bt_MapSave;
    QPushButton *bt_MapLoad;
    QPushButton *bt_LocInit;
    QPushButton *bt_LocInitAuto;
    QPushButton *bt_LocStart;
    QPushButton *bt_LocStop;
    QPushButton *bt_SimInit;
    QPushButton *bt_ObsClear;
    QPushButton *bt_ConfigLoad;
    QPushButton *bt_LocInitSemiAuto;
    QPushButton *bt_MapLastLc;
    QWidget *tab_2;
    QPushButton *bt_AutoMove;
    QPushButton *bt_AutoMove2;
    QPushButton *bt_AutoMove3;
    QPushButton *bt_AutoStop;
    QPushButton *bt_AutoPause;
    QPushButton *bt_AutoResume;
    QPushButton *bt_DockingMove;
    QPushButton *bt_DockingStop;
    QLabel *lb_CodeInfo;
    QPushButton *bt_Undock;
    QWidget *tab_3;
    QPushButton *bt_TaskDel;
    QPushButton *bt_TaskPause;
    QCheckBox *ckb_Looping;
    QPushButton *bt_TaskPlay;
    QPushButton *bt_TaskAdd;
    QPushButton *bt_TaskCancel;
    QPushButton *bt_TaskLoad;
    QPushButton *bt_TaskResume;
    QPushButton *bt_TaskSave;
    QListWidget *lw_TaskList;
    QLabel *label_7;
    QComboBox *cb_TaskDrivingMode;
    QWidget *tab_6;
    QLabel *lb_FmsInfo;
    QPushButton *bt_SendMap;
    QWidget *tab_4;
    QLabel *lb_SystemInfo;
    QCheckBox *ckb_RecordSystemInfo;
    QWidget *tab_5;
    QPushButton *bt_Sync;
    QPushButton *bt_MoveLinear;
    QPushButton *bt_MoveRotate;
    QDoubleSpinBox *dsb_MoveLinearDist;
    QDoubleSpinBox *dsb_MoveLinearV;
    QDoubleSpinBox *dsb_MoveRotateDeg;
    QDoubleSpinBox *dsb_MoveRotateW;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QPushButton *bt_Test;
    QSpinBox *spb_Led;
    QPushButton *bt_TestLed;
    QCheckBox *ckb_TestDebug;
    QPushButton *bt_TestImgSave;
    QLabel *lb_SlamInfo;
    QLabel *lb_MobilePoseInfo;
    QLabel *lb_MobileStatusInfo;
    QLabel *lb_QueInfo;
    QLabel *lb_AutoInfo;
    QWidget *tab_A;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_5;
    QFrame *frame_2;
    QPushButton *bt_MapLoad2;
    QPushButton *bt_MapSave2;
    QPushButton *bt_ClearTopo;
    QPushButton *bt_QuickAnnotStart;
    QPushButton *bt_QuickAnnotStop;
    QPushButton *bt_MapReload;
    QPushButton *bt_QuickAddNode;
    QPushButton *bt_QuickAddAruco;
    QPushButton *bt_QuickAddCloud;
    QTabWidget *annot_tab;
    QWidget *page_edit_map;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_21;
    QDoubleSpinBox *dsb_MapPosX;
    QDoubleSpinBox *dsb_MapPosY;
    QDoubleSpinBox *dsb_MapPosTh;
    QDoubleSpinBox *dsb_MapEraseSize;
    QLabel *label_22;
    QPushButton *bt_SetMapTf;
    QWidget *page_edit_topo;
    QPushButton *bt_EditNodeInfo;
    QPushButton *bt_AddLink2;
    QPushButton *bt_EditNodePos;
    QPushButton *bt_AddNode;
    QPushButton *bt_AddLink1;
    QComboBox *cb_NodeType;
    QPushButton *bt_AlignNodeX;
    QPushButton *bt_MinGapNodeY;
    QPushButton *bt_MinGapNodeX;
    QDoubleSpinBox *spb_NodeGap;
    QPushButton *bt_AlignNodeY;
    QPushButton *bt_AlignNodeTh;
    QLabel *label_6;
    QLabel *label_8;
    QLabel *label_23;
    QTextEdit *te_NodeInfo;
    QPushButton *bt_EditNodeType;
    QLabel *lb_PreNodeInfo;
    QLabel *lb_CurNodeInfo;
    QDoubleSpinBox *spb_NodeSizeX;
    QDoubleSpinBox *spb_NodeSizeY;
    QDoubleSpinBox *spb_NodeSizeZ;
    QLabel *label_24;
    QGroupBox *groupBox;
    QLabel *label_15;
    QLineEdit *le_CurNodeTh;
    QLabel *label_16;
    QPushButton *bt_NodePoseThDown;
    QLineEdit *le_CurNodeY;
    QLabel *label_14;
    QLineEdit *le_CurNodeX;
    QDoubleSpinBox *spb_NodeMovementXY;
    QPushButton *bt_NodePoseYDown;
    QPushButton *bt_NodePoseYUp;
    QCheckBox *ckb_TransformEntireNode;
    QDoubleSpinBox *spb_NodeMovementTh;
    QPushButton *bt_NodePoseXDown;
    QPushButton *bt_NodePoseThUp;
    QPushButton *bt_NodePoseXUp;
    QPushButton *bt_EditNodeName;
    QPushButton *bt_AutoLink;
    QComboBox *cb_NodeAdvanceType;
    QLabel *label_9;
    QCheckBox *ckb_UseMeasure;
    QPushButton *bt_DelNode;
    QPushButton *bt_SelectPreNodes;
    QPushButton *bt_SelectPostNodes;
    QPushButton *bt_AutoNode;
    QLabel *lb_NodeAlignInfo;
    QPushButton *bt_AddPoints;
    QFrame *frame_3;
    QPushButton *bt_JogF;
    QPushButton *bt_JogB;
    QPushButton *bt_JogL;
    QPushButton *bt_JogR;
    QDoubleSpinBox *spb_JogV;
    QDoubleSpinBox *spb_JogW;
    QPushButton *bt_MotorInit;
    QPushButton *bt_Emergency;
    QDoubleSpinBox *spb_AccV;
    QDoubleSpinBox *spb_DecelV;
    QDoubleSpinBox *spb_AccW;
    QDoubleSpinBox *spb_DecelW;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1250, 800);
        QFont font;
        font.setPointSize(8);
        MainWindow->setFont(font);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setSpacing(0);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        main_tab = new QTabWidget(centralwidget);
        main_tab->setObjectName(QString::fromUtf8("main_tab"));
        main_tab->setFont(font);
        main_tab->setTabPosition(QTabWidget::North);
        main_tab->setTabShape(QTabWidget::Rounded);
        page_slamnav = new QWidget();
        page_slamnav->setObjectName(QString::fromUtf8("page_slamnav"));
        page_slamnav->setMinimumSize(QSize(0, 0));
        verticalLayout_7 = new QVBoxLayout(page_slamnav);
        verticalLayout_7->setSpacing(0);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(0);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(-1, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(0);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label = new QLabel(page_slamnav);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font);

        horizontalLayout_2->addWidget(label);

        cb_ViewType = new QComboBox(page_slamnav);
        cb_ViewType->addItem(QString());
        cb_ViewType->addItem(QString());
        cb_ViewType->addItem(QString());
        cb_ViewType->setObjectName(QString::fromUtf8("cb_ViewType"));
        cb_ViewType->setFont(font);

        horizontalLayout_2->addWidget(cb_ViewType);

        cb_ViewHeight = new QComboBox(page_slamnav);
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->addItem(QString());
        cb_ViewHeight->setObjectName(QString::fromUtf8("cb_ViewHeight"));
        cb_ViewHeight->setFont(font);

        horizontalLayout_2->addWidget(cb_ViewHeight);

        bt_ViewReset = new QPushButton(page_slamnav);
        bt_ViewReset->setObjectName(QString::fromUtf8("bt_ViewReset"));
        bt_ViewReset->setFont(font);

        horizontalLayout_2->addWidget(bt_ViewReset);

        bt_SetTopView = new QPushButton(page_slamnav);
        bt_SetTopView->setObjectName(QString::fromUtf8("bt_SetTopView"));
        bt_SetTopView->setFont(font);

        horizontalLayout_2->addWidget(bt_SetTopView);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        lb_RobotId = new QLabel(page_slamnav);
        lb_RobotId->setObjectName(QString::fromUtf8("lb_RobotId"));
        lb_RobotId->setFont(font);

        horizontalLayout_2->addWidget(lb_RobotId);


        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(-1, 0, -1, -1);
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        ckb_PlotLive = new QCheckBox(page_slamnav);
        ckb_PlotLive->setObjectName(QString::fromUtf8("ckb_PlotLive"));
        ckb_PlotLive->setFont(font);
        ckb_PlotLive->setChecked(true);

        horizontalLayout->addWidget(ckb_PlotLive);

        ckb_PlotNodes = new QCheckBox(page_slamnav);
        ckb_PlotNodes->setObjectName(QString::fromUtf8("ckb_PlotNodes"));
        ckb_PlotNodes->setFont(font);
        ckb_PlotNodes->setChecked(true);

        horizontalLayout->addWidget(ckb_PlotNodes);

        ckb_PlotEdges = new QCheckBox(page_slamnav);
        ckb_PlotEdges->setObjectName(QString::fromUtf8("ckb_PlotEdges"));
        ckb_PlotEdges->setFont(font);
        ckb_PlotEdges->setChecked(true);

        horizontalLayout->addWidget(ckb_PlotEdges);

        ckb_PlotNames = new QCheckBox(page_slamnav);
        ckb_PlotNames->setObjectName(QString::fromUtf8("ckb_PlotNames"));
        ckb_PlotNames->setFont(font);
        ckb_PlotNames->setChecked(true);

        horizontalLayout->addWidget(ckb_PlotNames);

        ckb_PlotEnable = new QCheckBox(page_slamnav);
        ckb_PlotEnable->setObjectName(QString::fromUtf8("ckb_PlotEnable"));
        ckb_PlotEnable->setFont(font);
        ckb_PlotEnable->setChecked(true);

        horizontalLayout->addWidget(ckb_PlotEnable);

        ckb_Panning = new QCheckBox(page_slamnav);
        ckb_Panning->setObjectName(QString::fromUtf8("ckb_Panning"));
        ckb_Panning->setFont(font);
        ckb_Panning->setChecked(false);

        horizontalLayout->addWidget(ckb_Panning);


        verticalLayout_4->addLayout(horizontalLayout);

        qvtkWidget = new QVTKOpenGLNativeWidget(page_slamnav);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setFont(font);
        qvtkWidget->setMouseTracking(false);
        qvtkWidget->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_Screen1 = new QLabel(qvtkWidget);
        lb_Screen1->setObjectName(QString::fromUtf8("lb_Screen1"));
        lb_Screen1->setGeometry(QRect(0, 0, 180, 180));
        lb_Screen1->setFont(font);
        lb_Screen1->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));
        lb_Screen2 = new QLabel(qvtkWidget);
        lb_Screen2->setObjectName(QString::fromUtf8("lb_Screen2"));
        lb_Screen2->setGeometry(QRect(190, 0, 160, 90));
        lb_Screen2->setFont(font);
        lb_Screen2->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));
        lb_Screen3 = new QLabel(qvtkWidget);
        lb_Screen3->setObjectName(QString::fromUtf8("lb_Screen3"));
        lb_Screen3->setGeometry(QRect(350, 0, 160, 90));
        lb_Screen3->setFont(font);
        lb_Screen3->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));
        lb_Screen4 = new QLabel(qvtkWidget);
        lb_Screen4->setObjectName(QString::fromUtf8("lb_Screen4"));
        lb_Screen4->setGeometry(QRect(190, 90, 160, 90));
        lb_Screen4->setFont(font);
        lb_Screen4->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));
        lb_Screen5 = new QLabel(qvtkWidget);
        lb_Screen5->setObjectName(QString::fromUtf8("lb_Screen5"));
        lb_Screen5->setGeometry(QRect(350, 90, 160, 90));
        lb_Screen5->setFont(font);
        lb_Screen5->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));
        bt_ViewLeft = new QPushButton(qvtkWidget);
        bt_ViewLeft->setObjectName(QString::fromUtf8("bt_ViewLeft"));
        bt_ViewLeft->setGeometry(QRect(10, 240, 41, 41));
        bt_ViewLeft->setFont(font);
        bt_ViewLeft->setStyleSheet(QString::fromUtf8("background-color: rgb(237, 51, 59);"));
        bt_ViewLeft->setAutoRepeat(true);
        bt_ViewLeft->setAutoRepeatDelay(300);
        bt_ViewRight = new QPushButton(qvtkWidget);
        bt_ViewRight->setObjectName(QString::fromUtf8("bt_ViewRight"));
        bt_ViewRight->setGeometry(QRect(90, 240, 41, 41));
        bt_ViewRight->setFont(font);
        bt_ViewRight->setStyleSheet(QString::fromUtf8("background-color: rgb(237, 51, 59);"));
        bt_ViewRight->setAutoRepeat(true);
        bt_ViewRight->setAutoRepeatDelay(300);
        bt_ViewUp = new QPushButton(qvtkWidget);
        bt_ViewUp->setObjectName(QString::fromUtf8("bt_ViewUp"));
        bt_ViewUp->setGeometry(QRect(50, 200, 41, 41));
        bt_ViewUp->setFont(font);
        bt_ViewUp->setStyleSheet(QString::fromUtf8("background-color: rgb(38, 162, 105);"));
        bt_ViewUp->setAutoRepeat(true);
        bt_ViewUp->setAutoRepeatDelay(300);
        bt_ViewDown = new QPushButton(qvtkWidget);
        bt_ViewDown->setObjectName(QString::fromUtf8("bt_ViewDown"));
        bt_ViewDown->setGeometry(QRect(50, 280, 41, 41));
        bt_ViewDown->setFont(font);
        bt_ViewDown->setStyleSheet(QString::fromUtf8("background-color: rgb(38, 162, 105);"));
        bt_ViewDown->setAutoRepeat(true);
        bt_ViewDown->setAutoRepeatDelay(300);
        bt_ViewZoomIn = new QPushButton(qvtkWidget);
        bt_ViewZoomIn->setObjectName(QString::fromUtf8("bt_ViewZoomIn"));
        bt_ViewZoomIn->setGeometry(QRect(140, 200, 41, 41));
        bt_ViewZoomIn->setFont(font);
        bt_ViewZoomIn->setStyleSheet(QString::fromUtf8("background-color: rgb(26, 95, 180);"));
        bt_ViewZoomIn->setIconSize(QSize(16, 16));
        bt_ViewZoomIn->setAutoRepeat(true);
        bt_ViewZoomIn->setAutoRepeatDelay(300);
        bt_ViewZoomOut = new QPushButton(qvtkWidget);
        bt_ViewZoomOut->setObjectName(QString::fromUtf8("bt_ViewZoomOut"));
        bt_ViewZoomOut->setGeometry(QRect(140, 280, 41, 41));
        bt_ViewZoomOut->setFont(font);
        bt_ViewZoomOut->setStyleSheet(QString::fromUtf8("background-color: rgb(26, 95, 180);"));
        bt_ViewZoomOut->setIconSize(QSize(16, 16));
        bt_ViewZoomOut->setAutoRepeat(true);
        bt_ViewZoomOut->setAutoRepeatDelay(300);
        lb_NodeId = new QLabel(qvtkWidget);
        lb_NodeId->setObjectName(QString::fromUtf8("lb_NodeId"));
        lb_NodeId->setGeometry(QRect(190, 230, 321, 21));
        QFont font1;
        font1.setPointSize(11);
        lb_NodeId->setFont(font1);
        lb_RobotVel = new QLabel(qvtkWidget);
        lb_RobotVel->setObjectName(QString::fromUtf8("lb_RobotVel"));
        lb_RobotVel->setGeometry(QRect(190, 260, 321, 21));
        lb_RobotVel->setFont(font1);
        lb_PickPose = new QLabel(qvtkWidget);
        lb_PickPose->setObjectName(QString::fromUtf8("lb_PickPose"));
        lb_PickPose->setGeometry(QRect(190, 200, 321, 21));
        lb_PickPose->setFont(font1);

        verticalLayout_4->addWidget(qvtkWidget);

        lb_LogInfo = new QLabel(page_slamnav);
        lb_LogInfo->setObjectName(QString::fromUtf8("lb_LogInfo"));
        lb_LogInfo->setMinimumSize(QSize(0, 20));
        lb_LogInfo->setFont(font);
        lb_LogInfo->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_LogInfo->setFrameShape(QFrame::Box);
        lb_LogInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_4->addWidget(lb_LogInfo);

        verticalLayout_4->setStretch(2, 1);

        verticalLayout_7->addLayout(verticalLayout_4);

        main_tab->addTab(page_slamnav, QString());
        page_annot = new QWidget();
        page_annot->setObjectName(QString::fromUtf8("page_annot"));
        horizontalLayout_4 = new QHBoxLayout(page_annot);
        horizontalLayout_4->setSpacing(0);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(0);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setSizeConstraint(QLayout::SetNoConstraint);
        verticalLayout_3->setContentsMargins(-1, -1, -1, 0);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(0);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(-1, -1, -1, 0);
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);

        ckb_PlotNodes2 = new QCheckBox(page_annot);
        ckb_PlotNodes2->setObjectName(QString::fromUtf8("ckb_PlotNodes2"));
        ckb_PlotNodes2->setChecked(true);

        horizontalLayout_3->addWidget(ckb_PlotNodes2);

        ckb_PlotEdges2 = new QCheckBox(page_annot);
        ckb_PlotEdges2->setObjectName(QString::fromUtf8("ckb_PlotEdges2"));
        ckb_PlotEdges2->setChecked(true);

        horizontalLayout_3->addWidget(ckb_PlotEdges2);

        ckb_PlotNames2 = new QCheckBox(page_annot);
        ckb_PlotNames2->setObjectName(QString::fromUtf8("ckb_PlotNames2"));
        ckb_PlotNames2->setChecked(true);

        horizontalLayout_3->addWidget(ckb_PlotNames2);

        ckb_Panning2 = new QCheckBox(page_annot);
        ckb_Panning2->setObjectName(QString::fromUtf8("ckb_Panning2"));
        ckb_Panning2->setChecked(false);

        horizontalLayout_3->addWidget(ckb_Panning2);


        verticalLayout_3->addLayout(horizontalLayout_3);

        qvtkWidget2 = new QVTKOpenGLNativeWidget(page_annot);
        qvtkWidget2->setObjectName(QString::fromUtf8("qvtkWidget2"));
        qvtkWidget2->setMinimumSize(QSize(0, 0));
        qvtkWidget2->setMouseTracking(false);
        qvtkWidget2->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        bt_ViewLeft2 = new QPushButton(qvtkWidget2);
        bt_ViewLeft2->setObjectName(QString::fromUtf8("bt_ViewLeft2"));
        bt_ViewLeft2->setGeometry(QRect(10, 50, 41, 41));
        bt_ViewLeft2->setFont(font);
        bt_ViewLeft2->setStyleSheet(QString::fromUtf8("background-color: rgb(237, 51, 59);"));
        bt_ViewLeft2->setAutoRepeat(true);
        bt_ViewRight2 = new QPushButton(qvtkWidget2);
        bt_ViewRight2->setObjectName(QString::fromUtf8("bt_ViewRight2"));
        bt_ViewRight2->setGeometry(QRect(90, 50, 41, 41));
        bt_ViewRight2->setFont(font);
        bt_ViewRight2->setStyleSheet(QString::fromUtf8("background-color: rgb(237, 51, 59);"));
        bt_ViewRight2->setAutoRepeat(true);
        bt_ViewUp2 = new QPushButton(qvtkWidget2);
        bt_ViewUp2->setObjectName(QString::fromUtf8("bt_ViewUp2"));
        bt_ViewUp2->setGeometry(QRect(50, 10, 41, 41));
        bt_ViewUp2->setFont(font);
        bt_ViewUp2->setStyleSheet(QString::fromUtf8("background-color: rgb(38, 162, 105);"));
        bt_ViewUp2->setAutoRepeat(true);
        bt_ViewZoomIn2 = new QPushButton(qvtkWidget2);
        bt_ViewZoomIn2->setObjectName(QString::fromUtf8("bt_ViewZoomIn2"));
        bt_ViewZoomIn2->setGeometry(QRect(140, 10, 41, 41));
        bt_ViewZoomIn2->setFont(font);
        bt_ViewZoomIn2->setStyleSheet(QString::fromUtf8("background-color: rgb(26, 95, 180);"));
        bt_ViewZoomIn2->setIconSize(QSize(16, 16));
        bt_ViewZoomIn2->setAutoRepeat(true);
        bt_ViewZoomOut2 = new QPushButton(qvtkWidget2);
        bt_ViewZoomOut2->setObjectName(QString::fromUtf8("bt_ViewZoomOut2"));
        bt_ViewZoomOut2->setGeometry(QRect(140, 90, 41, 41));
        bt_ViewZoomOut2->setFont(font);
        bt_ViewZoomOut2->setStyleSheet(QString::fromUtf8("background-color: rgb(26, 95, 180);"));
        bt_ViewZoomOut2->setIconSize(QSize(16, 16));
        bt_ViewZoomOut2->setAutoRepeat(true);
        bt_ViewDown2 = new QPushButton(qvtkWidget2);
        bt_ViewDown2->setObjectName(QString::fromUtf8("bt_ViewDown2"));
        bt_ViewDown2->setGeometry(QRect(50, 90, 41, 41));
        bt_ViewDown2->setFont(font);
        bt_ViewDown2->setStyleSheet(QString::fromUtf8("background-color: rgb(38, 162, 105);"));
        bt_ViewDown2->setAutoRepeat(true);
        lb_Measure = new QLabel(qvtkWidget2);
        lb_Measure->setObjectName(QString::fromUtf8("lb_Measure"));
        lb_Measure->setGeometry(QRect(200, 10, 261, 21));
        lb_Measure->setFont(font1);
        lb_PickPose2 = new QLabel(qvtkWidget2);
        lb_PickPose2->setObjectName(QString::fromUtf8("lb_PickPose2"));
        lb_PickPose2->setGeometry(QRect(200, 40, 261, 21));
        lb_PickPose2->setFont(font1);

        verticalLayout_3->addWidget(qvtkWidget2);

        verticalLayout_3->setStretch(1, 1);

        horizontalLayout_4->addLayout(verticalLayout_3);

        horizontalLayout_4->setStretch(0, 1);
        main_tab->addTab(page_annot, QString());

        gridLayout->addWidget(main_tab, 0, 0, 2, 1);

        menu_tab = new QTabWidget(centralwidget);
        menu_tab->setObjectName(QString::fromUtf8("menu_tab"));
        menu_tab->setFont(font);
        tab_S = new QWidget();
        tab_S->setObjectName(QString::fromUtf8("tab_S"));
        verticalLayout_6 = new QVBoxLayout(tab_S);
        verticalLayout_6->setSpacing(0);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(-1, -1, 0, 0);
        tabWidget = new QTabWidget(tab_S);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy);
        tabWidget->setMinimumSize(QSize(330, 200));
        tabWidget->setFont(font);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        bt_MapBuild = new QPushButton(tab);
        bt_MapBuild->setObjectName(QString::fromUtf8("bt_MapBuild"));
        bt_MapBuild->setGeometry(QRect(20, 20, 71, 41));
        bt_MapBuild->setFont(font);
        bt_MapSave = new QPushButton(tab);
        bt_MapSave->setObjectName(QString::fromUtf8("bt_MapSave"));
        bt_MapSave->setGeometry(QRect(90, 20, 71, 41));
        bt_MapSave->setFont(font);
        bt_MapLoad = new QPushButton(tab);
        bt_MapLoad->setObjectName(QString::fromUtf8("bt_MapLoad"));
        bt_MapLoad->setGeometry(QRect(160, 20, 71, 41));
        bt_MapLoad->setFont(font);
        bt_LocInit = new QPushButton(tab);
        bt_LocInit->setObjectName(QString::fromUtf8("bt_LocInit"));
        bt_LocInit->setGeometry(QRect(20, 60, 71, 41));
        bt_LocInit->setFont(font);
        bt_LocInitAuto = new QPushButton(tab);
        bt_LocInitAuto->setObjectName(QString::fromUtf8("bt_LocInitAuto"));
        bt_LocInitAuto->setGeometry(QRect(160, 60, 71, 41));
        bt_LocInitAuto->setFont(font);
        bt_LocStart = new QPushButton(tab);
        bt_LocStart->setObjectName(QString::fromUtf8("bt_LocStart"));
        bt_LocStart->setGeometry(QRect(20, 100, 71, 41));
        bt_LocStart->setFont(font);
        bt_LocStop = new QPushButton(tab);
        bt_LocStop->setObjectName(QString::fromUtf8("bt_LocStop"));
        bt_LocStop->setGeometry(QRect(90, 100, 71, 41));
        bt_LocStop->setFont(font);
        bt_SimInit = new QPushButton(tab);
        bt_SimInit->setObjectName(QString::fromUtf8("bt_SimInit"));
        bt_SimInit->setEnabled(false);
        bt_SimInit->setGeometry(QRect(230, 20, 71, 41));
        bt_SimInit->setFont(font);
        bt_ObsClear = new QPushButton(tab);
        bt_ObsClear->setObjectName(QString::fromUtf8("bt_ObsClear"));
        bt_ObsClear->setGeometry(QRect(160, 100, 71, 41));
        bt_ObsClear->setFont(font);
        bt_ConfigLoad = new QPushButton(tab);
        bt_ConfigLoad->setObjectName(QString::fromUtf8("bt_ConfigLoad"));
        bt_ConfigLoad->setGeometry(QRect(230, 100, 71, 41));
        bt_ConfigLoad->setFont(font);
        bt_LocInitSemiAuto = new QPushButton(tab);
        bt_LocInitSemiAuto->setObjectName(QString::fromUtf8("bt_LocInitSemiAuto"));
        bt_LocInitSemiAuto->setGeometry(QRect(90, 60, 71, 41));
        bt_LocInitSemiAuto->setFont(font);
        bt_MapLastLc = new QPushButton(tab);
        bt_MapLastLc->setObjectName(QString::fromUtf8("bt_MapLastLc"));
        bt_MapLastLc->setGeometry(QRect(230, 60, 71, 41));
        bt_MapLastLc->setFont(font);
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        bt_AutoMove = new QPushButton(tab_2);
        bt_AutoMove->setObjectName(QString::fromUtf8("bt_AutoMove"));
        bt_AutoMove->setGeometry(QRect(20, 10, 61, 41));
        bt_AutoMove2 = new QPushButton(tab_2);
        bt_AutoMove2->setObjectName(QString::fromUtf8("bt_AutoMove2"));
        bt_AutoMove2->setGeometry(QRect(90, 10, 61, 41));
        bt_AutoMove3 = new QPushButton(tab_2);
        bt_AutoMove3->setObjectName(QString::fromUtf8("bt_AutoMove3"));
        bt_AutoMove3->setGeometry(QRect(160, 10, 61, 41));
        bt_AutoStop = new QPushButton(tab_2);
        bt_AutoStop->setObjectName(QString::fromUtf8("bt_AutoStop"));
        bt_AutoStop->setGeometry(QRect(230, 10, 61, 41));
        bt_AutoPause = new QPushButton(tab_2);
        bt_AutoPause->setObjectName(QString::fromUtf8("bt_AutoPause"));
        bt_AutoPause->setGeometry(QRect(20, 60, 61, 41));
        bt_AutoResume = new QPushButton(tab_2);
        bt_AutoResume->setObjectName(QString::fromUtf8("bt_AutoResume"));
        bt_AutoResume->setGeometry(QRect(90, 60, 61, 41));
        bt_DockingMove = new QPushButton(tab_2);
        bt_DockingMove->setObjectName(QString::fromUtf8("bt_DockingMove"));
        bt_DockingMove->setGeometry(QRect(160, 60, 61, 41));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(bt_DockingMove->sizePolicy().hasHeightForWidth());
        bt_DockingMove->setSizePolicy(sizePolicy1);
        bt_DockingMove->setFocusPolicy(Qt::StrongFocus);
        bt_DockingStop = new QPushButton(tab_2);
        bt_DockingStop->setObjectName(QString::fromUtf8("bt_DockingStop"));
        bt_DockingStop->setGeometry(QRect(230, 60, 61, 41));
        lb_CodeInfo = new QLabel(tab_2);
        lb_CodeInfo->setObjectName(QString::fromUtf8("lb_CodeInfo"));
        lb_CodeInfo->setGeometry(QRect(20, 110, 201, 41));
        bt_Undock = new QPushButton(tab_2);
        bt_Undock->setObjectName(QString::fromUtf8("bt_Undock"));
        bt_Undock->setGeometry(QRect(230, 110, 61, 41));
        sizePolicy1.setHeightForWidth(bt_Undock->sizePolicy().hasHeightForWidth());
        bt_Undock->setSizePolicy(sizePolicy1);
        bt_Undock->setFocusPolicy(Qt::StrongFocus);
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        bt_TaskDel = new QPushButton(tab_3);
        bt_TaskDel->setObjectName(QString::fromUtf8("bt_TaskDel"));
        bt_TaskDel->setGeometry(QRect(10, 30, 121, 21));
        bt_TaskPause = new QPushButton(tab_3);
        bt_TaskPause->setObjectName(QString::fromUtf8("bt_TaskPause"));
        bt_TaskPause->setGeometry(QRect(10, 80, 61, 21));
        ckb_Looping = new QCheckBox(tab_3);
        ckb_Looping->setObjectName(QString::fromUtf8("ckb_Looping"));
        ckb_Looping->setGeometry(QRect(10, 122, 71, 23));
        ckb_Looping->setChecked(true);
        bt_TaskPlay = new QPushButton(tab_3);
        bt_TaskPlay->setObjectName(QString::fromUtf8("bt_TaskPlay"));
        bt_TaskPlay->setGeometry(QRect(150, 144, 161, 21));
        bt_TaskAdd = new QPushButton(tab_3);
        bt_TaskAdd->setObjectName(QString::fromUtf8("bt_TaskAdd"));
        bt_TaskAdd->setGeometry(QRect(10, 10, 121, 21));
        bt_TaskCancel = new QPushButton(tab_3);
        bt_TaskCancel->setObjectName(QString::fromUtf8("bt_TaskCancel"));
        bt_TaskCancel->setGeometry(QRect(10, 100, 121, 21));
        bt_TaskCancel->setFont(font);
        bt_TaskCancel->setStyleSheet(QString::fromUtf8(""));
        bt_TaskLoad = new QPushButton(tab_3);
        bt_TaskLoad->setObjectName(QString::fromUtf8("bt_TaskLoad"));
        bt_TaskLoad->setGeometry(QRect(70, 50, 61, 21));
        bt_TaskResume = new QPushButton(tab_3);
        bt_TaskResume->setObjectName(QString::fromUtf8("bt_TaskResume"));
        bt_TaskResume->setGeometry(QRect(70, 80, 61, 21));
        bt_TaskSave = new QPushButton(tab_3);
        bt_TaskSave->setObjectName(QString::fromUtf8("bt_TaskSave"));
        bt_TaskSave->setGeometry(QRect(10, 50, 61, 21));
        lw_TaskList = new QListWidget(tab_3);
        lw_TaskList->setObjectName(QString::fromUtf8("lw_TaskList"));
        lw_TaskList->setGeometry(QRect(150, 20, 161, 121));
        label_7 = new QLabel(tab_3);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(150, 0, 141, 20));
        cb_TaskDrivingMode = new QComboBox(tab_3);
        cb_TaskDrivingMode->addItem(QString());
        cb_TaskDrivingMode->addItem(QString());
        cb_TaskDrivingMode->addItem(QString());
        cb_TaskDrivingMode->setObjectName(QString::fromUtf8("cb_TaskDrivingMode"));
        cb_TaskDrivingMode->setGeometry(QRect(10, 143, 121, 22));
        tabWidget->addTab(tab_3, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName(QString::fromUtf8("tab_6"));
        lb_FmsInfo = new QLabel(tab_6);
        lb_FmsInfo->setObjectName(QString::fromUtf8("lb_FmsInfo"));
        lb_FmsInfo->setGeometry(QRect(10, 10, 311, 91));
        lb_FmsInfo->setFont(font);
        lb_FmsInfo->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_FmsInfo->setFrameShape(QFrame::Box);
        lb_FmsInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        bt_SendMap = new QPushButton(tab_6);
        bt_SendMap->setObjectName(QString::fromUtf8("bt_SendMap"));
        bt_SendMap->setGeometry(QRect(10, 110, 71, 41));
        tabWidget->addTab(tab_6, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        lb_SystemInfo = new QLabel(tab_4);
        lb_SystemInfo->setObjectName(QString::fromUtf8("lb_SystemInfo"));
        lb_SystemInfo->setGeometry(QRect(8, 30, 311, 131));
        lb_SystemInfo->setFont(font);
        lb_SystemInfo->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_SystemInfo->setFrameShape(QFrame::Box);
        lb_SystemInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        ckb_RecordSystemInfo = new QCheckBox(tab_4);
        ckb_RecordSystemInfo->setObjectName(QString::fromUtf8("ckb_RecordSystemInfo"));
        ckb_RecordSystemInfo->setGeometry(QRect(12, 5, 71, 23));
        tabWidget->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        bt_Sync = new QPushButton(tab_5);
        bt_Sync->setObjectName(QString::fromUtf8("bt_Sync"));
        bt_Sync->setGeometry(QRect(10, 10, 41, 31));
        bt_MoveLinear = new QPushButton(tab_5);
        bt_MoveLinear->setObjectName(QString::fromUtf8("bt_MoveLinear"));
        bt_MoveLinear->setGeometry(QRect(220, 50, 91, 20));
        bt_MoveRotate = new QPushButton(tab_5);
        bt_MoveRotate->setObjectName(QString::fromUtf8("bt_MoveRotate"));
        bt_MoveRotate->setGeometry(QRect(220, 80, 91, 21));
        dsb_MoveLinearDist = new QDoubleSpinBox(tab_5);
        dsb_MoveLinearDist->setObjectName(QString::fromUtf8("dsb_MoveLinearDist"));
        dsb_MoveLinearDist->setGeometry(QRect(50, 50, 65, 21));
        dsb_MoveLinearDist->setMinimum(-5.000000000000000);
        dsb_MoveLinearDist->setMaximum(5.000000000000000);
        dsb_MoveLinearDist->setSingleStep(0.010000000000000);
        dsb_MoveLinearV = new QDoubleSpinBox(tab_5);
        dsb_MoveLinearV->setObjectName(QString::fromUtf8("dsb_MoveLinearV"));
        dsb_MoveLinearV->setGeometry(QRect(150, 50, 65, 21));
        dsb_MoveLinearV->setMinimum(-1.500000000000000);
        dsb_MoveLinearV->setMaximum(1.500000000000000);
        dsb_MoveLinearV->setSingleStep(0.010000000000000);
        dsb_MoveRotateDeg = new QDoubleSpinBox(tab_5);
        dsb_MoveRotateDeg->setObjectName(QString::fromUtf8("dsb_MoveRotateDeg"));
        dsb_MoveRotateDeg->setGeometry(QRect(50, 80, 65, 21));
        dsb_MoveRotateDeg->setMinimum(-180.000000000000000);
        dsb_MoveRotateDeg->setMaximum(180.000000000000000);
        dsb_MoveRotateDeg->setSingleStep(0.100000000000000);
        dsb_MoveRotateW = new QDoubleSpinBox(tab_5);
        dsb_MoveRotateW->setObjectName(QString::fromUtf8("dsb_MoveRotateW"));
        dsb_MoveRotateW->setGeometry(QRect(150, 80, 65, 21));
        dsb_MoveRotateW->setMinimum(-180.000000000000000);
        dsb_MoveRotateW->setMaximum(180.000000000000000);
        dsb_MoveRotateW->setSingleStep(0.100000000000000);
        label_2 = new QLabel(tab_5);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 50, 31, 20));
        label_3 = new QLabel(tab_5);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(130, 50, 21, 16));
        label_4 = new QLabel(tab_5);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 80, 31, 21));
        label_5 = new QLabel(tab_5);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(130, 80, 21, 17));
        bt_Test = new QPushButton(tab_5);
        bt_Test->setObjectName(QString::fromUtf8("bt_Test"));
        bt_Test->setGeometry(QRect(60, 10, 41, 31));
        spb_Led = new QSpinBox(tab_5);
        spb_Led->setObjectName(QString::fromUtf8("spb_Led"));
        spb_Led->setGeometry(QRect(10, 109, 44, 21));
        spb_Led->setMaximum(14);
        bt_TestLed = new QPushButton(tab_5);
        bt_TestLed->setObjectName(QString::fromUtf8("bt_TestLed"));
        bt_TestLed->setGeometry(QRect(60, 110, 71, 20));
        ckb_TestDebug = new QCheckBox(tab_5);
        ckb_TestDebug->setObjectName(QString::fromUtf8("ckb_TestDebug"));
        ckb_TestDebug->setGeometry(QRect(220, 110, 92, 20));
        bt_TestImgSave = new QPushButton(tab_5);
        bt_TestImgSave->setObjectName(QString::fromUtf8("bt_TestImgSave"));
        bt_TestImgSave->setGeometry(QRect(140, 110, 71, 20));
        tabWidget->addTab(tab_5, QString());

        verticalLayout_2->addWidget(tabWidget);

        lb_SlamInfo = new QLabel(tab_S);
        lb_SlamInfo->setObjectName(QString::fromUtf8("lb_SlamInfo"));
        lb_SlamInfo->setFont(font);
        lb_SlamInfo->setFocusPolicy(Qt::NoFocus);
        lb_SlamInfo->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_SlamInfo->setFrameShape(QFrame::Box);
        lb_SlamInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_2->addWidget(lb_SlamInfo);

        lb_MobilePoseInfo = new QLabel(tab_S);
        lb_MobilePoseInfo->setObjectName(QString::fromUtf8("lb_MobilePoseInfo"));
        lb_MobilePoseInfo->setFont(font);
        lb_MobilePoseInfo->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_MobilePoseInfo->setFrameShape(QFrame::Box);
        lb_MobilePoseInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_2->addWidget(lb_MobilePoseInfo);

        lb_MobileStatusInfo = new QLabel(tab_S);
        lb_MobileStatusInfo->setObjectName(QString::fromUtf8("lb_MobileStatusInfo"));
        lb_MobileStatusInfo->setFont(font);
        lb_MobileStatusInfo->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_MobileStatusInfo->setFrameShape(QFrame::Box);
        lb_MobileStatusInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_2->addWidget(lb_MobileStatusInfo);

        lb_QueInfo = new QLabel(tab_S);
        lb_QueInfo->setObjectName(QString::fromUtf8("lb_QueInfo"));
        lb_QueInfo->setFont(font);
        lb_QueInfo->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_QueInfo->setFrameShape(QFrame::Box);
        lb_QueInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_2->addWidget(lb_QueInfo);

        lb_AutoInfo = new QLabel(tab_S);
        lb_AutoInfo->setObjectName(QString::fromUtf8("lb_AutoInfo"));
        lb_AutoInfo->setFont(font);
        lb_AutoInfo->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        lb_AutoInfo->setFrameShape(QFrame::Box);
        lb_AutoInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_2->addWidget(lb_AutoInfo);

        verticalLayout_2->setStretch(2, 1);
        verticalLayout_2->setStretch(3, 1);
        verticalLayout_2->setStretch(4, 1);
        verticalLayout_2->setStretch(5, 1);

        verticalLayout_6->addLayout(verticalLayout_2);

        menu_tab->addTab(tab_S, QString());
        tab_A = new QWidget();
        tab_A->setObjectName(QString::fromUtf8("tab_A"));
        verticalLayout = new QVBoxLayout(tab_A);
        verticalLayout->setSpacing(0);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(0);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(-1, -1, -1, 0);
        frame_2 = new QFrame(tab_A);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setMinimumSize(QSize(330, 100));
        QFont font2;
        font2.setPointSize(9);
        frame_2->setFont(font2);
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        bt_MapLoad2 = new QPushButton(frame_2);
        bt_MapLoad2->setObjectName(QString::fromUtf8("bt_MapLoad2"));
        bt_MapLoad2->setGeometry(QRect(10, 10, 61, 31));
        bt_MapLoad2->setFont(font);
        bt_MapSave2 = new QPushButton(frame_2);
        bt_MapSave2->setObjectName(QString::fromUtf8("bt_MapSave2"));
        bt_MapSave2->setGeometry(QRect(70, 10, 61, 31));
        bt_MapSave2->setFont(font);
        bt_ClearTopo = new QPushButton(frame_2);
        bt_ClearTopo->setObjectName(QString::fromUtf8("bt_ClearTopo"));
        bt_ClearTopo->setGeometry(QRect(130, 10, 61, 31));
        bt_ClearTopo->setFont(font);
        bt_QuickAnnotStart = new QPushButton(frame_2);
        bt_QuickAnnotStart->setObjectName(QString::fromUtf8("bt_QuickAnnotStart"));
        bt_QuickAnnotStart->setGeometry(QRect(70, 50, 61, 31));
        bt_QuickAnnotStart->setFont(font);
        bt_QuickAnnotStop = new QPushButton(frame_2);
        bt_QuickAnnotStop->setObjectName(QString::fromUtf8("bt_QuickAnnotStop"));
        bt_QuickAnnotStop->setGeometry(QRect(130, 50, 61, 31));
        bt_QuickAnnotStop->setFont(font);
        bt_MapReload = new QPushButton(frame_2);
        bt_MapReload->setObjectName(QString::fromUtf8("bt_MapReload"));
        bt_MapReload->setGeometry(QRect(10, 50, 61, 31));
        bt_MapReload->setFont(font);
        bt_QuickAddNode = new QPushButton(frame_2);
        bt_QuickAddNode->setObjectName(QString::fromUtf8("bt_QuickAddNode"));
        bt_QuickAddNode->setGeometry(QRect(190, 10, 61, 31));
        bt_QuickAddNode->setFont(font);
        bt_QuickAddAruco = new QPushButton(frame_2);
        bt_QuickAddAruco->setObjectName(QString::fromUtf8("bt_QuickAddAruco"));
        bt_QuickAddAruco->setGeometry(QRect(190, 50, 61, 31));
        bt_QuickAddAruco->setFont(font);
        bt_QuickAddCloud = new QPushButton(frame_2);
        bt_QuickAddCloud->setObjectName(QString::fromUtf8("bt_QuickAddCloud"));
        bt_QuickAddCloud->setGeometry(QRect(250, 10, 61, 31));
        bt_QuickAddCloud->setFont(font);

        verticalLayout_5->addWidget(frame_2);

        annot_tab = new QTabWidget(tab_A);
        annot_tab->setObjectName(QString::fromUtf8("annot_tab"));
        annot_tab->setMinimumSize(QSize(0, 0));
        annot_tab->setFont(font);
        page_edit_map = new QWidget();
        page_edit_map->setObjectName(QString::fromUtf8("page_edit_map"));
        label_12 = new QLabel(page_edit_map);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(30, 77, 41, 31));
        label_13 = new QLabel(page_edit_map);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(30, 50, 41, 31));
        label_21 = new QLabel(page_edit_map);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(30, 30, 31, 31));
        dsb_MapPosX = new QDoubleSpinBox(page_edit_map);
        dsb_MapPosX->setObjectName(QString::fromUtf8("dsb_MapPosX"));
        dsb_MapPosX->setGeometry(QRect(60, 30, 65, 26));
        dsb_MapPosX->setMinimum(-99.000000000000000);
        dsb_MapPosX->setSingleStep(0.010000000000000);
        dsb_MapPosY = new QDoubleSpinBox(page_edit_map);
        dsb_MapPosY->setObjectName(QString::fromUtf8("dsb_MapPosY"));
        dsb_MapPosY->setGeometry(QRect(60, 55, 65, 26));
        dsb_MapPosY->setMinimum(-99.000000000000000);
        dsb_MapPosY->setSingleStep(0.010000000000000);
        dsb_MapPosTh = new QDoubleSpinBox(page_edit_map);
        dsb_MapPosTh->setObjectName(QString::fromUtf8("dsb_MapPosTh"));
        dsb_MapPosTh->setGeometry(QRect(60, 80, 65, 26));
        dsb_MapPosTh->setMinimum(-180.000000000000000);
        dsb_MapPosTh->setMaximum(180.000000000000000);
        dsb_MapEraseSize = new QDoubleSpinBox(page_edit_map);
        dsb_MapEraseSize->setObjectName(QString::fromUtf8("dsb_MapEraseSize"));
        dsb_MapEraseSize->setGeometry(QRect(248, 31, 65, 26));
        dsb_MapEraseSize->setMinimum(0.150000000000000);
        dsb_MapEraseSize->setSingleStep(0.010000000000000);
        label_22 = new QLabel(page_edit_map);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setGeometry(QRect(180, 30, 61, 31));
        bt_SetMapTf = new QPushButton(page_edit_map);
        bt_SetMapTf->setObjectName(QString::fromUtf8("bt_SetMapTf"));
        bt_SetMapTf->setGeometry(QRect(30, 120, 101, 25));
        annot_tab->addTab(page_edit_map, QString());
        page_edit_topo = new QWidget();
        page_edit_topo->setObjectName(QString::fromUtf8("page_edit_topo"));
        bt_EditNodeInfo = new QPushButton(page_edit_topo);
        bt_EditNodeInfo->setObjectName(QString::fromUtf8("bt_EditNodeInfo"));
        bt_EditNodeInfo->setGeometry(QRect(110, 100, 91, 21));
        bt_AddLink2 = new QPushButton(page_edit_topo);
        bt_AddLink2->setObjectName(QString::fromUtf8("bt_AddLink2"));
        bt_AddLink2->setGeometry(QRect(10, 100, 91, 21));
        bt_EditNodePos = new QPushButton(page_edit_topo);
        bt_EditNodePos->setObjectName(QString::fromUtf8("bt_EditNodePos"));
        bt_EditNodePos->setGeometry(QRect(110, 60, 91, 21));
        bt_AddNode = new QPushButton(page_edit_topo);
        bt_AddNode->setObjectName(QString::fromUtf8("bt_AddNode"));
        bt_AddNode->setGeometry(QRect(10, 60, 91, 21));
        bt_AddLink1 = new QPushButton(page_edit_topo);
        bt_AddLink1->setObjectName(QString::fromUtf8("bt_AddLink1"));
        bt_AddLink1->setGeometry(QRect(10, 80, 91, 21));
        cb_NodeType = new QComboBox(page_edit_topo);
        cb_NodeType->addItem(QString());
        cb_NodeType->addItem(QString());
        cb_NodeType->addItem(QString());
        cb_NodeType->addItem(QString());
        cb_NodeType->addItem(QString());
        cb_NodeType->setObjectName(QString::fromUtf8("cb_NodeType"));
        cb_NodeType->setGeometry(QRect(50, 30, 91, 21));
        cb_NodeType->setStyleSheet(QString::fromUtf8(""));
        bt_AlignNodeX = new QPushButton(page_edit_topo);
        bt_AlignNodeX->setObjectName(QString::fromUtf8("bt_AlignNodeX"));
        bt_AlignNodeX->setGeometry(QRect(40, 310, 71, 21));
        bt_MinGapNodeY = new QPushButton(page_edit_topo);
        bt_MinGapNodeY->setObjectName(QString::fromUtf8("bt_MinGapNodeY"));
        bt_MinGapNodeY->setGeometry(QRect(200, 330, 71, 21));
        bt_MinGapNodeX = new QPushButton(page_edit_topo);
        bt_MinGapNodeX->setObjectName(QString::fromUtf8("bt_MinGapNodeX"));
        bt_MinGapNodeX->setGeometry(QRect(120, 330, 71, 21));
        spb_NodeGap = new QDoubleSpinBox(page_edit_topo);
        spb_NodeGap->setObjectName(QString::fromUtf8("spb_NodeGap"));
        spb_NodeGap->setGeometry(QRect(40, 330, 71, 21));
        spb_NodeGap->setMinimum(-5.000000000000000);
        spb_NodeGap->setMaximum(5.000000000000000);
        spb_NodeGap->setSingleStep(0.050000000000000);
        spb_NodeGap->setValue(0.700000000000000);
        bt_AlignNodeY = new QPushButton(page_edit_topo);
        bt_AlignNodeY->setObjectName(QString::fromUtf8("bt_AlignNodeY"));
        bt_AlignNodeY->setGeometry(QRect(120, 310, 71, 21));
        bt_AlignNodeTh = new QPushButton(page_edit_topo);
        bt_AlignNodeTh->setObjectName(QString::fromUtf8("bt_AlignNodeTh"));
        bt_AlignNodeTh->setGeometry(QRect(200, 310, 71, 21));
        label_6 = new QLabel(page_edit_topo);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 30, 41, 21));
        QFont font3;
        font3.setPointSize(10);
        label_6->setFont(font3);
        label_8 = new QLabel(page_edit_topo);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 330, 31, 21));
        label_23 = new QLabel(page_edit_topo);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setGeometry(QRect(10, 140, 31, 21));
        label_23->setFont(font2);
        te_NodeInfo = new QTextEdit(page_edit_topo);
        te_NodeInfo->setObjectName(QString::fromUtf8("te_NodeInfo"));
        te_NodeInfo->setGeometry(QRect(50, 130, 151, 41));
        bt_EditNodeType = new QPushButton(page_edit_topo);
        bt_EditNodeType->setObjectName(QString::fromUtf8("bt_EditNodeType"));
        bt_EditNodeType->setGeometry(QRect(110, 80, 91, 21));
        lb_PreNodeInfo = new QLabel(page_edit_topo);
        lb_PreNodeInfo->setObjectName(QString::fromUtf8("lb_PreNodeInfo"));
        lb_PreNodeInfo->setGeometry(QRect(170, 210, 151, 81));
        lb_PreNodeInfo->setFont(font);
        lb_PreNodeInfo->setFrameShape(QFrame::Box);
        lb_PreNodeInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        lb_PreNodeInfo->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);
        lb_CurNodeInfo = new QLabel(page_edit_topo);
        lb_CurNodeInfo->setObjectName(QString::fromUtf8("lb_CurNodeInfo"));
        lb_CurNodeInfo->setGeometry(QRect(10, 210, 151, 81));
        lb_CurNodeInfo->setFont(font);
        lb_CurNodeInfo->setFrameShape(QFrame::Box);
        lb_CurNodeInfo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        lb_CurNodeInfo->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);
        spb_NodeSizeX = new QDoubleSpinBox(page_edit_topo);
        spb_NodeSizeX->setObjectName(QString::fromUtf8("spb_NodeSizeX"));
        spb_NodeSizeX->setGeometry(QRect(80, 180, 65, 21));
        spb_NodeSizeX->setMinimum(0.100000000000000);
        spb_NodeSizeX->setMaximum(100.000000000000000);
        spb_NodeSizeX->setSingleStep(0.100000000000000);
        spb_NodeSizeY = new QDoubleSpinBox(page_edit_topo);
        spb_NodeSizeY->setObjectName(QString::fromUtf8("spb_NodeSizeY"));
        spb_NodeSizeY->setGeometry(QRect(150, 180, 65, 21));
        spb_NodeSizeY->setMinimum(0.100000000000000);
        spb_NodeSizeY->setMaximum(100.000000000000000);
        spb_NodeSizeY->setSingleStep(0.100000000000000);
        spb_NodeSizeZ = new QDoubleSpinBox(page_edit_topo);
        spb_NodeSizeZ->setObjectName(QString::fromUtf8("spb_NodeSizeZ"));
        spb_NodeSizeZ->setGeometry(QRect(220, 180, 65, 21));
        spb_NodeSizeZ->setMinimum(1.000000000000000);
        spb_NodeSizeZ->setMaximum(10.000000000000000);
        spb_NodeSizeZ->setSingleStep(0.100000000000000);
        label_24 = new QLabel(page_edit_topo);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setGeometry(QRect(10, 180, 61, 21));
        label_24->setFont(font2);
        groupBox = new QGroupBox(page_edit_topo);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 350, 271, 111));
        label_15 = new QLabel(groupBox);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(20, 60, 21, 21));
        le_CurNodeTh = new QLineEdit(groupBox);
        le_CurNodeTh->setObjectName(QString::fromUtf8("le_CurNodeTh"));
        le_CurNodeTh->setGeometry(QRect(50, 80, 51, 21));
        le_CurNodeTh->setDragEnabled(true);
        le_CurNodeTh->setReadOnly(true);
        label_16 = new QLabel(groupBox);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(20, 80, 21, 21));
        bt_NodePoseThDown = new QPushButton(groupBox);
        bt_NodePoseThDown->setObjectName(QString::fromUtf8("bt_NodePoseThDown"));
        bt_NodePoseThDown->setGeometry(QRect(110, 80, 21, 21));
        le_CurNodeY = new QLineEdit(groupBox);
        le_CurNodeY->setObjectName(QString::fromUtf8("le_CurNodeY"));
        le_CurNodeY->setGeometry(QRect(50, 60, 51, 21));
        le_CurNodeY->setDragEnabled(true);
        le_CurNodeY->setReadOnly(true);
        label_14 = new QLabel(groupBox);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(20, 40, 21, 21));
        le_CurNodeX = new QLineEdit(groupBox);
        le_CurNodeX->setObjectName(QString::fromUtf8("le_CurNodeX"));
        le_CurNodeX->setGeometry(QRect(50, 40, 51, 21));
        le_CurNodeX->setDragEnabled(true);
        le_CurNodeX->setReadOnly(true);
        spb_NodeMovementXY = new QDoubleSpinBox(groupBox);
        spb_NodeMovementXY->setObjectName(QString::fromUtf8("spb_NodeMovementXY"));
        spb_NodeMovementXY->setGeometry(QRect(180, 40, 81, 21));
        spb_NodeMovementXY->setMaximum(5.000000000000000);
        spb_NodeMovementXY->setSingleStep(0.010000000000000);
        spb_NodeMovementXY->setValue(0.100000000000000);
        bt_NodePoseYDown = new QPushButton(groupBox);
        bt_NodePoseYDown->setObjectName(QString::fromUtf8("bt_NodePoseYDown"));
        bt_NodePoseYDown->setGeometry(QRect(110, 60, 21, 21));
        bt_NodePoseYUp = new QPushButton(groupBox);
        bt_NodePoseYUp->setObjectName(QString::fromUtf8("bt_NodePoseYUp"));
        bt_NodePoseYUp->setGeometry(QRect(140, 60, 21, 21));
        ckb_TransformEntireNode = new QCheckBox(groupBox);
        ckb_TransformEntireNode->setObjectName(QString::fromUtf8("ckb_TransformEntireNode"));
        ckb_TransformEntireNode->setGeometry(QRect(10, 20, 81, 23));
        ckb_TransformEntireNode->setFont(font2);
        spb_NodeMovementTh = new QDoubleSpinBox(groupBox);
        spb_NodeMovementTh->setObjectName(QString::fromUtf8("spb_NodeMovementTh"));
        spb_NodeMovementTh->setGeometry(QRect(180, 80, 81, 21));
        spb_NodeMovementTh->setMinimum(-180.000000000000000);
        spb_NodeMovementTh->setMaximum(180.000000000000000);
        spb_NodeMovementTh->setSingleStep(1.000000000000000);
        spb_NodeMovementTh->setValue(1.000000000000000);
        bt_NodePoseXDown = new QPushButton(groupBox);
        bt_NodePoseXDown->setObjectName(QString::fromUtf8("bt_NodePoseXDown"));
        bt_NodePoseXDown->setGeometry(QRect(110, 40, 21, 21));
        bt_NodePoseThUp = new QPushButton(groupBox);
        bt_NodePoseThUp->setObjectName(QString::fromUtf8("bt_NodePoseThUp"));
        bt_NodePoseThUp->setGeometry(QRect(140, 80, 21, 21));
        bt_NodePoseXUp = new QPushButton(groupBox);
        bt_NodePoseXUp->setObjectName(QString::fromUtf8("bt_NodePoseXUp"));
        bt_NodePoseXUp->setGeometry(QRect(140, 40, 21, 21));
        bt_EditNodeName = new QPushButton(page_edit_topo);
        bt_EditNodeName->setObjectName(QString::fromUtf8("bt_EditNodeName"));
        bt_EditNodeName->setGeometry(QRect(210, 100, 91, 21));
        bt_AutoLink = new QPushButton(page_edit_topo);
        bt_AutoLink->setObjectName(QString::fromUtf8("bt_AutoLink"));
        bt_AutoLink->setGeometry(QRect(210, 60, 91, 21));
        cb_NodeAdvanceType = new QComboBox(page_edit_topo);
        cb_NodeAdvanceType->addItem(QString());
        cb_NodeAdvanceType->addItem(QString());
        cb_NodeAdvanceType->addItem(QString());
        cb_NodeAdvanceType->setObjectName(QString::fromUtf8("cb_NodeAdvanceType"));
        cb_NodeAdvanceType->setGeometry(QRect(210, 30, 91, 21));
        cb_NodeAdvanceType->setStyleSheet(QString::fromUtf8(""));
        label_9 = new QLabel(page_edit_topo);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(150, 30, 51, 21));
        label_9->setFont(font3);
        ckb_UseMeasure = new QCheckBox(page_edit_topo);
        ckb_UseMeasure->setObjectName(QString::fromUtf8("ckb_UseMeasure"));
        ckb_UseMeasure->setGeometry(QRect(20, 5, 92, 16));
        bt_DelNode = new QPushButton(page_edit_topo);
        bt_DelNode->setObjectName(QString::fromUtf8("bt_DelNode"));
        bt_DelNode->setGeometry(QRect(210, 80, 91, 21));
        bt_SelectPreNodes = new QPushButton(page_edit_topo);
        bt_SelectPreNodes->setObjectName(QString::fromUtf8("bt_SelectPreNodes"));
        bt_SelectPreNodes->setGeometry(QRect(160, 0, 71, 21));
        bt_SelectPostNodes = new QPushButton(page_edit_topo);
        bt_SelectPostNodes->setObjectName(QString::fromUtf8("bt_SelectPostNodes"));
        bt_SelectPostNodes->setGeometry(QRect(230, 0, 71, 21));
        bt_AutoNode = new QPushButton(page_edit_topo);
        bt_AutoNode->setObjectName(QString::fromUtf8("bt_AutoNode"));
        bt_AutoNode->setGeometry(QRect(210, 120, 91, 21));
        lb_NodeAlignInfo = new QLabel(page_edit_topo);
        lb_NodeAlignInfo->setObjectName(QString::fromUtf8("lb_NodeAlignInfo"));
        lb_NodeAlignInfo->setGeometry(QRect(10, 290, 311, 21));
        bt_AddPoints = new QPushButton(page_edit_topo);
        bt_AddPoints->setObjectName(QString::fromUtf8("bt_AddPoints"));
        bt_AddPoints->setGeometry(QRect(210, 150, 91, 21));
        annot_tab->addTab(page_edit_topo, QString());

        verticalLayout_5->addWidget(annot_tab);


        verticalLayout->addLayout(verticalLayout_5);

        menu_tab->addTab(tab_A, QString());

        gridLayout->addWidget(menu_tab, 0, 1, 1, 1);

        frame_3 = new QFrame(centralwidget);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        sizePolicy.setHeightForWidth(frame_3->sizePolicy().hasHeightForWidth());
        frame_3->setSizePolicy(sizePolicy);
        frame_3->setMinimumSize(QSize(0, 100));
        frame_3->setFont(font);
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        bt_JogF = new QPushButton(frame_3);
        bt_JogF->setObjectName(QString::fromUtf8("bt_JogF"));
        bt_JogF->setGeometry(QRect(50, 10, 41, 41));
        bt_JogF->setFont(font);
        bt_JogF->setAutoRepeat(false);
        bt_JogF->setAutoRepeatDelay(100);
        bt_JogB = new QPushButton(frame_3);
        bt_JogB->setObjectName(QString::fromUtf8("bt_JogB"));
        bt_JogB->setGeometry(QRect(50, 50, 41, 41));
        bt_JogB->setFont(font);
        bt_JogB->setAutoRepeat(false);
        bt_JogB->setAutoRepeatDelay(100);
        bt_JogL = new QPushButton(frame_3);
        bt_JogL->setObjectName(QString::fromUtf8("bt_JogL"));
        bt_JogL->setGeometry(QRect(10, 50, 41, 41));
        bt_JogL->setFont(font);
        bt_JogL->setAutoRepeat(false);
        bt_JogL->setAutoRepeatDelay(100);
        bt_JogR = new QPushButton(frame_3);
        bt_JogR->setObjectName(QString::fromUtf8("bt_JogR"));
        bt_JogR->setGeometry(QRect(90, 50, 41, 41));
        bt_JogR->setFont(font);
        bt_JogR->setAutoRepeat(false);
        bt_JogR->setAutoRepeatDelay(100);
        spb_JogV = new QDoubleSpinBox(frame_3);
        spb_JogV->setObjectName(QString::fromUtf8("spb_JogV"));
        spb_JogV->setGeometry(QRect(140, 10, 101, 20));
        spb_JogV->setFont(font);
        spb_JogV->setMaximum(2.000000000000000);
        spb_JogV->setSingleStep(0.100000000000000);
        spb_JogV->setValue(1.000000000000000);
        spb_JogW = new QDoubleSpinBox(frame_3);
        spb_JogW->setObjectName(QString::fromUtf8("spb_JogW"));
        spb_JogW->setGeometry(QRect(140, 50, 101, 21));
        spb_JogW->setFont(font);
        spb_JogW->setMaximum(120.000000000000000);
        spb_JogW->setValue(45.000000000000000);
        bt_MotorInit = new QPushButton(frame_3);
        bt_MotorInit->setObjectName(QString::fromUtf8("bt_MotorInit"));
        bt_MotorInit->setGeometry(QRect(10, 10, 31, 31));
        bt_MotorInit->setFont(font);
        bt_Emergency = new QPushButton(frame_3);
        bt_Emergency->setObjectName(QString::fromUtf8("bt_Emergency"));
        bt_Emergency->setGeometry(QRect(250, 10, 71, 81));
        QFont font4;
        font4.setPointSize(8);
        font4.setBold(false);
        font4.setWeight(50);
        bt_Emergency->setFont(font4);
        bt_Emergency->setStyleSheet(QString::fromUtf8("background-color: rgb(246, 97, 81);"));
        spb_AccV = new QDoubleSpinBox(frame_3);
        spb_AccV->setObjectName(QString::fromUtf8("spb_AccV"));
        spb_AccV->setGeometry(QRect(140, 30, 51, 21));
        spb_AccV->setFont(font);
        spb_AccV->setDecimals(1);
        spb_AccV->setMaximum(120.000000000000000);
        spb_AccV->setSingleStep(0.100000000000000);
        spb_AccV->setValue(0.300000000000000);
        spb_DecelV = new QDoubleSpinBox(frame_3);
        spb_DecelV->setObjectName(QString::fromUtf8("spb_DecelV"));
        spb_DecelV->setGeometry(QRect(190, 30, 51, 21));
        spb_DecelV->setFont(font);
        spb_DecelV->setDecimals(1);
        spb_DecelV->setMaximum(120.000000000000000);
        spb_DecelV->setSingleStep(0.100000000000000);
        spb_DecelV->setValue(0.600000000000000);
        spb_AccW = new QDoubleSpinBox(frame_3);
        spb_AccW->setObjectName(QString::fromUtf8("spb_AccW"));
        spb_AccW->setGeometry(QRect(140, 70, 51, 21));
        spb_AccW->setFont(font);
        spb_AccW->setDecimals(1);
        spb_AccW->setMaximum(120.000000000000000);
        spb_AccW->setSingleStep(5.000000000000000);
        spb_AccW->setValue(30.000000000000000);
        spb_DecelW = new QDoubleSpinBox(frame_3);
        spb_DecelW->setObjectName(QString::fromUtf8("spb_DecelW"));
        spb_DecelW->setGeometry(QRect(190, 70, 51, 21));
        spb_DecelW->setFont(font);
        spb_DecelW->setDecimals(1);
        spb_DecelW->setMaximum(120.000000000000000);
        spb_DecelW->setSingleStep(5.000000000000000);
        spb_DecelW->setValue(60.000000000000000);

        gridLayout->addWidget(frame_3, 1, 1, 1, 1);

        gridLayout->setColumnStretch(0, 1);
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        main_tab->setCurrentIndex(0);
        menu_tab->setCurrentIndex(0);
        tabWidget->setCurrentIndex(3);
        annot_tab->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "SLAMNAV2", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "View: ", nullptr));
        cb_ViewType->setItemText(0, QCoreApplication::translate("MainWindow", "VIEW_2D", nullptr));
        cb_ViewType->setItemText(1, QCoreApplication::translate("MainWindow", "VIEW_3D", nullptr));
        cb_ViewType->setItemText(2, QCoreApplication::translate("MainWindow", "VIEW_MAPPING", nullptr));

        cb_ViewHeight->setItemText(0, QCoreApplication::translate("MainWindow", "20", nullptr));
        cb_ViewHeight->setItemText(1, QCoreApplication::translate("MainWindow", "40", nullptr));
        cb_ViewHeight->setItemText(2, QCoreApplication::translate("MainWindow", "60", nullptr));
        cb_ViewHeight->setItemText(3, QCoreApplication::translate("MainWindow", "80", nullptr));
        cb_ViewHeight->setItemText(4, QCoreApplication::translate("MainWindow", "100", nullptr));
        cb_ViewHeight->setItemText(5, QCoreApplication::translate("MainWindow", "120", nullptr));
        cb_ViewHeight->setItemText(6, QCoreApplication::translate("MainWindow", "140", nullptr));
        cb_ViewHeight->setItemText(7, QCoreApplication::translate("MainWindow", "160", nullptr));
        cb_ViewHeight->setItemText(8, QCoreApplication::translate("MainWindow", "180", nullptr));
        cb_ViewHeight->setItemText(9, QCoreApplication::translate("MainWindow", "200", nullptr));

        bt_ViewReset->setText(QCoreApplication::translate("MainWindow", "View reset", nullptr));
        bt_SetTopView->setText(QCoreApplication::translate("MainWindow", "Set top view", nullptr));
        lb_RobotId->setText(QCoreApplication::translate("MainWindow", "[ROBOT ID]", nullptr));
        ckb_PlotLive->setText(QCoreApplication::translate("MainWindow", "live", nullptr));
        ckb_PlotNodes->setText(QCoreApplication::translate("MainWindow", "nodes", nullptr));
        ckb_PlotEdges->setText(QCoreApplication::translate("MainWindow", "edges", nullptr));
        ckb_PlotNames->setText(QCoreApplication::translate("MainWindow", "names", nullptr));
        ckb_PlotEnable->setText(QCoreApplication::translate("MainWindow", "Plot enable", nullptr));
        ckb_Panning->setText(QCoreApplication::translate("MainWindow", "Panning", nullptr));
        lb_Screen1->setText(QString());
        lb_Screen2->setText(QString());
        lb_Screen3->setText(QString());
        lb_Screen4->setText(QString());
        lb_Screen5->setText(QString());
        bt_ViewLeft->setText(QCoreApplication::translate("MainWindow", "L", nullptr));
        bt_ViewRight->setText(QCoreApplication::translate("MainWindow", "R", nullptr));
        bt_ViewUp->setText(QCoreApplication::translate("MainWindow", "Up", nullptr));
        bt_ViewDown->setText(QCoreApplication::translate("MainWindow", "Dn", nullptr));
        bt_ViewZoomIn->setText(QCoreApplication::translate("MainWindow", "zoom\n"
"in", nullptr));
        bt_ViewZoomOut->setText(QCoreApplication::translate("MainWindow", "zoom\n"
"out", nullptr));
        lb_NodeId->setText(QCoreApplication::translate("MainWindow", "Node id", nullptr));
        lb_RobotVel->setText(QCoreApplication::translate("MainWindow", "Robot vel", nullptr));
        lb_PickPose->setText(QCoreApplication::translate("MainWindow", "Pick pose", nullptr));
        lb_LogInfo->setText(QCoreApplication::translate("MainWindow", "[LOG_INFO]", nullptr));
        main_tab->setTabText(main_tab->indexOf(page_slamnav), QCoreApplication::translate("MainWindow", "SLAMNAV", nullptr));
        ckb_PlotNodes2->setText(QCoreApplication::translate("MainWindow", "nodes", nullptr));
        ckb_PlotEdges2->setText(QCoreApplication::translate("MainWindow", "edges", nullptr));
        ckb_PlotNames2->setText(QCoreApplication::translate("MainWindow", "names", nullptr));
        ckb_Panning2->setText(QCoreApplication::translate("MainWindow", "Panning", nullptr));
        bt_ViewLeft2->setText(QCoreApplication::translate("MainWindow", "L", nullptr));
        bt_ViewRight2->setText(QCoreApplication::translate("MainWindow", "R", nullptr));
        bt_ViewUp2->setText(QCoreApplication::translate("MainWindow", "Up", nullptr));
        bt_ViewZoomIn2->setText(QCoreApplication::translate("MainWindow", "zoom\n"
"in", nullptr));
        bt_ViewZoomOut2->setText(QCoreApplication::translate("MainWindow", "zoom\n"
"out", nullptr));
        bt_ViewDown2->setText(QCoreApplication::translate("MainWindow", "Dn", nullptr));
        lb_Measure->setText(QCoreApplication::translate("MainWindow", "Measure", nullptr));
        lb_PickPose2->setText(QCoreApplication::translate("MainWindow", "Pick pose", nullptr));
        main_tab->setTabText(main_tab->indexOf(page_annot), QCoreApplication::translate("MainWindow", "ANNOTATION", nullptr));
        bt_MapBuild->setText(QCoreApplication::translate("MainWindow", "Build", nullptr));
        bt_MapSave->setText(QCoreApplication::translate("MainWindow", "Save", nullptr));
        bt_MapLoad->setText(QCoreApplication::translate("MainWindow", "Load", nullptr));
        bt_LocInit->setText(QCoreApplication::translate("MainWindow", "Init", nullptr));
        bt_LocInitAuto->setText(QCoreApplication::translate("MainWindow", "Auto init\n"
"(global)", nullptr));
        bt_LocStart->setText(QCoreApplication::translate("MainWindow", "LOC start", nullptr));
        bt_LocStop->setText(QCoreApplication::translate("MainWindow", "LOC stop", nullptr));
        bt_SimInit->setText(QCoreApplication::translate("MainWindow", "Sim init", nullptr));
        bt_ObsClear->setText(QCoreApplication::translate("MainWindow", "OBS clear", nullptr));
        bt_ConfigLoad->setText(QCoreApplication::translate("MainWindow", "Reload\n"
"config", nullptr));
        bt_LocInitSemiAuto->setText(QCoreApplication::translate("MainWindow", "Auto init\n"
"(semi)", nullptr));
        bt_MapLastLc->setText(QCoreApplication::translate("MainWindow", "Last\n"
"LC", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "Mapping", nullptr));
        bt_AutoMove->setText(QCoreApplication::translate("MainWindow", "Move\n"
"(pp)", nullptr));
        bt_AutoMove2->setText(QCoreApplication::translate("MainWindow", "Move\n"
"(hpp)", nullptr));
        bt_AutoMove3->setText(QCoreApplication::translate("MainWindow", "Move\n"
"(tng)", nullptr));
        bt_AutoStop->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        bt_AutoPause->setText(QCoreApplication::translate("MainWindow", "Pause", nullptr));
        bt_AutoResume->setText(QCoreApplication::translate("MainWindow", "Resume", nullptr));
        bt_DockingMove->setText(QCoreApplication::translate("MainWindow", "Docking\n"
"run", nullptr));
        bt_DockingStop->setText(QCoreApplication::translate("MainWindow", "Docking\n"
"Stop", nullptr));
        lb_CodeInfo->setText(QCoreApplication::translate("MainWindow", "[CODE_INFO]", nullptr));
        bt_Undock->setText(QCoreApplication::translate("MainWindow", "Undock", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Driving", nullptr));
        bt_TaskDel->setText(QCoreApplication::translate("MainWindow", "Task Delete [I]", nullptr));
        bt_TaskPause->setText(QCoreApplication::translate("MainWindow", "Pause", nullptr));
        ckb_Looping->setText(QCoreApplication::translate("MainWindow", "looping", nullptr));
        bt_TaskPlay->setText(QCoreApplication::translate("MainWindow", "Task Play [O]", nullptr));
        bt_TaskAdd->setText(QCoreApplication::translate("MainWindow", "Task Add [U]", nullptr));
        bt_TaskCancel->setText(QCoreApplication::translate("MainWindow", "Cancel", nullptr));
        bt_TaskLoad->setText(QCoreApplication::translate("MainWindow", "Load", nullptr));
        bt_TaskResume->setText(QCoreApplication::translate("MainWindow", "Resume", nullptr));
        bt_TaskSave->setText(QCoreApplication::translate("MainWindow", "Save", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "node list", nullptr));
        cb_TaskDrivingMode->setItemText(0, QCoreApplication::translate("MainWindow", "basic", nullptr));
        cb_TaskDrivingMode->setItemText(1, QCoreApplication::translate("MainWindow", "holonomic", nullptr));
        cb_TaskDrivingMode->setItemText(2, QCoreApplication::translate("MainWindow", "tng", nullptr));

        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("MainWindow", "Task", nullptr));
        lb_FmsInfo->setText(QCoreApplication::translate("MainWindow", "[FMS_INFO]", nullptr));
        bt_SendMap->setText(QCoreApplication::translate("MainWindow", "send map\n"
"(sn -> fms)", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_6), QCoreApplication::translate("MainWindow", "fms", nullptr));
        lb_SystemInfo->setText(QCoreApplication::translate("MainWindow", "[SYSTEM_INFO]", nullptr));
        ckb_RecordSystemInfo->setText(QCoreApplication::translate("MainWindow", "record", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QCoreApplication::translate("MainWindow", "System", nullptr));
        bt_Sync->setText(QCoreApplication::translate("MainWindow", "Sync", nullptr));
        bt_MoveLinear->setText(QCoreApplication::translate("MainWindow", "move linear", nullptr));
        bt_MoveRotate->setText(QCoreApplication::translate("MainWindow", "move rotate", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "dist:", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "v:", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "deg:", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "w:", nullptr));
        bt_Test->setText(QCoreApplication::translate("MainWindow", "Test", nullptr));
        bt_TestLed->setText(QCoreApplication::translate("MainWindow", "LED test", nullptr));
        ckb_TestDebug->setText(QCoreApplication::translate("MainWindow", "Debug", nullptr));
        bt_TestImgSave->setText(QCoreApplication::translate("MainWindow", "IMG save", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QCoreApplication::translate("MainWindow", "Test", nullptr));
        lb_SlamInfo->setText(QCoreApplication::translate("MainWindow", "[SLAM_INFO]", nullptr));
        lb_MobilePoseInfo->setText(QCoreApplication::translate("MainWindow", "[MOBILE_POSE]", nullptr));
        lb_MobileStatusInfo->setText(QCoreApplication::translate("MainWindow", "[MOBILE_STATUS]", nullptr));
        lb_QueInfo->setText(QCoreApplication::translate("MainWindow", "[QUE_STATUS]", nullptr));
        lb_AutoInfo->setText(QCoreApplication::translate("MainWindow", "[AUTO_INFO]", nullptr));
        menu_tab->setTabText(menu_tab->indexOf(tab_S), QCoreApplication::translate("MainWindow", "S", nullptr));
        bt_MapLoad2->setText(QCoreApplication::translate("MainWindow", "Load", nullptr));
        bt_MapSave2->setText(QCoreApplication::translate("MainWindow", "Save", nullptr));
        bt_ClearTopo->setText(QCoreApplication::translate("MainWindow", "Clear", nullptr));
        bt_QuickAnnotStart->setText(QCoreApplication::translate("MainWindow", "QA start", nullptr));
        bt_QuickAnnotStop->setText(QCoreApplication::translate("MainWindow", "QA stop", nullptr));
        bt_MapReload->setText(QCoreApplication::translate("MainWindow", "Reload", nullptr));
        bt_QuickAddNode->setText(QCoreApplication::translate("MainWindow", "QA Node", nullptr));
        bt_QuickAddAruco->setText(QCoreApplication::translate("MainWindow", "QA Aruco", nullptr));
        bt_QuickAddCloud->setText(QCoreApplication::translate("MainWindow", "QA Cloud", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "th : ", nullptr));
        label_13->setText(QCoreApplication::translate("MainWindow", "y : ", nullptr));
        label_21->setText(QCoreApplication::translate("MainWindow", "x : ", nullptr));
        dsb_MapEraseSize->setSuffix(QCoreApplication::translate("MainWindow", "m", nullptr));
        label_22->setText(QCoreApplication::translate("MainWindow", "erase size : ", nullptr));
        bt_SetMapTf->setText(QCoreApplication::translate("MainWindow", "set map tf", nullptr));
        annot_tab->setTabText(annot_tab->indexOf(page_edit_map), QCoreApplication::translate("MainWindow", "EDIT_MAP", nullptr));
        bt_EditNodeInfo->setText(QCoreApplication::translate("MainWindow", "Edit info", nullptr));
        bt_AddLink2->setText(QCoreApplication::translate("MainWindow", "Add link2[B]", nullptr));
#if QT_CONFIG(shortcut)
        bt_AddLink2->setShortcut(QCoreApplication::translate("MainWindow", "B", nullptr));
#endif // QT_CONFIG(shortcut)
        bt_EditNodePos->setText(QCoreApplication::translate("MainWindow", "Edit pos[E]", nullptr));
#if QT_CONFIG(shortcut)
        bt_EditNodePos->setShortcut(QCoreApplication::translate("MainWindow", "E", nullptr));
#endif // QT_CONFIG(shortcut)
        bt_AddNode->setText(QCoreApplication::translate("MainWindow", "Add node[N]", nullptr));
#if QT_CONFIG(shortcut)
        bt_AddNode->setShortcut(QCoreApplication::translate("MainWindow", "N", nullptr));
#endif // QT_CONFIG(shortcut)
        bt_AddLink1->setText(QCoreApplication::translate("MainWindow", "Add link1[L]", nullptr));
#if QT_CONFIG(shortcut)
        bt_AddLink1->setShortcut(QCoreApplication::translate("MainWindow", "L", nullptr));
#endif // QT_CONFIG(shortcut)
        cb_NodeType->setItemText(0, QCoreApplication::translate("MainWindow", "GOAL", nullptr));
        cb_NodeType->setItemText(1, QCoreApplication::translate("MainWindow", "ROUTE", nullptr));
        cb_NodeType->setItemText(2, QCoreApplication::translate("MainWindow", "INIT", nullptr));
        cb_NodeType->setItemText(3, QCoreApplication::translate("MainWindow", "OBS", nullptr));
        cb_NodeType->setItemText(4, QCoreApplication::translate("MainWindow", "ZONE", nullptr));

        bt_AlignNodeX->setText(QCoreApplication::translate("MainWindow", "Align X", nullptr));
        bt_MinGapNodeY->setText(QCoreApplication::translate("MainWindow", "Gap Y", nullptr));
        bt_MinGapNodeX->setText(QCoreApplication::translate("MainWindow", "Gap X", nullptr));
        spb_NodeGap->setSuffix(QCoreApplication::translate("MainWindow", " m", nullptr));
        bt_AlignNodeY->setText(QCoreApplication::translate("MainWindow", "Align Y", nullptr));
        bt_AlignNodeTh->setText(QCoreApplication::translate("MainWindow", "Align Th", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "type:", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "gap:", nullptr));
        label_23->setText(QCoreApplication::translate("MainWindow", "info:", nullptr));
        bt_EditNodeType->setText(QCoreApplication::translate("MainWindow", "Edit type", nullptr));
        lb_PreNodeInfo->setText(QCoreApplication::translate("MainWindow", "pre node info", nullptr));
        lb_CurNodeInfo->setText(QCoreApplication::translate("MainWindow", "cur node info", nullptr));
        spb_NodeSizeX->setSuffix(QCoreApplication::translate("MainWindow", " m", nullptr));
        spb_NodeSizeY->setSuffix(QCoreApplication::translate("MainWindow", " m", nullptr));
        spb_NodeSizeZ->setSuffix(QCoreApplication::translate("MainWindow", " m", nullptr));
        label_24->setText(QCoreApplication::translate("MainWindow", "size(x,y,z):", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "Move node", nullptr));
        label_15->setText(QCoreApplication::translate("MainWindow", "y :", nullptr));
        label_16->setText(QCoreApplication::translate("MainWindow", "th :", nullptr));
        bt_NodePoseThDown->setText(QCoreApplication::translate("MainWindow", "-", nullptr));
        label_14->setText(QCoreApplication::translate("MainWindow", "x :", nullptr));
        spb_NodeMovementXY->setSuffix(QCoreApplication::translate("MainWindow", " m", nullptr));
        bt_NodePoseYDown->setText(QCoreApplication::translate("MainWindow", "-", nullptr));
        bt_NodePoseYUp->setText(QCoreApplication::translate("MainWindow", "+", nullptr));
        ckb_TransformEntireNode->setText(QCoreApplication::translate("MainWindow", "all node", nullptr));
        spb_NodeMovementTh->setSuffix(QCoreApplication::translate("MainWindow", " deg", nullptr));
        bt_NodePoseXDown->setText(QCoreApplication::translate("MainWindow", "-", nullptr));
        bt_NodePoseThUp->setText(QCoreApplication::translate("MainWindow", "+", nullptr));
        bt_NodePoseXUp->setText(QCoreApplication::translate("MainWindow", "+", nullptr));
        bt_EditNodeName->setText(QCoreApplication::translate("MainWindow", "Edit name", nullptr));
        bt_AutoLink->setText(QCoreApplication::translate("MainWindow", "Auto link", nullptr));
        cb_NodeAdvanceType->setItemText(0, QCoreApplication::translate("MainWindow", "Forward", nullptr));
        cb_NodeAdvanceType->setItemText(1, QCoreApplication::translate("MainWindow", "Backward", nullptr));
        cb_NodeAdvanceType->setItemText(2, QCoreApplication::translate("MainWindow", "Offset", nullptr));

        label_9->setText(QCoreApplication::translate("MainWindow", "advance:", nullptr));
        ckb_UseMeasure->setText(QCoreApplication::translate("MainWindow", "use measure", nullptr));
        bt_DelNode->setText(QCoreApplication::translate("MainWindow", "Del node[D]", nullptr));
#if QT_CONFIG(shortcut)
        bt_DelNode->setShortcut(QCoreApplication::translate("MainWindow", "D", nullptr));
#endif // QT_CONFIG(shortcut)
        bt_SelectPreNodes->setText(QCoreApplication::translate("MainWindow", "Pre select", nullptr));
        bt_SelectPostNodes->setText(QCoreApplication::translate("MainWindow", "Post select", nullptr));
        bt_AutoNode->setText(QCoreApplication::translate("MainWindow", "Auto node", nullptr));
        lb_NodeAlignInfo->setText(QCoreApplication::translate("MainWindow", "Node align info", nullptr));
        bt_AddPoints->setText(QCoreApplication::translate("MainWindow", "Add points", nullptr));
        annot_tab->setTabText(annot_tab->indexOf(page_edit_topo), QCoreApplication::translate("MainWindow", "EDIT_TOPO", nullptr));
        menu_tab->setTabText(menu_tab->indexOf(tab_A), QCoreApplication::translate("MainWindow", "A", nullptr));
        bt_JogF->setText(QCoreApplication::translate("MainWindow", "F", nullptr));
        bt_JogB->setText(QCoreApplication::translate("MainWindow", "B", nullptr));
        bt_JogL->setText(QCoreApplication::translate("MainWindow", "L", nullptr));
        bt_JogR->setText(QCoreApplication::translate("MainWindow", "R", nullptr));
        spb_JogV->setSuffix(QCoreApplication::translate("MainWindow", " mps", nullptr));
        spb_JogW->setSuffix(QCoreApplication::translate("MainWindow", " dps", nullptr));
        bt_MotorInit->setText(QCoreApplication::translate("MainWindow", "Init", nullptr));
        bt_Emergency->setText(QCoreApplication::translate("MainWindow", "Emergency\n"
"stop", nullptr));
        spb_AccV->setSuffix(QString());
        spb_DecelV->setSuffix(QString());
        spb_AccW->setSuffix(QString());
        spb_DecelW->setSuffix(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
