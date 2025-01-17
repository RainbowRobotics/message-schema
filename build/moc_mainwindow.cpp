/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[102];
    char stringdata0[1347];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 14), // "slot_write_log"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 8), // "user_log"
QT_MOC_LITERAL(4, 36, 10), // "color_code"
QT_MOC_LITERAL(5, 47, 21), // "slot_main_tab_changed"
QT_MOC_LITERAL(6, 69, 21), // "slot_menu_tab_changed"
QT_MOC_LITERAL(7, 91, 10), // "map_update"
QT_MOC_LITERAL(8, 102, 10), // "obs_update"
QT_MOC_LITERAL(9, 113, 11), // "topo_update"
QT_MOC_LITERAL(10, 125, 11), // "pick_update"
QT_MOC_LITERAL(11, 137, 10), // "all_update"
QT_MOC_LITERAL(12, 148, 12), // "bt_ViewReset"
QT_MOC_LITERAL(13, 161, 13), // "bt_SetTopView"
QT_MOC_LITERAL(14, 175, 9), // "plot_loop"
QT_MOC_LITERAL(15, 185, 10), // "plot_loop2"
QT_MOC_LITERAL(16, 196, 7), // "qa_loop"
QT_MOC_LITERAL(17, 204, 13), // "bt_ConfigLoad"
QT_MOC_LITERAL(18, 218, 12), // "bt_MotorInit"
QT_MOC_LITERAL(19, 231, 12), // "bt_Emergency"
QT_MOC_LITERAL(20, 244, 7), // "bt_Sync"
QT_MOC_LITERAL(21, 252, 13), // "bt_MoveLinear"
QT_MOC_LITERAL(22, 266, 13), // "bt_MoveRotate"
QT_MOC_LITERAL(23, 280, 7), // "bt_JogF"
QT_MOC_LITERAL(24, 288, 7), // "bt_JogB"
QT_MOC_LITERAL(25, 296, 7), // "bt_JogL"
QT_MOC_LITERAL(26, 304, 7), // "bt_JogR"
QT_MOC_LITERAL(27, 312, 14), // "bt_JogReleased"
QT_MOC_LITERAL(28, 327, 12), // "bt_MapReload"
QT_MOC_LITERAL(29, 340, 11), // "bt_MapSave2"
QT_MOC_LITERAL(30, 352, 10), // "bt_AddNode"
QT_MOC_LITERAL(31, 363, 10), // "bt_DelNode"
QT_MOC_LITERAL(32, 374, 11), // "bt_AddLink1"
QT_MOC_LITERAL(33, 386, 11), // "bt_AddLink2"
QT_MOC_LITERAL(34, 398, 11), // "bt_AutoLink"
QT_MOC_LITERAL(35, 410, 11), // "bt_AutoNode"
QT_MOC_LITERAL(36, 422, 12), // "bt_AddPoints"
QT_MOC_LITERAL(37, 435, 14), // "bt_EditNodePos"
QT_MOC_LITERAL(38, 450, 15), // "bt_EditNodeType"
QT_MOC_LITERAL(39, 466, 15), // "bt_EditNodeInfo"
QT_MOC_LITERAL(40, 482, 15), // "bt_EditNodeName"
QT_MOC_LITERAL(41, 498, 14), // "bt_MinGapNodeX"
QT_MOC_LITERAL(42, 513, 14), // "bt_MinGapNodeY"
QT_MOC_LITERAL(43, 528, 13), // "bt_AlignNodeX"
QT_MOC_LITERAL(44, 542, 13), // "bt_AlignNodeY"
QT_MOC_LITERAL(45, 556, 14), // "bt_AlignNodeTh"
QT_MOC_LITERAL(46, 571, 12), // "bt_ClearTopo"
QT_MOC_LITERAL(47, 584, 14), // "bt_NodePoseXUp"
QT_MOC_LITERAL(48, 599, 14), // "bt_NodePoseYUp"
QT_MOC_LITERAL(49, 614, 15), // "bt_NodePoseThUp"
QT_MOC_LITERAL(50, 630, 16), // "bt_NodePoseXDown"
QT_MOC_LITERAL(51, 647, 16), // "bt_NodePoseYDown"
QT_MOC_LITERAL(52, 664, 17), // "bt_NodePoseThDown"
QT_MOC_LITERAL(53, 682, 18), // "bt_QuickAnnotStart"
QT_MOC_LITERAL(54, 701, 17), // "bt_QuickAnnotStop"
QT_MOC_LITERAL(55, 719, 15), // "bt_QuickAddNode"
QT_MOC_LITERAL(56, 735, 16), // "bt_QuickAddAruco"
QT_MOC_LITERAL(57, 752, 16), // "bt_QuickAddCloud"
QT_MOC_LITERAL(58, 769, 11), // "bt_SetMapTf"
QT_MOC_LITERAL(59, 781, 11), // "bt_MapBuild"
QT_MOC_LITERAL(60, 793, 10), // "bt_MapSave"
QT_MOC_LITERAL(61, 804, 10), // "bt_MapLoad"
QT_MOC_LITERAL(62, 815, 12), // "bt_MapLastLc"
QT_MOC_LITERAL(63, 828, 10), // "bt_LocInit"
QT_MOC_LITERAL(64, 839, 18), // "bt_LocInitSemiAuto"
QT_MOC_LITERAL(65, 858, 14), // "bt_LocInitAuto"
QT_MOC_LITERAL(66, 873, 11), // "bt_LocStart"
QT_MOC_LITERAL(67, 885, 10), // "bt_LocStop"
QT_MOC_LITERAL(68, 896, 10), // "bt_SimInit"
QT_MOC_LITERAL(69, 907, 7), // "bt_Test"
QT_MOC_LITERAL(70, 915, 10), // "bt_TestLed"
QT_MOC_LITERAL(71, 926, 13), // "ckb_TestDebug"
QT_MOC_LITERAL(72, 940, 14), // "bt_TestImgSave"
QT_MOC_LITERAL(73, 955, 11), // "bt_AutoMove"
QT_MOC_LITERAL(74, 967, 12), // "bt_AutoMove2"
QT_MOC_LITERAL(75, 980, 12), // "bt_AutoMove3"
QT_MOC_LITERAL(76, 993, 11), // "bt_AutoStop"
QT_MOC_LITERAL(77, 1005, 12), // "bt_AutoPause"
QT_MOC_LITERAL(78, 1018, 13), // "bt_AutoResume"
QT_MOC_LITERAL(79, 1032, 23), // "slot_local_path_updated"
QT_MOC_LITERAL(80, 1056, 24), // "slot_global_path_updated"
QT_MOC_LITERAL(81, 1081, 11), // "bt_ObsClear"
QT_MOC_LITERAL(82, 1093, 10), // "bt_TaskAdd"
QT_MOC_LITERAL(83, 1104, 10), // "bt_TaskDel"
QT_MOC_LITERAL(84, 1115, 11), // "bt_TaskSave"
QT_MOC_LITERAL(85, 1127, 11), // "bt_TaskLoad"
QT_MOC_LITERAL(86, 1139, 11), // "bt_TaskPlay"
QT_MOC_LITERAL(87, 1151, 12), // "bt_TaskPause"
QT_MOC_LITERAL(88, 1164, 13), // "bt_TaskCancel"
QT_MOC_LITERAL(89, 1178, 11), // "cb_NodeType"
QT_MOC_LITERAL(90, 1190, 4), // "type"
QT_MOC_LITERAL(91, 1195, 17), // "bt_SelectPreNodes"
QT_MOC_LITERAL(92, 1213, 18), // "bt_SelectPostNodes"
QT_MOC_LITERAL(93, 1232, 10), // "bt_SendMap"
QT_MOC_LITERAL(94, 1243, 14), // "slot_resist_id"
QT_MOC_LITERAL(95, 1258, 2), // "id"
QT_MOC_LITERAL(96, 1261, 20), // "slot_sim_random_init"
QT_MOC_LITERAL(97, 1282, 4), // "seed"
QT_MOC_LITERAL(98, 1287, 19), // "slot_sim_random_seq"
QT_MOC_LITERAL(99, 1307, 14), // "bt_DockingMove"
QT_MOC_LITERAL(100, 1322, 14), // "bt_DockingStop"
QT_MOC_LITERAL(101, 1337, 9) // "bt_Undock"

    },
    "MainWindow\0slot_write_log\0\0user_log\0"
    "color_code\0slot_main_tab_changed\0"
    "slot_menu_tab_changed\0map_update\0"
    "obs_update\0topo_update\0pick_update\0"
    "all_update\0bt_ViewReset\0bt_SetTopView\0"
    "plot_loop\0plot_loop2\0qa_loop\0bt_ConfigLoad\0"
    "bt_MotorInit\0bt_Emergency\0bt_Sync\0"
    "bt_MoveLinear\0bt_MoveRotate\0bt_JogF\0"
    "bt_JogB\0bt_JogL\0bt_JogR\0bt_JogReleased\0"
    "bt_MapReload\0bt_MapSave2\0bt_AddNode\0"
    "bt_DelNode\0bt_AddLink1\0bt_AddLink2\0"
    "bt_AutoLink\0bt_AutoNode\0bt_AddPoints\0"
    "bt_EditNodePos\0bt_EditNodeType\0"
    "bt_EditNodeInfo\0bt_EditNodeName\0"
    "bt_MinGapNodeX\0bt_MinGapNodeY\0"
    "bt_AlignNodeX\0bt_AlignNodeY\0bt_AlignNodeTh\0"
    "bt_ClearTopo\0bt_NodePoseXUp\0bt_NodePoseYUp\0"
    "bt_NodePoseThUp\0bt_NodePoseXDown\0"
    "bt_NodePoseYDown\0bt_NodePoseThDown\0"
    "bt_QuickAnnotStart\0bt_QuickAnnotStop\0"
    "bt_QuickAddNode\0bt_QuickAddAruco\0"
    "bt_QuickAddCloud\0bt_SetMapTf\0bt_MapBuild\0"
    "bt_MapSave\0bt_MapLoad\0bt_MapLastLc\0"
    "bt_LocInit\0bt_LocInitSemiAuto\0"
    "bt_LocInitAuto\0bt_LocStart\0bt_LocStop\0"
    "bt_SimInit\0bt_Test\0bt_TestLed\0"
    "ckb_TestDebug\0bt_TestImgSave\0bt_AutoMove\0"
    "bt_AutoMove2\0bt_AutoMove3\0bt_AutoStop\0"
    "bt_AutoPause\0bt_AutoResume\0"
    "slot_local_path_updated\0"
    "slot_global_path_updated\0bt_ObsClear\0"
    "bt_TaskAdd\0bt_TaskDel\0bt_TaskSave\0"
    "bt_TaskLoad\0bt_TaskPlay\0bt_TaskPause\0"
    "bt_TaskCancel\0cb_NodeType\0type\0"
    "bt_SelectPreNodes\0bt_SelectPostNodes\0"
    "bt_SendMap\0slot_resist_id\0id\0"
    "slot_sim_random_init\0seed\0slot_sim_random_seq\0"
    "bt_DockingMove\0bt_DockingStop\0bt_Undock"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      95,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,  489,    2, 0x0a /* Public */,
       5,    0,  494,    2, 0x0a /* Public */,
       6,    0,  495,    2, 0x0a /* Public */,
       7,    0,  496,    2, 0x0a /* Public */,
       8,    0,  497,    2, 0x0a /* Public */,
       9,    0,  498,    2, 0x0a /* Public */,
      10,    0,  499,    2, 0x0a /* Public */,
      11,    0,  500,    2, 0x0a /* Public */,
      12,    0,  501,    2, 0x0a /* Public */,
      13,    0,  502,    2, 0x0a /* Public */,
      14,    0,  503,    2, 0x0a /* Public */,
      15,    0,  504,    2, 0x0a /* Public */,
      16,    0,  505,    2, 0x0a /* Public */,
      17,    0,  506,    2, 0x0a /* Public */,
      18,    0,  507,    2, 0x0a /* Public */,
      19,    0,  508,    2, 0x0a /* Public */,
      20,    0,  509,    2, 0x0a /* Public */,
      21,    0,  510,    2, 0x0a /* Public */,
      22,    0,  511,    2, 0x0a /* Public */,
      23,    0,  512,    2, 0x0a /* Public */,
      24,    0,  513,    2, 0x0a /* Public */,
      25,    0,  514,    2, 0x0a /* Public */,
      26,    0,  515,    2, 0x0a /* Public */,
      27,    0,  516,    2, 0x0a /* Public */,
      28,    0,  517,    2, 0x0a /* Public */,
      29,    0,  518,    2, 0x0a /* Public */,
      30,    0,  519,    2, 0x0a /* Public */,
      31,    0,  520,    2, 0x0a /* Public */,
      32,    0,  521,    2, 0x0a /* Public */,
      33,    0,  522,    2, 0x0a /* Public */,
      34,    0,  523,    2, 0x0a /* Public */,
      35,    0,  524,    2, 0x0a /* Public */,
      36,    0,  525,    2, 0x0a /* Public */,
      37,    0,  526,    2, 0x0a /* Public */,
      38,    0,  527,    2, 0x0a /* Public */,
      39,    0,  528,    2, 0x0a /* Public */,
      40,    0,  529,    2, 0x0a /* Public */,
      41,    0,  530,    2, 0x0a /* Public */,
      42,    0,  531,    2, 0x0a /* Public */,
      43,    0,  532,    2, 0x0a /* Public */,
      44,    0,  533,    2, 0x0a /* Public */,
      45,    0,  534,    2, 0x0a /* Public */,
      46,    0,  535,    2, 0x0a /* Public */,
      47,    0,  536,    2, 0x0a /* Public */,
      48,    0,  537,    2, 0x0a /* Public */,
      49,    0,  538,    2, 0x0a /* Public */,
      50,    0,  539,    2, 0x0a /* Public */,
      51,    0,  540,    2, 0x0a /* Public */,
      52,    0,  541,    2, 0x0a /* Public */,
      53,    0,  542,    2, 0x0a /* Public */,
      54,    0,  543,    2, 0x0a /* Public */,
      55,    0,  544,    2, 0x0a /* Public */,
      56,    0,  545,    2, 0x0a /* Public */,
      57,    0,  546,    2, 0x0a /* Public */,
      58,    0,  547,    2, 0x0a /* Public */,
      59,    0,  548,    2, 0x0a /* Public */,
      60,    0,  549,    2, 0x0a /* Public */,
      61,    0,  550,    2, 0x0a /* Public */,
      62,    0,  551,    2, 0x0a /* Public */,
      63,    0,  552,    2, 0x0a /* Public */,
      64,    0,  553,    2, 0x0a /* Public */,
      65,    0,  554,    2, 0x0a /* Public */,
      66,    0,  555,    2, 0x0a /* Public */,
      67,    0,  556,    2, 0x0a /* Public */,
      68,    0,  557,    2, 0x0a /* Public */,
      69,    0,  558,    2, 0x0a /* Public */,
      70,    0,  559,    2, 0x0a /* Public */,
      71,    0,  560,    2, 0x0a /* Public */,
      72,    0,  561,    2, 0x0a /* Public */,
      73,    0,  562,    2, 0x0a /* Public */,
      74,    0,  563,    2, 0x0a /* Public */,
      75,    0,  564,    2, 0x0a /* Public */,
      76,    0,  565,    2, 0x0a /* Public */,
      77,    0,  566,    2, 0x0a /* Public */,
      78,    0,  567,    2, 0x0a /* Public */,
      79,    0,  568,    2, 0x0a /* Public */,
      80,    0,  569,    2, 0x0a /* Public */,
      81,    0,  570,    2, 0x0a /* Public */,
      82,    0,  571,    2, 0x0a /* Public */,
      83,    0,  572,    2, 0x0a /* Public */,
      84,    0,  573,    2, 0x0a /* Public */,
      85,    0,  574,    2, 0x0a /* Public */,
      86,    0,  575,    2, 0x0a /* Public */,
      87,    0,  576,    2, 0x0a /* Public */,
      88,    0,  577,    2, 0x0a /* Public */,
      89,    1,  578,    2, 0x0a /* Public */,
      91,    0,  581,    2, 0x0a /* Public */,
      92,    0,  582,    2, 0x0a /* Public */,
      93,    0,  583,    2, 0x0a /* Public */,
      94,    1,  584,    2, 0x0a /* Public */,
      96,    1,  587,    2, 0x0a /* Public */,
      98,    0,  590,    2, 0x0a /* Public */,
      99,    0,  591,    2, 0x0a /* Public */,
     100,    0,  592,    2, 0x0a /* Public */,
     101,    0,  593,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   90,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   95,
    QMetaType::Void, QMetaType::QString,   97,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->slot_write_log((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 1: _t->slot_main_tab_changed(); break;
        case 2: _t->slot_menu_tab_changed(); break;
        case 3: _t->map_update(); break;
        case 4: _t->obs_update(); break;
        case 5: _t->topo_update(); break;
        case 6: _t->pick_update(); break;
        case 7: _t->all_update(); break;
        case 8: _t->bt_ViewReset(); break;
        case 9: _t->bt_SetTopView(); break;
        case 10: _t->plot_loop(); break;
        case 11: _t->plot_loop2(); break;
        case 12: _t->qa_loop(); break;
        case 13: _t->bt_ConfigLoad(); break;
        case 14: _t->bt_MotorInit(); break;
        case 15: _t->bt_Emergency(); break;
        case 16: _t->bt_Sync(); break;
        case 17: _t->bt_MoveLinear(); break;
        case 18: _t->bt_MoveRotate(); break;
        case 19: _t->bt_JogF(); break;
        case 20: _t->bt_JogB(); break;
        case 21: _t->bt_JogL(); break;
        case 22: _t->bt_JogR(); break;
        case 23: _t->bt_JogReleased(); break;
        case 24: _t->bt_MapReload(); break;
        case 25: _t->bt_MapSave2(); break;
        case 26: _t->bt_AddNode(); break;
        case 27: _t->bt_DelNode(); break;
        case 28: _t->bt_AddLink1(); break;
        case 29: _t->bt_AddLink2(); break;
        case 30: _t->bt_AutoLink(); break;
        case 31: _t->bt_AutoNode(); break;
        case 32: _t->bt_AddPoints(); break;
        case 33: _t->bt_EditNodePos(); break;
        case 34: _t->bt_EditNodeType(); break;
        case 35: _t->bt_EditNodeInfo(); break;
        case 36: _t->bt_EditNodeName(); break;
        case 37: _t->bt_MinGapNodeX(); break;
        case 38: _t->bt_MinGapNodeY(); break;
        case 39: _t->bt_AlignNodeX(); break;
        case 40: _t->bt_AlignNodeY(); break;
        case 41: _t->bt_AlignNodeTh(); break;
        case 42: _t->bt_ClearTopo(); break;
        case 43: _t->bt_NodePoseXUp(); break;
        case 44: _t->bt_NodePoseYUp(); break;
        case 45: _t->bt_NodePoseThUp(); break;
        case 46: _t->bt_NodePoseXDown(); break;
        case 47: _t->bt_NodePoseYDown(); break;
        case 48: _t->bt_NodePoseThDown(); break;
        case 49: _t->bt_QuickAnnotStart(); break;
        case 50: _t->bt_QuickAnnotStop(); break;
        case 51: _t->bt_QuickAddNode(); break;
        case 52: _t->bt_QuickAddAruco(); break;
        case 53: _t->bt_QuickAddCloud(); break;
        case 54: _t->bt_SetMapTf(); break;
        case 55: _t->bt_MapBuild(); break;
        case 56: _t->bt_MapSave(); break;
        case 57: _t->bt_MapLoad(); break;
        case 58: _t->bt_MapLastLc(); break;
        case 59: _t->bt_LocInit(); break;
        case 60: _t->bt_LocInitSemiAuto(); break;
        case 61: _t->bt_LocInitAuto(); break;
        case 62: _t->bt_LocStart(); break;
        case 63: _t->bt_LocStop(); break;
        case 64: _t->bt_SimInit(); break;
        case 65: _t->bt_Test(); break;
        case 66: _t->bt_TestLed(); break;
        case 67: _t->ckb_TestDebug(); break;
        case 68: _t->bt_TestImgSave(); break;
        case 69: _t->bt_AutoMove(); break;
        case 70: _t->bt_AutoMove2(); break;
        case 71: _t->bt_AutoMove3(); break;
        case 72: _t->bt_AutoStop(); break;
        case 73: _t->bt_AutoPause(); break;
        case 74: _t->bt_AutoResume(); break;
        case 75: _t->slot_local_path_updated(); break;
        case 76: _t->slot_global_path_updated(); break;
        case 77: _t->bt_ObsClear(); break;
        case 78: _t->bt_TaskAdd(); break;
        case 79: _t->bt_TaskDel(); break;
        case 80: _t->bt_TaskSave(); break;
        case 81: _t->bt_TaskLoad(); break;
        case 82: _t->bt_TaskPlay(); break;
        case 83: _t->bt_TaskPause(); break;
        case 84: _t->bt_TaskCancel(); break;
        case 85: _t->cb_NodeType((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 86: _t->bt_SelectPreNodes(); break;
        case 87: _t->bt_SelectPostNodes(); break;
        case 88: _t->bt_SendMap(); break;
        case 89: _t->slot_resist_id((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 90: _t->slot_sim_random_init((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 91: _t->slot_sim_random_seq(); break;
        case 92: _t->bt_DockingMove(); break;
        case 93: _t->bt_DockingStop(); break;
        case 94: _t->bt_Undock(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 95)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 95;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 95)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 95;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
