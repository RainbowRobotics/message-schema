/****************************************************************************
** Meta object code from reading C++ file 'comm_ui.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../comm_ui.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'comm_ui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_COMM_UI_t {
    QByteArrayData data[73];
    char stringdata0[1249];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_COMM_UI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_COMM_UI_t qt_meta_stringdata_COMM_UI = {
    {
QT_MOC_LITERAL(0, 0, 7), // "COMM_UI"
QT_MOC_LITERAL(1, 8, 18), // "signal_send_status"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 16), // "signal_motorinit"
QT_MOC_LITERAL(4, 45, 4), // "time"
QT_MOC_LITERAL(5, 50, 15), // "signal_move_jog"
QT_MOC_LITERAL(6, 66, 2), // "vx"
QT_MOC_LITERAL(7, 69, 2), // "vy"
QT_MOC_LITERAL(8, 72, 2), // "wz"
QT_MOC_LITERAL(9, 75, 18), // "signal_move_target"
QT_MOC_LITERAL(10, 94, 1), // "x"
QT_MOC_LITERAL(11, 96, 1), // "y"
QT_MOC_LITERAL(12, 98, 1), // "z"
QT_MOC_LITERAL(13, 100, 2), // "rz"
QT_MOC_LITERAL(14, 103, 6), // "preset"
QT_MOC_LITERAL(15, 110, 6), // "method"
QT_MOC_LITERAL(16, 117, 16), // "signal_move_goal"
QT_MOC_LITERAL(17, 134, 7), // "node_id"
QT_MOC_LITERAL(18, 142, 17), // "signal_move_pause"
QT_MOC_LITERAL(19, 160, 18), // "signal_move_resume"
QT_MOC_LITERAL(20, 179, 16), // "signal_move_stop"
QT_MOC_LITERAL(21, 196, 20), // "signal_mapping_start"
QT_MOC_LITERAL(22, 217, 19), // "signal_mapping_stop"
QT_MOC_LITERAL(23, 237, 19), // "signal_mapping_save"
QT_MOC_LITERAL(24, 257, 4), // "name"
QT_MOC_LITERAL(25, 262, 21), // "signal_mapping_reload"
QT_MOC_LITERAL(26, 284, 14), // "signal_mapload"
QT_MOC_LITERAL(27, 299, 28), // "signal_localization_autoinit"
QT_MOC_LITERAL(28, 328, 32), // "signal_localization_semiautoinit"
QT_MOC_LITERAL(29, 361, 24), // "signal_localization_init"
QT_MOC_LITERAL(30, 386, 25), // "signal_localization_start"
QT_MOC_LITERAL(31, 412, 24), // "signal_localization_stop"
QT_MOC_LITERAL(32, 437, 14), // "reconnect_loop"
QT_MOC_LITERAL(33, 452, 9), // "connected"
QT_MOC_LITERAL(34, 462, 12), // "disconnected"
QT_MOC_LITERAL(35, 475, 12), // "recv_message"
QT_MOC_LITERAL(36, 488, 7), // "message"
QT_MOC_LITERAL(37, 496, 14), // "slot_motorinit"
QT_MOC_LITERAL(38, 511, 13), // "slot_move_jog"
QT_MOC_LITERAL(39, 525, 16), // "slot_move_target"
QT_MOC_LITERAL(40, 542, 14), // "slot_move_goal"
QT_MOC_LITERAL(41, 557, 15), // "slot_move_pause"
QT_MOC_LITERAL(42, 573, 16), // "slot_move_resume"
QT_MOC_LITERAL(43, 590, 14), // "slot_move_stop"
QT_MOC_LITERAL(44, 605, 17), // "slot_move_succeed"
QT_MOC_LITERAL(45, 623, 16), // "slot_move_failed"
QT_MOC_LITERAL(46, 640, 18), // "slot_mapping_start"
QT_MOC_LITERAL(47, 659, 17), // "slot_mapping_stop"
QT_MOC_LITERAL(48, 677, 17), // "slot_mapping_save"
QT_MOC_LITERAL(49, 695, 19), // "slot_mapping_reload"
QT_MOC_LITERAL(50, 715, 12), // "slot_mapload"
QT_MOC_LITERAL(51, 728, 26), // "slot_localization_autoinit"
QT_MOC_LITERAL(52, 755, 30), // "slot_localization_semiautoinit"
QT_MOC_LITERAL(53, 786, 38), // "slot_localization_semiautoini..."
QT_MOC_LITERAL(54, 825, 37), // "slot_localization_semiautoini..."
QT_MOC_LITERAL(55, 863, 22), // "slot_localization_init"
QT_MOC_LITERAL(56, 886, 23), // "slot_localization_start"
QT_MOC_LITERAL(57, 910, 22), // "slot_localization_stop"
QT_MOC_LITERAL(58, 933, 11), // "send_status"
QT_MOC_LITERAL(59, 945, 16), // "send_global_path"
QT_MOC_LITERAL(60, 962, 15), // "send_local_path"
QT_MOC_LITERAL(61, 978, 27), // "send_mapping_start_response"
QT_MOC_LITERAL(62, 1006, 6), // "result"
QT_MOC_LITERAL(63, 1013, 26), // "send_mapping_stop_response"
QT_MOC_LITERAL(64, 1040, 26), // "send_mapping_save_response"
QT_MOC_LITERAL(65, 1067, 21), // "send_mapload_response"
QT_MOC_LITERAL(66, 1089, 26), // "send_localization_response"
QT_MOC_LITERAL(67, 1116, 7), // "command"
QT_MOC_LITERAL(68, 1124, 25), // "send_move_target_response"
QT_MOC_LITERAL(69, 1150, 23), // "send_move_goal_response"
QT_MOC_LITERAL(70, 1174, 24), // "send_move_pause_response"
QT_MOC_LITERAL(71, 1199, 25), // "send_move_resume_response"
QT_MOC_LITERAL(72, 1225, 23) // "send_move_stop_response"

    },
    "COMM_UI\0signal_send_status\0\0"
    "signal_motorinit\0time\0signal_move_jog\0"
    "vx\0vy\0wz\0signal_move_target\0x\0y\0z\0rz\0"
    "preset\0method\0signal_move_goal\0node_id\0"
    "signal_move_pause\0signal_move_resume\0"
    "signal_move_stop\0signal_mapping_start\0"
    "signal_mapping_stop\0signal_mapping_save\0"
    "name\0signal_mapping_reload\0signal_mapload\0"
    "signal_localization_autoinit\0"
    "signal_localization_semiautoinit\0"
    "signal_localization_init\0"
    "signal_localization_start\0"
    "signal_localization_stop\0reconnect_loop\0"
    "connected\0disconnected\0recv_message\0"
    "message\0slot_motorinit\0slot_move_jog\0"
    "slot_move_target\0slot_move_goal\0"
    "slot_move_pause\0slot_move_resume\0"
    "slot_move_stop\0slot_move_succeed\0"
    "slot_move_failed\0slot_mapping_start\0"
    "slot_mapping_stop\0slot_mapping_save\0"
    "slot_mapping_reload\0slot_mapload\0"
    "slot_localization_autoinit\0"
    "slot_localization_semiautoinit\0"
    "slot_localization_semiautoinit_succeed\0"
    "slot_localization_semiautoinit_failed\0"
    "slot_localization_init\0slot_localization_start\0"
    "slot_localization_stop\0send_status\0"
    "send_global_path\0send_local_path\0"
    "send_mapping_start_response\0result\0"
    "send_mapping_stop_response\0"
    "send_mapping_save_response\0"
    "send_mapload_response\0send_localization_response\0"
    "command\0send_move_target_response\0"
    "send_move_goal_response\0"
    "send_move_pause_response\0"
    "send_move_resume_response\0"
    "send_move_stop_response"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_COMM_UI[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      56,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      18,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,  294,    2, 0x06 /* Public */,
       3,    1,  295,    2, 0x06 /* Public */,
       5,    4,  298,    2, 0x06 /* Public */,
       9,    7,  307,    2, 0x06 /* Public */,
      16,    4,  322,    2, 0x06 /* Public */,
      18,    1,  331,    2, 0x06 /* Public */,
      19,    1,  334,    2, 0x06 /* Public */,
      20,    1,  337,    2, 0x06 /* Public */,
      21,    1,  340,    2, 0x06 /* Public */,
      22,    1,  343,    2, 0x06 /* Public */,
      23,    2,  346,    2, 0x06 /* Public */,
      25,    1,  351,    2, 0x06 /* Public */,
      26,    2,  354,    2, 0x06 /* Public */,
      27,    1,  359,    2, 0x06 /* Public */,
      28,    1,  362,    2, 0x06 /* Public */,
      29,    5,  365,    2, 0x06 /* Public */,
      30,    1,  376,    2, 0x06 /* Public */,
      31,    1,  379,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      32,    0,  382,    2, 0x0a /* Public */,
      33,    0,  383,    2, 0x0a /* Public */,
      34,    0,  384,    2, 0x0a /* Public */,
      35,    1,  385,    2, 0x0a /* Public */,
      37,    1,  388,    2, 0x0a /* Public */,
      38,    4,  391,    2, 0x0a /* Public */,
      39,    7,  400,    2, 0x0a /* Public */,
      40,    4,  415,    2, 0x0a /* Public */,
      41,    1,  424,    2, 0x0a /* Public */,
      42,    1,  427,    2, 0x0a /* Public */,
      43,    1,  430,    2, 0x0a /* Public */,
      44,    1,  433,    2, 0x0a /* Public */,
      45,    1,  436,    2, 0x0a /* Public */,
      46,    1,  439,    2, 0x0a /* Public */,
      47,    1,  442,    2, 0x0a /* Public */,
      48,    2,  445,    2, 0x0a /* Public */,
      49,    1,  450,    2, 0x0a /* Public */,
      50,    2,  453,    2, 0x0a /* Public */,
      51,    1,  458,    2, 0x0a /* Public */,
      52,    1,  461,    2, 0x0a /* Public */,
      53,    1,  464,    2, 0x0a /* Public */,
      54,    1,  467,    2, 0x0a /* Public */,
      55,    5,  470,    2, 0x0a /* Public */,
      56,    1,  481,    2, 0x0a /* Public */,
      57,    1,  484,    2, 0x0a /* Public */,
      58,    0,  487,    2, 0x0a /* Public */,
      59,    0,  488,    2, 0x0a /* Public */,
      60,    0,  489,    2, 0x0a /* Public */,
      61,    1,  490,    2, 0x0a /* Public */,
      63,    0,  493,    2, 0x0a /* Public */,
      64,    2,  494,    2, 0x0a /* Public */,
      65,    2,  499,    2, 0x0a /* Public */,
      66,    2,  504,    2, 0x0a /* Public */,
      68,    8,  509,    2, 0x0a /* Public */,
      69,    5,  526,    2, 0x0a /* Public */,
      70,    1,  537,    2, 0x0a /* Public */,
      71,    1,  540,    2, 0x0a /* Public */,
      72,    1,  543,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    4,    6,    7,    8,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Int, QMetaType::QString,    4,   10,   11,   12,   13,   14,   15,
    QMetaType::Void, QMetaType::Double, QMetaType::QString, QMetaType::Int, QMetaType::QString,    4,   17,   14,   15,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    4,   24,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    4,   24,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    4,   10,   11,   12,   13,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   36,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    4,    6,    7,    8,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Int, QMetaType::QString,    4,   10,   11,   12,   13,   14,   15,
    QMetaType::Void, QMetaType::Double, QMetaType::QString, QMetaType::Int, QMetaType::QString,    4,   17,   14,   15,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::QString,   36,
    QMetaType::Void, QMetaType::QString,   36,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    4,   24,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    4,   24,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::QString,   36,
    QMetaType::Void, QMetaType::QString,   36,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    4,   10,   11,   12,   13,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   62,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   24,   62,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   24,   62,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   67,   62,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Int, QMetaType::QString, QMetaType::QString, QMetaType::QString,   10,   11,   12,   13,   14,   15,   62,   36,
    QMetaType::Void, QMetaType::QString, QMetaType::Int, QMetaType::QString, QMetaType::QString, QMetaType::QString,   17,   14,   15,   62,   36,
    QMetaType::Void, QMetaType::QString,   62,
    QMetaType::Void, QMetaType::QString,   62,
    QMetaType::Void, QMetaType::QString,   62,

       0        // eod
};

void COMM_UI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<COMM_UI *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_send_status(); break;
        case 1: _t->signal_motorinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->signal_move_jog((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        case 3: _t->signal_move_target((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< int(*)>(_a[6])),(*reinterpret_cast< QString(*)>(_a[7]))); break;
        case 4: _t->signal_move_goal((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 5: _t->signal_move_pause((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: _t->signal_move_resume((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->signal_move_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: _t->signal_mapping_start((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->signal_mapping_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 10: _t->signal_mapping_save((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 11: _t->signal_mapping_reload((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: _t->signal_mapload((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 13: _t->signal_localization_autoinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 14: _t->signal_localization_semiautoinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 15: _t->signal_localization_init((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5]))); break;
        case 16: _t->signal_localization_start((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 17: _t->signal_localization_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 18: _t->reconnect_loop(); break;
        case 19: _t->connected(); break;
        case 20: _t->disconnected(); break;
        case 21: _t->recv_message((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 22: _t->slot_motorinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 23: _t->slot_move_jog((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        case 24: _t->slot_move_target((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< int(*)>(_a[6])),(*reinterpret_cast< QString(*)>(_a[7]))); break;
        case 25: _t->slot_move_goal((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 26: _t->slot_move_pause((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 27: _t->slot_move_resume((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 28: _t->slot_move_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 29: _t->slot_move_succeed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 30: _t->slot_move_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 31: _t->slot_mapping_start((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 32: _t->slot_mapping_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 33: _t->slot_mapping_save((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 34: _t->slot_mapping_reload((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 35: _t->slot_mapload((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 36: _t->slot_localization_autoinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 37: _t->slot_localization_semiautoinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 38: _t->slot_localization_semiautoinit_succeed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 39: _t->slot_localization_semiautoinit_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 40: _t->slot_localization_init((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5]))); break;
        case 41: _t->slot_localization_start((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 42: _t->slot_localization_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 43: _t->send_status(); break;
        case 44: _t->send_global_path(); break;
        case 45: _t->send_local_path(); break;
        case 46: _t->send_mapping_start_response((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 47: _t->send_mapping_stop_response(); break;
        case 48: _t->send_mapping_save_response((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 49: _t->send_mapload_response((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 50: _t->send_localization_response((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 51: _t->send_move_target_response((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5])),(*reinterpret_cast< QString(*)>(_a[6])),(*reinterpret_cast< QString(*)>(_a[7])),(*reinterpret_cast< QString(*)>(_a[8]))); break;
        case 52: _t->send_move_goal_response((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4])),(*reinterpret_cast< QString(*)>(_a[5]))); break;
        case 53: _t->send_move_pause_response((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 54: _t->send_move_resume_response((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 55: _t->send_move_stop_response((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (COMM_UI::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_send_status)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_motorinit)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double , double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_move_jog)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double , double , double , double , double , int , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_move_target)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double , QString , int , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_move_goal)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_move_pause)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_move_resume)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_move_stop)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_mapping_start)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_mapping_stop)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_mapping_save)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_mapping_reload)) {
                *result = 11;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_mapload)) {
                *result = 12;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_localization_autoinit)) {
                *result = 13;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_localization_semiautoinit)) {
                *result = 14;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double , double , double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_localization_init)) {
                *result = 15;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_localization_start)) {
                *result = 16;
                return;
            }
        }
        {
            using _t = void (COMM_UI::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_UI::signal_localization_stop)) {
                *result = 17;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject COMM_UI::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_COMM_UI.data,
    qt_meta_data_COMM_UI,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *COMM_UI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *COMM_UI::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_COMM_UI.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int COMM_UI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 56)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 56;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 56)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 56;
    }
    return _id;
}

// SIGNAL 0
void COMM_UI::signal_send_status()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void COMM_UI::signal_motorinit(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void COMM_UI::signal_move_jog(double _t1, double _t2, double _t3, double _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void COMM_UI::signal_move_target(double _t1, double _t2, double _t3, double _t4, double _t5, int _t6, QString _t7)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t5))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t6))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t7))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void COMM_UI::signal_move_goal(double _t1, QString _t2, int _t3, QString _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void COMM_UI::signal_move_pause(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void COMM_UI::signal_move_resume(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void COMM_UI::signal_move_stop(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void COMM_UI::signal_mapping_start(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void COMM_UI::signal_mapping_stop(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void COMM_UI::signal_mapping_save(double _t1, QString _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void COMM_UI::signal_mapping_reload(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void COMM_UI::signal_mapload(double _t1, QString _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void COMM_UI::signal_localization_autoinit(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}

// SIGNAL 14
void COMM_UI::signal_localization_semiautoinit(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 14, _a);
}

// SIGNAL 15
void COMM_UI::signal_localization_init(double _t1, double _t2, double _t3, double _t4, double _t5)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t5))) };
    QMetaObject::activate(this, &staticMetaObject, 15, _a);
}

// SIGNAL 16
void COMM_UI::signal_localization_start(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 16, _a);
}

// SIGNAL 17
void COMM_UI::signal_localization_stop(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 17, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
