/****************************************************************************
** Meta object code from reading C++ file 'comm_ms.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../comm_ms.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'comm_ms.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_COMM_MS_t {
    QByteArrayData data[79];
    char stringdata0[1269];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_COMM_MS_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_COMM_MS_t qt_meta_stringdata_COMM_MS = {
    {
QT_MOC_LITERAL(0, 0, 7), // "COMM_MS"
QT_MOC_LITERAL(1, 8, 16), // "signal_motorinit"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 4), // "time"
QT_MOC_LITERAL(4, 31, 15), // "signal_move_jog"
QT_MOC_LITERAL(5, 47, 2), // "vx"
QT_MOC_LITERAL(6, 50, 2), // "vy"
QT_MOC_LITERAL(7, 53, 2), // "wz"
QT_MOC_LITERAL(8, 56, 18), // "signal_move_target"
QT_MOC_LITERAL(9, 75, 1), // "x"
QT_MOC_LITERAL(10, 77, 1), // "y"
QT_MOC_LITERAL(11, 79, 1), // "z"
QT_MOC_LITERAL(12, 81, 2), // "rz"
QT_MOC_LITERAL(13, 84, 6), // "preset"
QT_MOC_LITERAL(14, 91, 6), // "method"
QT_MOC_LITERAL(15, 98, 16), // "signal_move_goal"
QT_MOC_LITERAL(16, 115, 7), // "node_id"
QT_MOC_LITERAL(17, 123, 17), // "signal_move_pause"
QT_MOC_LITERAL(18, 141, 18), // "signal_move_resume"
QT_MOC_LITERAL(19, 160, 16), // "signal_move_stop"
QT_MOC_LITERAL(20, 177, 20), // "signal_mapping_start"
QT_MOC_LITERAL(21, 198, 19), // "signal_mapping_stop"
QT_MOC_LITERAL(22, 218, 19), // "signal_mapping_save"
QT_MOC_LITERAL(23, 238, 4), // "name"
QT_MOC_LITERAL(24, 243, 21), // "signal_mapping_reload"
QT_MOC_LITERAL(25, 265, 14), // "signal_mapload"
QT_MOC_LITERAL(26, 280, 28), // "signal_localization_autoinit"
QT_MOC_LITERAL(27, 309, 32), // "signal_localization_semiautoinit"
QT_MOC_LITERAL(28, 342, 24), // "signal_localization_init"
QT_MOC_LITERAL(29, 367, 25), // "signal_localization_start"
QT_MOC_LITERAL(30, 393, 24), // "signal_localization_stop"
QT_MOC_LITERAL(31, 418, 19), // "signal_docking_dock"
QT_MOC_LITERAL(32, 438, 21), // "signal_docking_undock"
QT_MOC_LITERAL(33, 460, 13), // "sio_connected"
QT_MOC_LITERAL(34, 474, 16), // "sio_disconnected"
QT_MOC_LITERAL(35, 491, 25), // "sio::client::close_reason"
QT_MOC_LITERAL(36, 517, 6), // "reason"
QT_MOC_LITERAL(37, 524, 9), // "sio_error"
QT_MOC_LITERAL(38, 534, 14), // "recv_motorinit"
QT_MOC_LITERAL(39, 549, 11), // "std::string"
QT_MOC_LITERAL(40, 561, 17), // "sio::message::ptr"
QT_MOC_LITERAL(41, 579, 4), // "data"
QT_MOC_LITERAL(42, 584, 6), // "hasAck"
QT_MOC_LITERAL(43, 591, 19), // "sio::message::list&"
QT_MOC_LITERAL(44, 611, 8), // "ack_resp"
QT_MOC_LITERAL(45, 620, 9), // "recv_move"
QT_MOC_LITERAL(46, 630, 12), // "recv_mapping"
QT_MOC_LITERAL(47, 643, 12), // "recv_mapload"
QT_MOC_LITERAL(48, 656, 17), // "recv_localization"
QT_MOC_LITERAL(49, 674, 17), // "recv_docking_dock"
QT_MOC_LITERAL(50, 692, 19), // "recv_docking_undock"
QT_MOC_LITERAL(51, 712, 14), // "slot_motorinit"
QT_MOC_LITERAL(52, 727, 13), // "slot_move_jog"
QT_MOC_LITERAL(53, 741, 16), // "slot_move_target"
QT_MOC_LITERAL(54, 758, 14), // "slot_move_goal"
QT_MOC_LITERAL(55, 773, 15), // "slot_move_pause"
QT_MOC_LITERAL(56, 789, 16), // "slot_move_resume"
QT_MOC_LITERAL(57, 806, 14), // "slot_move_stop"
QT_MOC_LITERAL(58, 821, 17), // "slot_move_succeed"
QT_MOC_LITERAL(59, 839, 7), // "message"
QT_MOC_LITERAL(60, 847, 16), // "slot_move_failed"
QT_MOC_LITERAL(61, 864, 18), // "slot_mapping_start"
QT_MOC_LITERAL(62, 883, 17), // "slot_mapping_stop"
QT_MOC_LITERAL(63, 901, 17), // "slot_mapping_save"
QT_MOC_LITERAL(64, 919, 19), // "slot_mapping_reload"
QT_MOC_LITERAL(65, 939, 12), // "slot_mapload"
QT_MOC_LITERAL(66, 952, 26), // "slot_localization_autoinit"
QT_MOC_LITERAL(67, 979, 30), // "slot_localization_semiautoinit"
QT_MOC_LITERAL(68, 1010, 38), // "slot_localization_semiautoini..."
QT_MOC_LITERAL(69, 1049, 37), // "slot_localization_semiautoini..."
QT_MOC_LITERAL(70, 1087, 22), // "slot_localization_init"
QT_MOC_LITERAL(71, 1110, 23), // "slot_localization_start"
QT_MOC_LITERAL(72, 1134, 22), // "slot_localization_stop"
QT_MOC_LITERAL(73, 1157, 17), // "slot_docking_dock"
QT_MOC_LITERAL(74, 1175, 19), // "slot_docking_undock"
QT_MOC_LITERAL(75, 1195, 17), // "slot_dock_success"
QT_MOC_LITERAL(76, 1213, 16), // "slot_dock_failed"
QT_MOC_LITERAL(77, 1230, 19), // "slot_undock_success"
QT_MOC_LITERAL(78, 1250, 18) // "slot_undock_failed"

    },
    "COMM_MS\0signal_motorinit\0\0time\0"
    "signal_move_jog\0vx\0vy\0wz\0signal_move_target\0"
    "x\0y\0z\0rz\0preset\0method\0signal_move_goal\0"
    "node_id\0signal_move_pause\0signal_move_resume\0"
    "signal_move_stop\0signal_mapping_start\0"
    "signal_mapping_stop\0signal_mapping_save\0"
    "name\0signal_mapping_reload\0signal_mapload\0"
    "signal_localization_autoinit\0"
    "signal_localization_semiautoinit\0"
    "signal_localization_init\0"
    "signal_localization_start\0"
    "signal_localization_stop\0signal_docking_dock\0"
    "signal_docking_undock\0sio_connected\0"
    "sio_disconnected\0sio::client::close_reason\0"
    "reason\0sio_error\0recv_motorinit\0"
    "std::string\0sio::message::ptr\0data\0"
    "hasAck\0sio::message::list&\0ack_resp\0"
    "recv_move\0recv_mapping\0recv_mapload\0"
    "recv_localization\0recv_docking_dock\0"
    "recv_docking_undock\0slot_motorinit\0"
    "slot_move_jog\0slot_move_target\0"
    "slot_move_goal\0slot_move_pause\0"
    "slot_move_resume\0slot_move_stop\0"
    "slot_move_succeed\0message\0slot_move_failed\0"
    "slot_mapping_start\0slot_mapping_stop\0"
    "slot_mapping_save\0slot_mapping_reload\0"
    "slot_mapload\0slot_localization_autoinit\0"
    "slot_localization_semiautoinit\0"
    "slot_localization_semiautoinit_succeed\0"
    "slot_localization_semiautoinit_failed\0"
    "slot_localization_init\0slot_localization_start\0"
    "slot_localization_stop\0slot_docking_dock\0"
    "slot_docking_undock\0slot_dock_success\0"
    "slot_dock_failed\0slot_undock_success\0"
    "slot_undock_failed"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_COMM_MS[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      56,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      19,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  294,    2, 0x06 /* Public */,
       4,    4,  297,    2, 0x06 /* Public */,
       8,    7,  306,    2, 0x06 /* Public */,
      15,    4,  321,    2, 0x06 /* Public */,
      17,    1,  330,    2, 0x06 /* Public */,
      18,    1,  333,    2, 0x06 /* Public */,
      19,    1,  336,    2, 0x06 /* Public */,
      20,    1,  339,    2, 0x06 /* Public */,
      21,    1,  342,    2, 0x06 /* Public */,
      22,    2,  345,    2, 0x06 /* Public */,
      24,    1,  350,    2, 0x06 /* Public */,
      25,    2,  353,    2, 0x06 /* Public */,
      26,    1,  358,    2, 0x06 /* Public */,
      27,    1,  361,    2, 0x06 /* Public */,
      28,    5,  364,    2, 0x06 /* Public */,
      29,    1,  375,    2, 0x06 /* Public */,
      30,    1,  378,    2, 0x06 /* Public */,
      31,    1,  381,    2, 0x06 /* Public */,
      32,    1,  384,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      33,    0,  387,    2, 0x08 /* Private */,
      34,    1,  388,    2, 0x08 /* Private */,
      37,    0,  391,    2, 0x08 /* Private */,
      38,    4,  392,    2, 0x08 /* Private */,
      45,    4,  401,    2, 0x08 /* Private */,
      46,    4,  410,    2, 0x08 /* Private */,
      47,    4,  419,    2, 0x08 /* Private */,
      48,    4,  428,    2, 0x08 /* Private */,
      49,    4,  437,    2, 0x08 /* Private */,
      50,    4,  446,    2, 0x08 /* Private */,
      51,    1,  455,    2, 0x08 /* Private */,
      52,    4,  458,    2, 0x08 /* Private */,
      53,    7,  467,    2, 0x08 /* Private */,
      54,    4,  482,    2, 0x08 /* Private */,
      55,    1,  491,    2, 0x08 /* Private */,
      56,    1,  494,    2, 0x08 /* Private */,
      57,    1,  497,    2, 0x08 /* Private */,
      58,    1,  500,    2, 0x08 /* Private */,
      60,    1,  503,    2, 0x08 /* Private */,
      61,    1,  506,    2, 0x08 /* Private */,
      62,    1,  509,    2, 0x08 /* Private */,
      63,    2,  512,    2, 0x08 /* Private */,
      64,    1,  517,    2, 0x08 /* Private */,
      65,    2,  520,    2, 0x08 /* Private */,
      66,    1,  525,    2, 0x08 /* Private */,
      67,    1,  528,    2, 0x08 /* Private */,
      68,    1,  531,    2, 0x08 /* Private */,
      69,    1,  534,    2, 0x08 /* Private */,
      70,    5,  537,    2, 0x08 /* Private */,
      71,    1,  548,    2, 0x08 /* Private */,
      72,    1,  551,    2, 0x08 /* Private */,
      73,    1,  554,    2, 0x08 /* Private */,
      74,    1,  557,    2, 0x08 /* Private */,
      75,    1,  560,    2, 0x08 /* Private */,
      76,    1,  563,    2, 0x08 /* Private */,
      77,    1,  566,    2, 0x08 /* Private */,
      78,    1,  569,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    3,    5,    6,    7,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Int, QMetaType::QString,    3,    9,   10,   11,   12,   13,   14,
    QMetaType::Void, QMetaType::Double, QMetaType::QString, QMetaType::Int, QMetaType::QString,    3,   16,   13,   14,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    3,   23,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    3,   23,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    3,    9,   10,   11,   12,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 35,   36,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 39, 0x80000000 | 40, QMetaType::Bool, 0x80000000 | 43,   23,   41,   42,   44,
    QMetaType::Void, 0x80000000 | 39, 0x80000000 | 40, QMetaType::Bool, 0x80000000 | 43,   23,   41,   42,   44,
    QMetaType::Void, 0x80000000 | 39, 0x80000000 | 40, QMetaType::Bool, 0x80000000 | 43,   23,   41,   42,   44,
    QMetaType::Void, 0x80000000 | 39, 0x80000000 | 40, QMetaType::Bool, 0x80000000 | 43,   23,   41,   42,   44,
    QMetaType::Void, 0x80000000 | 39, 0x80000000 | 40, QMetaType::Bool, 0x80000000 | 43,   23,   41,   42,   44,
    QMetaType::Void, 0x80000000 | 39, 0x80000000 | 40, QMetaType::Bool, 0x80000000 | 43,   23,   41,   42,   44,
    QMetaType::Void, 0x80000000 | 39, 0x80000000 | 40, QMetaType::Bool, 0x80000000 | 43,   23,   41,   42,   44,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    3,    5,    6,    7,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Int, QMetaType::QString,    3,    9,   10,   11,   12,   13,   14,
    QMetaType::Void, QMetaType::Double, QMetaType::QString, QMetaType::Int, QMetaType::QString,    3,   16,   13,   14,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::QString,   59,
    QMetaType::Void, QMetaType::QString,   59,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    3,   23,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    3,   23,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::QString,   59,
    QMetaType::Void, QMetaType::QString,   59,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    3,    9,   10,   11,   12,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::QString,   59,
    QMetaType::Void, QMetaType::QString,   59,
    QMetaType::Void, QMetaType::QString,   59,
    QMetaType::Void, QMetaType::QString,   59,

       0        // eod
};

void COMM_MS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<COMM_MS *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_motorinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 1: _t->signal_move_jog((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        case 2: _t->signal_move_target((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< int(*)>(_a[6])),(*reinterpret_cast< QString(*)>(_a[7]))); break;
        case 3: _t->signal_move_goal((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 4: _t->signal_move_pause((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->signal_move_resume((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: _t->signal_move_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->signal_mapping_start((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: _t->signal_mapping_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->signal_mapping_save((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 10: _t->signal_mapping_reload((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: _t->signal_mapload((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 12: _t->signal_localization_autoinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->signal_localization_semiautoinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 14: _t->signal_localization_init((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5]))); break;
        case 15: _t->signal_localization_start((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 16: _t->signal_localization_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 17: _t->signal_docking_dock((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 18: _t->signal_docking_undock((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 19: _t->sio_connected(); break;
        case 20: _t->sio_disconnected((*reinterpret_cast< const sio::client::close_reason(*)>(_a[1]))); break;
        case 21: _t->sio_error(); break;
        case 22: _t->recv_motorinit((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const sio::message::ptr(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< sio::message::list(*)>(_a[4]))); break;
        case 23: _t->recv_move((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const sio::message::ptr(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< sio::message::list(*)>(_a[4]))); break;
        case 24: _t->recv_mapping((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const sio::message::ptr(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< sio::message::list(*)>(_a[4]))); break;
        case 25: _t->recv_mapload((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const sio::message::ptr(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< sio::message::list(*)>(_a[4]))); break;
        case 26: _t->recv_localization((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const sio::message::ptr(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< sio::message::list(*)>(_a[4]))); break;
        case 27: _t->recv_docking_dock((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const sio::message::ptr(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< sio::message::list(*)>(_a[4]))); break;
        case 28: _t->recv_docking_undock((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const sio::message::ptr(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< sio::message::list(*)>(_a[4]))); break;
        case 29: _t->slot_motorinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 30: _t->slot_move_jog((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        case 31: _t->slot_move_target((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< int(*)>(_a[6])),(*reinterpret_cast< QString(*)>(_a[7]))); break;
        case 32: _t->slot_move_goal((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 33: _t->slot_move_pause((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 34: _t->slot_move_resume((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 35: _t->slot_move_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 36: _t->slot_move_succeed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 37: _t->slot_move_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 38: _t->slot_mapping_start((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 39: _t->slot_mapping_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 40: _t->slot_mapping_save((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 41: _t->slot_mapping_reload((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 42: _t->slot_mapload((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 43: _t->slot_localization_autoinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 44: _t->slot_localization_semiautoinit((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 45: _t->slot_localization_semiautoinit_succeed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 46: _t->slot_localization_semiautoinit_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 47: _t->slot_localization_init((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5]))); break;
        case 48: _t->slot_localization_start((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 49: _t->slot_localization_stop((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 50: _t->slot_docking_dock((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 51: _t->slot_docking_undock((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 52: _t->slot_dock_success((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 53: _t->slot_dock_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 54: _t->slot_undock_success((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 55: _t->slot_undock_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_motorinit)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double , double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_move_jog)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double , double , double , double , double , int , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_move_target)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double , QString , int , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_move_goal)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_move_pause)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_move_resume)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_move_stop)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_mapping_start)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_mapping_stop)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_mapping_save)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_mapping_reload)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_mapload)) {
                *result = 11;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_localization_autoinit)) {
                *result = 12;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_localization_semiautoinit)) {
                *result = 13;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double , double , double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_localization_init)) {
                *result = 14;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_localization_start)) {
                *result = 15;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_localization_stop)) {
                *result = 16;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_docking_dock)) {
                *result = 17;
                return;
            }
        }
        {
            using _t = void (COMM_MS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_MS::signal_docking_undock)) {
                *result = 18;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject COMM_MS::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_COMM_MS.data,
    qt_meta_data_COMM_MS,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *COMM_MS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *COMM_MS::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_COMM_MS.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int COMM_MS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void COMM_MS::signal_motorinit(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void COMM_MS::signal_move_jog(double _t1, double _t2, double _t3, double _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void COMM_MS::signal_move_target(double _t1, double _t2, double _t3, double _t4, double _t5, int _t6, QString _t7)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t5))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t6))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t7))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void COMM_MS::signal_move_goal(double _t1, QString _t2, int _t3, QString _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void COMM_MS::signal_move_pause(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void COMM_MS::signal_move_resume(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void COMM_MS::signal_move_stop(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void COMM_MS::signal_mapping_start(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void COMM_MS::signal_mapping_stop(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void COMM_MS::signal_mapping_save(double _t1, QString _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void COMM_MS::signal_mapping_reload(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void COMM_MS::signal_mapload(double _t1, QString _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void COMM_MS::signal_localization_autoinit(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void COMM_MS::signal_localization_semiautoinit(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}

// SIGNAL 14
void COMM_MS::signal_localization_init(double _t1, double _t2, double _t3, double _t4, double _t5)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t5))) };
    QMetaObject::activate(this, &staticMetaObject, 14, _a);
}

// SIGNAL 15
void COMM_MS::signal_localization_start(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 15, _a);
}

// SIGNAL 16
void COMM_MS::signal_localization_stop(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 16, _a);
}

// SIGNAL 17
void COMM_MS::signal_docking_dock(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 17, _a);
}

// SIGNAL 18
void COMM_MS::signal_docking_undock(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 18, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
