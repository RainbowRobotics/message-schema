/****************************************************************************
** Meta object code from reading C++ file 'comm_fms.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../comm_fms.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'comm_fms.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_COMM_FMS_t {
    QByteArrayData data[22];
    char stringdata0[252];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_COMM_FMS_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_COMM_FMS_t qt_meta_stringdata_COMM_FMS = {
    {
QT_MOC_LITERAL(0, 0, 8), // "COMM_FMS"
QT_MOC_LITERAL(1, 9, 16), // "signal_send_info"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 14), // "signal_mapload"
QT_MOC_LITERAL(4, 42, 4), // "time"
QT_MOC_LITERAL(5, 47, 4), // "name"
QT_MOC_LITERAL(6, 52, 11), // "signal_init"
QT_MOC_LITERAL(7, 64, 18), // "signal_random_init"
QT_MOC_LITERAL(8, 83, 4), // "seed"
QT_MOC_LITERAL(9, 88, 17), // "signal_random_seq"
QT_MOC_LITERAL(10, 106, 16), // "signal_regist_id"
QT_MOC_LITERAL(11, 123, 2), // "id"
QT_MOC_LITERAL(12, 126, 14), // "reconnect_loop"
QT_MOC_LITERAL(13, 141, 9), // "connected"
QT_MOC_LITERAL(14, 151, 12), // "disconnected"
QT_MOC_LITERAL(15, 164, 12), // "recv_message"
QT_MOC_LITERAL(16, 177, 3), // "buf"
QT_MOC_LITERAL(17, 181, 14), // "slot_send_info"
QT_MOC_LITERAL(18, 196, 12), // "slot_mapload"
QT_MOC_LITERAL(19, 209, 9), // "slot_init"
QT_MOC_LITERAL(20, 219, 16), // "slot_random_init"
QT_MOC_LITERAL(21, 236, 15) // "slot_random_seq"

    },
    "COMM_FMS\0signal_send_info\0\0signal_mapload\0"
    "time\0name\0signal_init\0signal_random_init\0"
    "seed\0signal_random_seq\0signal_regist_id\0"
    "id\0reconnect_loop\0connected\0disconnected\0"
    "recv_message\0buf\0slot_send_info\0"
    "slot_mapload\0slot_init\0slot_random_init\0"
    "slot_random_seq"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_COMM_FMS[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   89,    2, 0x06 /* Public */,
       3,    2,   90,    2, 0x06 /* Public */,
       6,    1,   95,    2, 0x06 /* Public */,
       7,    2,   98,    2, 0x06 /* Public */,
       9,    1,  103,    2, 0x06 /* Public */,
      10,    1,  106,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      12,    0,  109,    2, 0x08 /* Private */,
      13,    0,  110,    2, 0x08 /* Private */,
      14,    0,  111,    2, 0x08 /* Private */,
      15,    1,  112,    2, 0x08 /* Private */,
      17,    0,  115,    2, 0x08 /* Private */,
      18,    2,  116,    2, 0x08 /* Private */,
      19,    1,  121,    2, 0x08 /* Private */,
      20,    2,  124,    2, 0x08 /* Private */,
      21,    1,  129,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    4,    5,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    4,    8,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::QString,   11,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QByteArray,   16,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    4,    5,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::QString,    4,    8,
    QMetaType::Void, QMetaType::Double,    4,

       0        // eod
};

void COMM_FMS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<COMM_FMS *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_send_info(); break;
        case 1: _t->signal_mapload((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 2: _t->signal_init((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->signal_random_init((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 4: _t->signal_random_seq((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->signal_regist_id((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->reconnect_loop(); break;
        case 7: _t->connected(); break;
        case 8: _t->disconnected(); break;
        case 9: _t->recv_message((*reinterpret_cast< const QByteArray(*)>(_a[1]))); break;
        case 10: _t->slot_send_info(); break;
        case 11: _t->slot_mapload((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 12: _t->slot_init((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->slot_random_init((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 14: _t->slot_random_seq((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (COMM_FMS::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_FMS::signal_send_info)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (COMM_FMS::*)(double , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_FMS::signal_mapload)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (COMM_FMS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_FMS::signal_init)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (COMM_FMS::*)(double , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_FMS::signal_random_init)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (COMM_FMS::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_FMS::signal_random_seq)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (COMM_FMS::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&COMM_FMS::signal_regist_id)) {
                *result = 5;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject COMM_FMS::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_COMM_FMS.data,
    qt_meta_data_COMM_FMS,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *COMM_FMS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *COMM_FMS::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_COMM_FMS.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int COMM_FMS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void COMM_FMS::signal_send_info()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void COMM_FMS::signal_mapload(double _t1, QString _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void COMM_FMS::signal_init(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void COMM_FMS::signal_random_init(double _t1, QString _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void COMM_FMS::signal_random_seq(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void COMM_FMS::signal_regist_id(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
