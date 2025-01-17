/****************************************************************************
** Meta object code from reading C++ file 'docking.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../docking.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'docking.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DOCKING_t {
    QByteArrayData data[7];
    char stringdata0[99];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DOCKING_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DOCKING_t qt_meta_stringdata_DOCKING = {
    {
QT_MOC_LITERAL(0, 0, 7), // "DOCKING"
QT_MOC_LITERAL(1, 8, 19), // "signal_dock_succeed"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 7), // "message"
QT_MOC_LITERAL(4, 37, 18), // "signal_dock_failed"
QT_MOC_LITERAL(5, 56, 21), // "signal_undock_succeed"
QT_MOC_LITERAL(6, 78, 20) // "signal_undock_failed"

    },
    "DOCKING\0signal_dock_succeed\0\0message\0"
    "signal_dock_failed\0signal_undock_succeed\0"
    "signal_undock_failed"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DOCKING[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       4,    1,   37,    2, 0x06 /* Public */,
       5,    1,   40,    2, 0x06 /* Public */,
       6,    1,   43,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,

       0        // eod
};

void DOCKING::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DOCKING *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_dock_succeed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->signal_dock_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->signal_undock_succeed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->signal_undock_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DOCKING::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DOCKING::signal_dock_succeed)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (DOCKING::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DOCKING::signal_dock_failed)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (DOCKING::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DOCKING::signal_undock_succeed)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (DOCKING::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DOCKING::signal_undock_failed)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject DOCKING::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_DOCKING.data,
    qt_meta_data_DOCKING,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DOCKING::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DOCKING::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DOCKING.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int DOCKING::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void DOCKING::signal_dock_succeed(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void DOCKING::signal_dock_failed(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void DOCKING::signal_undock_succeed(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void DOCKING::signal_undock_failed(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
