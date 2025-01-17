/****************************************************************************
** Meta object code from reading C++ file 'autocontrol.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../autocontrol.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'autocontrol.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_AUTOCONTROL_t {
    QByteArrayData data[7];
    char stringdata0[113];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_AUTOCONTROL_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_AUTOCONTROL_t qt_meta_stringdata_AUTOCONTROL = {
    {
QT_MOC_LITERAL(0, 0, 11), // "AUTOCONTROL"
QT_MOC_LITERAL(1, 12, 26), // "signal_global_path_updated"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 25), // "signal_local_path_updated"
QT_MOC_LITERAL(4, 66, 19), // "signal_move_succeed"
QT_MOC_LITERAL(5, 86, 7), // "message"
QT_MOC_LITERAL(6, 94, 18) // "signal_move_failed"

    },
    "AUTOCONTROL\0signal_global_path_updated\0"
    "\0signal_local_path_updated\0"
    "signal_move_succeed\0message\0"
    "signal_move_failed"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AUTOCONTROL[] = {

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
       1,    0,   34,    2, 0x06 /* Public */,
       3,    0,   35,    2, 0x06 /* Public */,
       4,    1,   36,    2, 0x06 /* Public */,
       6,    1,   39,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::QString,    5,

       0        // eod
};

void AUTOCONTROL::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AUTOCONTROL *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_global_path_updated(); break;
        case 1: _t->signal_local_path_updated(); break;
        case 2: _t->signal_move_succeed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->signal_move_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AUTOCONTROL::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AUTOCONTROL::signal_global_path_updated)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AUTOCONTROL::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AUTOCONTROL::signal_local_path_updated)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (AUTOCONTROL::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AUTOCONTROL::signal_move_succeed)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (AUTOCONTROL::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AUTOCONTROL::signal_move_failed)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject AUTOCONTROL::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_AUTOCONTROL.data,
    qt_meta_data_AUTOCONTROL,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *AUTOCONTROL::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AUTOCONTROL::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_AUTOCONTROL.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int AUTOCONTROL::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void AUTOCONTROL::signal_global_path_updated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void AUTOCONTROL::signal_local_path_updated()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void AUTOCONTROL::signal_move_succeed(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void AUTOCONTROL::signal_move_failed(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
