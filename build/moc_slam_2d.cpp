/****************************************************************************
** Meta object code from reading C++ file 'slam_2d.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../slam_2d.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'slam_2d.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLAM_2D_t {
    QByteArrayData data[5];
    char stringdata0[98];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLAM_2D_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLAM_2D_t qt_meta_stringdata_SLAM_2D = {
    {
QT_MOC_LITERAL(0, 0, 7), // "SLAM_2D"
QT_MOC_LITERAL(1, 8, 40), // "signal_localization_semiautoi..."
QT_MOC_LITERAL(2, 49, 0), // ""
QT_MOC_LITERAL(3, 50, 7), // "message"
QT_MOC_LITERAL(4, 58, 39) // "signal_localization_semiautoi..."

    },
    "SLAM_2D\0signal_localization_semiautoinit_succeed\0"
    "\0message\0signal_localization_semiautoinit_failed"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLAM_2D[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,
       4,    1,   27,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,

       0        // eod
};

void SLAM_2D::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SLAM_2D *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_localization_semiautoinit_succeed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->signal_localization_semiautoinit_failed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (SLAM_2D::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLAM_2D::signal_localization_semiautoinit_succeed)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (SLAM_2D::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLAM_2D::signal_localization_semiautoinit_failed)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SLAM_2D::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_SLAM_2D.data,
    qt_meta_data_SLAM_2D,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SLAM_2D::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLAM_2D::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLAM_2D.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SLAM_2D::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void SLAM_2D::signal_localization_semiautoinit_succeed(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLAM_2D::signal_localization_semiautoinit_failed(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
