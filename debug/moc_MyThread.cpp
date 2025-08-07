/****************************************************************************
** Meta object code from reading C++ file 'MyThread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.16)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../MyThread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MyThread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.16. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MyThread_1_t {
    QByteArrayData data[15];
    char stringdata0[207];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MyThread_1_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MyThread_1_t qt_meta_stringdata_MyThread_1 = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MyThread_1"
QT_MOC_LITERAL(1, 11, 12), // "dataReceived"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 4), // "data"
QT_MOC_LITERAL(4, 30, 10), // "portClosed"
QT_MOC_LITERAL(5, 41, 13), // "errorOccurred"
QT_MOC_LITERAL(6, 55, 5), // "error"
QT_MOC_LITERAL(7, 61, 24), // "signal_ComportSearchBack"
QT_MOC_LITERAL(8, 86, 14), // "QList<QString>"
QT_MOC_LITERAL(9, 101, 25), // "signal_ComportConnectBack"
QT_MOC_LITERAL(10, 127, 23), // "signal_ComportCloseBack"
QT_MOC_LITERAL(11, 151, 16), // "on_ComPortSearch"
QT_MOC_LITERAL(12, 168, 17), // "on_ComPortConnect"
QT_MOC_LITERAL(13, 186, 8), // "baudRate"
QT_MOC_LITERAL(14, 195, 11) // "onReadyRead"

    },
    "MyThread_1\0dataReceived\0\0data\0portClosed\0"
    "errorOccurred\0error\0signal_ComportSearchBack\0"
    "QList<QString>\0signal_ComportConnectBack\0"
    "signal_ComportCloseBack\0on_ComPortSearch\0"
    "on_ComPortConnect\0baudRate\0onReadyRead"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MyThread_1[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x06 /* Public */,
       4,    0,   62,    2, 0x06 /* Public */,
       5,    1,   63,    2, 0x06 /* Public */,
       7,    1,   66,    2, 0x06 /* Public */,
       9,    1,   69,    2, 0x06 /* Public */,
      10,    1,   72,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    0,   75,    2, 0x0a /* Public */,
      12,    2,   76,    2, 0x0a /* Public */,
      14,    0,   81,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QByteArray,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, 0x80000000 | 8,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    2,   13,
    QMetaType::Void,

       0        // eod
};

void MyThread_1::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MyThread_1 *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->dataReceived((*reinterpret_cast< const QByteArray(*)>(_a[1]))); break;
        case 1: _t->portClosed(); break;
        case 2: _t->errorOccurred((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->signal_ComportSearchBack((*reinterpret_cast< QList<QString>(*)>(_a[1]))); break;
        case 4: _t->signal_ComportConnectBack((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->signal_ComportCloseBack((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->on_ComPortSearch(); break;
        case 7: _t->on_ComPortConnect((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< qint32(*)>(_a[2]))); break;
        case 8: _t->onReadyRead(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QList<QString> >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MyThread_1::*)(const QByteArray & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::dataReceived)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::portClosed)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::errorOccurred)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)(QList<QString> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::signal_ComportSearchBack)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::signal_ComportConnectBack)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::signal_ComportCloseBack)) {
                *result = 5;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MyThread_1::staticMetaObject = { {
    QMetaObject::SuperData::link<QThread::staticMetaObject>(),
    qt_meta_stringdata_MyThread_1.data,
    qt_meta_data_MyThread_1,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MyThread_1::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MyThread_1::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MyThread_1.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int MyThread_1::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void MyThread_1::dataReceived(const QByteArray & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MyThread_1::portClosed()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void MyThread_1::errorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MyThread_1::signal_ComportSearchBack(QList<QString> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void MyThread_1::signal_ComportConnectBack(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void MyThread_1::signal_ComportCloseBack(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
