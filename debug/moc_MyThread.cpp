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
    QByteArrayData data[22];
    char stringdata0[352];
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
QT_MOC_LITERAL(3, 25, 20), // "std::vector<uint8_t>"
QT_MOC_LITERAL(4, 46, 4), // "data"
QT_MOC_LITERAL(5, 51, 10), // "portClosed"
QT_MOC_LITERAL(6, 62, 13), // "errorOccurred"
QT_MOC_LITERAL(7, 76, 5), // "error"
QT_MOC_LITERAL(8, 82, 24), // "signal_ComportSearchBack"
QT_MOC_LITERAL(9, 107, 14), // "QList<QString>"
QT_MOC_LITERAL(10, 122, 25), // "signal_ComportConnectBack"
QT_MOC_LITERAL(11, 148, 23), // "signal_ComportCloseBack"
QT_MOC_LITERAL(12, 172, 23), // "signal_ComportWriteBack"
QT_MOC_LITERAL(13, 196, 22), // "signal_ComportReadBack"
QT_MOC_LITERAL(14, 219, 15), // "signal_PaintADC"
QT_MOC_LITERAL(15, 235, 20), // "signal_PaintFeedBack"
QT_MOC_LITERAL(16, 256, 16), // "on_ComportSearch"
QT_MOC_LITERAL(17, 273, 17), // "on_ComportConnect"
QT_MOC_LITERAL(18, 291, 8), // "baudRate"
QT_MOC_LITERAL(19, 300, 15), // "on_ComportClose"
QT_MOC_LITERAL(20, 316, 15), // "on_ComportWrite"
QT_MOC_LITERAL(21, 332, 19) // "on_ComportStartRead"

    },
    "MyThread_1\0dataReceived\0\0std::vector<uint8_t>\0"
    "data\0portClosed\0errorOccurred\0error\0"
    "signal_ComportSearchBack\0QList<QString>\0"
    "signal_ComportConnectBack\0"
    "signal_ComportCloseBack\0signal_ComportWriteBack\0"
    "signal_ComportReadBack\0signal_PaintADC\0"
    "signal_PaintFeedBack\0on_ComportSearch\0"
    "on_ComportConnect\0baudRate\0on_ComportClose\0"
    "on_ComportWrite\0on_ComportStartRead"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MyThread_1[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      10,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   89,    2, 0x06 /* Public */,
       5,    0,   92,    2, 0x06 /* Public */,
       6,    1,   93,    2, 0x06 /* Public */,
       8,    1,   96,    2, 0x06 /* Public */,
      10,    1,   99,    2, 0x06 /* Public */,
      11,    1,  102,    2, 0x06 /* Public */,
      12,    0,  105,    2, 0x06 /* Public */,
      13,    0,  106,    2, 0x06 /* Public */,
      14,    0,  107,    2, 0x06 /* Public */,
      15,    0,  108,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      16,    0,  109,    2, 0x0a /* Public */,
      17,    2,  110,    2, 0x0a /* Public */,
      19,    0,  115,    2, 0x0a /* Public */,
      20,    1,  116,    2, 0x0a /* Public */,
      21,    0,  119,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void, 0x80000000 | 9,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    2,   18,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void,

       0        // eod
};

void MyThread_1::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MyThread_1 *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->dataReceived((*reinterpret_cast< const std::vector<uint8_t>(*)>(_a[1]))); break;
        case 1: _t->portClosed(); break;
        case 2: _t->errorOccurred((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->signal_ComportSearchBack((*reinterpret_cast< QList<QString>(*)>(_a[1]))); break;
        case 4: _t->signal_ComportConnectBack((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->signal_ComportCloseBack((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->signal_ComportWriteBack(); break;
        case 7: _t->signal_ComportReadBack(); break;
        case 8: _t->signal_PaintADC(); break;
        case 9: _t->signal_PaintFeedBack(); break;
        case 10: _t->on_ComportSearch(); break;
        case 11: _t->on_ComportConnect((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< qint32(*)>(_a[2]))); break;
        case 12: _t->on_ComportClose(); break;
        case 13: _t->on_ComportWrite((*reinterpret_cast< std::vector<uint8_t>(*)>(_a[1]))); break;
        case 14: _t->on_ComportStartRead(); break;
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
            using _t = void (MyThread_1::*)(const std::vector<uint8_t> & );
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
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::signal_ComportWriteBack)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::signal_ComportReadBack)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::signal_PaintADC)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::signal_PaintFeedBack)) {
                *result = 9;
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
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void MyThread_1::dataReceived(const std::vector<uint8_t> & _t1)
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

// SIGNAL 6
void MyThread_1::signal_ComportWriteBack()
{
    QMetaObject::activate(this, &staticMetaObject, 6, nullptr);
}

// SIGNAL 7
void MyThread_1::signal_ComportReadBack()
{
    QMetaObject::activate(this, &staticMetaObject, 7, nullptr);
}

// SIGNAL 8
void MyThread_1::signal_PaintADC()
{
    QMetaObject::activate(this, &staticMetaObject, 8, nullptr);
}

// SIGNAL 9
void MyThread_1::signal_PaintFeedBack()
{
    QMetaObject::activate(this, &staticMetaObject, 9, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
