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
    QByteArrayData data[28];
    char stringdata0[416];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MyThread_1_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MyThread_1_t qt_meta_stringdata_MyThread_1 = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MyThread_1"
QT_MOC_LITERAL(1, 11, 17), // "PaintGraph_signal"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 18), // "PaintGraph2_signal"
QT_MOC_LITERAL(4, 49, 24), // "ComportDataUpdate_signal"
QT_MOC_LITERAL(5, 74, 21), // "ComportConnect_signal"
QT_MOC_LITERAL(6, 96, 19), // "ComportClose_signal"
QT_MOC_LITERAL(7, 116, 21), // "ComPortConnect_signal"
QT_MOC_LITERAL(8, 138, 18), // "ComportRead_signal"
QT_MOC_LITERAL(9, 157, 19), // "ComPortWrite_signal"
QT_MOC_LITERAL(10, 177, 4), // "back"
QT_MOC_LITERAL(11, 182, 15), // "ComPortReadData"
QT_MOC_LITERAL(12, 198, 16), // "QVector<uint8_t>"
QT_MOC_LITERAL(13, 215, 19), // "ComPort_handleError"
QT_MOC_LITERAL(14, 235, 28), // "QSerialPort::SerialPortError"
QT_MOC_LITERAL(15, 264, 5), // "error"
QT_MOC_LITERAL(16, 270, 13), // "ComPortSearch"
QT_MOC_LITERAL(17, 284, 14), // "QList<QString>"
QT_MOC_LITERAL(18, 299, 16), // "ComPortFoundPort"
QT_MOC_LITERAL(19, 316, 14), // "ComPortConnect"
QT_MOC_LITERAL(20, 331, 12), // "ComPortClose"
QT_MOC_LITERAL(21, 344, 12), // "ComPortWrite"
QT_MOC_LITERAL(22, 357, 8), // "uint8_t*"
QT_MOC_LITERAL(23, 366, 4), // "data"
QT_MOC_LITERAL(24, 371, 8), // "uint32_t"
QT_MOC_LITERAL(25, 380, 7), // "cntdata"
QT_MOC_LITERAL(26, 388, 14), // "onCheckConnect"
QT_MOC_LITERAL(27, 403, 12) // "onPortClosed"

    },
    "MyThread_1\0PaintGraph_signal\0\0"
    "PaintGraph2_signal\0ComportDataUpdate_signal\0"
    "ComportConnect_signal\0ComportClose_signal\0"
    "ComPortConnect_signal\0ComportRead_signal\0"
    "ComPortWrite_signal\0back\0ComPortReadData\0"
    "QVector<uint8_t>\0ComPort_handleError\0"
    "QSerialPort::SerialPortError\0error\0"
    "ComPortSearch\0QList<QString>\0"
    "ComPortFoundPort\0ComPortConnect\0"
    "ComPortClose\0ComPortWrite\0uint8_t*\0"
    "data\0uint32_t\0cntdata\0onCheckConnect\0"
    "onPortClosed"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MyThread_1[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       8,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   99,    2, 0x06 /* Public */,
       3,    0,  100,    2, 0x06 /* Public */,
       4,    0,  101,    2, 0x06 /* Public */,
       5,    0,  102,    2, 0x06 /* Public */,
       6,    0,  103,    2, 0x06 /* Public */,
       7,    0,  104,    2, 0x06 /* Public */,
       8,    0,  105,    2, 0x06 /* Public */,
       9,    1,  106,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    0,  109,    2, 0x0a /* Public */,
      13,    1,  110,    2, 0x0a /* Public */,
      16,    0,  113,    2, 0x0a /* Public */,
      18,    0,  114,    2, 0x0a /* Public */,
      19,    0,  115,    2, 0x0a /* Public */,
      20,    0,  116,    2, 0x0a /* Public */,
      21,    2,  117,    2, 0x0a /* Public */,
      26,    0,  122,    2, 0x0a /* Public */,
      27,    0,  123,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   10,

 // slots: parameters
    0x80000000 | 12,
    QMetaType::Void, 0x80000000 | 14,   15,
    0x80000000 | 17,
    QMetaType::QString,
    QMetaType::Bool,
    QMetaType::Void,
    QMetaType::QString, 0x80000000 | 22, 0x80000000 | 24,   23,   25,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MyThread_1::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MyThread_1 *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->PaintGraph_signal(); break;
        case 1: _t->PaintGraph2_signal(); break;
        case 2: _t->ComportDataUpdate_signal(); break;
        case 3: _t->ComportConnect_signal(); break;
        case 4: _t->ComportClose_signal(); break;
        case 5: _t->ComPortConnect_signal(); break;
        case 6: _t->ComportRead_signal(); break;
        case 7: _t->ComPortWrite_signal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: { QVector<uint8_t> _r = _t->ComPortReadData();
            if (_a[0]) *reinterpret_cast< QVector<uint8_t>*>(_a[0]) = std::move(_r); }  break;
        case 9: _t->ComPort_handleError((*reinterpret_cast< QSerialPort::SerialPortError(*)>(_a[1]))); break;
        case 10: { QList<QString> _r = _t->ComPortSearch();
            if (_a[0]) *reinterpret_cast< QList<QString>*>(_a[0]) = std::move(_r); }  break;
        case 11: { QString _r = _t->ComPortFoundPort();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 12: { bool _r = _t->ComPortConnect();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 13: _t->ComPortClose(); break;
        case 14: { QString _r = _t->ComPortWrite((*reinterpret_cast< uint8_t*(*)>(_a[1])),(*reinterpret_cast< uint32_t(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 15: _t->onCheckConnect(); break;
        case 16: _t->onPortClosed(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::PaintGraph_signal)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::PaintGraph2_signal)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::ComportDataUpdate_signal)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::ComportConnect_signal)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::ComportClose_signal)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::ComPortConnect_signal)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::ComportRead_signal)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (MyThread_1::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MyThread_1::ComPortWrite_signal)) {
                *result = 7;
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
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}

// SIGNAL 0
void MyThread_1::PaintGraph_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void MyThread_1::PaintGraph2_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void MyThread_1::ComportDataUpdate_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void MyThread_1::ComportConnect_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void MyThread_1::ComportClose_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void MyThread_1::ComPortConnect_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void MyThread_1::ComportRead_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 6, nullptr);
}

// SIGNAL 7
void MyThread_1::ComPortWrite_signal(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}
struct qt_meta_stringdata_MyThread_2_t {
    QByteArrayData data[1];
    char stringdata0[11];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MyThread_2_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MyThread_2_t qt_meta_stringdata_MyThread_2 = {
    {
QT_MOC_LITERAL(0, 0, 10) // "MyThread_2"

    },
    "MyThread_2"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MyThread_2[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void MyThread_2::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    (void)_o;
    (void)_id;
    (void)_c;
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject MyThread_2::staticMetaObject = { {
    QMetaObject::SuperData::link<QThread::staticMetaObject>(),
    qt_meta_stringdata_MyThread_2.data,
    qt_meta_data_MyThread_2,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MyThread_2::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MyThread_2::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MyThread_2.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int MyThread_2::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
