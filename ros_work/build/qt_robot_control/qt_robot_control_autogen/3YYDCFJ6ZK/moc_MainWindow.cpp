/****************************************************************************
** Meta object code from reading C++ file 'MainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/qt_robot_control/include/qt_robot_control/MainWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[18];
    char stringdata0[304];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 18), // "onStartBaseClicked"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 19), // "onStartLidarClicked"
QT_MOC_LITERAL(4, 51, 20), // "onStartCameraClicked"
QT_MOC_LITERAL(5, 72, 20), // "onStartGeminiClicked"
QT_MOC_LITERAL(6, 93, 24), // "onStartAllSensorsClicked"
QT_MOC_LITERAL(7, 118, 29), // "onStartKeyboardControlClicked"
QT_MOC_LITERAL(8, 148, 25), // "onStartGpioControlClicked"
QT_MOC_LITERAL(9, 174, 17), // "onRgbTopicChanged"
QT_MOC_LITERAL(10, 192, 5), // "index"
QT_MOC_LITERAL(11, 198, 19), // "onDepthTopicChanged"
QT_MOC_LITERAL(12, 218, 13), // "onUpdateTimer"
QT_MOC_LITERAL(13, 232, 17), // "onProcessFinished"
QT_MOC_LITERAL(14, 250, 8), // "exitCode"
QT_MOC_LITERAL(15, 259, 20), // "QProcess::ExitStatus"
QT_MOC_LITERAL(16, 280, 10), // "exitStatus"
QT_MOC_LITERAL(17, 291, 12) // "process_name"

    },
    "MainWindow\0onStartBaseClicked\0\0"
    "onStartLidarClicked\0onStartCameraClicked\0"
    "onStartGeminiClicked\0onStartAllSensorsClicked\0"
    "onStartKeyboardControlClicked\0"
    "onStartGpioControlClicked\0onRgbTopicChanged\0"
    "index\0onDepthTopicChanged\0onUpdateTimer\0"
    "onProcessFinished\0exitCode\0"
    "QProcess::ExitStatus\0exitStatus\0"
    "process_name"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x08 /* Private */,
       3,    0,   70,    2, 0x08 /* Private */,
       4,    0,   71,    2, 0x08 /* Private */,
       5,    0,   72,    2, 0x08 /* Private */,
       6,    0,   73,    2, 0x08 /* Private */,
       7,    0,   74,    2, 0x08 /* Private */,
       8,    0,   75,    2, 0x08 /* Private */,
       9,    1,   76,    2, 0x08 /* Private */,
      11,    1,   79,    2, 0x08 /* Private */,
      12,    0,   82,    2, 0x08 /* Private */,
      13,    3,   83,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 15, QMetaType::QString,   14,   16,   17,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onStartBaseClicked(); break;
        case 1: _t->onStartLidarClicked(); break;
        case 2: _t->onStartCameraClicked(); break;
        case 3: _t->onStartGeminiClicked(); break;
        case 4: _t->onStartAllSensorsClicked(); break;
        case 5: _t->onStartKeyboardControlClicked(); break;
        case 6: _t->onStartGpioControlClicked(); break;
        case 7: _t->onRgbTopicChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->onDepthTopicChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->onUpdateTimer(); break;
        case 10: _t->onProcessFinished((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QProcess::ExitStatus(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3]))); break;
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
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
