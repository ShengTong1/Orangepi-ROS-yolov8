/****************************************************************************
** Meta object code from reading C++ file 'SystemMonitorWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/qt_robot_control/include/qt_robot_control/SystemMonitorWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SystemMonitorWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SystemMonitorWidget_t {
    QByteArrayData data[29];
    char stringdata0[200];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SystemMonitorWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SystemMonitorWidget_t qt_meta_stringdata_SystemMonitorWidget = {
    {
QT_MOC_LITERAL(0, 0, 19), // "SystemMonitorWidget"
QT_MOC_LITERAL(1, 20, 20), // "updateBatteryVoltage"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 7), // "voltage"
QT_MOC_LITERAL(4, 50, 14), // "updateOdometry"
QT_MOC_LITERAL(5, 65, 2), // "vx"
QT_MOC_LITERAL(6, 68, 2), // "vy"
QT_MOC_LITERAL(7, 71, 2), // "vz"
QT_MOC_LITERAL(8, 74, 1), // "x"
QT_MOC_LITERAL(9, 76, 1), // "y"
QT_MOC_LITERAL(10, 78, 3), // "yaw"
QT_MOC_LITERAL(11, 82, 13), // "updateImuData"
QT_MOC_LITERAL(12, 96, 2), // "ax"
QT_MOC_LITERAL(13, 99, 2), // "ay"
QT_MOC_LITERAL(14, 102, 2), // "az"
QT_MOC_LITERAL(15, 105, 2), // "wx"
QT_MOC_LITERAL(16, 108, 2), // "wy"
QT_MOC_LITERAL(17, 111, 2), // "wz"
QT_MOC_LITERAL(18, 114, 2), // "qx"
QT_MOC_LITERAL(19, 117, 2), // "qy"
QT_MOC_LITERAL(20, 120, 2), // "qz"
QT_MOC_LITERAL(21, 123, 2), // "qw"
QT_MOC_LITERAL(22, 126, 16), // "updateTopicsList"
QT_MOC_LITERAL(23, 143, 6), // "topics"
QT_MOC_LITERAL(24, 150, 15), // "updateNodesList"
QT_MOC_LITERAL(25, 166, 5), // "nodes"
QT_MOC_LITERAL(26, 172, 13), // "addLogMessage"
QT_MOC_LITERAL(27, 186, 5), // "level"
QT_MOC_LITERAL(28, 192, 7) // "message"

    },
    "SystemMonitorWidget\0updateBatteryVoltage\0"
    "\0voltage\0updateOdometry\0vx\0vy\0vz\0x\0y\0"
    "yaw\0updateImuData\0ax\0ay\0az\0wx\0wy\0wz\0"
    "qx\0qy\0qz\0qw\0updateTopicsList\0topics\0"
    "updateNodesList\0nodes\0addLogMessage\0"
    "level\0message"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SystemMonitorWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x0a /* Public */,
       4,    6,   47,    2, 0x0a /* Public */,
      11,   10,   60,    2, 0x0a /* Public */,
      22,    1,   81,    2, 0x0a /* Public */,
      24,    1,   84,    2, 0x0a /* Public */,
      26,    2,   87,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,    8,    9,   10,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   12,   13,   14,   15,   16,   17,   18,   19,   20,   21,
    QMetaType::Void, QMetaType::QStringList,   23,
    QMetaType::Void, QMetaType::QStringList,   25,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   27,   28,

       0        // eod
};

void SystemMonitorWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SystemMonitorWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateBatteryVoltage((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: _t->updateOdometry((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6]))); break;
        case 2: _t->updateImuData((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6])),(*reinterpret_cast< double(*)>(_a[7])),(*reinterpret_cast< double(*)>(_a[8])),(*reinterpret_cast< double(*)>(_a[9])),(*reinterpret_cast< double(*)>(_a[10]))); break;
        case 3: _t->updateTopicsList((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 4: _t->updateNodesList((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 5: _t->addLogMessage((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SystemMonitorWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_SystemMonitorWidget.data,
    qt_meta_data_SystemMonitorWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SystemMonitorWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SystemMonitorWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SystemMonitorWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int SystemMonitorWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
