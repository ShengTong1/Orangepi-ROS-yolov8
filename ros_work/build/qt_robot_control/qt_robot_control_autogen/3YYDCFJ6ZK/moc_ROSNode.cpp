/****************************************************************************
** Meta object code from reading C++ file 'ROSNode.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/qt_robot_control/include/qt_robot_control/ROSNode.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ROSNode.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ROSNode_t {
    QByteArrayData data[31];
    char stringdata0[217];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROSNode_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROSNode_t qt_meta_stringdata_ROSNode = {
    {
QT_MOC_LITERAL(0, 0, 7), // "ROSNode"
QT_MOC_LITERAL(1, 8, 21), // "batteryVoltageUpdated"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 7), // "voltage"
QT_MOC_LITERAL(4, 39, 15), // "odometryUpdated"
QT_MOC_LITERAL(5, 55, 2), // "vx"
QT_MOC_LITERAL(6, 58, 2), // "vy"
QT_MOC_LITERAL(7, 61, 2), // "vz"
QT_MOC_LITERAL(8, 64, 1), // "x"
QT_MOC_LITERAL(9, 66, 1), // "y"
QT_MOC_LITERAL(10, 68, 1), // "z"
QT_MOC_LITERAL(11, 70, 10), // "imuUpdated"
QT_MOC_LITERAL(12, 81, 2), // "ax"
QT_MOC_LITERAL(13, 84, 2), // "ay"
QT_MOC_LITERAL(14, 87, 2), // "az"
QT_MOC_LITERAL(15, 90, 2), // "wx"
QT_MOC_LITERAL(16, 93, 2), // "wy"
QT_MOC_LITERAL(17, 96, 2), // "wz"
QT_MOC_LITERAL(18, 99, 2), // "qx"
QT_MOC_LITERAL(19, 102, 2), // "qy"
QT_MOC_LITERAL(20, 105, 2), // "qz"
QT_MOC_LITERAL(21, 108, 2), // "qw"
QT_MOC_LITERAL(22, 111, 17), // "topicsListUpdated"
QT_MOC_LITERAL(23, 129, 6), // "topics"
QT_MOC_LITERAL(24, 136, 16), // "nodesListUpdated"
QT_MOC_LITERAL(25, 153, 5), // "nodes"
QT_MOC_LITERAL(26, 159, 10), // "logMessage"
QT_MOC_LITERAL(27, 170, 5), // "level"
QT_MOC_LITERAL(28, 176, 7), // "message"
QT_MOC_LITERAL(29, 184, 16), // "updateTopicsList"
QT_MOC_LITERAL(30, 201, 15) // "updateNodesList"

    },
    "ROSNode\0batteryVoltageUpdated\0\0voltage\0"
    "odometryUpdated\0vx\0vy\0vz\0x\0y\0z\0"
    "imuUpdated\0ax\0ay\0az\0wx\0wy\0wz\0qx\0qy\0"
    "qz\0qw\0topicsListUpdated\0topics\0"
    "nodesListUpdated\0nodes\0logMessage\0"
    "level\0message\0updateTopicsList\0"
    "updateNodesList"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROSNode[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,
       4,    6,   57,    2, 0x06 /* Public */,
      11,   10,   70,    2, 0x06 /* Public */,
      22,    1,   91,    2, 0x06 /* Public */,
      24,    1,   94,    2, 0x06 /* Public */,
      26,    2,   97,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      29,    0,  102,    2, 0x08 /* Private */,
      30,    0,  103,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,    8,    9,   10,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   12,   13,   14,   15,   16,   17,   18,   19,   20,   21,
    QMetaType::Void, QMetaType::QStringList,   23,
    QMetaType::Void, QMetaType::QStringList,   25,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   27,   28,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ROSNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ROSNode *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->batteryVoltageUpdated((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: _t->odometryUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6]))); break;
        case 2: _t->imuUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6])),(*reinterpret_cast< double(*)>(_a[7])),(*reinterpret_cast< double(*)>(_a[8])),(*reinterpret_cast< double(*)>(_a[9])),(*reinterpret_cast< double(*)>(_a[10]))); break;
        case 3: _t->topicsListUpdated((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 4: _t->nodesListUpdated((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 5: _t->logMessage((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 6: _t->updateTopicsList(); break;
        case 7: _t->updateNodesList(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ROSNode::*)(float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROSNode::batteryVoltageUpdated)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ROSNode::*)(double , double , double , double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROSNode::odometryUpdated)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ROSNode::*)(double , double , double , double , double , double , double , double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROSNode::imuUpdated)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (ROSNode::*)(const QStringList & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROSNode::topicsListUpdated)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (ROSNode::*)(const QStringList & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROSNode::nodesListUpdated)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (ROSNode::*)(const QString & , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROSNode::logMessage)) {
                *result = 5;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ROSNode::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ROSNode.data,
    qt_meta_data_ROSNode,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ROSNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROSNode::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ROSNode.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "rclcpp::Node"))
        return static_cast< rclcpp::Node*>(this);
    return QObject::qt_metacast(_clname);
}

int ROSNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void ROSNode::batteryVoltageUpdated(float _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ROSNode::odometryUpdated(double _t1, double _t2, double _t3, double _t4, double _t5, double _t6)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t5))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t6))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ROSNode::imuUpdated(double _t1, double _t2, double _t3, double _t4, double _t5, double _t6, double _t7, double _t8, double _t9, double _t10)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t5))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t6))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t7))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t8))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t9))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t10))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ROSNode::topicsListUpdated(const QStringList & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void ROSNode::nodesListUpdated(const QStringList & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void ROSNode::logMessage(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
