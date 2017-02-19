/****************************************************************************
** Meta object code from reading C++ file 'urdfdialog.h'
**
** Created: Tue Jan 14 12:06:55 2014
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "urdfdialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'urdfdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CUrdfDialog[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x08,
      35,   12,   12,   12, 0x08,
      71,   12,   12,   12, 0x08,
      99,   12,   12,   12, 0x08,
     130,   12,   12,   12, 0x08,
     164,   12,   12,   12, 0x08,
     197,   12,   12,   12, 0x08,
     224,   12,   12,   12, 0x08,
     255,   12,   12,   12, 0x08,
     285,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_CUrdfDialog[] = {
    "CUrdfDialog\0\0on_qqImport_clicked()\0"
    "on_qqCollisionLinksHidden_clicked()\0"
    "on_qqJointsHidden_clicked()\0"
    "on_qqConvexDecompose_clicked()\0"
    "on_qqConvexDecomposeDlg_clicked()\0"
    "on_qqCreateVisualLinks_clicked()\0"
    "on_qqCenterModel_clicked()\0"
    "on_qqModelDefinition_clicked()\0"
    "on_qqAlternateMasks_clicked()\0"
    "on_qqPositionCtrl_clicked()\0"
};

const QMetaObject CUrdfDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_CUrdfDialog,
      qt_meta_data_CUrdfDialog, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CUrdfDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CUrdfDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CUrdfDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CUrdfDialog))
        return static_cast<void*>(const_cast< CUrdfDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int CUrdfDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: on_qqImport_clicked(); break;
        case 1: on_qqCollisionLinksHidden_clicked(); break;
        case 2: on_qqJointsHidden_clicked(); break;
        case 3: on_qqConvexDecompose_clicked(); break;
        case 4: on_qqConvexDecomposeDlg_clicked(); break;
        case 5: on_qqCreateVisualLinks_clicked(); break;
        case 6: on_qqCenterModel_clicked(); break;
        case 7: on_qqModelDefinition_clicked(); break;
        case 8: on_qqAlternateMasks_clicked(); break;
        case 9: on_qqPositionCtrl_clicked(); break;
        default: ;
        }
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
