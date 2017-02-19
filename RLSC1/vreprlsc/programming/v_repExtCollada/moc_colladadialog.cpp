/****************************************************************************
** Meta object code from reading C++ file 'colladadialog.h'
**
** Created: Tue Jan 14 12:07:39 2014
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "colladadialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'colladadialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CColladaDialog[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x08,
      43,   15,   15,   15, 0x08,
      75,   15,   15,   15, 0x08,
     118,   15,   15,   15, 0x08,
     140,   15,   15,   15, 0x08,
     171,   15,   15,   15, 0x08,
     203,   15,   15,   15, 0x08,
     246,   15,   15,   15, 0x08,
     268,   15,   15,   15, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_CColladaDialog[] = {
    "CColladaDialog\0\0on_qqMergeImport_clicked()\0"
    "on_qqImportMeshesOnly_clicked()\0"
    "on_qqImportScalingFactor_editingFinished()\0"
    "on_qqImport_clicked()\0"
    "on_qqExportAnimation_clicked()\0"
    "on_qqExportShapesOnly_clicked()\0"
    "on_qqExportScalingFactor_editingFinished()\0"
    "on_qqExport_clicked()\0"
    "on_qqImportUngroup_clicked()\0"
};

const QMetaObject CColladaDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_CColladaDialog,
      qt_meta_data_CColladaDialog, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CColladaDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CColladaDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CColladaDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CColladaDialog))
        return static_cast<void*>(const_cast< CColladaDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int CColladaDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: on_qqMergeImport_clicked(); break;
        case 1: on_qqImportMeshesOnly_clicked(); break;
        case 2: on_qqImportScalingFactor_editingFinished(); break;
        case 3: on_qqImport_clicked(); break;
        case 4: on_qqExportAnimation_clicked(); break;
        case 5: on_qqExportShapesOnly_clicked(); break;
        case 6: on_qqExportScalingFactor_editingFinished(); break;
        case 7: on_qqExport_clicked(); break;
        case 8: on_qqImportUngroup_clicked(); break;
        default: ;
        }
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
