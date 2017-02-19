/********************************************************************************
** Form generated from reading UI file 'urdfdialog.ui'
**
** Created: Tue Jan 14 12:06:46 2014
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_URDFDIALOG_H
#define UI_URDFDIALOG_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_CUrdfDialog
{
public:
    QPushButton *qqImport;
    QCheckBox *qqCenterModel;
    QCheckBox *qqAlternateMasks;
    QCheckBox *qqConvexDecomposeDlg;
    QCheckBox *qqConvexDecompose;
    QCheckBox *qqCreateVisualLinks;
    QCheckBox *qqCollisionLinksHidden;
    QCheckBox *qqModelDefinition;
    QCheckBox *qqJointsHidden;
    QCheckBox *qqPositionCtrl;

    void setupUi(QDialog *CUrdfDialog)
    {
        if (CUrdfDialog->objectName().isEmpty())
            CUrdfDialog->setObjectName(QString::fromUtf8("CUrdfDialog"));
        CUrdfDialog->resize(311, 234);
        CUrdfDialog->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
        qqImport = new QPushButton(CUrdfDialog);
        qqImport->setObjectName(QString::fromUtf8("qqImport"));
        qqImport->setGeometry(QRect(220, 200, 75, 23));
        qqCenterModel = new QCheckBox(CUrdfDialog);
        qqCenterModel->setObjectName(QString::fromUtf8("qqCenterModel"));
        qqCenterModel->setGeometry(QRect(10, 110, 271, 17));
        qqAlternateMasks = new QCheckBox(CUrdfDialog);
        qqAlternateMasks->setObjectName(QString::fromUtf8("qqAlternateMasks"));
        qqAlternateMasks->setGeometry(QRect(10, 150, 271, 17));
        qqConvexDecomposeDlg = new QCheckBox(CUrdfDialog);
        qqConvexDecomposeDlg->setObjectName(QString::fromUtf8("qqConvexDecomposeDlg"));
        qqConvexDecomposeDlg->setGeometry(QRect(10, 70, 271, 17));
        qqConvexDecompose = new QCheckBox(CUrdfDialog);
        qqConvexDecompose->setObjectName(QString::fromUtf8("qqConvexDecompose"));
        qqConvexDecompose->setGeometry(QRect(10, 50, 271, 17));
        qqCreateVisualLinks = new QCheckBox(CUrdfDialog);
        qqCreateVisualLinks->setObjectName(QString::fromUtf8("qqCreateVisualLinks"));
        qqCreateVisualLinks->setGeometry(QRect(10, 90, 271, 17));
        qqCollisionLinksHidden = new QCheckBox(CUrdfDialog);
        qqCollisionLinksHidden->setObjectName(QString::fromUtf8("qqCollisionLinksHidden"));
        qqCollisionLinksHidden->setGeometry(QRect(10, 10, 271, 17));
        qqModelDefinition = new QCheckBox(CUrdfDialog);
        qqModelDefinition->setObjectName(QString::fromUtf8("qqModelDefinition"));
        qqModelDefinition->setGeometry(QRect(10, 130, 271, 17));
        qqJointsHidden = new QCheckBox(CUrdfDialog);
        qqJointsHidden->setObjectName(QString::fromUtf8("qqJointsHidden"));
        qqJointsHidden->setGeometry(QRect(10, 30, 271, 17));
        qqPositionCtrl = new QCheckBox(CUrdfDialog);
        qqPositionCtrl->setObjectName(QString::fromUtf8("qqPositionCtrl"));
        qqPositionCtrl->setGeometry(QRect(10, 170, 291, 17));
        QWidget::setTabOrder(qqCollisionLinksHidden, qqJointsHidden);
        QWidget::setTabOrder(qqJointsHidden, qqConvexDecompose);
        QWidget::setTabOrder(qqConvexDecompose, qqConvexDecomposeDlg);
        QWidget::setTabOrder(qqConvexDecomposeDlg, qqCreateVisualLinks);
        QWidget::setTabOrder(qqCreateVisualLinks, qqCenterModel);
        QWidget::setTabOrder(qqCenterModel, qqModelDefinition);
        QWidget::setTabOrder(qqModelDefinition, qqAlternateMasks);
        QWidget::setTabOrder(qqAlternateMasks, qqImport);

        retranslateUi(CUrdfDialog);

        QMetaObject::connectSlotsByName(CUrdfDialog);
    } // setupUi

    void retranslateUi(QDialog *CUrdfDialog)
    {
        CUrdfDialog->setWindowTitle(QApplication::translate("CUrdfDialog", "URDF Import", 0, QApplication::UnicodeUTF8));
        qqImport->setText(QApplication::translate("CUrdfDialog", "Import", 0, QApplication::UnicodeUTF8));
        qqCenterModel->setText(QApplication::translate("CUrdfDialog", "Center model above ground", 0, QApplication::UnicodeUTF8));
        qqAlternateMasks->setText(QApplication::translate("CUrdfDialog", "Alternate local respondable masks", 0, QApplication::UnicodeUTF8));
        qqConvexDecomposeDlg->setText(QApplication::translate("CUrdfDialog", "Show convex decomposition dialog", 0, QApplication::UnicodeUTF8));
        qqConvexDecompose->setText(QApplication::translate("CUrdfDialog", "Convex decompose non-convex collision links", 0, QApplication::UnicodeUTF8));
        qqCreateVisualLinks->setText(QApplication::translate("CUrdfDialog", "Create visual links if none", 0, QApplication::UnicodeUTF8));
        qqCollisionLinksHidden->setText(QApplication::translate("CUrdfDialog", "Assign collision links to layer 9 ", 0, QApplication::UnicodeUTF8));
        qqModelDefinition->setText(QApplication::translate("CUrdfDialog", "Prepare model definition if feasable", 0, QApplication::UnicodeUTF8));
        qqJointsHidden->setText(QApplication::translate("CUrdfDialog", "Assign joints to layer 10", 0, QApplication::UnicodeUTF8));
        qqPositionCtrl->setText(QApplication::translate("CUrdfDialog", "Enable position control for revolute and prismatic joints", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CUrdfDialog: public Ui_CUrdfDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_URDFDIALOG_H
