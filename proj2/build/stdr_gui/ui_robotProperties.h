/********************************************************************************
** Form generated from reading UI file 'robotProperties.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROBOTPROPERTIES_H
#define UI_ROBOTPROPERTIES_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RobotProperties
{
public:
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QLineEdit *robotRadius;
    QLabel *label_3;
    QLabel *label_2;
    QLineEdit *robotOrientation;
    QHBoxLayout *horizontalLayout;
    QPushButton *updateButton;
    QPushButton *refresh_robot;

    void setupUi(QWidget *RobotProperties)
    {
        if (RobotProperties->objectName().isEmpty())
            RobotProperties->setObjectName(QString::fromUtf8("RobotProperties"));
        RobotProperties->resize(348, 124);
        gridLayout_2 = new QGridLayout(RobotProperties);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        robotRadius = new QLineEdit(RobotProperties);
        robotRadius->setObjectName(QString::fromUtf8("robotRadius"));

        gridLayout->addWidget(robotRadius, 0, 1, 1, 1);

        label_3 = new QLabel(RobotProperties);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 1, 0, 1, 1);

        label_2 = new QLabel(RobotProperties);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        robotOrientation = new QLineEdit(RobotProperties);
        robotOrientation->setObjectName(QString::fromUtf8("robotOrientation"));

        gridLayout->addWidget(robotOrientation, 1, 1, 1, 1);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        updateButton = new QPushButton(RobotProperties);
        updateButton->setObjectName(QString::fromUtf8("updateButton"));
        updateButton->setMaximumSize(QSize(100, 16777215));

        horizontalLayout->addWidget(updateButton);

        refresh_robot = new QPushButton(RobotProperties);
        refresh_robot->setObjectName(QString::fromUtf8("refresh_robot"));
        refresh_robot->setMaximumSize(QSize(100, 16777215));

        horizontalLayout->addWidget(refresh_robot);


        gridLayout_2->addLayout(horizontalLayout, 2, 0, 1, 1);

        QWidget::setTabOrder(robotRadius, robotOrientation);

        retranslateUi(RobotProperties);

        QMetaObject::connectSlotsByName(RobotProperties);
    } // setupUi

    void retranslateUi(QWidget *RobotProperties)
    {
        RobotProperties->setWindowTitle(QApplication::translate("RobotProperties", "Robot", nullptr));
        robotRadius->setText(QApplication::translate("RobotProperties", "0.15", nullptr));
        label_3->setText(QApplication::translate("RobotProperties", "Robot orientation", nullptr));
        label_2->setText(QApplication::translate("RobotProperties", "Robot radius", nullptr));
        robotOrientation->setText(QApplication::translate("RobotProperties", "0", nullptr));
        updateButton->setText(QApplication::translate("RobotProperties", "Update", nullptr));
        refresh_robot->setText(QApplication::translate("RobotProperties", "Refresh", nullptr));
    } // retranslateUi

};

namespace Ui {
    class RobotProperties: public Ui_RobotProperties {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROBOTPROPERTIES_H
