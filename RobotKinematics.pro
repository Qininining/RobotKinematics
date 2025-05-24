QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Kinematics/KinematicsSolver.cpp \
    Kinematics/PoEKinematics.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    Kinematics/KinematicsSolver.h \
    Kinematics/PoEKinematics.h \
    mainwindow.h

FORMS += \
    mainwindow.ui


INCLUDEPATH += \
    $$PWD/Kinematics \
    $$PWD/3rdparty/NanoDrive2.8.12/04_SDK/include \
    $$PWD/3rdparty/opencv/include \



unix:!macx|win32: LIBS += -L$$PWD/3rdparty/NanoDrive2.8.12/04_SDK/lib64/ -lNTControl
INCLUDEPATH += $$PWD/3rdparty/NanoDrive2.8.12/04_SDK/lib64
DEPENDPATH += $$PWD/3rdparty/NanoDrive2.8.12/04_SDK/lib64

# OpenCV
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/3rdparty/opencv/x64/vc16/lib/ -lopencv_world4100
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/3rdparty/opencv/x64/vc16/lib/ -lopencv_world4100d
INCLUDEPATH += $$PWD/3rdparty/opencv/include
DEPENDPATH += $$PWD/3rdparty/opencv/include


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
