QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

INCLUDEPATH += /opt/xenomai-2.6.4/include
LIBS += -lpthread -L/opt/xenomai-2.6.4/lib -lxenomai -lnative
#DEFINES += __LINUX_ARM_ARCH__

target.path = /mnt/mtd5/daincube/KETI
INSTALLS += target

myconfig.path = /mnt/mtd5/daincube/KETI
myconfig.files = $$PWD/config.json
INSTALLS += myconfig

TARGET = DARController

SOURCES += main.cpp \
    ControlMain/controlfunc.cpp \
    DataControl/diningwaypoints.cpp \
    RobotArm/robotarm.cpp \
    RobotArm/numerical.cpp \
    TcpServer/tcpserver.cpp \
    ControlMain/controlmain.cpp \
    DataControl/datacontrol.cpp \
    DataControl/fileio.cpp \
    EtherCAT/custom_slave_information.cpp \
    EtherCAT/userinterface.cpp \
    EtherCAT/ecatmaster.cpp \
    EtherCAT/ecatslave.cpp \
    EtherCAT/daininterface.cpp \

HEADERS += \
    RobotArm/robotarm.h \
    RobotArm/numerical.h \
    TcpServer/tcpserver.h \
    ControlMain/controlmain.h \
    DataControl/datacontrol.h \
    DataControl/fileio.h \
    EtherCAT/globalstruct.h \
    EtherCAT/esi.h \
    EtherCAT/userinterface.h \
    EtherCAT/ecatmaster.h \
    EtherCAT/ecatslave.h \
    EtherCAT/daininterface.h
