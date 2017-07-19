KINEMATICS_NAMESPACE = kinematics
KINEMATICS_URDF = Hunker
KINEMATICS_IKFASTDEPTH = 0

BOARD = opencm904
BOOTLOADER = robotis

SRC_FILES = $(wildcard *.cpp)
USER_MODULES += $(CURDIR)/../../Libraries/ToyWalker
#USER_MODULES += $(CURDIR)/../../Libraries/orocos_kdl
USER_INCLUDES += $(CURDIR)/../../Libraries/Eigen31

include ../common.mk

include $(RHOBANMAPLE_DIR)/skeleton/Makefile

kinematics: kinematics_ikfast.backLeftFoot.translation3d.full.cpp kinematics_ikfast.backRightFoot.translation3d.full.cpp kinematics_ikfast.frontLeftFoot.translation3d.full.cpp kinematics_ikfast.frontRightFoot.translation3d.full.cpp
