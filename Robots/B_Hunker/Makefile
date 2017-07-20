KINEMATICS_NAMESPACE = kinematics
KINEMATICS_URDF = Hunker
KINEMATICS_IKFASTDEPTH = 4
KINEMATICS_IKFASTLIMITSHEADER = $(KINEMATICS_NAMESPACE)_ikfast.limits.hpp
IKFAST_REAL = float

BOARD = opencm904
BOOTLOADER = robotis

SRC_FILES = $(wildcard *.cpp)
USER_MODULES += $(CURDIR)/../../Libraries/ToyWalker
#USER_MODULES += $(CURDIR)/../../Libraries/orocos_kdl
USER_INCLUDES += $(CURDIR)/../../Libraries/Eigen31

include ../common.mk

include $(RHOBANMAPLE_DIR)/skeleton/Makefile

kinematics: kinematics_ikfast.backLeftFoot.translation3d.gcov.cpp kinematics_ikfast.backRightFoot.translation3d.gcov.cpp kinematics_ikfast.frontLeftFoot.translation3d.gcov.cpp kinematics_ikfast.frontRightFoot.translation3d.gcov.cpp
