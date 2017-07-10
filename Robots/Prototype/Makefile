KINEMATICS_NAMESPACE = kinematics
KINEMATICS_URDF = Prototype
KINEMATICS_IKFASTDEPTH = 1
KINEMATICS_IKFASTLIMITSHEADER = $(KINEMATICS_NAMESPACE)_ikfast.limits.hpp

BOARD_TAG = pro5v328
#BOARD_TAG = pro
#BOARD_SUB = 16MHzatmega328

ARDUINO_LIBS = Servo SoftwareSerial stlport stlport_missing Eigen31 boost_1_51_0 ToyWalker

include ../common.mk

include $(ARDMK_DIR)/Arduino.mk

kinematics: kinematics_ikfast.back_foot.translation3d.gcov.cpp kinematics_ikfast.left_foot.translation3d.gcov.cpp kinematics_ikfast.right_foot.translation3d.gcov.cpp
