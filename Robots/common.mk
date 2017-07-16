ARDMK_DIR = /usr/share/arduino
#ARDMK_DIR = /home/user/src/Arduino-Makefile
ARDUINO_DIR = /opt/arduino-1.8.2
RHOBANMAPLE_DIR = /home/user/src/Maple

USER_LIB_PATH = ../../Libraries
BOARDS_TXT = ../boards.txt
override LIB_MAPLE_HOME = $(RHOBANMAPLE_DIR)/LibMaple
override LIB_ROBOT_HOME = $(RHOBANMAPLE_DIR)/LibRhoban
override PATH := $(PATH):$(RHOBANMAPLE_DIR)/BinutilsArm/bin
override SRC_FILES := $(SRC_FILES)

upload:

.PHONY: preview
preview: $(KINEMATICS_URDF).urdf
	check_urdf $<
	roslaunch urdf_tutorial display.launch model:=$<

$(KINEMATICS_NAMESPACE)_kdl.gen.cpp: $(KINEMATICS_URDF).urdf
	../../Tools/urdf_to_kdl/build/devel/lib/urdf_to_kdl/urdf_to_kdl "$(KINEMATICS_NAMESPACE)_kdl" get $< > $@

%.urdf: %.urdf.xacro
	rosrun xacro xacro --inorder $< > $@

%.dae.links: %.dae
	openrave-robot.py $< --info links

%.dae: %.urdf
	rosrun collada_urdf urdf_to_collada $< $@


#Rules for generating files like `kinematics_ikfast.back_foot.translation3d.full.cpp`
.PRECIOUS: $(KINEMATICS_NAMESPACE)_ikfast.%.hpp $(KINEMATICS_NAMESPACE)_ikfast.%.cpp
$(KINEMATICS_NAMESPACE)_ikfast.%.cpp: $(KINEMATICS_NAMESPACE)_ikfast.%.ikfast.styled.cpp
	sed '1s/^/#include <ToyWalker.h>\n#pragma GCC diagnostic ignored "-Wunused-variable"\n#pragma GCC diagnostic ignored "-Wreturn-type"\n#define IKFAST_NO_MAIN\n#define IKFAST_REAL double\n#define IKFAST_NAMESPACE $(KINEMATICS_NAMESPACE)_ikfast_$(word 2,$(subst ., ,$@))_$(word 3,$(subst ., ,$@))\n/; s/=IKPowWithIntegerCheck(/=IKPowWithIntegerCheck<IkReal>(/g; s/std::vector<\(.*\)> \(.*\)(\(.*\));/\1 \2[\3];/' $< > $@

ifdef KINEMATICS_IKFASTLIMITSHEADER
	KINEMATICS_IKFASTLIMITSHEADERINCLUDE = #include "$(KINEMATICS_IKFASTLIMITSHEADER)"\n\n
endif

$(KINEMATICS_NAMESPACE)_ikfast.%.hpp:
	printf '#define IKFAST_NO_MAIN\n#define IKFAST_REAL double\n#define IKFAST_NAMESPACE $(KINEMATICS_NAMESPACE)_ikfast_$(word 2,$(subst ., ,$@))_$(word 3,$(subst ., ,$@))\n#define IKFAST_HAS_LIBRARY\n\n$(KINEMATICS_IKFASTLIMITSHEADERINCLUDE)#include "ikfast.h"\n\n#undef IKFAST_NO_MAIN\n#undef IKFAST_REAL\n#undef IKFAST_NAMESPACE\n#undef IKFAST_HAS_LIBRARY\n' > $@

%.translation3d.gcov.ikfast.styled.cpp: %.translation3d.full.ikfast.styled.cpp $(KINEMATICS_IKFASTLIMITSHEADER)
	mkdir -p $@_gcov
	cd $@_gcov && g++ -DDECIMATION=64 -DIKFAST_NO_MAIN -DIKFAST_HAS_LIBRARY -DIKFAST_NAMESPACE=$(KINEMATICS_NAMESPACE)_ikfast_$(word 2,$(subst ., ,$@))_$(word 3,$(subst ., ,$@)) -I ../../../Libraries/ToyWalker -include ../$(KINEMATICS_IKFASTLIMITSHEADER) ../../ikfast_gcov_translation3d.cpp ../$< -o gcov_testrun -fprofile-arcs -ftest-coverage
	cd $@_gcov && ./gcov_testrun
	cd $@_gcov && gcov $< >/dev/null
	sed -ne 's!    #####: *\([0-9]*\):\s.*!\1s/^/\\/\\//!p' $@_gcov/$*.translation3d.full.ikfast.styled.cpp.gcov > $@_gcov/$*.sed
	sed -f $@_gcov/$*.sed -e 's!^//\(\s*\)if!\1if (false)//if!; s!^//\(\s*\)else if!\1else if (false)//else if!; s!^//\(\s*bool \)!\1!' < $< > $@
	rm -rf $@_gcov


%.full.ikfast.styled.cpp: %.full.ikfast %.hpp
	astyle --style=allman -j < $< > $@

DAELINKID = $$(openrave-robot.py $(KINEMATICS_URDF).dae --info links | sed -ne 's/^$(1)\s*\([0-9]*\)\s*\S*/\1/p')
$(KINEMATICS_NAMESPACE)_ikfast.%.full.ikfast: $(KINEMATICS_URDF).dae
	python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=$< --iktype=$(word 3,$(subst ., ,$@)) --savefile=$@ --baselink=$(call DAELINKID,base_link) --eelink=$(call DAELINKID,$(word 2,$(subst ., ,$@))) --maxcasedepth=$(KINEMATICS_IKFASTDEPTH)
