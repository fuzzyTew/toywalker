IKFAST = python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py

Default: Prototype.urdf.preview

%.urdf.preview: %.urdf
	check_urdf $<
	roslaunch urdf_tutorial display.launch model:=$<

%.urdf: %.urdf.xacro
	rosrun xacro xacro --inorder $< > $@

%.dae: %.urdf
	rosrun collada_urdf urdf_to_collada $< $@

backleg.ikfast.cpp: Prototype.robot.xml
	$(IKFAST) --robot=$< --iktype=translation3d --savefile=$@ --baselink=0 --eelink=3

