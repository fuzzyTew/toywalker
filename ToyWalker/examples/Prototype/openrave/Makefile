IKFAST = python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py
backleg.ikfast.cpp: Prototype.robot.xml
	$(IKFAST) --robot=$< --iktype=translation3d --savefile=$@ --baselink=0 --eelink=4 --freeindex=4

