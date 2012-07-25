# LDW visualizer #

Visualization stuff for the FHWA2 project in the GAVlab

Datum is roughly center of track
	
	E: 	  659 300  		m
	N: 	3 607 850   	m


### Installation ###

1. 	If you don't have ROS, you are an idiot. [Fix this.](http://www.ros.org/wiki/ROS/Installation)
2.	Create a ROS workspace in your home folder.

		mkdir devel
		cd devel
		mkdir fhwa2_ws
		cd fhwa2_ws
		rosws init
		source setup.bash

3. 	Install the latest version of this stack.

		rosws merge https://raw.github.com/GAVLab/fhwa2_viz/master/fhwa2_viz.rosinstall
		source setup.bash
		source /opt/ros/fuerte/setup.bash
		rosinstall --verbose . /opt/ros/fuerte/
		source setup.bash

Note that once you have done this, subsequent updating is simpler.

		rosinstall .
		source setup.bash


### Running Playback From alog ###

1.	Launch the playback file.
		
		roslaunch ~/devel/fhwa2_ws/fhwa2_viz/fhwa2_MOOS_to_ROS/launch/playback.launch

2. 	In uPlayBack, select the desired alog file. Click "?" to set host and port. Click "MOOS".


### Running Playback Live ###

You'll know when I do.