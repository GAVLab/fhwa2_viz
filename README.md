# LDW visualizer #

Visualization stuff for the FHWA2 project in the GAVlab
Datum is roughly center of track
	E: 	  659 300  		m
	N: 	3 607 850   	m


### Installation ###

1. If you don't have ROS, you are an idiot. [Fix this.](http://www.ros.org/wiki/ROS/Installation)
2.	Create a ROS workspace in your home folder:
	mkdir devel
	cd devel
	mkdir fhwa2_ws
	cd fhwa2_ws
	rosws init
	source setup.bash
3. Install the latest version of this stack:
	rosws merge https://raw.github.com/GAVLab/fhwa2_viz/master/fhwa2_viz.rosinstall
	source setup.bash
	source /opt/ros/fuerte/setup.bash
	rosinstall --verbose . /opt/ros/fuerte/
	source setup.bash