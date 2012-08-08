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
It is assumed that the G computer will be running Windows XP, and that the viz computer will be running Ubuntu 12.04 or later with ROS Fuerte or greater.

1. Hook up all equipment:
	
	-Novatel requires COM port on G computer (recommend 5), power, and GPS antenna to be plugged in
	
	-Ethernet switch between G computer and viz computer

2. On the G computer, open a terminal and determine the local static IP address and subnet mask with

		ipconfig/all

4. Ensure the wired connection is enabled on the viz computer.

		sudo ifconfig eth0 up

3. Set a temporary static IP on the viz computer. It is recommended to pick one that is very close to the G computer's. Use the same subnet mask.
		
		sudo ifconfig eth0 169.254.142.190 netmask 255.255.255.0

5. Ping each computer from the other to ensure proper connection