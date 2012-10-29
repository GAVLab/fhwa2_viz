LDW visualizer
==============

Visualization stuff for the FHWA2 project in the GAVlab

Datum is roughly center of track
	
	E: 	  659 300  		m
	N: 	3 607 850   	m


## Installation

1. 	If you don't have ROS, you are an idiot. [Fix this.](http://www.ros.org/wiki/ROS/Installation)

2. Install Git.

	`sudo apt-get install git`

3.	Create a ROS workspace in your home folder.
	
	`mkdir devel`
	`cd devel`
	`mkdir fhwa2_ws`
	`cd fhwa2_ws`
	`rosws init . /opt/ros/fuerte`
	`source ./setup.bash`

4. 	Install the latest version of this stack.

	`rosws merge https://raw.github.com/GAVLab/fhwa2_viz/master/fhwa2_viz.rosinstall`
	`source setup.bash`
	`rosinstall --verbose . /opt/ros/fuerte/`
	`source setup.bash`
	`rosmake MOOS pyMOOS`

Note that once you have done this, subsequent updating is simpler.

`rosws update`
`rosinstall .`
`source ./setup.bash`


## Running Playback From alog

1.	Launch the playback file.
		
	`roslaunch ~/devel/fhwa2_ws/fhwa2_viz/fhwa2_MOOS_to_ROS/launch/playback.launch`

2. 	In uPlayBack, select the desired alog file. Click "?" to set host and port. Click "MOOS".


## Running Playback Live

### Network Configuration

1. Set a static IP for both boxes.

2. Changes the "source_ip" param in the live.launch or live_2_pos.launch file to
the IP of the car (MOOSDB) computer