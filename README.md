fhwa2_viz
=========

Visualization stuff for the FHWA2 project in the GAVlab

Datum is roughly center of track

E: 	  659 300  		m
N: 	3 607 850   	m


### Procedures for running latest version ###

##### For MOOSDB on separate machine from visualizer: #####

1. modify guiBridge.py (will run on visualizer machine) main function to reflect IP address and port number of computer with MOOSDB

2. 


##### For MOOSDB and visualizer on same machine: #####
	
1. Ensure your IP and port for MOOSDB are singular in guiBridge.py (127.0.0.1 , 9000)

2. Open a terminal and initialize roscore:
	
	"gavlab@sim-pc:~$ roscore"
	
3. In another terminal, initialize MOOSDB:
	
	"gavlab@sim-pc:~$ rosrun MOOS MOOSDB"
	
4. [For visualizing from log file]: In another terminal, open uPlayBack:
	
	"gavlab@sim-pc:~$ rosrun MOOS uPlayBack"

Select the *.alog file by clicking "File"

Click the "?" icon to select host (agree with IP in guiBridge.py) and port (...)

Click "MOOS" to MOOS it up, ..yo

5. In yet another terminal intialize rViz:

	"gavlab@sim-pc:~$ rosrun rviz rviz"

Load the rViz configuration from /home/__user__/__rosworkspace__/fhwa2_viz/fhwa2_MOOS_to_ros/

6. Run guiBridge.py:
	"gavlab@sim-pc:~/rgc0003rosws/fhwa2_viz$ python fhwa2_MOOS_to_ROS/scripts/guiBridge.py"



#################################################

Note: when changing log files, restart uPlayBack