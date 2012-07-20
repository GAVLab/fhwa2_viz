#!/bin/bash
cd ~/rgc0003rosws/fhwa2_viz/fhwa2_MOOS_to_ROS

# Start ros master
gnome-terminal --window roscore
echo 'roscore Initialized'

# Initialize MOOSDB
gnome-terminal --window rosrun MOOS MOOSDB
echo 'MOOSDB initialized'

gnome-terminal --window
rosrun MOOS uPlayBack

gnome-terminal --window
rosrun rviz rviz

gnome-terminal --window
python scripts/mapBridge.py cfg/survey.yaml

gnome-terminal --window
python sctipts/liveBridge.py cfg/gavlab.yaml

gnome-terminal --window
python scripts/liveBridge.py cfg/pennst.yaml

gnome-terminal --window
python scripts/liveBridge.py cfg/sri.yaml

gnome-terminal --window
python scripts/liveBridge.py cfg/dsrc.yaml