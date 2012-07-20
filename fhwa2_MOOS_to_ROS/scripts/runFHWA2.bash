#!/bin/bash
# source /home/gavlab/rgc0003rosws ## should already be in the bashrc
cd ~/rgc0003rosws/fhwa2_viz/fhwa2_MOOS_to_ROS

# ROS launch
# -MOOSDB
# -uPlayBack or uMS
# -rViz with proper configuratio settings
gnome-terminal -x bash -c "roslaunch ./launch/run_everything.launch"

# Put the NCAT track data in rViz
sleep 3 # allow rviz to initialize
gnome-terminal -x bash -c "python scripts/mapBridge.py cfg/survey.yaml"

# show gavlab
sleep 3
gnome-terminal -x bash -c "python sctipts/liveBridge.py cfg/gavlab.yaml"

# gnome-terminal -x bash -c "python scripts/liveBridge.py cfg/pennst.yaml"

# gnome-terminal -x bash -c "python scripts/liveBridge.py cfg/sri.yaml"

# gnome-terminal -x bash -c "python scripts/liveBridge.py cfg/dsrc.yaml"