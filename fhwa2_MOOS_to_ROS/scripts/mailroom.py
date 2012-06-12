#!/usr/bin/env python

"""
This file contains modules which are at the core of MOOS ==> ROS communication

Created 5/30/2012
Author: Robert Cofield
"""

def handle_msg(self, msg):
    if msg.IsDouble():
        var_type = "Double"
        value = str(msg.GetDouble()) #store all values as strings until handled specifically
    elif msg.IsString():
        var_type = "String"
        value = msg.GetString()
    else:
        rospy.logwarn('wtf? Unknown variable type')

    time = msg.GetTime()
    name = msg.GetKey() # type of measurement "z______"
    if name in self.desired_variables: # where desired messages are scooped
        #send to appropriate variable handler
        if name in self.odometry_variables:
            handle_odom_var(self, name, var_type, value, time)
        # elif #..... other types of msgs to be done later
    else:
        rospy.logwarn("Unhandled msg: %(name)s of type %(var_type)s carries value %(value)s" %locals())

#############################################################################################

def handle_odom_var(self, name, var_type, value, time):
    import rospy
    from randmcnally import ll2utm  
    from math import radians  

    # Need to include covariance info from here throughout
    time = int(time*1000.0)/1000.0 #rounding to 3 decimal places so that the msg will groove...

    # this function should only receive msgs with name 'zLat' & 'zLong' & 'zCourse'
    # So missing info should be there
    if name not in self.LatLong_holder: 
        self.LatLong_holder[name] = dict([['var_type', var_type],
                                          ['value', float(value)],
                                          ['time', time]])#need to make sure time stamp is the same        
    # Problems might arise here if time steps and info flow doesn't match up
    all_present = True # Now determine if there's any msgs missing
    for var_name in self.odometry_variables:
        if var_name not in self.LatLong_holder:
            all_present = False
    if all_present: # time step has all required infos
        time_lat = self.LatLong_holder['zLat']['time']
        time_lon = self.LatLong_holder['zLong']['time']
        time_crs = self.LatLong_holder['zCourse']['time']
        if (time_lat != time_lon) or (time_lat != time_crs):
            rospy.logwarn("Lat/Long/Course mismatch:: time steps aren't being handled properly")
        
        # Position Conversions
        (Easting, Northing) = ll2utm(self.LatLong_holder['zLat']['value'], 
                                             self.LatLong_holder['zLong']['value'])
        
        # Covariances - Lat/Lon Std Dev are output in meters already
        NorthingStdDev = self.LatLong_holder['zLatStdDev']['value']
        EastingStdDev = self.LatLong_holder['zLongStdDev']['value']
                    
        print('EastingStdDev: %(EastingStdDev)f' %locals())
        print('NorthingStdDev %(NorthingStdDev)f' %locals())

        # Consolidate into packaging dictionary & send off
        self.NE_holder = {}
        self.NE_holder['N'] = Northing
        self.NE_holder['E'] = Easting
        self.NE_holder['Nsd'] = NorthingStdDev
        self.NE_holder['Esd'] = EastingStdDev
        self.NE_holder['crs'] = radians(self.LatLong_holder['zCourse']['value']) # Will orient odom arrows to velocity direction
        self.NE_holder['time'] = self.LatLong_holder['zLong']['time']
        package_odom_var(self, self.NE_holder) # Send to shipping function
        self.LatLong_holder = {} # clear holder for location at next time step
        del self.NE_holder

#########################################################################################

def package_odom_var(self, NE_holder):
    """
    Puts odom var (from UTM N & E) into a ros message.
    """
    import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
    import rospy
    from nav_msgs.msg import Odometry # this will need to be repeated for other message types?
    from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
    import tf
    from tf.transformations import quaternion_from_euler as qfe
    from math import sqrt, pi, degrees
    import bridge_tf

    time = NE_holder['time']
    # Assume that the odom msg for this time step doesn't yet exist, create it
    self.odom_msgs[time] = Odometry()
    self.odom_msgs[time].header.stamp = rospy.Time(time)
    self.odom_msgs[time].header.frame_id = "odom"
    # when NE_holder reaches this function, it should have all the necessary info
    self.odom_msgs[time].pose.pose.position.x = NE_holder['E']
    self.odom_msgs[time].pose.pose.position.y = NE_holder['N']
    self.odom_msgs[time].pose.pose.position.z = 0 # constrain to xy axis for top-down view (this may not need to be stated)
    quat = qfe(-pi, -pi, -NE_holder['crs']-pi/2)
    self.odom_msgs[time].pose.pose.orientation.x = quat[0]
    self.odom_msgs[time].pose.pose.orientation.y = quat[1]
    self.odom_msgs[time].pose.pose.orientation.z = quat[2]
    self.odom_msgs[time].pose.pose.orientation.w = quat[3]
    self.odom_msgs[time].pose.covariance[0] = NE_holder['Nsd']
    self.odom_msgs[time].pose.covariance[7] = NE_holder['Esd']
    self.odom_msgs[time].pose.covariance[14] = 0 # constrain to xy axis for top-down view (this may not need to be stated)
    # Send to positions display function
    self.odom_publisher.publish(self.odom_msgs[time]) # ship it! # this action moved to do_current position
    # send to error ellipse & vehicle model function
    pub_at_position(self, time)
    # tell camera tf where the look
    bridge_tf.cameraFollow_tf(self, time)
    del self.odom_msgs[time]


def pub_at_position(self, time):
    from visualization_msgs.msg import Marker
    import rospy
    from math import
    """ 
    Handles necessary information for displaying error ellipses and vehicle model at current position
    """
    marker = Marker()
    pub = self.curpos_publisher
    msg = self.odom_msgs[time]
    marker.header = msg.header
    marker.id = 0 # enumerate subsequent markers here
    marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
    marker.pose = msg.pose.pose
    marker.lifetime = rospy.Duration() # will last forever unless modified

    # Error Ellipse
    marker.ns = "Error_Ellipses"
    marker.type = Marker.CYLINDER     
    marker.scale.x = sqrt(msg.pose.covariance[0]) *10
    marker.scale.y = sqrt(msg.pose.covariance[7]) *10
    marker.scale.z = 0.000001
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0 # transparency            
    pub.publish(marker)
    
    # Vehicle Model
    marker.ns = "vehicle_model"
    marker.type = Marker.MESH_RESOURCE
    marker.scale.x = 0.0254 # artifact of sketchup export
    marker.scale.y = 0.0254 # artifact of sketchup export
    marker.scale.z = 0.0254 # artifact of sketchup export
    marker.color.r = .5
    marker.color.g = .5
    marker.color.b = .5
    marker.color.a = .5
    marker.mesh_resource = "package://fhwa2_MOOS_to_ROS/mesh/infiniti_03_novatel_centered.dae" #robustify here
    marker.mesh_use_embedded_materials = False
    pub.publish(marker)