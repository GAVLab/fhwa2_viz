#!/usr/bin/env python

"""
This file contains modules which are at the core of MOOS ==> ROS communication

The function flow structure is roughly a daisy-chain with respect to each message received

Created 5/30/2012
Author: Robert Cofield
"""

def handle_msg(self, msg):
    print("In handle_msg")
    if msg.IsDouble():
        var_type = "Double"
        value = str(msg.GetDouble()) #store all values as strings until handled specifically
    elif msg.IsString():
        var_type = "String"
        value = msg.GetString()
    else:
        rospy.logwarn('wtf? Unknown variable type')

    # obtain info from msg via functions defined in pyMOOSMsg.cpp
    time = msg.GetTime()
    name = msg.GetKey() # type of measurement "z______"
    sens = msg.GetSource() # hopefully will yield "g______"
    # if sens not in self.sensors:
    #     rospy.logwarn("mailroom.handle_msg :: unknown source/sensor: %(sens)s of yielding variable type %(name)s, carrying value %(value)s" %locals())
    if name in self.desired_variables: # where desired messages are scooped
        gather_odom_var(self, name, var_type, sens, value, time)
        # elif #..... other types of msgs to be done later
    else:
        rospy.logwarn("mailroom.handle_msg :: Unhandled msg: %(name)s of type %(var_type)s carries value %(value)s" %locals())


#############################################################################################


def gather_odom_var(self, name, var_type, sens, value, time):
    """
    This function will only invoke publishing when it can send off all necessary info for a single source at once
    """
    import rospy
    from randmcnally import ll2utm  
    import math
    from pprint import pprint

    print("In gather_odom_var")
    # Need to include covariance info from here throughout
    time = int(time*1000.0)/1000.0 #rounding to 3 decimal places so that the msg will groove...

    # this function should only receive msgs with name 'zLat' & 'zLong' & 'zCourse'
    # So missing info should be there
    if sens not in self.LatLong_holder: # hold lat/long for each sensor
        self.LatLong_holder[sens] = {}
        if name not in self.LatLong_holder[sens]: 
            self.LatLong_holder[sens][name] = dict([['var_type', var_type], #double/string
                                                    ['value', float(value)],
                                                    ['time', time]])#need to make sure time stamp is the same        

    # Now determine if there's any msgs missing
    all_present = [True]*len(self.sensors)
    for sens_ind in range(len(self.sensors)):
        if self.sensors[sens_ind] not in self.LatLong_holder: # don't even have any info from this sensor yet
            all_present[sens_ind] = False
        else: # see if we have all the info for this time step from this sensor
            for var_name in self.desired_variables: # everything needed for each time step from each sensor
                if var_name not in self.LatLong_holder[self.sensors[sens_ind]]: # no we don't
                    all_present[sens_ind] = False
                    break

        if all_present[sens_ind]: # time step has all required infos
            convert_odom_var(self, self.sensors[sens_ind]) # the second argument should be equivalent to 'sens'
            self.LatLong_holder[self.sensors] = {}

    print('LatLong_holder:')
    pprint(self.LatLong_holder)
    print('\n')

    # if all info present, send it down the line
    # for sens_ind in range(len(self.sensors)):
        # if all_present[sens_ind]: # time step has all required infos
        #     sens_str = self.sensors[sens_ind]
        #     convert_odom_var(self, sens_str) #


###########################################################################################
# msgs aren't getting past here - 6/19

def convert_odom_var(self, sens_str):
    """
    Once all the information for a single odom msg from a single sensor(source) is built together, this function converts it to UTM in a special UTM holder and sends it to the publishing function: mailroom.package_odom_var()
    """
    print("In convert_odom_var")
    time_lat = self.LatLong_holder[sens_str]['zLat']['time']
    time_lon = self.LatLong_holder[sens_str]['zLong']['time']
    time_crs = self.LatLong_holder[sens_str]['zCourse']['time']
    if (time_lat != time_lon) or (time_lat != time_crs):
        rospy.logwarn("Lat/Long/Course mismatch:: time steps aren't being handled properly")
    
    # Position Conversions
    (Easting, Northing) = ll2utm(self.LatLong_holder[sens_str]['zLat']['value'], 
                                 self.LatLong_holder[sens_str]['zLong']['value'])
    
    # Covariances - Lat/Lon Std Dev are output in meters already
    NorthingStdDev = self.LatLong_holder[sens_str]['zLatStdDev']['value']
    EastingStdDev = self.LatLong_holder[sens_str]['zLongStdDev']['value']
                
    print('EastingStdDev: %(EastingStdDev)f' %locals())
    print('NorthingStdDev %(NorthingStdDev)f' %locals())

    # Consolidate into packaging dictionary & send off
    self.NE_holder = {}
    self.NE_holder['sens'] = sens_str
    self.NE_holder['N'] = Northing
    self.NE_holder['E'] = Easting
    self.NE_holder['Nsd'] = NorthingStdDev
    self.NE_holder['Esd'] = EastingStdDev
    self.NE_holder['crs'] = math.radians(self.LatLong_holder[sens_str]['zCourse']['value']) # Will orient odom arrows to velocity direction
    self.NE_holder['time'] = self.LatLong_holder[sens_str]['zLong']['time'] # could be any of the timestamps
    
    package_odom_var(self, self.NE_holder) # Send to shipping function
    
    # self.LatLong_holder = {} # clear holder for location at next time step
    del self.NE_holder


#########################################################################################


def package_odom_var(self, NE_holder):
    """
    Puts odom var (from UTM N & E) into a ros message and sends to the publishing decider.
    """
    import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
    import rospy
    from nav_msgs.msg import Odometry # this will need to be repeated for other message types?
    from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
    import tf
    from tf.transformations import quaternion_from_euler as qfe
    from math import sqrt, pi, degrees
    import bridge_tf

    print("In package_odom_var")
    time = NE_holder['time']
    # Assume that the odom msg for this time step doesn't yet exist, create it
    # May need another way of differentiating between sources other than time - for dummy-generated sources, times will be the same
    self.odom_msgs[time] = Odometry()
    self.odom_msgs[time].header.stamp = rospy.Time(time)
    self.odom_msgs[time].header.frame_id = "odom" # may need to expand?
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
       
    # send to multiple odometry messaging & error ellipse publish function
    demarcateSource(self, time, NE_holder['sens'])

    # send to vehicle model function
    pub_at_position(self, time)
    
    # tell camera tf where the look
    bridge_tf.cameraFollow_tf(self, time)
    del self.odom_msgs[time]


####################################################################################


def demarcateSource(self, time, sensor):
    """
    Makes odom msgs from each of the various sources look different on screen
    -puts each source's odom arrow and covar ellipse together
    -publishes it with the appropriate publisher
    - input 'sensor' : string from one of self.sensors list
    """
    print("In demarcateSource")
    msg = self.odom_msgs[time]
    
    marker = Marker() #error ellipse
    marker.header = msg.header
    marker.id = 0 # enumerate subsequent markers here 
    marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
    marker.pose = msg.pose.pose # will need to stack up different types so they're not on top of one another
    marker.lifetime = rospy.Duration() # will last forever unless modified
    marker.ns = "Error_Ellipses" + '___' + sensor
    marker.type = Marker.CYLINDER     
    marker.scale.x = sqrt(msg.pose.covariance[0]) *10
    marker.scale.y = sqrt(msg.pose.covariance[7]) *10
    marker.scale.z = 0.000001
    marker.color.a = 1.0 # transparency            

    # decide which publisher to use for the odom msg, and what color to make the ellipse
    if NE_holder['sens'] == 'gNovatel':
        pub = self.odom_novatel_publisher # this will probably need another 
        marker.color.r = 0.0 # accepted position solution will be white
        marker.color.g = 0.0
        marker.color.b = 1.0
    elif NE_holder['sens'] == 'gPennSt':
        pub = self.odom_pennst_publisher
        marker.color.r = self.rgb_pennst['r']
        marker.color.g = self.rgb_pennst['g']
        marker.color.b = self.rgb_pennst['b']
    elif NE_holder['sens'] == 'gSRI':
        pub = self.odom_sri_publisher
        marker.color.r = self.rgb_sri['r']
        marker.color.g = self.rgb_sri['g']
        marker.color.b = self.rgb_sri['b']
    elif NE_holder['sens'] == 'gDSRC':
        pub = self.odom_dsrc_publisher
        marker.color.r = self.rgb_dsrc['r']
        marker.color.g = self.rgb_dsrc['g']
        marker.color.b = self.rgb_dsrc['b']
    else:
        rospy.logwarn('mailroom.package_odom_var :: unknown msg sensor/source, not published')   

    pub.publish(msg)
    pub.publish(marker)

   
####################################################################################


def pub_at_position(self, time):
    from visualization_msgs.msg import Marker
    import rospy
    """ 
    Handles necessary information for displaying things at accepted (Novatel) position solution:
        -vehicle mesh only for now
    """
    print("In pub_at_position")
    marker = Marker()
    pub = self.curpos_publisher
    msg = self.odom_msgs[time]
    marker.header = msg.header
    marker.id = 0 # enumerate subsequent markers here
    marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
    marker.pose = msg.pose.pose
    marker.lifetime = rospy.Duration() # will last forever unless modified

    # # Error Ellipse - need to be one for each gSensor
    # marker.ns = "Error_Ellipses"
    # marker.type = Marker.CYLINDER     
    # marker.scale.x = sqrt(msg.pose.covariance[0]) *10
    # marker.scale.y = sqrt(msg.pose.covariance[7]) *10
    # marker.scale.z = 0.000001
    # marker.color.r = 0.0
    # marker.color.g = 0.0
    # marker.color.b = 1.0
    # marker.color.a = 1.0 # transparency            
    # pub.publish(marker)
    
    # Vehicle Model - only
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