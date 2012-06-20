#!/usr/bin/env python

"""
This file contains modules which are at the core of MOOS ==> ROS communication

The function flow structure is roughly a daisy-chain with respect to each message received

Created 5/30/2012
Author: Robert Cofield
"""

def handle_msg( self, msg ):
    """
    takes a msg and passes usable info to gather_odom_var
    """
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
    time = msg.GetTime() # this may be grabbing the wrong time (not from the msg)
    name = msg.GetKey() # type of measurement "z______" string
    sens = msg.GetSource() # will yield "g______" string
    # if sens not in self.sensors:
    #     rospy.logwarn("mailroom.handle_msg :: unknown source/sensor: %(sens)s of yielding variable type %(name)s, carrying value %(value)s" %locals())
    if name in self.desired_variables: # where desired messages are scooped
        gather_odom_var(self, name, var_type, sens, value, time)
        # elif #..... other types of msgs to be done later
    else:
        rospy.logwarn("mailroom.handle_msg :: Unhandled msg: %(name)s of type %(var_type)s carries value %(value)s" %locals())


#############################################################################################


def gather_odom_var( self, name, var_type, sens, value, time ):
    """
    This function will only invoke publishing when it can send off all necessary info for a single source at once
    """
    import rospy
    from pprint import pprint

    print("\n\nIn gather_odom_var")
    # Need to include covariance info from here throughout'

    time = int(time*1000.0)/1000.0 #rounding to 3 decimal places so that the msg will groove...
    # # debug attempt:
    # time = rospy.Time(time)


    ###################################################################################################
    # # this function should only receive msgs with name 'zLat' & 'zLong' & 'zCourse'
    # # So missing info should be there
    # if sens not in self.LatLong_holder: # hold lat/long for each sensor
    #     self.LatLong_holder[sens] = {}
    #     if name not in self.LatLong_holder[sens]: 
    #         self.LatLong_holder[sens][name] = dict([['var_type', var_type], #double/string
    #                                                 ['value', float(value)],
    #                                                 ['time', time]])#need to make sure time stamp is the same        

    # # Now determine if there's any msgs missing
    # all_present = [True]*len(self.sensors)
    # for sens_ind in range(len(self.sensors)):
    #     if self.sensors[sens_ind] not in self.LatLong_holder: # don't even have any info from this sensor yet
    #         all_present[sens_ind] = False
    #     else: # see if we have all the info for this time step from this sensor
    #         for var_name in self.desired_variables: # everything needed for each time step from each sensor
    #             if var_name not in self.LatLong_holder[self.sensors[sens_ind]]: # no we don't
    #                 all_present[sens_ind] = False
    #                 break

    #     if all_present[sens_ind]: # time step has all required infos
    #         convert_odom_var(self, self.sensors[sens_ind]) # the second argument should be equivalent to 'sens'
    #         self.LatLong_holder[self.sensors] = {}

    # print('LatLong_holder:')
    # pprint(self.LatLong_holder)
    # print('\n')

    ###########################################################################################
    # msgs aren't getting past here - 6/19
    # debugging re-code

    # Determine which holder to use for this message
    # TIME STAMPS ARE CRITICAL HERE!!
    # may build up unused messages if they aren't complete enough to publish
    # also there has to be a more robust way of handling these names and properties and strings
    if sens == "gNovatel":
        holder = self.gNovatel_holder
    elif sens == "gPennSt":
        holder = self.gPennSt_holder
    elif sens == "gSRI":
        holder = self.gSRI_holder
    elif sens == "gDSRC":
        holder = self.gDSRC_holder
    else:
        rospy.logwarn("unknown sensor type. GET IT TOGETHER.")
    
    if time not in holder: # no messages from this timestep yet, initialize a dictionary for this time
        holder[time] = {}

    # stick the message in a holder specific to its sensor/source, according to msg time
    holder[time][name] = dict([['var_type', var_type], ['value', value]])

    # the latest addition may have made something publishable - check to see if the HOLDER WE JUST ADDED TO now meets the req's at ANY OF ITS TIMESTAMPS
    tstamps_sent = []
    for tstamp in holder: # loop through all times (may be async)
        holder_attime_publishable = [False] *len(self.desired_variables) # assume that it isn't publishable until proven otherwise
        for des_var_ind in range(len(self.desired_variables)):
            if self.desired_variables[des_var_ind] in holder[tstamp]:
                holder_attime_publishable[des_var_ind] = True
        if all(holder_attime_publishable): # ka-ching
            skateboard = holder[tstamp] # this will just roll the publishable group of msgs from sensor "holder" at time "tstamp" over to convert_odom_var; note that it doesn't contain the sensor/source "g_______"
            convert_odom_var(self, skateboard, sens, tstamp) # send it off



            ## Debug stuff
            print('Now we would publish stuff')
            print("sensor string: %(sens)s \nholder:" %locals())
            pprint(holder)
            print("skateboard for %(sens)s" %locals())
            print("\tat timestamp: %(tstamp)f" %locals())
            pprint(skateboard)
            #############


            # remember what we've sent so it can be cleaned
            tstamps_sent.append(tstamp)
    

    # clean up the holder of anything we already printed        
    for tstamp in tstamps_sent:
        del holder[tstamp] # don't need this time's msgs anymore
        print("\n~deleted dict from timestamp %(tstamp)f" %locals())



############################################################################################

def convert_odom_var( self, skateboard, sens, time ):
    """
    Once all the information for a single odom msg from a single sensor(source) is built together, this function converts it to UTM in a special UTM holder and sends it to the publishing function: mailroom.package_odom_var()
    """
    from randmcnally import ll2utm  
    from math import radians
    from pprint import pprint

    print("In convert_odom_var")
    
    #############################################################################
    ### the following was used before making multi-input support, and did not prove to be an easy modification
    # time_lat = self.LatLong_holder[sens_str]['zLat']['time']
    # time_lon = self.LatLong_holder[sens_str]['zLong']['time']
    # time_crs = self.LatLong_holder[sens_str]['zCourse']['time']
    # if (time_lat != time_lon) or (time_lat != time_crs):
    #     rospy.logwarn("Lat/Long/Course mismatch:: time steps aren't being handled properly")
    
    # # Position Conversions
    # (Easting, Northing) = ll2utm(self.LatLong_holder[sens_str]['zLat']['value'], 
    #                              self.LatLong_holder[sens_str]['zLong']['value'])
    
    # # Covariances - Lat/Lon Std Dev are output in meters already
    # NorthingStdDev = self.LatLong_holder[sens_str]['zLatStdDev']['value']
    # EastingStdDev = self.LatLong_holder[sens_str]['zLongStdDev']['value']
                
    # print('EastingStdDev: %(EastingStdDev)f' %locals())
    # print('NorthingStdDev %(NorthingStdDev)f' %locals())

    # # Consolidate into packaging dictionary & send off
    # self.NE_holder = {}
    # self.NE_holder['sens'] = sens_str
    # self.NE_holder['N'] = Northing
    # self.NE_holder['E'] = Easting
    # self.NE_holder['Nsd'] = NorthingStdDev
    # self.NE_holder['Esd'] = EastingStdDev
    # self.NE_holder['crs'] = radians(self.LatLong_holder[sens_str]['zCourse']['value']) # Will orient odom arrows to velocity direction
    # self.NE_holder['time'] = self.LatLong_holder[sens_str]['zLong']['time'] # could be any of the timestamps
    
    # package_odom_var(self, self.NE_holder) # Send to shipping function
    
    # # self.LatLong_holder = {} # clear holder for location at next time step
    # del self.NE_holder



    #########################################################################
    ### debugging multi-input support:
    #########################################################################

    UTMtoPub = dict([['time', time], ['sens', sens]]) # is time really necessary? Not using as dictionary index in self...

    # Convert to UTM
    (Easting, Northing) = ll2utm(float(skateboard['zLat']['value']), float(skateboard['zLong']['value']))
    UTMtoPub['N'] = Northing
    UTMtoPub['E'] = Easting

    # Covariances - Lat/Lon Std Dev are output in meters already
    UTMtoPub['Nsd'] = float(skateboard['zLatStdDev']['value'])
    UTMtoPub['Esd'] = float(skateboard['zLongStdDev']['value'])

    UTMtoPub['crs'] = radians(float(skateboard['zCourse']['value'])) # convert course to radians for ROS; Will orient odom arrows to velocity direction

    package_odom_var(self, UTMtoPub)

    print("\nUTMtoPub:")
    pprint(UTMtoPub)


#########################################################################################


def package_odom_var( self, UTMtoPub ):
    """
    Puts odom var (from UTM N & E) into a ros message and sends to the publishing decider.
        -does all the ROS packaging common to all sources
    """
    import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
    import rospy
    from nav_msgs.msg import Odometry # this will need to be repeated for other message types?
    from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
    import tf
    from tf.transformations import quaternion_from_euler as qfe
    from math import sqrt, pi, degrees
    import bridge_tf
    from pprint import pprint

    print("In package_odom_var")

    (odom_pub, ell_pub, r, g, b, stackup_elev, upd_veh) = demarcateSource( self, UTMtoPub['sens'] ) # get source-specific information
 
    ### Odometry Arrows ###
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time(UTMtoPub['time'])
    # odom_msg.header.stamp = UTMtoPub['time'] # rospy.Time 'ed it already
    odom_msg.header.frame_id = "odom" # may need to expand?
    odom_msg.pose.pose.position.x = UTMtoPub['E']
    odom_msg.pose.pose.position.y = UTMtoPub['N']
    odom_msg.pose.pose.position.z = stackup_elev # stack by source so all can be visible if coincident
    quat = qfe(-pi, -pi, -UTMtoPub['crs']-pi/2) # make a quaternion :: the (-pi, -pi, ...) came from trial-and-error => explore later
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]
    odom_msg.pose.covariance[0] = UTMtoPub['Nsd']
    odom_msg.pose.covariance[7] = UTMtoPub['Esd']
    odom_msg.pose.covariance[14] = 0 # constrain to xy axis for top-down view (this may not need to be stated)
    #######################

    ### Error Ellipses ### - pulls from the odom msg
    ell_marker = Marker()
    ell_marker.header = odom_msg.header # clarify?
    ell_marker.id = 0 # enumerate subsequent markers here 
    ell_marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
    ell_marker.pose = odom_msg.pose.pose # put at same place as its odom arrow
    ell_marker.lifetime = rospy.Duration() # will last forever unless modified
    ell_marker.ns = "Error_Ellipses" + '___' + UTMtoPub['sens']
    ell_marker.type = Marker.CYLINDER     
    ell_marker.scale.x = sqrt(odom_msg.pose.covariance[0]) *10 # not visible unless scaled up
    ell_marker.scale.y = sqrt(odom_msg.pose.covariance[7]) *10 # not visible unless scaled up
    ell_marker.scale.z = 0.000001 # We just want a disk
    ell_marker.color.r = r
    ell_marker.color.g = g
    ell_marker.color.b = b
    ell_marker.color.a = 0.5 # transparency - they're on top of one another, potentially
    ######################

    odom_pub.publish(odom_msg)
    ell_pub.publish(ell_marker)
        
    # tell camera tf where the look
    if upd_veh: # accepted(novatel) position has been updated -> update the position of the G mesh
        pub_at_position(self, odom_msg)
        bridge_tf.cameraFollow_tf(self, odom_msg)

    pprint(odom_msg)
    print('\n')



####################################################################################

def demarcateSource( self, sens ):
    """
    Makes odom msgs from each of the various sources look different on screen
    -puts each source's odom arrow and covar ellipse together
    -publishes it with the appropriate publisher
    - input 'sensor' : string from one of self.sensors list
    """
    print("In demarcateSource")

    if sens == 'gNovatel':
        odom_pub = self.odom_novatel_publisher
        ell_pub = self.ell_novatel_publisher
        r = self.rgb_novatel['r']
        g = self.rgb_novatel['g']
        b = self.rgb_novatel['b']
        stackup_elev = 0.3 # on top
        upd_veh = True
    elif sens == 'gPennSt':
        odom_pub = self.odom_pennst_publisher
        ell_pub = self.ell_pennst_publisher
        r = self.rgb_pennst['r']
        g = self.rgb_pennst['g']
        b = self.rgb_pennst['b']
        stackup_elev = 0.2
        upd_veh = False
    elif sens == 'gSRI':
        odom_pub = self.odom_sri_publisher
        ell_pub = self.ell_sri_publisher
        r = self.rgb_sri['r']
        g = self.rgb_sri['g']
        b = self.rgb_sri['b']
        stackup_elev = 0.1
        upd_veh = False
    elif sens == 'gDSRC':
        odom_pub = self.odom_dsrc_publisher
        ell_pub = self.ell_dsrc_publisher
        r = self.rgb_dsrc['r']
        g = self.rgb_dsrc['g']
        b = self.rgb_dsrc['b']
        stackup_elev = 0.0
        upd_veh = False
    else:
        rospy.logwarn('mailroom.demarcateSource :: unknown msg sensor/source, seriously??!! COME ON!')   


    return odom_pub, ell_pub, r, g, b, stackup_elev, upd_veh

####################################################################################

def pub_at_position( self, odom_msg ):
    from visualization_msgs.msg import Marker
    import rospy
    """
    Handles necessary information for displaying things at ACCEPTED (NOVATEL) POSITION SOLUTION:
        -vehicle mesh only for now
        -this should only be invoked when the accepted (novatel) position is updated
    """
    print("In pub_at_position")
    marker = Marker()
    pub = self.curpos_publisher
    # msg = self.odom_msgs[time]
    
    marker.header = odom_msg.header
    marker.id = 0 # enumerate subsequent markers here
    marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
    marker.pose = odom_msg.pose.pose
    marker.lifetime = rospy.Duration() # will last forever unless modified
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