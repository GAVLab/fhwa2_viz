#!/usr/bin/env python
# Robert Cofield, GAVLab
# Python v2.7.3
# from pymoos.MOOSCommClient import MOOSCommClient # used this when pymoos wouldn't build on hp under ros
from pprint import pprint
import sys, os
from time import sleep, time
from yaml import load
from math import radians, sqrt, pi, degrees
# from mapping import ll2utm

#ROS Imports
import roslib
roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from nav_msgs.msg import Odometry # this will need to be repeated for other message types?
from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
import tf
from tf.transformations import quaternion_from_euler as qfe
import navpy.util

#MOOS Imports
# sys.path.append('../../../MOOS-ros-pkg/MOOS/pymoos/python') # location of one file named MOOSCommClient.py (other located in bin)

# sys.path.append('../../../MOOS-ros-pkg/MOOS/pymoos/lib')
from pymoos.MOOSCommClient import MOOSCommClient


class MOOS2RVIZ(MOOSCommClient):
    """Takes moos messages from an onboard moos db and displays the data in rViz"""
    def __init__(self, config):
        MOOSCommClient.__init__(self)
        self.SetOnConnectCallBack(self.onConnect)
        self.SetOnMailCallBack(self.onMail)
        self.get_config(config)
        self.set_publishers()
        self.holder = {}
        self.create_cap_freq_holders()
        self.navpy_gps = navpy.util.GPS()


    ### Init-related Functions #################################################
    def get_config(self, config):
        """saves the yaml config file info as instance attributes"""
        self.freq_max = config["freq_max"]
        self.UTMdatum = config["UTMdatum"] #dict
        self.coord_sys = config["coord_sys"]
        self.sensor_name = config["sensor_name"] # single string
        self.desired_variables = config["desired_variables"]
        self.color = config["color"] # dict with keys r, g, b on 0-255 scale
        self.legend_text_height = config["legend_text_height"]
        self.legend_text = config["display_name"]
        self.ismaster = config["dictate_pos"] # this instance will govern the current position
        if self.ismaster:
            self.veh_mesh_resource = config["veh_mesh_resource"]
        else:
            self.veh_mesh_resource = None


    def set_publishers(self):
        odom_topic = '/' + self.sensor_name + '/odom'
        self.odom_publisher = rospy.Publisher(odom_topic, Odometry)
        ell_topic = '/' + self.sensor_name + '/error_ellipse'
        self.ell_publisher = rospy.Publisher(ell_topic, Marker)
        legend_topic = '/' + self.sensor_name + '/legend'
        self.legend_publisher = rospy.Publisher(legend_topic, Marker)
        if self.ismaster: # this instance dictates accepted position (veh disp mesh)
            curpos_topic = '/' + self.sensor_name + ''
            self.curpos_publisher = rospy.Publisher('/novatel/current_position', Marker) # even though this is at the same position as the novatel error ellipse, we want it to have a different name in case the integrated solution is different
            rospy.Subscriber(odom_topic, Odometry, self.pub_at_position) # put the vehicle model at the accepted position solution


    def create_cap_freq_holders(self):
        """makes none value holders to keep track of when the last of each 
        measurement was published -- to limit frequency
        last_gNovatel_zLat = None --> will have last published timestamp
        -used in mailroom.cap_freq function
        """
        self.cap_freq_holders = {}
        for meas in range(len(self.desired_variables)):
            name = ''.join(['last_', self.desired_variables[meas]])
            self.cap_freq_holders[name] = time() # wall time
    ############################################################################

    def cameraFollow_tf(self, odom_msg):
        msg = odom_msg
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),\
                         (0, 0, 0, 1), # this is a unit quaternion\
                         msg.header.stamp,"base_footprint", "odom") 
                            #time,    #child frame , #parent frame

    # def FPV_tf(self, odom_msg):
    #     msg = odom_msg
    #     br = tf.TransformBroadcaster()
    #     br.sendTransform((msg.pose.pose.position.x,
    #                       msg.pose.pose.position.y, 
    #                       msg.pose.pose.position.z),
    #                      (odom_msg.pose.pose.orientation.x,
    #                       odom_msg.pose.pose.orientation.y,
    #                       odom_msg.pose.pose.orientation.z,
    #                       odom_msg.pose.pose.orientation.w),
    #                      msg.header.stamp, "fpv", "odom") 
    #                         #time,    #child frame , #parent frame


    ##### Mailroom Functions ###################################################
    def onConnect(self): 
        """Function required in every pyMOOS App"""
        # print('liveBridge.py :: In onConnect')
        for var in self.desired_variables: # expand later
            self.Register(var) #defined in MOOSCommClient.py

    def onMail(self):
        """Function required in every pyMOOS App"""
        # print('In onMail')
        messages = self.FetchRecentMail()
        # print('Recent Mail Fetched')
        for message in messages:
            self.handle_msg(message)
        # print('Messages Handled')
        return True


    def handle_msg(self, msg):
        """
        takes a msg and passes usable info to gather_odom_var
        """
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
        # print('Here')
        if (sens == self.sensor_name) and (name in self.desired_variables): # where desired messages are scooped
            if not self.cap_freq(name): # frequency capping
                self.gather_odom_var(name, var_type, value, time)
        # else:
        #     rospy.logwarn("handle_msg :: Unhandled msg: %(name)s of type %(var_type)s carries value %(value)s" % locals())


    def cap_freq(self, name):
        """enfores the maximum frequency by emitting boolean to send only messages
        arriving after the prescribed period following the previous message of 
        the same type
        """
        # minimum amount of time between msgs for each sensor
        this_time = time()
        tmin = 1/self.freq_max 
        ## reconstruct name of last time holder to find its contents (last time)
        last_one_name = ''.join(['last_', name])
        last_one = self.cap_freq_holders[last_one_name]

        # publish if first time or enough time elapsed
        if self.freq_max == 0: # disable freq capping
            catch = False
        elif (last_one is None) or ((this_time - last_one) >= tmin):
            catch = False
        else:
            catch = True
        # update the holder since we're now publishing
        if not catch:
            last_one = time()
        return catch


    def gather_odom_var(self, name, var_type, value, time):
        """
        This function will only invoke publishing when it can send off all necessary info for a single source at once
        """
        # print("In gather_odom_var")
        time = int(time*1000.0)/1000.0 #rounding to 3 decimal places 
        # TIME STAMPS ARE CRITICAL HERE!!
        # may build up unused messages if they aren't complete enough to publish
        if time not in self.holder: # no messages from this timestep yet, initialize a dictionary for this time
            self.holder[time] = {}
        # stick the message in a holder specific to its sensor/source, according to msg time
        self.holder[time][name] = dict([['var_type', var_type], ['value', value]]) # assume no multiple messages
        # the latest addition may have made something publishable - check to see if the HOLDER WE JUST ADDED TO now meets the req's at ANY OF ITS TIMESTAMPS
        tstamps_sent = []
        for tstamp in self.holder: # loop through all times (may be async)
            holder_attime_publishable = [False] *len(self.desired_variables) # assume that it isn't publishable until proven otherwise
            for des_var_ind in range(len(self.desired_variables)):
                if self.desired_variables[des_var_ind] in self.holder[tstamp]:
                    holder_attime_publishable[des_var_ind] = True
            if all(holder_attime_publishable): # ka-ching
                # this will just roll the publishable group of msgs from sensor "holder" at time "tstamp" over to convert_odom_var; note that it doesn't contain the sensor/source "g_______"
                skateboard = self.holder[tstamp] 
                # send it off
                self.convert_odom_var(skateboard, tstamp) 
                # remember what we've sent so it can be cleaned
                tstamps_sent.append(tstamp)  
        for tstamp in tstamps_sent:
            del self.holder[tstamp] # clean up the holder of anything we already printed        


    def convert_odom_var(self, skateboard, time):
        """
        Once all the information for a single odom msg from a single sensor(source) is built together, this function converts it to UTM in a special UTM holder and sends it to the publishing function: mailroom.package_odom_var()
        Covariances - Lat/Lon Std Dev are output in meters already
        converts course to radians for ROS; Will orient odom arrows to velocity direction
        """
        # print("In convert_odom_var")
        ## FIXME here an assumption is made about the order of variables - robustify later
        UTMtoPub = dict([['time', time]]) # is time really necessary? Not using as dictionary index in self...
        if self.coord_sys == 'ECEF':
            (Easting, Northing, _), info = self.navpy_gps.ecef2utm((float(skateboard[self.desired_variables[0]]),
                                                                    float(skateboard[self.desired_variables[1]]),
                                                                    float(skateboard[self.desired_variables[2]])))
        elif self.coord_sys == 'LLA':
            (Easting, Northing, _), info = self.navpy_gps.lla2utm((float(skateboard['zLat']['value']),
                                                                   float(skateboard['zLong']['value'])),
                                                                   0)
        # print("{} - {} = {}".format(Northing, self.UTMdatum['N'], Northing - self.UTMdatum['N']))
        UTMtoPub['N'] = Northing - self.UTMdatum['N']
        UTMtoPub['E'] = Easting  - self.UTMdatum['E']
        UTMtoPub['Nsd'] = float(skateboard['zLatStdDev']['value'])
        UTMtoPub['Esd'] = float(skateboard['zLongStdDev']['value'])
        UTMtoPub['crs'] = radians(float(skateboard['zCourse']['value']))
        self.package_odom_var(UTMtoPub)


    def package_odom_var(self, UTMtoPub):
        """
        Puts odom var (from UTM N & E) into a ros message and sends to the publishing decider.
            -does all the ROS packaging common to all sources
            -each individual legend is now
        """
        ### Odometry Arrows ####################################################
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time(UTMtoPub['time'])
        odom_msg.header.frame_id = "odom" # may need to expand?
        odom_msg.pose.pose.position.x = UTMtoPub['E']
        odom_msg.pose.pose.position.y = UTMtoPub['N']
        odom_msg.pose.pose.position.z = 1.55
        # make a quaternion :: the (-pi, -pi, ...) came from trial-and-error 
        quat = qfe(-pi, -pi, -UTMtoPub['crs']-pi/2)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.pose.covariance[0] = UTMtoPub['Nsd']
        odom_msg.pose.covariance[7] = UTMtoPub['Esd']
        odom_msg.pose.covariance[14] = 0
        self.odom_publisher.publish(odom_msg)

        ### Error Ellipses #####################################################
        ## pulls from the odom msg --
        ell_marker = Marker()
        ell_marker.header = odom_msg.header # clarify?
        ell_marker.id = 0 # enumerate subsequent markers here 
        ell_marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
        ell_marker.pose = odom_msg.pose.pose # put at same place as its odom arrow
        ell_marker.lifetime = rospy.Duration() # will last forever unless modified
        ell_marker.ns = ''.join(["Error_Ellipses", '___', self.sensor_name])
        ell_marker.type = Marker.CYLINDER     
        ell_marker.scale.x = sqrt(odom_msg.pose.covariance[0]) *10 # not visible unless scaled up
        ell_marker.scale.y = sqrt(odom_msg.pose.covariance[7]) *10 # not visible unless scaled up
        ell_marker.scale.z = 0.000001 # We just want a disk
        ell_marker.color.r = self.color['r']
        ell_marker.color.g = self.color['g']
        ell_marker.color.b = self.color['b']
        ell_marker.color.a = 0.95
        self.ell_publisher.publish(ell_marker)

        ### Legend #############################################################
        marker = Marker()
        marker.header = odom_msg.header
        marker.id = 0
        marker.ns = self.sensor_name + 'legend'
        marker.type = Marker.TEXT_VIEW_FACING
        if self.sensor_name == 'gDSRC':
            marker.text = 'gKapsch'
        else:
            marker.text = self.sensor_name
        marker.action = Marker.MODIFY
        marker.pose = odom_msg.pose.pose
        marker.pose.position.z = self.legend_text_height # elevate to spread
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.6
        marker.color.r = self.color['r']
        marker.color.g = self.color['g']
        marker.color.b = self.color['b']
        marker.color.a = 1.0
        self.legend_publisher.publish(marker)

        # tell camera tf where the look if master ##############################
        if self.ismaster: # update the vehicle mesh position
            self.pub_at_position(odom_msg)
            self.cameraFollow_tf(odom_msg)
            # self.FPV_tf(odom_msg) # add a view


    def pub_at_position(self, odom_msg):
        """
        Handles necessary information for displaying things at 
                                ACCEPTED (NOVATEL) POSITION SOLUTION:
            -vehicle mesh 
        !!!this should only be invoked when the accepted (novatel) position 
        is updated
        """
        ### G35 Mesh #############################################################
        marker = Marker()
        pub = self.curpos_publisher
        marker.header = odom_msg.header
        marker.id = 0 # enumerate subsequent markers here
        marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
        marker.pose = odom_msg.pose.pose
        marker.pose.position.z = 1.55
        marker.lifetime = rospy.Duration() # will last forever unless modified
        marker.ns = "vehicle_model"
        marker.type = Marker.MESH_RESOURCE
        marker.scale.x = 0.0254 # artifact of sketchup export
        marker.scale.y = 0.0254 # artifact of sketchup export
        marker.scale.z = 0.0254 # artifact of sketchup export
        marker.color.r = .05
        marker.color.g = .05
        marker.color.b = .05
        marker.color.a = .2
        marker.mesh_resource = self.veh_mesh_resource
        marker.mesh_use_embedded_materials = False
        pub.publish(marker)


################################################################################
################################################################################

def main():
    param_ip = rospy.get_param('~sender_ip')
    print('IP from launch parameter: %s' % param_ip)
    # ip = '192.168.1.100'
    ip = '127.0.0.1'
    port = rospy.get_param('~port')
    print('Port from launch parameter: %s' % port)
    
    ## setup config file
    config_file = sys.argv[1]
    if config_file[-4:] != 'yaml':
        print("Config file must be YAML format")
    stream = file(config_file,'r')
    this_config = load(stream) # loads as a dictionary

    #setup ROS node
    node_name = 'moos2rviz_' + this_config['sensor_name']
    rospy.init_node(node_name)

    #Setup MOOS App
    app = MOOS2RVIZ(this_config)
    app.Run(ip, int(port), node_name) # fixed IP of R2 - G computer
    # app.Run('127.0.0.1', 9000, node_name)
    for x in range(30): # allow 3 second to connect to MOOSDB
        sleep(0.1)
        if app.IsConnected():
            print("Connected to MOOSDB @ " + ip+":"+port)
            break
    # Make sure we Connected
    if not app.IsConnected():
        rospy.logerr("Failed to Connect to MOOSDB @ " + ip + ":" + port)
        sys.exit(-1)
    
    #Setup ROS Stuff
   
    #spin
    rospy.spin()
    
if __name__ == '__main__':
    main()