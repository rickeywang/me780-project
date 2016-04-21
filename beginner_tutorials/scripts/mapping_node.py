#!/usr/bin/env python
import matplotlib.pyplot as pp
import message_filters
import roslib
import rospy
from nav_msgs.msg import Odometry
from pylab import *
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
import colormaps as cmaps
plt.register_cmap(name='viridis', cmap=cmaps.viridis)

from oc import *

PKG = 'beginner_tutorials'
roslib.load_manifest(PKG)

# Global Variable Defines
M_global = 40
N_global = 40

odom_scale = array(10) # Odometry data [m]
laser_scale = array(10)  # LaserScan range data [m]
og_prob_global = None
ogl = None
icount = None
origin = None
origin_set = False

odom_cache = None
laser_cache = None

# Interactive mode ON for plot
pp.ion()

def setup_og():
    # Occupancy grid in both probability and log odds form
    global og_prob_global
    og_prob_global = 0.5 * ones((M_global, N_global))
    oglog_0 = log(og_prob_global/(1 - og_prob_global))
    global ogl # Log odds
    ogl=oglog_0
    global icount
    icount = 2

def imu_callback(data):
    #   print data
    #print rospy.get_name(), "Linear Accel X: %s" % str(data.linear_acceleration.x)
    data = data

def odom_callback(data):
    odom_x = data.pose.pose.position.x
    odom_y = data.pose.pose.position.y
    w = data.pose.pose.orientation.w
    print("Odom X " + str(odom_x) + " Y " + str(odom_y))


def laser_callback(laser_msg):
    global icount
    global ogl
    global og_prob_global
    global origin
    global origin_set

    print(icount)
    icount += 1

    # @TODO The laser_msg and odom_msg are not synchronized, causing bad maps

    laser_msg = laser_cache.getElemBeforeTime(laser_cache.getLastestTime())

    r_m = laser_msg.ranges * laser_scale
    phi_m = arange(laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment)
    r_max = laser_msg.range_max * laser_scale

    # State from odom. Offset by the origin.
    odom_msg = odom_cache.getElemBeforeTime(odom_cache.getLastestTime())
    if not origin_set:
        """
        Initializes the odometry offset to value that the robot first starts,
        because the ROS Node on Turtlebot keeps track of odom since first turning on.

        This coordinate becomes our origin. We offset actual /odom data by this origin.
        """
        origin = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)
        origin_set = True
    odom_x = ((odom_msg.pose.pose.position.x - origin[0]) * odom_scale)
    odom_y = ((odom_msg.pose.pose.position.y - origin[1]) * odom_scale)
    w = odom_msg.pose.pose.orientation.w
    odom_theta = w
    state = c_[odom_x, odom_y, odom_theta]
    state = state[0]

    ogout = ogmap(M_global, N_global, ogl, state, phi_m, r_m, r_max)

    ogl = ogout['ogl']
    imml = ogout['imml']

    # Recover the probabilities from log
    og_prob_global = exp(ogl) / (1 + exp(ogl))
    og_prob_mm = exp(imml) / (1 + exp(imml))

    plt.figure(0)
    aximg0 = plt.imshow(og_prob_global * 100, cmap='viridis')
    plt.show()
    plt.pause(0.0001)
    plt.figure(1)
    plt.clf()
    aximg1 = plt.imshow(og_prob_mm, cmap='viridis')
    state_now = state
    for i in range(len(phi_m)):
        if isnan(r_m[i]):
            continue
        else:
            plot(state_now[1] + r_m[i]*sin(phi_m[i] + state_now[2]),
                 state_now[0] + r_m[i]*cos(phi_m[i] + state_now[2]),
                 'g.')

    plt.show()
    plt.pause(0.0001)



def map_callback(msg):
    # Called when new map is received
    ogmap = msg.data
    # Now, use the laser info to update this map!



def listener():
    rospy.init_node('numpy_listener')
    #rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    #rospy.Subscriber("/odom", Odometry, odom_callback)
    # rospy.Subscriber("/map", OccupancyGrid, map_callback)
    #odom_init_once = None
    #odom_init_once = rospy.Subscriber("/odom", Odometry, odom_init_once)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    # rospy.Subscriber("floats", Floats, callback)
    # Time sync problem ... it seems that computer is too slow to process,
    # so the algorithm runs with latest state but older measurements!
    odom_sub = message_filters.Subscriber("/odom", Odometry)
    global odom_cache
    odom_cache = message_filters.Cache(odom_sub, 2)
    scan_sub = message_filters.Subscriber("/scan", LaserScan)
    global laser_cache
    laser_cache = message_filters.Cache(scan_sub, 2)
    ts = message_filters.TimeSynchronizer([odom_sub, scan_sub], 10)
    #ts.registerCallback(laser_callback, 1)
    rospy.spin()


if __name__ == '__main__':
    setup_og()
    listener()
