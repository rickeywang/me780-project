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

M_global = 25
N_global = 25

odom_x = 0.0
odom_y = 0.0
odom_theta = 0

# Define at Module Level
og_prob_global = 0.5 * ones((M_global, N_global))
ogl = log(og_prob_global/(1 - og_prob_global))

# Interactive mode ON
pp.ion()

def setup_og():
    # Occupancy grid in both probability and log odds form
    global og_prob_global
    og_prob_global = 0.5 * ones((M_global, N_global))
    oglog_0 = log(og_prob_global/(1 - og_prob_global))
    global ogl # Log odds
    ogl=oglog_0


def update_ogl(ogl_new):
    ogl = ogl_new


def imu_callback(data):
    #   print data
    #print rospy.get_name(), "Linear Accel X: %s" % str(data.linear_acceleration.x)
    data = data


def odom_callback(data):
    odom_x = data.pose.pose.position.x
    odom_y = data.pose.pose.position.y
    w = data.pose.pose.orientation.w
    odom_theta = 4*pi - 2*arccos(w) #Only z axis has rotation
    #print "Odom X:%s" % odom_x


def laser_callback(msg):

    r_m = msg.ranges * 10
    phi_m = arange(msg.angle_min, msg.angle_max, msg.angle_increment)
    r_max = msg.range_max * 10

    # State from odom
    odom_msg = odom_cache.getElemBeforeTime(odom_cache.getLastestTime())
    odom_x = (odom_msg.pose.pose.position.x * 10)
    odom_y = (odom_msg.pose.pose.position.y * 10)
    w = odom_msg.pose.pose.orientation.w
    odom_theta = 4 * pi - 2 * arccos(w)
    #print "Odom X:%s" % odom_x
    state = c_[odom_x, odom_y, odom_theta]
    state = state[0]

    ogout = ogmap(M_global, N_global, ogl, state, phi_m, r_m, r_max)

    update_ogl(ogout['ogl'])
    imml = ogout['imml']

    # Recover the probabilities from log
    og_prob_global = exp(ogl) / (1 + exp(ogl))
    og_prob_mm = exp(imml) / (1 + exp(imml))

    plt.figure(0)
    aximg0 = plt.imshow(og_prob_global * 100, cmap='viridis')
    plt.show()
    plt.pause(0.0001)
    plt.figure(1)
    aximg1 = plt.imshow(og_prob_mm, cmap='viridis')
    plt.show()
    plt.pause(0.0001)



def map_callback(msg):
    # Called when new map is received
    ogmap = msg.data
    # Now, use the laser info to update this map!



def listener():
    rospy.init_node('numpy_listener')
    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    # rospy.Subscriber("/odom", Odometry, odom_callback)
    # rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    # rospy.Subscriber("floats", Floats, callback)
    odom_sub = message_filters.Subscriber("/odom", Odometry)
    global odom_cache
    odom_cache = message_filters.Cache(odom_sub)
    scan_sub = message_filters.Subscriber("/scan", LaserScan)
    ts = message_filters.TimeSynchronizer([odom_sub, scan_sub], 10)
    ts.registerCallback(laser_callback)
    rospy.spin()


if __name__ == '__main__':
    setup_og()
    listener()
