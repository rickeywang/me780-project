#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from numpy import *
import matplotlib.pyplot as pp
import matplotlib.image as mpimg
from matplotlib import animation
from pylab import *
import message_filters


from oc import *

PKG = 'beginner_tutorials'
roslib.load_manifest(PKG)

M_global = 100
N_global = 100

odom_x = 0.0
odom_y = 0.0
odom_theta = 0


pp.ion()

def setup_og():
    # Occupancy grid in both probablity and log odds form
    global og_global
    og_global = 0.5 * ones((M_global, N_global))
    ogl0 = log(og_global/(1 - og_global))
    global ogl
    ogl=ogl0

def imu_callback(data):
    #   print data
    #print rospy.get_name(), "Linear Accel X: %s" % str(data.linear_acceleration.x)
    data = data


def odom_callback(data):
    odom_x = (data.pose.pose.position.x * 100)
    odom_y = data.pose.pose.position.y * 100
    w = data.pose.pose.orientation.w
    odom_theta = 4*pi - 2*arccos(w) #Only z axis has rotation
    #print "Odom X:%s" % odom_x


def laser_callback(msg):

    r_m = msg.ranges
    # Remove NaN
    #r_m = r_m[~isnan(r_m)]
    phi_m = arange(msg.angle_min,msg.angle_max,msg.angle_increment)
    r_max = msg.range_max

    #state from odom
    odom_msg =  odom_cache.getElemBeforeTime(odom_cache.getLastestTime())
    odom_x = (odom_msg.pose.pose.position.x * 100)
    odom_y = (odom_msg.pose.pose.position.y * 100)
    w = odom_msg.pose.pose.orientation.w
    odom_theta = 4 * pi - 2 * arccos(w)
    #print "Odom X:%s" % odom_x
    state = c_[odom_x, odom_y, odom_theta]
    state = state[0]

    ogout = ogmap(M_global, N_global, ogl, state, phi_m, r_m, r_max)

    og_global = ogout['og']

    aximg = plt.imshow(og_global)
    plt.show()
    plt.pause(0.0001)



def map_callback(msg):
    # Called when new map is received
    ogmap = msg.data
    # Now, use the laser info to update this map!



def listener():
    rospy.init_node('numpy_listener')
    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    #rospy.Subscriber("/odom", Odometry, odom_callback)
    #rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    # rospy.Subscriber("floats", Floats, callback)
    odom_sub = message_filters.Subscriber("/odom", Odometry)
    global odom_cache
    odom_cache = message_filters.Cache(odom_sub)
    # scan_sub = message_filters.Subscriber("/scan", LaserScan)
    # ts = message_filters.TimeSynchronizer([odom_sub, scan_sub], 10)
    # ts.registerCallback(laser_callback)
    rospy.spin()


if __name__ == '__main__':
    setup_og()
    listener()
