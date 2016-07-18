/*
 * Author: Christian Lee
 * - Information is published as a twist message whereas the 'linear' would
 *   contain [x=X y=Y z=Yaw] and the 'angular' would be [x=Vel y=AngVel z=N/A].
 *   This is to mirror the actual state estimator node
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/tf.h>

#define TURTLE 6 // the index to pull the turtle from the gazebo array
#define LOOP_RATE 10 // Hz. The husky does not move that quickly, thus a lower frequency is acceptable
#define POSITION_ERROR 1
#define ANG_ERROR 0.5
#define TOLERANCE 0.0001
#define YAW 2 // Yaw index in the Eigen::Vector3d
#define PI 3.141592654
#define HARDCODED_VEL 0.5

float x = 0;
float y = 0;
double yaw = 0;
// Two dummy variables
double roll = 0;
double pitch = 0;

double vel = 0;
double angVel = 0;

bool posInit = false;

// keep the angle between -pi and pi
double angle_wrap( double angle)
{
  while(angle<-PI || angle > PI)
  {
    if (angle < -PI)
    {
      angle = angle+2*PI;
    }
    else if (angle > PI)
    {
      angle = angle-2*PI;
    }
  }
  return angle;
}

void position_callback (const gazebo_msgs::ModelStates::ConstPtr turtleState)
{
  x = turtleState->pose[TURTLE].position.x;
  y = turtleState->pose[TURTLE].position.y;
  tf::Quaternion orientationQuat = tf::Quaternion (turtleState->pose[TURTLE].orientation.x, turtleState->pose[TURTLE].orientation.y, turtleState->pose[TURTLE].orientation.z, turtleState->pose[TURTLE].orientation.w);
  tf::Matrix3x3 orientationMat(orientationQuat);
  orientationMat.getRPY(roll, pitch, yaw);

  yaw = angle_wrap(yaw);
  posInit = true;
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "turtle_position");
  ros::NodeHandle node;
  // it publishes estimated_state because this is what the controller
  // will look for
  ros::Publisher estimatedPosition = node.advertise<geometry_msgs::TwistStamped>("/turtle/estimated_state", 1); 
  ros::Subscriber gazeboPose = node.subscribe("/gazebo/model_states", 1, position_callback);
 
  ros::Rate loopRate(LOOP_RATE);

  // Wait for the position to be initialized
  while (!posInit && ros::ok())
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  while (ros::ok())
  {
    // Use a twist message to deliver the state of the husky
    geometry_msgs::TwistStamped huskyXYYaw;
    huskyXYYaw.twist.linear.x = x;
    huskyXYYaw.twist.linear.y = y;
    huskyXYYaw.twist.linear.z = yaw;
    huskyXYYaw.twist.angular.x = vel;
    huskyXYYaw.twist.angular.y = angVel;
    estimatedPosition.publish(huskyXYYaw);
    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
