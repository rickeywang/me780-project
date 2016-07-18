/**
 * This node is supposed to receive the measurements from the sensors and
 * estimate the state using a Kalman filter.
 * - The state is defined as a vector with 3 values:
 *   [X-position, Y-position, Yaw]
 * - Information is published as a twist message whereas the 'linear' would
 *   contain [x=X y=Y z=Yaw] 
 *   
 */

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>

#define LOOP_RATE 20
#define PI 3.14159265359

using namespace Eigen;

bool newOdomRead = false;
bool newTeleopRead = false;

/**
 * Extended Kalman filter is required. Therefore, initialize a state transition
 * matrix as well as a linearized state transition matrix (A and G respectively)
 */
Eigen::MatrixXd stateTrans = Eigen::MatrixXd::Identity(5,5); // alias A matrix
Eigen::MatrixXd stateTransL = Eigen::MatrixXd::Identity(5,5); // alias G matrix

Eigen::MatrixXd measureCov = Eigen::MatrixXd::Identity(5,5); // alias Q matrix
Eigen::MatrixXd predictCov = Eigen::MatrixXd::Identity(5,5); // alias R matrix
Eigen::VectorXd measureVect = Eigen::VectorXd::Zero(5); // measurements from the odom
//Eigen::VectorXd inputVect = Eigen::VectorXd::Zero(5);

VectorXd mean = Eigen::VectorXd::Zero(5);
VectorXd pred_mean = Eigen::VectorXd::Zero(5);
MatrixXd sigma = Eigen::MatrixXd::Identity(5,5)*0.01;
MatrixXd pred_sigma = Eigen::MatrixXd::Identity(5,5)*0.01;
bool meas_update_last;

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

void odom_callback (const nav_msgs::Odometry& odomMsg)
{
  // save the most recent odometry reading
  measureVect(3) = odomMsg.twist.twist.linear.x;
  measureVect(4) = odomMsg.twist.twist.angular.z;
  newOdomRead = true;
}

/*
void teleop_callback (const geometry_msgs::Twist& teleopMsg)
{
  // save the most recent teleop reading
  inputVect(3) = teleopMsg.linear.x;
  inputVect(4) = teleopMsg.angular.z;
  newTeleopRead = true;
}*/

void ekf_measurement_update(MatrixXd matr_H, VectorXd meas_vector,
	    MatrixXd meas_cov)
{
  VectorXd innovation(matr_H.rows());
  innovation = meas_vector - matr_H * pred_mean;
	
  MatrixXd temp(matr_H.cols(), matr_H.rows());
  temp = matr_H * pred_sigma * matr_H.transpose() + meas_cov;
  
  MatrixXd matr_K(matr_H.cols(), matr_H.rows());
  matr_K = pred_sigma * matr_H.transpose() * temp.inverse();
  
  mean = pred_mean + matr_K*(innovation);
  sigma = (MatrixXd::Identity(sigma.rows(),sigma.rows())
		- matr_K*matr_H) * pred_sigma;
}

void ekf_prediction_update(MatrixXd matr_A, MatrixXd matr_G,
    VectorXd B, MatrixXd pred_cov)
{

  pred_sigma = matr_G*sigma*matr_G.transpose() + pred_cov;

  // X[t] = AX[t-1] + B*u;
  pred_mean = matr_A * mean + B;

  mean = pred_mean;
  sigma = pred_sigma;
}

int main(int argc, char **argv)
{
  // setup Kalman gains...
  measureCov *= 3.5; // m, m, rad, m/s, rad/s
  // odom values are trusted, but the rotation measurement is noisy
  measureCov(2,2) = 0.1;
  measureCov(4,4) = 0.5;
  predictCov *= 0.01; // m, m, rad, m/s, rad/s

  double t1 = 0;
  double t2 = 0;
  double dt = 0;
  double initX = 0;
  double initY = 0;

  ros::init(argc, argv, "state_estimator");
  ros::NodeHandle node;
  ros::Publisher turtleState = node.advertise<geometry_msgs::Twist>("/turtlebot/estimated_state",1);
  ros::Subscriber odomValues = node.subscribe("/odom", 1, odom_callback);
  //ros::Subscriber teleoValues = node.subscribe("/cmd_vel_mux/input/teleop", 1000, teleop_callback);
  ros::Rate loopRate(LOOP_RATE);

  // initialize t1, because t1 is used for a differential time measurement
  t1 = ros::Time::now().toSec();

  while(ros::ok())
  {
    // get new messages
    ros::spinOnce();

    geometry_msgs::Twist estimatedState;
    // set time for motion update
    t2 = ros::Time::now().toSec();
    dt = t2-t1;
    t1 = t2;

    // prepare state transition matrix
    stateTrans(0,3) = cos(mean(2))*dt;
    stateTrans(1,3) = sin(mean(2))*dt;
    stateTrans(2,4) = dt;

    // prepare a linearized state transition matrix (Jacobian)
    stateTransL = stateTrans;
    stateTransL(0,2) = -mean(3)*dt*sin(mean(2));
    stateTransL(1,2) = -mean(3)*dt*cos(mean(2));

    Eigen::VectorXd B = Eigen::VectorXd::Zero(5);

    // run a prediction update (propagate motion model)
    ekf_prediction_update(stateTrans, stateTransL, B, predictCov);

    if (newOdomRead /*|| newKinectRead*/) // check for any new measurements
    {
      // fill in this matrix based on what measurements have been returned
      Eigen::MatrixXd C = Eigen::MatrixXd::Zero(5,5);
		
	  /*if (newKinectRead)
	  {
		  // the kinect readings measures relative displacement
		  C(0,0) = 1;
		  C(1,1) = 1;
		  C(2,2) = 1;
		  newKinectRead = false;
	  }*/

      if (newOdomRead) // odometry measurement received
      {
        // the odometry readings measures the vel and ang vel
        C(3,3) = 1;
        C(4,4) = 1;
        newOdomRead = false;
      }

      	// perform measurement update with whichever measurements we received
	ekf_measurement_update(C, measureVect, measureCov);
    }

	
    /**
     * publish most updated mean (the source code was modified to always
     * write the best prediction into the 'mean')
     */
    mean(2) = angle_wrap(mean(2));
    estimatedState.linear.x = mean(0);
    estimatedState.linear.y = mean(1);
    estimatedState.linear.z = mean(2);
    estimatedState.angular.x = mean(3);
    estimatedState.angular.z = mean(4);

    turtleState.publish(estimatedState);
    loopRate.sleep();
  }

  return 0;
}
