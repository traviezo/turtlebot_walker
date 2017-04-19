/** 
 * Copyright 2017 Christian Ramos
 *
 * @file turtlebot_walker.cpp 
 *
 * @brief A simple node to control Turtlebot motion. This program
 * will make turtlebot move forward until it sees an obstacle in
 * front of it at a distance ~0.6 meters. It will then rotate 90
 * degrees and look to see if its clear of obstacles. If not, it
 * will continue rotating until finds a clear path.
 *
 * @author Christian Ramos
 *
 * @date 04/17/2017
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

/// Defining pi constant
const double PI = 3.1415926535897;

class TurtleBotWalker {
    /// Creating a node handle
    ros::NodeHandle n;

    /// A publisher to publish velocity commands in x axis
    ros::Publisher pub;

    /// A subscriber to to Laser scan
    ros::Subscriber sub;

 public:
    /// Declaring a constructor
    TurtleBotWalker();

    /// Declaring a destructor
    ~TurtleBotWalker();

   /**
    * This callback function is called every time a laser scan msg is received.
    * This function will override the linear and angular velocity command messages 
    * based on distance thresholds from obstacles.It will then publish those
    * messages to the network.
    * @author Jafar Al-Kofahi 
    * @param msg laser scan message received from ROS network
    * @date 04/17/2017
    */
    void rotateCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

int main(int argc, char **argv) {
  /// Initializing the ROS node
  ros::init(argc, argv, "move_forward");

  /// Creating an instance of the TurtleBotWalker class
  TurtleBotWalker tbWalker;

  /// Instatiating a ROS object that sets the loop frequency
  ros::Rate rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

/// Implementing constructor
TurtleBotWalker::TurtleBotWalker() {
  /// Subscriber to laser scan definition
  sub = n.subscribe<sensor_msgs::LaserScan>("/scan", \
                                                  1, \
                &TurtleBotWalker::rotateCallback, this);

  /// Publisher object to advertise moving forward messages
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
}

/// Implementing an empty destructor
TurtleBotWalker::~TurtleBotWalker() {
}


void TurtleBotWalker::rotateCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  geometry_msgs::Twist command;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.linear.x = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;

  if (msg->ranges[160] <= 1) {
    command.angular.z = (PI/2);
  } else if (msg->ranges[480] <= 1) {
      command.angular.z = (-PI/2);
  } else if (msg->ranges[320] <= 0.7) {
      command.angular.z = (PI/2);
  } else {
      command.linear.x = 0.15;
      command.angular.z = 0.0;
  }

  pub.publish(command);
}

