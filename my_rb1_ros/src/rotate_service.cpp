#include <cmath>
#include <geometry_msgs/Twist.h>
#include <my_rb1_message/Rotate.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class RotateMyRB1Robot {
public:
  RotateMyRB1Robot() {
    this->service = node.advertiseService(
        "/rotate_robot", &RotateMyRB1Robot::service_callback, this);
    this->subscriber = node.subscribe(
        "/odom", 1000, &RotateMyRB1Robot::subscriber_callback, this);
    this->publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ROS_INFO("The service /rotate_robot is available for request.");
  }

private:
  // ros objects
  ros::NodeHandle node;
  ros::ServiceServer service;
  ros::Publisher publisher;
  ros::Subscriber subscriber;

  // msg variables
  double yaw;

  bool service_callback(my_rb1_message::Rotate::Request &request,
                        my_rb1_message::Rotate::Response &response) {
    ROS_INFO("/rotate_robot request (degress : %d)", request.degrees);

    // direction of movement and angular distance (theta) in radians
    double direction = request.degrees / std::abs(request.degrees);
    double theta = std::abs((M_PI / 180) * request.degrees);

    // intial values of odometer
    double yaw_prime = this->yaw;
    double theta_covered = 0.0;

    // start robot movement
    // todo : damping can be used for speed control
    publisher_call(0.1, direction);
    // validate angular distance covered
    while (theta_covered < theta) {
      theta_covered += std::abs(this->yaw - yaw_prime);
      yaw_prime = this->yaw;
      ros::spinOnce();
    }
    // stop robot movement
    publisher_call(0);

    // return service call
    ROS_INFO("target %f, direction %f, movement %f", theta, direction,
             theta_covered);
    response.result = "success : rotation completed";
    return true;
  }

  void subscriber_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    // reading current position from /odom topic
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    // convert quaternion into euler angles
    // as an alternatie tf can also be used.
    this->yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  void publisher_call(double angular_z = 0.1, double direction = -1) {
    // publishing velocity along with direction to the topic /cmd_vel
    geometry_msgs::Twist velocity;
    velocity.angular.z = direction * angular_z;
    publisher.publish(velocity);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "service_rotate_robot");
  RotateMyRB1Robot rotate;
  ros::spin();
  return 0;
}