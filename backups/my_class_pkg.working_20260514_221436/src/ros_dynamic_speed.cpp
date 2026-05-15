#include <string>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <my_class_pkg/TutorialsConfig.h>
#include <ros/ros.h>

namespace
{
double g_robot_speed = 0.2;

void reconfigureCallback(my_class_pkg::TutorialsConfig& config, uint32_t /* level */)
{
  g_robot_speed = config.double_param;
  ROS_INFO(
      "Reconfigure Request: int=%d double=%.3f str=%s bool=%s size=%d",
      config.int_param,
      config.double_param,
      config.str_param.c_str(),
      config.bool_param ? "True" : "False",
      config.size);
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_dynamic_speed_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string cmd_vel_topic = "/cmd_vel";
  pnh.param("cmd_vel_topic", cmd_vel_topic, cmd_vel_topic);

  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);

  dynamic_reconfigure::Server<my_class_pkg::TutorialsConfig> server;
  dynamic_reconfigure::Server<my_class_pkg::TutorialsConfig>::CallbackType callback;
  callback = boost::bind(&reconfigureCallback, _1, _2);
  server.setCallback(callback);

  ROS_INFO("Publishing dynamic speed commands to %s", cmd_vel_topic.c_str());

  ros::Rate rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = g_robot_speed;
    cmd_pub.publish(cmd_vel);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
