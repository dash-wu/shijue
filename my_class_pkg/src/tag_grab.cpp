#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_srvs/Empty.h>
#include <upros_message/ArmPosition.h>

static void sleep_s(double second)
{
  ros::Duration(second).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tag_grab_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::ServiceClient arm_move_open_client =
      nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_open");
  ros::ServiceClient arm_zero_client =
      nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
  ros::ServiceClient arm_grab_client =
      nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
  ros::ServiceClient arm_release_client =
      nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

  arm_move_open_client.waitForExistence(ros::Duration(30.0));
  arm_zero_client.waitForExistence(ros::Duration(5.0));
  arm_grab_client.waitForExistence(ros::Duration(5.0));
  arm_release_client.waitForExistence(ros::Duration(5.0));

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ROS_INFO("Waiting for TF arm_base_link <- tag_1 ...");
  geometry_msgs::TransformStamped tfs;
  try
  {
    tfs = tf_buffer.lookupTransform("arm_base_link", "tag_1", ros::Time(0), ros::Duration(30.0));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_ERROR("TF lookup failed: %s", ex.what());
    ros::shutdown();
    return 1;
  }

  // 与实验说明一致：ROS 米 -> 逆解毫米及轴向约定
  const double tx = tfs.transform.translation.x;
  const double ty = tfs.transform.translation.y;
  const double tz = tfs.transform.translation.z;
  const float x = static_cast<float>(-ty * 1000.0);
  const float y = static_cast<float>(tx * 1000.0 + 30.0);
  const float z = static_cast<float>(tz * 1000.0 + 40.0);

  ROS_INFO("Target X: %.1f mm, Y: %.1f mm, Z: %.1f mm", x, y, z);

  std_srvs::Empty empty_srv;

  ROS_INFO("Step 1: release (open claw)");
  arm_release_client.call(empty_srv);
  sleep_s(5.0);

  ROS_INFO("Step 2: move to grasp pose");
  upros_message::ArmPosition move_srv;
  move_srv.request.x = x;
  move_srv.request.y = y;
  move_srv.request.z = z;
  if (!arm_move_open_client.call(move_srv))
    ROS_WARN("arm_pos_service_open call returned false");
  sleep_s(5.0);

  ROS_INFO("Step 3: grab");
  arm_grab_client.call(empty_srv);
  sleep_s(5.0);

  ROS_INFO("Step 4: zero");
  arm_zero_client.call(empty_srv);
  sleep_s(5.0);

  ros::shutdown();
  return 0;
}
