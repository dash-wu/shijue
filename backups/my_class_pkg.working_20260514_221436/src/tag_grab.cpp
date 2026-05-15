#include <cstdio>
#include <cmath>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <upros_message/ArmPosition.h>
#include <upros_message/TagCommand.h>

namespace
{
std::mutex g_queue_mutex;
std::queue<int> g_target_queue;
ros::Publisher g_status_pub;

static void publish_status(const std::string& msg)
{
  std_msgs::String s;
  s.data = msg;
  if (g_status_pub)
  {
    g_status_pub.publish(s);
  }
  ROS_INFO("%s", msg.c_str());
}

void cmd_callback(const upros_message::TagCommand::ConstPtr& msg)
{
  if (msg->intent != "pick")
  {
    ROS_WARN("tag_grab_node 仅处理抓取(pick)，忽略 intent=%s", msg->intent.c_str());
    return;
  }
  if (msg->target != 1 && msg->target != 2)
  {
    ROS_WARN("仅支持抓取一号/二号 (target=1 或 2)，收到 target=%d", msg->target);
    return;
  }
  std::lock_guard<std::mutex> lock(g_queue_mutex);
  g_target_queue.push(msg->target);
  publish_status("已加入抓取队列 target=" + std::to_string(msg->target) + " intent=" + msg->intent);
  ROS_INFO("已加入抓取队列: target=%d (AprilTag id=%d → frame tag_%d)", msg->target, msg->target,
           msg->target);
}

static void sleep_s(double second)
{
  ros::Duration(second).sleep();
}

static std::string tag_frame_for_target(const std::string& format, int target)
{
  char buf[128];
  if (std::snprintf(buf, sizeof(buf), format.c_str(), target) < 0)
    return "tag_" + std::to_string(target);
  return std::string(buf);
}

static double dist_mm(int ax, int ay, int az, int bx, int by, int bz)
{
  const double dx = static_cast<double>(ax - bx);
  const double dy = static_cast<double>(ay - by);
  const double dz = static_cast<double>(az - bz);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/** TF: arm_base_link <- tag_link，与历史 apriltag 例程一致的毫米目标（逆解输入） */
static bool lookup_target_mm(tf2_ros::Buffer& buffer, const std::string& tag_link, int offset_y_mm,
                             int offset_z_mm, int& out_x, int& out_y, int& out_z)
{
  geometry_msgs::TransformStamped tfs;
  try
  {
    tfs = buffer.lookupTransform("arm_base_link", tag_link, ros::Time(0), ros::Duration(0.6));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "lookupTransform arm_base_link <- %s: %s", tag_link.c_str(), ex.what());
    return false;
  }
  const double tx = tfs.transform.translation.x;
  const double ty = tfs.transform.translation.y;
  const double tz = tfs.transform.translation.z;
  out_x = static_cast<int>(-ty * 1000.0);
  out_y = static_cast<int>(tx * 1000.0) + offset_y_mm;
  out_z = static_cast<int>(tz * 1000.0) + offset_z_mm;
  return true;
}

static bool call_move_open(ros::ServiceClient& arm_move_open_client, int x, int y, int z)
{
  upros_message::ArmPosition move_srv;
  move_srv.request.x = static_cast<float>(x);
  move_srv.request.y = static_cast<float>(y);
  move_srv.request.z = static_cast<float>(z);
  if (!arm_move_open_client.call(move_srv))
  {
    ROS_WARN("arm_pos_service_open 调用失败");
    return false;
  }
  // 新版 control_center 在逆解失败时填 resp.status=0 且不调用关节运动；此处仅作提示（旧版未填 status 时不能据此判失败）
  if (move_srv.response.status == 0)
  {
    ROS_WARN(
        "arm_pos_service_open 返回 status=0（逆解可能未成功）。请重新编译 upros_arm 后查看 control_center 日志；或调高 "
        "~offset_z_mm / ~min_z_mm。");
  }
  return true;
}

/**
 * 等待标签进入视野，再在机械臂运动后反复用 TF 修正目标点（视觉位姿 → 手眼闭环）。
 */
static void run_grab_sequence(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_ros::Buffer& buffer,
                              ros::ServiceClient& arm_move_open_client, ros::ServiceClient& arm_zero_client,
                              ros::ServiceClient& arm_grab_client, int target_tag)
{
  int offset_y_mm = 30;
  int offset_z_mm = 100;
  int min_z_mm = 65;
  double tag_wait_sec = 30.0;
  int servo_max_iters = 22;
  double servo_settle_sec = 0.38;
  double servo_converge_mm = 12.0;
  int max_step_mm = 90;
  std::string tag_frame_format("tag_%d");
  std::string tag_frame_format_alt("tag_36h11_%d");

  pnh.param("offset_y_mm", offset_y_mm, 30);
  pnh.param("offset_z_mm", offset_z_mm, 100);
  pnh.param("min_z_mm", min_z_mm, 65);
  pnh.param("tag_wait_sec", tag_wait_sec, 30.0);
  pnh.param("servo_max_iters", servo_max_iters, 22);
  pnh.param("servo_settle_sec", servo_settle_sec, 0.38);
  pnh.param("servo_converge_mm", servo_converge_mm, 12.0);
  pnh.param("max_step_mm", max_step_mm, 90);
  pnh.param("tag_frame_format", tag_frame_format, std::string("tag_%d"));
  pnh.param("tag_frame_format_alt", tag_frame_format_alt, std::string("tag_36h11_%d"));

  std::vector<std::string> tag_candidates;
  tag_candidates.push_back(tag_frame_for_target(tag_frame_format, target_tag));
  if (!tag_frame_format_alt.empty())
  {
    tag_candidates.push_back(tag_frame_for_target(tag_frame_format_alt, target_tag));
  }
  publish_status("开始抓取: 将依次尝试 TF 帧: " + tag_candidates[0] +
                 (tag_candidates.size() > 1 ? " , " + tag_candidates[1] : ""));

  std::string tag_link;
  // 1) 等待 TF（AprilTag 可能发布为 tag_1 或 tag_36h11_1 等）
  const ros::Time wait_until = ros::Time::now() + ros::Duration(tag_wait_sec);
  int x = 0, y = 0, z = 0;
  bool have_pose = false;
  ros::Time last_hint = ros::Time::now();
  while (ros::ok() && ros::Time::now() < wait_until)
  {
    for (const auto& candidate : tag_candidates)
    {
      if (lookup_target_mm(buffer, candidate, offset_y_mm, offset_z_mm, x, y, z))
      {
        tag_link = candidate;
        have_pose = true;
        publish_status("已找到 TF 帧: " + tag_link + "，目标(mm) X=" + std::to_string(x) + " Y=" +
                       std::to_string(y) + " Z=" + std::to_string(z));
        break;
      }
    }
    if (have_pose)
      break;
    const double remain = (wait_until - ros::Time::now()).toSec();
    if ((ros::Time::now() - last_hint).toSec() > 3.0)
    {
      last_hint = ros::Time::now();
      ROS_WARN(
          "仍在等待 TF: arm_base_link <- %s (约 %.0fs 内超时)。若一直如此：请把标签放在相机前、"
          "确认 tags.yaml 里含对应 id；若 TF 名不同请设 ~tag_frame_format；"
          "自检: rosrun tf tf_echo arm_base_link %s",
          tag_link.c_str(), std::max(0.0, remain), tag_link.c_str());
    }
    ros::Duration(0.25).sleep();
  }
  if (!have_pose)
  {
    publish_status("错误: 超时仍无 TF（已试 " + std::to_string(tag_candidates.size()) +
                   " 种帧名）。请把标签放相机前，或 rostopic echo /tf | grep tag 看实际帧名后设置 "
                   "~tag_frame_format / ~tag_frame_format_alt");
    ROS_ERROR(
        "超时未检测到 tag TF — 机械臂不会动。已尝试多种帧名；请确认 AprilTag 在视野内且 TF 链完整。");
    return;
  }

  if (z < min_z_mm)
  {
    ROS_WARN("目标 Z=%d mm 低于逆解可用高度，已钳位到 min_z_mm=%d（标签在 TF 中往往偏「低」，可调 ~offset_z_mm）", z,
             min_z_mm);
    publish_status("提示: Z 过低已钳位到 " + std::to_string(min_z_mm) + " mm");
    z = min_z_mm;
  }

  ROS_INFO("已检测到标签(%s)，初始目标(mm): X=%d Y=%d Z=%d — 开始视觉微调", tag_link.c_str(), x, y, z);

  // 2) 迭代：移动到估计点 → 静止后再次观测 → 偏差小于阈值则收敛
  int prev_x = x, prev_y = y, prev_z = z;
  for (int iter = 0; iter < servo_max_iters && ros::ok(); ++iter)
  {
    int cx = x, cy = y, cz = z;
    if (cz < min_z_mm)
    {
      cz = min_z_mm;
    }
    // 限制单步位移，避免一次跳变过大
    if (iter > 0)
    {
      const int dx = cx - prev_x;
      const int dy = cy - prev_y;
      const int dz = cz - prev_z;
      const double step = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
      if (step > static_cast<double>(max_step_mm))
      {
        const double s = static_cast<double>(max_step_mm) / step;
        cx = prev_x + static_cast<int>(dx * s);
        cy = prev_y + static_cast<int>(dy * s);
        cz = prev_z + static_cast<int>(dz * s);
      }
    }

    ROS_INFO("调用 arm_pos_service_open: X=%d Y=%d Z=%d (mm)", cx, cy, cz);
    publish_status("调用 arm_pos_service_open: X=" + std::to_string(cx) + " Y=" + std::to_string(cy) +
                     " Z=" + std::to_string(cz) + " (mm)");
    if (!call_move_open(arm_move_open_client, cx, cy, cz))
    {
      publish_status("错误: arm_pos_service_open 调用失败，机械臂未动");
      return;
    }
    sleep_s(servo_settle_sec);

    if (!lookup_target_mm(buffer, tag_link, offset_y_mm, offset_z_mm, x, y, z))
    {
      ROS_WARN_THROTTLE(0.5, "微调过程中短暂丢失 %s，继续尝试", tag_link.c_str());
      continue;
    }
    if (z < min_z_mm)
    {
      z = min_z_mm;
    }

    const double err = dist_mm(x, y, z, prev_x, prev_y, prev_z);
    ROS_INFO("视觉微调 iter=%d 目标(mm): X=%d Y=%d Z=%d 与上次偏差=%.1f mm", iter + 1, x, y, z, err);

    if (iter > 0 && err < servo_converge_mm)
    {
      int fx = x, fy = y, fz = z;
      if (fz < min_z_mm)
      {
        fz = min_z_mm;
      }
      // 最终再到位一次
      call_move_open(arm_move_open_client, fx, fy, fz);
      sleep_s(servo_settle_sec);
      ROS_INFO("视觉微调收敛，执行抓取");
      publish_status("视觉微调收敛，即将闭合夹爪");
      break;
    }
    prev_x = x;
    prev_y = y;
    prev_z = z;
  }

  std_srvs::Empty empty_srv;
  publish_status("调用 grab_service 闭合夹爪");
  if (!arm_grab_client.call(empty_srv))
  {
    publish_status("警告: grab_service 调用失败");
    ROS_WARN("grab_service 调用失败");
  }
  sleep_s(3.0);

  publish_status("调用 zero_service 回零");
  if (!arm_zero_client.call(empty_srv))
  {
    publish_status("警告: zero_service 调用失败");
    ROS_WARN("zero_service 调用失败");
  }
  sleep_s(3.0);

  publish_status("抓取序列完成 target=" + std::to_string(target_tag));
  ROS_INFO("抓取序列完成 (target=%d)", target_tag);
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tag_grab_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  g_status_pub = nh.advertise<std_msgs::String>("/tag_grab_status", 20, true);
  publish_status("tag_grab_node 已连接 /tag_grab_status；等待 /voice_control (pick, target 1/2)");

  ros::Subscriber cmd_sub = nh.subscribe("/voice_control", 10, cmd_callback);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::ServiceClient arm_move_open_client =
      nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_open");
  ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
  ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");

  arm_move_open_client.waitForExistence(ros::Duration(30.0));
  arm_zero_client.waitForExistence(ros::Duration(10.0));
  arm_grab_client.waitForExistence(ros::Duration(10.0));

  ROS_INFO("tag_grab_node 就绪：请说「抓取一号」或「抓取二号」。AprilTag 帧名为 tag_1 / tag_2。");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok())
  {
    int next_target = 0;
    {
      std::lock_guard<std::mutex> lock(g_queue_mutex);
      if (g_target_queue.empty())
      {
        ros::Duration(0.05).sleep();
        continue;
      }
      next_target = g_target_queue.front();
      g_target_queue.pop();
    }
    run_grab_sequence(nh, pnh, tf_buffer, arm_move_open_client, arm_zero_client, arm_grab_client,
                      next_target);
  }

  return 0;
}
