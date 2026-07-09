#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <human_follow_rknn/TargetInfo.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <cmath>

class PersonFollowController
{
public:
  PersonFollowController(ros::NodeHandle& nh)
  {
    nh.param("k_yaw", k_yaw_, 0.8);
    nh.param("k_v",   k_v_,   2.0);
    nh.param("desired_h_norm", desired_h_norm_, 0.3);
    nh.param("follow_height",  z_hold_, 3.0);
    nh.param("v_max", v_max_, 2.0);
    nh.param("yaw_rate_max", yaw_rate_max_, 0.8);
    nh.param("xy_bound", xy_bound_, 10.0);

    pose_sub_ = nh.subscribe("/mavros/local_position/pose", 1,
                             &PersonFollowController::poseCb, this);
    target_sub_ = nh.subscribe("person_target", 1,
                               &PersonFollowController::targetCb, this);
    cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);

    has_pose_ = false;
    ref_initialized_ = false;
    last_time_ = ros::Time::now();
  }

private:
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    has_pose_ = true;
    cur_pos_[0] = msg->pose.position.x;
    cur_pos_[1] = msg->pose.position.y;
    cur_pos_[2] = msg->pose.position.z;

    const auto& q = msg->pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    cur_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    if(!ref_initialized_)
    {
      ref_pos_[0] = cur_pos_[0];
      ref_pos_[1] = cur_pos_[1];
      ref_pos_[2] = z_hold_;
      home_pos_[0] = cur_pos_[0];
      home_pos_[1] = cur_pos_[1];
      home_pos_[2] = cur_pos_[2];
      ref_initialized_ = true;
    }
  }

  void targetCb(const human_follow_rknn::TargetInfo::ConstPtr& msg)
  {
    if(!has_pose_ || !ref_initialized_) return;

    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    last_time_ = now;
    if(dt <= 0.0) dt = 0.01;

    if(!msg->target_acquired)
    {
      // 没有目标：保持当前位置不变（实际上就是悬停）
      publishCmd(ref_pos_);
      return;
    }

    double e_yaw  = msg->u_norm;
    double e_dist = desired_h_norm_ - msg->box_height_norm;

    double yaw_rate = -k_yaw_ * e_yaw;
    double vx_body  =  k_v_   * e_dist;
    double vy_body  = 0.0;

    // 限幅
    if(vx_body > v_max_) vx_body = v_max_;
    if(vx_body < -v_max_) vx_body = -v_max_;
    if(yaw_rate > yaw_rate_max_) yaw_rate = yaw_rate_max_;
    if(yaw_rate < -yaw_rate_max_) yaw_rate = -yaw_rate_max_;

    double cy = std::cos(cur_yaw_);
    double sy = std::sin(cur_yaw_);
    double vx_world = cy * vx_body - sy * vy_body;
    double vy_world = sy * vx_body + cy * vy_body;

    ref_pos_[0] += vx_world * dt;
    ref_pos_[1] += vy_world * dt;
    ref_pos_[2]  = z_hold_;

    // 限制在 home 附近
    double dx = ref_pos_[0] - home_pos_[0];
    double dy = ref_pos_[1] - home_pos_[1];
    double r  = std::hypot(dx, dy);
    if(r > xy_bound_)
    {
      double scale = xy_bound_ / r;
      ref_pos_[0] = home_pos_[0] + dx * scale;
      ref_pos_[1] = home_pos_[1] + dy * scale;
    }

    publishCmd(ref_pos_);
  }

  void publishCmd(const double pos[3])
  {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.position.x = pos[0];
    cmd.position.y = pos[1];
    cmd.position.z = pos[2];
    cmd.yaw        = cur_yaw_;
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.jerk.x = cmd.jerk.y = cmd.jerk.z = 0.0;
    cmd.snap.x = cmd.snap.y = cmd.snap.z = 0.0;
    cmd.yaw_dot = 0.0;
    cmd.trajectory_status = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd_pub_.publish(cmd);
  }

  ros::Subscriber pose_sub_;
  ros::Subscriber target_sub_;
  ros::Publisher  cmd_pub_;

  bool has_pose_;
  bool ref_initialized_;
  double cur_pos_[3];
  double ref_pos_[3];
  double home_pos_[3];
  double cur_yaw_;
  ros::Time last_time_;

  double k_yaw_, k_v_, desired_h_norm_, z_hold_;
  double v_max_, yaw_rate_max_, xy_bound_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_follow_px4ctrl_node");
  ros::NodeHandle nh("~");
  PersonFollowController ctrl(nh);
  ros::spin();
  return 0;
}
