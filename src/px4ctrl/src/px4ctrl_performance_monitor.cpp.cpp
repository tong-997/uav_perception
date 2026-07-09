// px4ctrl_performance_monitor.cpp
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>

class PX4CtrlMonitor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber traj_sub_, odom_sub_, att_cmd_sub_;
    ros::Publisher error_pub_;
    
    trajectory_msgs::JointTrajectoryPoint current_desired_;
    bool has_desired_;
    
public:
    PX4CtrlMonitor() {
        traj_sub_ = nh_.subscribe("/px4ctrl/trajectory", 10, &PX4CtrlMonitor::trajCallback, this);
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &PX4CtrlMonitor::odomCallback, this);
        att_cmd_sub_ = nh_.subscribe("/mavros/setpoint_raw/attitude", 10, &PX4CtrlMonitor::attCmdCallback, this);
        error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/px4ctrl/tracking_error", 10);
        
        has_desired_ = false;
    }
    
    void trajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
        if (!msg->points.empty()) {
            current_desired_ = msg->points[0];
            has_desired_ = true;
        }
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!has_desired_) return;
        
        // 计算位置跟踪误差
        Eigen::Vector3d desired_pos(current_desired_.positions[0],
                                   current_desired_.positions[1],
                                   current_desired_.positions[2]);
        
        Eigen::Vector3d actual_pos(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z);
        
        Eigen::Vector3d pos_error = desired_pos - actual_pos;
        
        // 计算速度跟踪误差
        Eigen::Vector3d desired_vel(current_desired_.velocities[0],
                                   current_desired_.velocities[1],
                                   current_desired_.velocities[2]);
        
        Eigen::Vector3d actual_vel(msg->twist.twist.linear.x,
                                  msg->twist.twist.linear.y,
                                  msg->twist.twist.linear.z);
        
        Eigen::Vector3d vel_error = desired_vel - actual_vel;
        
        // 发布误差信息
        geometry_msgs::Vector3Stamped error_msg;
        error_msg.header.stamp = ros::Time::now();
        error_msg.vector.x = pos_error.norm();  // 位置误差模长
        error_msg.vector.y = vel_error.norm();  // 速度误差模长
        error_msg.vector.z = 0.0;  // 可用于其他指标
        
        error_pub_.publish(error_msg);
        
        ROS_INFO_THROTTLE(2.0, "PX4Ctrl Tracking - Pos Error: %.3fm, Vel Error: %.3fm/s", 
                         pos_error.norm(), vel_error.norm());
    }
    
    void attCmdCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
        // 监控 px4ctrl 输出的控制指令
        ROS_INFO_THROTTLE(2.0, "PX4Ctrl Output - Thrust: %.3f, Attitude: (%.3f, %.3f, %.3f, %.3f)",
                         msg->thrust, msg->orientation.x, msg->orientation.y, 
                         msg->orientation.z, msg->orientation.w);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "px4ctrl_performance_monitor");
    PX4CtrlMonitor monitor;
    ros::spin();
    return 0;
}