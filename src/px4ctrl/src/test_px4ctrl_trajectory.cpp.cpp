// test_px4ctrl_trajectory.cpp
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>

class PX4CtrlTest {
private:
    ros::NodeHandle nh_;
    ros::Publisher traj_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;
    double test_start_time_;
    bool odom_received_;
    Eigen::Vector3d start_position_;
    
public:
    PX4CtrlTest() {
        traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/px4ctrl/trajectory", 10);
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &PX4CtrlTest::odomCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.02), &PX4CtrlTest::timerCallback, this);
        test_start_time_ = ros::Time::now().toSec();
        odom_received_ = false;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!odom_received_) {
            start_position_ << msg->pose.pose.position.x, 
                             msg->pose.pose.position.y, 
                             msg->pose.pose.position.z;
            odom_received_ = true;
            ROS_INFO("Received odometry, start position: (%.2f, %.2f, %.2f)", 
                    start_position_.x(), start_position_.y(), start_position_.z());
        }
    }
    
    void timerCallback(const ros::TimerEvent& e) {
        if (!odom_received_) return;
        
        double t = ros::Time::now().toSec() - test_start_time_;
        
        trajectory_msgs::JointTrajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.header.frame_id = "world";
        
        // 添加轨迹点
        trajectory_msgs::JointTrajectoryPoint point;
        
        if (t < 5.0) {
            // 测试1: 悬停测试 - 保持当前位置
            testHover(point, t);
        } else if (t < 10.0) {
            // 测试2: 位置阶跃响应
            testPositionStep(point, t);
        } else if (t < 15.0) {
            // 测试3: 正弦轨迹跟踪
            testSineTrajectory(point, t);
        } else if (t < 20.0) {
            // 测试4: 圆形轨迹
            testCircleTrajectory(point, t);
        } else if (t < 25.0) {
            // 测试5: 速度指令测试
            testVelocityCommand(point, t);
        } else {
            // 回到悬停
            testHover(point, t);
        }
        
        traj_msg.points.push_back(point);
        traj_pub_.publish(traj_msg);
    }
    
    void testHover(trajectory_msgs::JointTrajectoryPoint& point, double t) {
        // 10个double: [px, py, pz, vx, vy, vz, ax, ay, az, yaw]
        point.positions.resize(10);
        point.velocities.resize(10);
        point.accelerations.resize(10);
        
        // 位置: 保持起始位置
        point.positions[0] = start_position_.x();
        point.positions[1] = start_position_.y();
        point.positions[2] = start_position_.z();
        
        // 速度: 0
        point.velocities[0] = point.velocities[1] = point.velocities[2] = 0.0;
        
        // 加速度: 0
        point.accelerations[0] = point.accelerations[1] = point.accelerations[2] = 0.0;
        
        // 偏航角: 0
        point.positions[9] = 0.0;
        
        ROS_INFO_THROTTLE(2.0, "Testing: Hover at (%.2f, %.2f, %.2f)", 
                         start_position_.x(), start_position_.y(), start_position_.z());
    }
    
    void testPositionStep(trajectory_msgs::JointTrajectoryPoint& point, double t) {
        point.positions.resize(10);
        point.velocities.resize(10);
        point.accelerations.resize(10);
        
        // 位置: 向前2米
        point.positions[0] = start_position_.x() + 2.0;
        point.positions[1] = start_position_.y();
        point.positions[2] = start_position_.z() + 1.0;  // 上升1米
        
        // 速度: 0 (期望到达后静止)
        point.velocities[0] = point.velocities[1] = point.velocities[2] = 0.0;
        
        // 加速度: 0
        point.accelerations[0] = point.accelerations[1] = point.accelerations[2] = 0.0;
        
        // 偏航角: 0
        point.positions[9] = 0.0;
        
        ROS_INFO_THROTTLE(2.0, "Testing: Position Step to (%.2f, %.2f, %.2f)", 
                         point.positions[0], point.positions[1], point.positions[2]);
    }
    
    void testSineTrajectory(trajectory_msgs::JointTrajectoryPoint& point, double t) {
        point.positions.resize(10);
        point.velocities.resize(10);
        point.accelerations.resize(10);
        
        double freq = 0.5;  // 0.5 Hz
        double amplitude = 2.0;
        
        // X方向正弦运动
        point.positions[0] = start_position_.x() + amplitude * sin(2 * M_PI * freq * (t-10));
        point.positions[1] = start_position_.y();
        point.positions[2] = start_position_.z() + 1.0;
        
        // 速度
        point.velocities[0] = amplitude * 2 * M_PI * freq * cos(2 * M_PI * freq * (t-10));
        point.velocities[1] = 0.0;
        point.velocities[2] = 0.0;
        
        // 加速度
        point.accelerations[0] = -amplitude * pow(2 * M_PI * freq, 2) * sin(2 * M_PI * freq * (t-10));
        point.accelerations[1] = 0.0;
        point.accelerations[2] = 0.0;
        
        point.positions[9] = 0.0;
        
        ROS_INFO_THROTTLE(2.0, "Testing: Sine Trajectory, X=%.2f, Vx=%.2f", 
                         point.positions[0], point.velocities[0]);
    }
    
    void testCircleTrajectory(trajectory_msgs::JointTrajectoryPoint& point, double t) {
        point.positions.resize(10);
        point.velocities.resize(10);
        point.accelerations.resize(10);
        
        double radius = 2.0;
        double freq = 0.2;  // 0.2 Hz
        double angle = 2 * M_PI * freq * (t-15);
        
        // 圆形轨迹
        point.positions[0] = start_position_.x() + radius * cos(angle);
        point.positions[1] = start_position_.y() + radius * sin(angle);
        point.positions[2] = start_position_.z() + 1.0;
        
        // 速度 (切线方向)
        point.velocities[0] = -radius * 2 * M_PI * freq * sin(angle);
        point.velocities[1] = radius * 2 * M_PI * freq * cos(angle);
        point.velocities[2] = 0.0;
        
        // 加速度 (向心加速度)
        point.accelerations[0] = -radius * pow(2 * M_PI * freq, 2) * cos(angle);
        point.accelerations[1] = -radius * pow(2 * M_PI * freq, 2) * sin(angle);
        point.accelerations[2] = 0.0;
        
        // 偏航角朝向运动方向
        point.positions[9] = angle + M_PI/2;
        
        ROS_INFO_THROTTLE(2.0, "Testing: Circle Trajectory, Angle=%.1f deg", angle * 180/M_PI);
    }
    
    void testVelocityCommand(trajectory_msgs::JointTrajectoryPoint& point, double t) {
        point.positions.resize(10);
        point.velocities.resize(10);
        point.accelerations.resize(10);
        
        // 恒定速度指令
        point.positions[0] = start_position_.x();  // 位置会被忽略，主要看速度跟踪
        point.positions[1] = start_position_.y();
        point.positions[2] = start_position_.z() + 1.0;
        
        // 恒定速度
        point.velocities[0] = 1.0;  // 1 m/s 向前
        point.velocities[1] = 0.0;
        point.velocities[2] = 0.0;
        
        point.accelerations[0] = point.accelerations[1] = point.accelerations[2] = 0.0;
        point.positions[9] = 0.0;
        
        ROS_INFO_THROTTLE(2.0, "Testing: Velocity Command (%.1f, %.1f, %.1f) m/s", 
                         point.velocities[0], point.velocities[1], point.velocities[2]);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "px4ctrl_trajectory_test");
    PX4CtrlTest test;
    ros::spin();
    return 0;
}