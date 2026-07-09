#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    //初始化节点
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");
    //检测终止信号
    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();
    //初始化参数类，从参数服务器读取参数
    Parameter_t param;
    param.config_from_ros_handle(nh);

    // Controller controller(param);
    // 初始化线性控制器
    LinearControl controller(param);
    //	初始化状态机
    PX4CtrlFSM fsm(param, controller);

    //订阅当前状态，以参数传给fsm.state_data.feed()		
    //接收状态消息
    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
    //订阅当前扩展状态，以参数传给fsm.extened_state_data.feed()	
    //接收扩展状态消息
    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));
    //订阅里程计话题odmo，由/vins_fusion/imu_propagate映射而来
    //回调函数	->读取位置、速度、四元数数据传入fsm.odmo_data，监控数据帧率，小于100hz警告
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    //订阅位置命令话题cmd	，由/position_cmd映射而来
    //回调函数	->读取位置、速度、加速度、加加速度(jerk)、yaw角给fsm.cmd_data
    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());
    //  订阅imu数据话题	，/mavros/imu/data
    //  回调函数	->读取姿态、角速度、角加速度给fsm.imu_data
    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    /*接收机通道定义
    ch[0]:	横滚
    ch[1]:	俯仰
    ch[2]:	油门
    ch[3]:	偏航
    开关  打到最上面为0，打到最下面为1.0
	两档：0~1.0
	三档：0~0.5~1.0
    ch[4]:	5通道，飞行模式，三档  0 手动模式，1.定点模式 2.指令控制模式
    ch[5]:	6通道，命令控制，两档
    ch[7]:	8通道，用于重启px4
    */
    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }
    // 	订阅电池状态话题	，/mavros/battery
	//回调函数	->读取电压、计算百分比并打印
    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    // 订阅起飞命令话题，takeoff_land
	// 回调函数	->fsm.takeoff_land_data.triggered = 1，接收起飞命令
    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());
    
    // /mavros/setpoint_raw/attitude 对应有两种方式控制：1.姿态+油门；2.机体角速度+油门
    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    //首先飞机要处在定点模式下，此时把6通道拨杆从控制指令拒绝状态拨到使能状态，此刻px4ctrl会发一个`/traj_start_trigger`出来，
    // 当下还尚未切换到指令控制模式，px4ctrl会等待外部指令，一旦接收到指令，模式就会切换，屏幕也会打印绿色的"[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)" 
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    // debug信息 作为日志信息记录的
    fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug
     // mavros通信，模式切换
    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // mavros通信，解锁与上锁
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // 重启
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Duration(0.5).sleep();
    //判断从参数服务器读过来的参数设置是否需要遥控：1不需要，0需要
    if (param.takeoff_land.no_RC)
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");//不需要，警告：遥控失效，小心
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC"); //需要：等待接收机信号
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }
    //进入死循环，检查px4的连接，连接正常跳出循环
    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }
    //主循环，以固定帧率进入fsm.process()进程
    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}
