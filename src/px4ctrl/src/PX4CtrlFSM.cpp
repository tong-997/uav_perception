#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

// u就是直接给px4的控制量，des是期望的位姿
//takeoff是在L1就实现的，在启动px4ctrl之前就把遥控器5、6通道拨到最下面，遥杆居中
// 状态识别	状态机的灵魂，值得借鉴！！！
// 整个状态机在做的就是根据模式选择相应的期望位姿des，后面会传给控制器
/* 这里将控制大致分为了三级：
		L1：手动控制
		L2：自动悬停
		L3：命令控制（跟踪轨迹）
		介于L1和L2之间的：自动起飞\降落，L3模式不允许自动降落

	5通道: 切换px4ctrl控制（Offboard模式）或飞控的原飞行模式
    6通道: 是否允许px4ctrl接收你的代码发给px4ctrl的控制指令
    7通道: 紧急停桨（Emergency Stop）
    8通道: 在飞控未解锁的状态下一键重启（飞控内选择ekf2估计器时常用）
		
*/


PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_) : param(param_), controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}

/* 
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/

void PX4CtrlFSM::process()
{

	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	Desired_State_t des(odom_data);
	bool rotor_low_speed_during_land = false;

	// STEP1: state machine runs
	switch (state)
	{
	case MANUAL_CTRL: //手动模式
	{
		if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER 尝试进入自动悬停模式 ，5通道刚刚打下来了
		{
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 3.0)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}

			state = AUTO_HOVER;//更新状态为AUTO_HOVER
			controller.resetThrustMapping();// 推力映射
			set_hov_with_odom();// 设置悬停位姿的位置、方向
			toggle_offboard_mode(true);//设置px4进入offboard模式  只要是L2以上的控制，px4都运行在offboard模式

			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}

		//  在L1收到了起飞命令的触发	
		// 使能自动起飞，且起飞命令被触发，且起飞命令等于1	
		else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
		{
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 0.1)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
				break;
			}
			if (!get_landed())
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
				break;
			}
			if (rc_is_received(now_time)) // Check this only if RC is connected.
			{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					// 检查遥控器是否悬停状态（5通道要在下侧）
				// 以及能否接收控制指令模式，允许px4ctrl接收你的代码发给px4ctrl的控制指令 （6通道也要在下侧）
				// rc_data.check_centered() 这里怀疑代码写错了，他都是检查0通道是否归中，应该是检查0，1，2，3通道摇杆是否在中间
					ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
					while (ros::ok())
					{
						ros::Duration(0.01).sleep();
						ros::spinOnce();
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
						{
							ROS_INFO("\033[32m[px4ctrl] OK, you can takeoff again.\033[32m");
							break;
						}
					}
					break;
				}
			}
             //满足了自动起飞要求
			state = AUTO_TAKEOFF;
			// 推力映射，这个在后面会细讲，他对推力模型做了在线调整机制，
			// 悬停和轨迹跟踪时推力选用的模型不同
			controller.resetThrustMapping();
			set_start_pose_for_takeoff_land(odom_data);
			toggle_offboard_mode(true);				  // toggle on offboard before arm
			for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
			{
				ros::Duration(0.01).sleep();
				ros::spinOnce();
			}
			// 如果使能了auto_arm，则px4进入ARM模式（解锁px4）
			if (param.takeoff_land.enable_auto_arm)
			{
				toggle_arm_disarm(true);
			}
			takeoff_land.toggle_takeoff_land_time = now_time; //记录切换到自动起飞的时间戳

			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
		}
        // 如果想要重启px4
		if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
		{
			if (state_data.current_state.armed)
			{
				ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
				break;
			}
			reboot_FCU();
		}

		break;
	}

	case AUTO_HOVER://自动悬停（AUTO_HOVER）
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))//5通道不在最下面，或没有收到里程计信息
		{
			state = MANUAL_CTRL;// 更新状态为MANUAL_CTRL
			toggle_offboard_mode(false);// 设置px4退出offboard模式

			ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
		}
		else if (rc_data.is_command_mode && cmd_is_received(now_time))// 当前6通道在最下面，且正在接收命令
		{
			if (state_data.current_state.mode == "OFFBOARD")//如果px4当前状态为offboard
			{
				state = CMD_CTRL;// 更新状态为CMD_CTRL
				des = get_cmd_des();//把/position_cmd发来的数据作为des，从控制命令读取期望的位置、速度、加速度、加加速度、yaw角、角加速度
				ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			}
		}
		//自动起飞命令被触发，且起飞降落命令 为降落
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{

			state = AUTO_LAND;// 更新状态为AUTO_LAND
			set_start_pose_for_takeoff_land(odom_data);//设置降落的起始位姿

			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
		}
		else
		{
			set_hov_with_rc();// 从遥控器接收期望控制量
			des = get_hover_des();// 将遥控器接收期望控制量，设置为期望位姿
			// 如果6通打了下来，或者使能自动起飞\降落延迟触发 
			if ((rc_data.enter_command_mode) ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
			}

			// cout << "des.p=" << des.p.transpose() << endl;
		}

		break;
	}
    // 命令控制模式
	case CMD_CTRL:
	{
		// L2的前提已经要求5通道在下了（进入offbord模式），L3更要求了
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))// 5通不在最下面，或没有收到里程计信息
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);//更新状态为MANUAL_CTRL

			ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time))// 6通不在最下面，或没有收到控制命令
		{
			state = AUTO_HOVER; // 更新状态为AUTO_HOVER
			set_hov_with_odom(); //  设置从里程计获得位姿作为悬停位姿
			des = get_hover_des();// 设置悬停位姿为期望位姿 
			ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		// 5、6通道拨下来了（进入了offboard 且能接收指令），/position_cmd不断地发控制命令
		else
		{
			des = get_cmd_des();
		}
        // 这时自动起飞\降落命令被触发，且要自动降落
		if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
					Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
					  param.msg_timeout.cmd); //报错，不能在命令控制模式触发自动降落，只能在自动悬停下启动
		}

		break;
	}
    // 自动起飞（AUTO_TAKEOFF）：
	case AUTO_TAKEOFF:
	{
		// 刚收到自动起飞命令的前几秒怠速
		if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
		{
			des = get_rotor_speed_up_des(now_time);
		}
		
		//起飞高度大于设置的悬停高度
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
		{
			state = AUTO_HOVER;// 更新状态为AUTO_HOVER
			set_hov_with_odom();// 从里程计获得当前位姿作为悬停位姿
			ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");
         //使能并设置触发延迟 ，可能是进入自动悬停模式一段时间后告诉规划器我ok了
			//这段延迟时间被宏定义为2S
			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
		}
		//计算期望位姿des
		else
		{
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}

		break;
	}
    // 自动降落（AUTO_LAND）
	case AUTO_LAND:
	{
		// 如果5通不在最下面，或没有收到里程计信息
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;//更新状态为MANUAL_CTRL
			toggle_offboard_mode(false); //设置px4退出offboard模式

			ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		//  6通不在最下面
		else if (!rc_data.is_command_mode)
		{
			state = AUTO_HOVER;// 更新状态为AUTO_HOVER
			set_hov_with_odom();//从里程计获得当前位姿作为悬停位姿
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
		}
		else if (!get_landed())// 还没降落
		{
			des = get_takeoff_land_des(-param.takeoff_land.speed);//	计算期望位姿
		}
		else
		{
			rotor_low_speed_during_land = true;

			static bool print_once_flag = true;
			if (print_once_flag)
			{
				ROS_INFO("\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
				print_once_flag = false;
			}
             //  如果px4扩展状态为已经着陆，PX4系统检测到已经着陆
			if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
			{
				static double last_trial_time = 0; // Avoid too frequent calls
				if (now_time.toSec() - last_trial_time > 1.0)
				{
					if (toggle_arm_disarm(false)) // disarm
					{
						print_once_flag = true;
						state = MANUAL_CTRL;//给px4上锁，状态更新为MANUAL_CTRL，px4退出offboard模式
						toggle_offboard_mode(false); // toggle off offboard after disarm
						ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
					}

					last_trial_time = now_time.toSec();
				}
			}
		}

		break;
	}

	default:
		break;
	}

	// STEP2: estimate thrust model
	// 估计油门推力用哪种模式，有两种模式，一个是悬停下的油门推力，一种轨迹跟踪下的油门推力
	if (state == AUTO_HOVER || state == CMD_CTRL)
	{
		// controller.estimateThrustModel(imu_data.a, bat_data.volt, param);
		controller.estimateThrustModel(imu_data.a,param);// 具体识别方法在控制器中介绍

	}

	// STEP3: solve and update new control commands 更新控制指令
	if (rotor_low_speed_during_land) // used at the start of auto takeoff 开始自动起飞
	{
		motors_idling(imu_data, u);//电机怠速
	}
	else
	{
		debug_msg = controller.calculateControl(des, odom_data, imu_data, u);// 更新控制命令并记录发送
		debug_msg.header.stamp = now_time;
		debug_pub.publish(debug_msg);
	}

	// STEP4: publish control commands to mavros
	if (param.use_bodyrate_ctrl)
	{
		publish_bodyrate_ctrl(u, now_time); 
	}
	else
	{
		publish_attitude_ctrl(u, now_time);
	}

	// STEP5: Detect if the drone has landed
	land_detector(state, des, odom_data);// 检测是否已经落地
	// cout << takeoff_land.landed << " ";
	// fflush(stdout);

	// STEP6: Clear flags beyound their lifetime
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
	u.q = imu.q;
	u.bodyrates = Eigen::Vector3d::Zero();
	u.thrust = 0.04;
}

void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
	{
		takeoff_land.landed = false; // Always holds
	}
	last_state = state;

	if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
	{
		takeoff_land.landed = true;
		return; // No need of other decisions
	}

	// land_detector parameters
	// 降落检测三个条件：
	//1.目标位置和实际位置范围在0.5m范围以内
    //2.速度限定在0.1m/s以内
	//3.上述条件需满足 3s以上
	constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
	constexpr double VELOCITY_THR_C = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
	constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

	static ros::Time time_C12_reached; // time_Constraints12_reached
	static bool is_last_C12_satisfy;
	if (takeoff_land.landed)
	{
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) //Constraint 3 reached
			{
				takeoff_land.landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	des.yaw = cmd_data.yaw;
	des.yaw_rate = cmd_data.yaw_rate;

	return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
	if (des_a_z > 0.1)
	{
		ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
	// takeoff_land.last_set_cmd_time = now;

	// takeoff_land.start_pose(2) += speed * delta_t;

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;

	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;

	// if (param.print_dbg)
	// {
	// 	static unsigned int count = 0;
	// 	if (count++ % 100 == 0)
	// 	{
	// 		cout << "hover_pose=" << hover_pose.transpose() << endl;
	// 		cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
	// 	}
	// }
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

	takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
	mavros_msgs::SetMode offb_set_mode;

	if (on_off)
	{
		state_data.state_before_offboard = state_data.current_state;
		if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
			state_data.state_before_offboard.mode = "MANUAL";

		offb_set_mode.request.custom_mode = "OFFBOARD";
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else
	{
		offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Exit OFFBOARD rejected by PX4!");
			return false;
		}
	}

	return true;

	// if (param.print_dbg)
	// 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

	return true;
}

void PX4CtrlFSM::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO("Reboot FCU");

	// if (param.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}
