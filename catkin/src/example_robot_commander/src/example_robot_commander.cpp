#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char **argv)
{
	ros::init(argc,argv,"robot0_commander_wcb38");
	ros::NodeHandle nh;
	ros::Publisher velcmd_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);
	// change topic to command abby...
	//ros::Publisher velcmd_pub = nh.advertise<geometry_msgs::Twist>("abby/cmd_vel",1);
	ros::Rate sleep_timer(111); //111Hz timer

	geometry_msgs::Twist twist_cmd;
	twist_cmd.linear.x = 0.0;
	twist_cmd.linear.y = 0.0;
	twist_cmd.linear.z = 0.0;
	twist_cmd.angular.x = 0.0;
	twist_cmd.angular.y = 0.0;
	twist_cmd.angular.z = 0.0;

	ROS_INFO("count-down 3 seconds");
	for (int j=3;j>0;j--) {
		ROS_INFO("%d!",j);
		for (int i = 0; i<111;i++)
		    sleep_timer.sleep();
	}
	ROS_INFO("0!");
`	ROS_INFO("Onward March -->");
	int niters = 1000; 
/*
	1000 iters at 111Hz is 9.009 seconds; 
	at 9.009ms/iteration, and 0.222m/sec, expect 2.0mm/iter ; 
	should move by 2m over 9.009 sec 
*/
	velcmd_pub.publish(twist_cmd); //0|0
	for (int i=0;i<niters;i++) {
		twist_cmd.linear.x = 0.222;
		velcmd_pub.publish(twist_cmd); //.222|0
		sleep_timer.sleep();
	}
	twist_cmd.linear.x = 0.0;
	twist_cmd.angular.z = 0.0;
	velcmd_pub.publish(twist_cmd); //0|0
	ROS_INFO(" --> End March");

	ROS_INFO("Turn to April --> ");
	niters=500; // 4.5045 sec, 90deg
	for (int i=0;i<niters;i++) {
		twist_cmd.angular.z = -0.3487; //0|-.3487
		velcmd_pub.publish(twist_cmd);
		sleep_timer.sleep();
	}
	twist_cmd.linear.x = 0.0;
	twist_cmd.angular.z = 0.0;
	velcmd_pub.publish(twist_cmd); //0|0
	ROS_INFO(" --> End April");
	
	ROS_INFO("Onward May -->");
	niters = 1000; //9.009sec, 2 m
	for (int i=0;i<niters;i++) {
		twist_cmd.linear.x = 0.222;
		velcmd_pub.publish(twist_cmd); //.222|0
		sleep_timer.sleep();
	}
	twist_cmd.linear.x = 0.0;
	twist_cmd.angular.z = 0.0;
	velcmd_pub.publish(twist_cmd); //0|0
	ROS_INFO(" --> End May");

	ROS_INFO("Turn to June --> ");
	niters=500; // 4.5045 sec, 90deg
	for (int i=0;i<niters;i++) {
		twist_cmd.angular.z = -0.3487; //0|-.3487
		velcmd_pub.publish(twist_cmd);
		sleep_timer.sleep();
	}
	twist_cmd.linear.x = 0.0;
	twist_cmd.angular.z = 0.0;
	velcmd_pub.publish(twist_cmd); //0|0
	ROS_INFO(" --> End June");

	ROS_INFO("Onward July -->");
	niters = 1000; //9.009sec, 2 m
	for (int i=0;i<niters;i++) {
		twist_cmd.linear.x = 0.222;
		velcmd_pub.publish(twist_cmd); //.222|0
		sleep_timer.sleep();
	}
	twist_cmd.linear.x = 0.0;
	twist_cmd.angular.z = 0.0;
	velcmd_pub.publish(twist_cmd); //0|0
	ROS_INFO(" --> End July");

	ROS_INFO("Tired August.")
	twist_cmd.linear.x = 0.0;
	twist_cmd.angular.z = 0;
	velcmd_pub.publish(twist_cmd); //0|0 and halt
	return 0;
} 
