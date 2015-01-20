#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char **argv)
{
	ros::init(argc,argv,"robot0_commander_wcb38");
	ros::NodeHandle nh;
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);
	// change topic to command abby...
	//ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("abby/cmd_vel",1);
	ros::Rate sleep_timer(100); //let's make a 100Hz timer

	//create a variable of type "Twist", as defined in: /opt/ros/hydro/share/geometry_msgs
	// look at the components of a message of type geometry_msgs::Twist by typing:
	// rosmsg show geometry_msgs/Twist/
	// It has 6 fields.  Let's fill them all in with some data:
	geometry_msgs::Twist twist_cmd;
	twist_cmd.linear.x = 0.0;
	twist_cmd.linear.y = 0.0;
	twist_cmd.linear.z = 0.0;
	twist_cmd.angular.x = 0.0;
	twist_cmd.angular.y = 0.0;
	twist_cmd.angular.z = 0.0;

	ROS_INFO("count-down");
	for (int j=3;j>0;j--) {
		ROS_INFO("j start sleep --> %d",j);
		for (int i = 0; i<100;i++)
		    sleep_timer.sleep();
	}

	int niters = 1200; //1000 iters at 100Hz is 10 seconds;
		//iteration counter; at 10ms/iteration, and 0.2m/sec, expect 2mm/iter
		// should move by 2m over 10 sec
	for (int i=0;i<niters;i++) {
		cmd_publisher.publish(twist_cmd); // really, should only need to publish this once, but no hard done
		sleep_timer.sleep(); // sleep for (remainder of) 10m
	}
	twist_cmd.linear.x = 0.0;
	twist_cmd.angular.z = -0.314;
	niters=500; // 5 sec
	ROS_INFO("Time to rotate negative");
	for (int i=0;i<niters;i++) {
		cmd_publisher.publish(twist_cmd); // really, should only need to publish this once, but no hard done
		sleep_timer.sleep(); // sleep for (remainder of) 10m
	}
	ROS_INFO("my work here is done");
	//while (ros::ok()) 
	{
		twist_cmd.linear.x = 0.0;
		twist_cmd.angular.z = 0;
		cmd_publisher.publish(twist_cmd); // and halt
	}


	return 0;
} 
