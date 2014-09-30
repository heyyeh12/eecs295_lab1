#include "ros/ros.h"
#include "fw_wrapper/command.h"
#include "fw_wrapper/getcmd.h"
#include "std_msgs/Int32.h"

void testcb(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("wow");
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sample_driver");

	ros::NodeHandle n;

	ros::Publisher driver_pub = n.advertise<fw_wrapper::command>("commandsender",1000);

	ros::Subscriber sub = n.subscribe ("test", 1, testcb);
	ros::ServiceClient client = n.serviceClient<fw_wrapper::getcmd>("getcmd");
	fw_wrapper::getcmd srv;

	ros::Rate loop_rate(40);
	int count = 0;
	int count2 = 0;
	std::vector<int8_t> dev_ids;
	std::vector<int16_t> target_vals;
	fw_wrapper::command msg;
	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();
	loop_rate.sleep();
	while(ros::ok()) {
		msg.command_type = "SetMotorTargetSpeed";
		msg.device_id = 6;
		msg.target_val = 255+count*50;
		driver_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count2 = 0;
		while(1) {
			msg.command_type = "SetMotorTargetPosition";
			msg.device_id = 6;
			msg.target_val = 255+count2*5;
			driver_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			if (count2 == 100) break;
			count2++;
		}
		if(count == 10) break;
		count++;
	}
	return 0;
}
