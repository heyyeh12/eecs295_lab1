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
	std::vector<int16_t> target_vals1;
	std::vector<int16_t> target_vals2;
	dev_ids.push_back(3);
	dev_ids.push_back(5);
	dev_ids.push_back(4);
	dev_ids.push_back(6);
	target_vals1.push_back(512);
	target_vals1.push_back(512);
	target_vals1.push_back(512);
	target_vals1.push_back(512);
	target_vals2.push_back(300);
	target_vals2.push_back(300);
	target_vals2.push_back(300);
	target_vals2.push_back(300);
	fw_wrapper::command msg;
	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();
	loop_rate.sleep();

	srv.request.command_type = "GetSensorValue";
	srv.request.device_id = 5;
	msg.command_type = "SetMotorTargetPositionsSync";
	msg.n_dev = 4;
	msg.dev_ids = dev_ids;
/*
	while(ros::ok()) {
		msg.target_vals = target_vals1;
		driver_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		msg.target_vals = target_vals2;
		driver_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
*/
	while(ros::ok()) {
		if(client.call(srv)) {
			ROS_INFO("%d", srv.response.val);
			if(srv.response.val > 1000) {
				msg.target_vals = target_vals2;
				driver_pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			} else {
				msg.target_vals = target_vals1;
				driver_pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
	}
	return 0;
}
