#include "ros/ros.h"
#include "fw_wrapper/allcmd.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sample_driver");

	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<fw_wrapper::allcmd>("allcmd");
	fw_wrapper::allcmd srv;

	ros::Rate loop_rate(5);
	srv.request.device_id = 1;
	srv.request.command_type = "SetMotorWheelSpeed";
	srv.request.target_val = 512;
	client.call(srv);

	srv.request.device_id = 1;
	srv.request.command_type = "GetMotorWheelSpeed";
	for(int i = 0; i < 100; i++) 
	{
			if(client.call(srv)) {
				ROS_INFO("Wheelspeed is %d", srv.response.val);
			}
			ros::spinOnce();
			loop_rate.sleep();
		}

	srv.request.device_id = 1;
	srv.request.command_type = "SetMotorWheelSpeed";
	srv.request.target_val = 0;
	client.call(srv);


	return 0;
}
