#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "fw_wrapper/command.h"
#include "fw_wrapper/allcmd.h"
#include "zigbee.h"
#include <termios.h>
#include <unistd.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#define ACK (short)0
#define RDY (short)-1

#define MAX_LOOP_COUNT 5
#define MAX_INIT_LOOP_COUNT 100

boost::mutex radio_in_use;

void commandsenderCallback (const fw_wrapper::command::ConstPtr& msg)
{
}

bool allcmdCallback(fw_wrapper::allcmd::Request &req, fw_wrapper::allcmd::Response &res)
{
  boost::mutex::scoped_lock lock(radio_in_use);
	short RxData, TxData;
	char type, id;
	int i;
	//std_msgs::Int32 x;
	//testpub.publish(x);
	ROS_INFO("Got Request: %s\n, DevID: %d\n", req.command_type.c_str(), req.device_id);
	ros::Rate r(1000);
	int flag = 0;
	if (req.command_type == "GetMotorTargetPosition") {
		type = 'p';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		while(1) {
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("got get_target_position response\n");
					res.val = RxData;
					flag = 1;
					break;
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		TxData = ACK;
		while(flag) {
			zgb_tx_data(TxData);
			for (i = 0; i < 30; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("ACK is lost.. resending\n");
					break;
				}
				r.sleep();
			}
			if (i == 30)
				flag = 0;
		}
	} else if (req.command_type == "GetMotorCurrentPosition") {
		type = 'q';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		while(1) {
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("got get_current_position response\n");
					res.val = RxData;
					flag = 1;
					break;
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		TxData = ACK;
		while(flag) {
			zgb_tx_data(TxData);
			for (i = 0; i < 30; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("ACK is lost.. resending\n");
					break;
				}
				r.sleep();
			}
			if (i == 30)
				flag = 0;
		}
	} 
	else if (req.command_type == "GetIsMotorMoving") 
	{
	  int loop_count = 0;
		type = 'm';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		while(loop_count < MAX_LOOP_COUNT) {
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("got is_motor_moving response\n");
					res.val = RxData;
					flag = 1;
					break;
				}
				r.sleep();
			}
			if (flag)
				break;
				
			loop_count++;
		}
		
		loop_count = 0;
		TxData = ACK;
		while(flag) 
		{
			zgb_tx_data(TxData);
			for (i = 0; i < 30; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("ACK is lost.. resending\n");
					break;
				}
				r.sleep();
			}
			if (i == 30)
				flag = 0;
		}
	} 
	else if (req.command_type == "GetSensorValue") 
	{
	  int loop_count = 0;
		type = 'v';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		while(loop_count < MAX_LOOP_COUNT) 
		{
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("got get_sensor_value response %d\n", RxData);
					res.val = RxData;
					flag = 1;
					break;
				}
				r.sleep();
			}
			if (flag)
				break;
				
			loop_count++;
		}
		
		if(loop_count == MAX_LOOP_COUNT)
		{
		  ROS_ERROR("Response from robot timed out");
		}
		loop_count = 0;
		TxData = ACK;
		while(flag) 
		{
			zgb_tx_data(TxData);
			for (i = 0; i < 50; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("ACK is lost.. resending... got %d\n", RxData);
					break;
				}
				r.sleep();
			}
			if (i == 50)
				flag = 0;
		}
		ROS_INFO("successfully acknowledged get_sensor_value response \n");
	}
	else if (req.command_type == "GetMotorWheelSpeed") 
	{
	  int loop_count = 0;
		type = 's';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		while(loop_count < MAX_LOOP_COUNT) 
		{
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("got get_wheel_speed_value response %d\n", RxData);
					res.val = RxData;
					flag = 1;
					break;
				}
				r.sleep();
			}
			if (flag)
				break;
				
			loop_count++;
		}
		
		if(loop_count == MAX_LOOP_COUNT)
		{
		  ROS_ERROR("Response from robot timed out");
		}
		loop_count = 0;
		TxData = ACK;
		while(flag) 
		{
			zgb_tx_data(TxData);
			for (i = 0; i < 50; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("ACK is lost.. resending... got %d\n", RxData);
					break;
				}
				r.sleep();
			}
			if (i == 50)
				flag = 0;
		}
		ROS_INFO("successfully acknowledged get_wheel_speed_value response \n");
	} 
	else if (req.command_type == "SetMotorTargetPosition") 
	{
		type = 'P';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		
		while(zgb_rx_check() == 1)
		{
		  RxData = zgb_rx_data(); 
		}
		
		while(1) 
		{
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("got motor position first ack %d\n", RxData);
					if(RxData == ACK) 
					{
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		TxData = req.target_val;
		flag = 0;
		while(1) 
		{
			zgb_tx_data(TxData);
			ROS_INFO("send data %d\n", TxData);
			for (i = 0; i < 20; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("got motor position second ack\n");
					if(RxData == ACK) 
					{
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
			ROS_INFO("Timeout... resend\n");
		}
		res.val = 0;
	}
	else if(req.command_type == "SetMotorTargetSpeed") 
	{
		type = 'S';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		while(1) 
		{
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("got motor speed first ack\n");
					if(RxData == ACK) 
					{
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		TxData = req.target_val;
		flag = 0;
		while(1) 
		{
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("got motor speed second ack\n");
					if(RxData == ACK) {
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		res.val = 0;
	} 
	else if(req.command_type == "SetMotorTargetPositionsSync") 
	{
		type = 'C';
		TxData = ((short)type << 8) | ((short)req.n_dev & 0xff);
		while(1) {
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) 
			{
				if(zgb_rx_check() == 1) 
				{
					RxData = zgb_rx_data();
					ROS_INFO("got motor position sync first ack\n");
					if(RxData == ACK) {
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		int n = req.n_dev/2;
		if (req.n_dev != n*2)
			n = n + 1;
		int cnt = 0;
		for (int j = 0; j < n; j++) 
		{
			short hi = req.dev_ids[cnt++];
			short lo = -1;
			if (cnt < req.n_dev)
				lo = req.dev_ids[cnt++];
			TxData = (hi << 8) | (lo & 0xff);
			while(1) 
			{
				zgb_tx_data(TxData);
				for (i = 0; i < 20; i++) 
				{
					if(zgb_rx_check() == 1) 
					{
						RxData = zgb_rx_data();
						if(RxData == ACK) {
							flag = 1;
							break;
						}
						else
						{
						  ROS_INFO("RxData does not equal ACK!");
						  zgb_tx_data(ACK);
						  break;
						}
					}
					r.sleep();
				}
				if (i < 20 && flag)
					break;
			}
		}
		ROS_INFO("sent id list\n");
		for (int j = 0; j < req.n_dev; j++) 
		{
			TxData = req.target_vals[j];
			while(1) 
			{
				zgb_tx_data(TxData);
				for (i = 0; i < 20; i++) 
				{
					if(zgb_rx_check() == 1) 
					{
						RxData = zgb_rx_data();
						if(RxData == ACK) {
							flag = 1;
							break;
						}
						else
						{
						  ROS_INFO("RxData does not equal ACK!");
						  zgb_tx_data(ACK);
						  break;
						}
					}
					r.sleep();
				}
				if (i < 20 && flag)
					break;
			}
		}
		ROS_INFO("sent target values\n");
		res.val = 0;
	} 
	else if(req.command_type == "SetMotorMode") 
	{
		type = 'M';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		while(1) {
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("got motor mode first ack\n");
					if(RxData == ACK) {
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		TxData = req.target_val;
		flag = 0;
		while(1) {
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("got motor mode second ack\n");
					if(RxData == ACK) {
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		res.val = 0;
	} else if(req.command_type == "SetMotorWheelSpeed") {
		type = 'W';
		id = req.device_id;
		TxData = ((short)type << 8) | ((short)id & 0xff);
		while(1) {
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("got wheel speed first ack\n");
					if(RxData == ACK) {
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		TxData = req.target_val;
		flag = 0;
		while(1) {
			zgb_tx_data(TxData);
			for (i = 0; i < 20; i++) {
				if(zgb_rx_check() == 1) {
					RxData = zgb_rx_data();
					ROS_INFO("got wheel speed second ack\n");
					if(RxData == ACK) {
						flag = 1;
						break;
					}
					else
					{
					  ROS_INFO("RxData does not equal ACK!");
					  zgb_tx_data(ACK);
					  break;
					}
				}
				r.sleep();
			}
			if (i < 20 && flag)
				break;
		}
		res.val = 0;
	}
	else {
		ROS_INFO("Unrecognized Command Type\n");
	}
	return true;
}


int main(int argc, char **argv)
{
	short RxData;
	ros::init(argc, argv, "wrapper");
	ros::NodeHandle n;
	int init_loop_count = 0;
	if ( zgb_initialize(0) == 0) {
		ROS_INFO( "Failed to open Zig2Serial!\n");
		ROS_INFO( "Quitting...\n");
		return 0;
	} else {
		ROS_INFO( "Succeeded to open Zig2Serial!\n");
	}
	ros::Rate rate(50);
	zgb_tx_data(RDY);
	while(init_loop_count < MAX_INIT_LOOP_COUNT) {
		if(zgb_rx_check() == 1) {
			RxData = zgb_rx_data();
			if (RxData == ACK)
				break;
		}
		init_loop_count++;
		rate.sleep();
	}
	
	if(init_loop_count == MAX_INIT_LOOP_COUNT)
	{
	  ROS_ERROR("Could not establish connection with robot, please restart the robot controller, check the zigbee connection is made and then restart this node.");
	  return 0;
	}

	ROS_INFO("Robot is ready to take commands\n");

	ros::Subscriber sub = n.subscribe ("commandsender", 5, commandsenderCallback);

	ros::ServiceServer serv = n.advertiseService("allcmd", allcmdCallback);

	ros::spin();
	return 0;
}
