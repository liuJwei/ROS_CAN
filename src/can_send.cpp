
//This file send the data via usb-can device.
//liuwein@126.com

#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"

//CAN header
#include "controlcan.h"

#include <iostream>
using namespace std;
void CAN_init();

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "can_send");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Rate loop_rate(100);
  
  CAN_init();

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  int send_frame = 0;

  ros::Subscriber sub_1 = n.subscribe("camera_chatter", 1, chatterCallback);
  ros::Subscriber sub_2 = n.subscribe("lidar_chatter", 1, chatterCallback);
  ros::Subscriber sub_3 = n.subscribe("radar_chatter", 1, chatterCallback);

  while (ros::ok())
  {

  //需要发送的帧，结构体设置
  int frame_number = 5;
  VCI_CAN_OBJ send[frame_number];
  for(int it = 0; it < frame_number ; it++)
  {
	  send[it].ID = 0x0A510101 + it ;
	  send[it].SendType = 0;
	  send[it].RemoteFlag = 0; //数据帧
	  send[it].ExternFlag = 1; //扩展帧
	  send[it].DataLen = 8;

	 //send[0].data =  01 01 02 FF 00 00 00 00
	 int i=0;
	for(i = 0; i < send[it].DataLen; i++)
	{
		send[it].Data[i] = 0x00;
	}
	send[it].Data[0] = it+1;
	send[it].Data[1] = 0x02;  //02：转速控制; 01:扭矩控制模式
	send[it].Data[4] = 0x00;  //控制量 Data[2~3]扭矩控制量 Data[4~5]转速控制量
	send[it].Data[5] = 0x1F;  //控制量
  }
  send[1].Data[4] = 0xFF;   //左右电机方向相反, 左右电机对应位数值相加 = FFFF;
  send[1].Data[5] = 0xE0;

  send[3].Data[4] = 0xFF;   //左右电机方向相反
  send[3].Data[5] = 0xE0;

  //转向电机第一帧设置为goto控制模式
  if(count == 0)
  {
	  send[4].ID = 0x0401 ;
	  send[4].SendType = 0;
	  send[4].RemoteFlag = 0; //数据帧
	  send[4].ExternFlag = 0; //标准帧
	  send[4].DataLen = 8;  

	  send[4].Data[0] = 0x0F;
	  for(int i = 1; i< 8; i++)
	  {
              send[4].Data[i] = 0x00;
	  }
  }
  else
  {
	  send[4].ID = 0x0401 ;
	  send[4].SendType = 0;
	  send[4].RemoteFlag = 0; //数据帧
	  send[4].ExternFlag = 0; //标准帧
	  send[4].DataLen = 8;  

	  send[4].Data[0] = 0x1F;
	  for(int i = 1; i< 8; i++)
          {
		 send[4].Data[i] = 0x00;
	  }
	  send[4].Data[4] = 0x60;  //控制量 Data[4～7] 
  }


  send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, send, frame_number);
  ROS_INFO_STREAM("1st node: "<<count<<" loop, sent_frames:"<< send_frame);
  
 // cout << send[0].ID<<" "<<send[1].ID<<" "<<send[2].ID<<" "<<send[3].ID<<endl;

/*
	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
		printf("Index:%04d  ",count);
		printf("CAN2 TX ID:0x%08X", send[0].ID);
		if(send[0].ExternFlag==0) printf(" Standard ");
		if(send[0].ExternFlag==1) printf(" Extend   ");
		if(send[0].RemoteFlag==0) printf(" Data   ");
		if(send[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send[0].DataLen);
		printf(" data:0x");

		for(i=0;i<send[0].DataLen;i++)
		{
			printf(" %02X", send[0].Data[i]);
		}

		printf("\n");
	}
	else
	{
		break;
	}
*/
//------------------------------------------------------------

    /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

    ros::spinOnce();

    loop_rate.sleep();

    count++;
	

  }

  VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
  usleep(100000);//延时100ms。
  VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。

  return 0;
}


//--------------------CAN配置-------------------------------------------------//
void CAN_init()
{
	ROS_INFO_STREAM(">>this is hello !\r\n ");//指示程序已运行
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		ROS_INFO_STREAM(">>open deivce success!\n");//打开设备成功
	}else
	{
		ROS_INFO_STREAM(">>open deivce error!\n");
		exit(1);		
	}	
		
	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0 = 0x00;/*波特率500 Kbps  0x00  0x1C*/
	config.Timing1 = 0x1C;
	config.Mode = 0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
}

