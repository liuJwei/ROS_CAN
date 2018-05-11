#include "ros/ros.h"
#include <sstream>

//CAN header
#include "controlcan.h"

#include <iostream>
using namespace std;
void CAN_init();

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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

  while (ros::ok())
  {

//需要发送的帧，结构体设置
  VCI_CAN_OBJ send[4];
  for(int it = 0; it < 4 ; it++)
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
        send[it].Data[1] = 0x02;
        send[it].Data[4] = 0x01;
        send[it].Data[5] = 0xFF;
  }
  send[1].Data[4] = 0xFE;   //左右电机方向相反
  send[1].Data[5] = 0x00;

  send[3].Data[4] = 0xFE;   //左右电机方向相反
  send[3].Data[5] = 0x00;

  send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, send, 4);
  cout << send_frame << endl;
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

    ros::spinOnce();

    loop_rate.sleep();
    count++;
	

  }

  VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
  usleep(100000);//延时100ms。
  VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。

  return 0;
}

void CAN_init()
{
//-------------------------------------------CAN配置---------------------------
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
//----------------------------------------------------------------------
}


