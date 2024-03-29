/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#include "usart.h"
#include "sysinit.h"
#include "key.h"
#include "led.h"
#include "delay.h"
#include "oled.h"
#include "bsp_mpu6050.h"
#include "inv_mpu.h"

char tmp_buf[33];			//字符串数组
float pitch,roll,yaw; 		//欧拉角:俯仰角，偏航角，滚转角
short aacx,aacy,aacz;		//加速度传感器原始数据  angular acceleration
short gyrox,gyroy,gyroz;	//陀螺仪原始数据  gyroscope
short temp;					//温度

struct MPU6050				//MPU6050结构体
{
	uint8_t flag;				//采集成功标志位
	uint8_t speed;				//上报速度
}mpu6050;					//唯一结构体变量

void MPU_Read(void);		//MPU6050数据读取函数	
void DATA_Report(void);		//MPU6050数据上报	

void MPU_Read(void)
{
	
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//dmp处理得到数据，对返回值进行判断
	{ 
		temp=MPU_Get_Temperature();	                //得到温度值
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		mpu6050.speed++;                            //上报速度自加
		if(mpu6050.speed == 4)						//上报速度阈值设置
		{
			mpu6050.flag = 1;						//采集成功标志位设置为有效
			mpu6050.speed = 0;						//上报速度归零
		}	
	}
	else 											//采集不成功										
	{
		mpu6050.flag = 0;							//采集成功标志位设置为无效
	}	
}

void DATA_Report(void)
{
	if(mpu6050.flag == 1)						//采集成功时
	{ 
		if(temp<0)								//对数据正负判断，判断为负时
		{
			temp=-temp;							//对负数据取反
		}
		else                                    //判断为正时
		{
		}
		printf("temp:%d.%d,",temp/100,temp%10); //通过串口1输出温度
		
		temp=pitch*10;							 //赋temp为pitch
		if(temp<0)								//对数据正负判断，判断为负时
		{
			temp=-temp;						    //对负数据取反		
		}
		else                                    //判断为正时 
		{
		}
		sprintf((char *)tmp_buf,"Pitch:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,16,(uint8_t *)tmp_buf,16);
		printf("pitch:%d.%d,",temp/10,temp%10); //通过串口1输出pitch	
		
		temp=roll*10;                            //赋temp为roll
		if(temp<0)								//对数据正负判断，判断为负时
		{
			temp=-temp;						    //对负数据取反	
		}
		else                                    //判断为正时
		{
		}
		sprintf((char *)tmp_buf,"roll:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,32,(uint8_t *)tmp_buf,16);
		printf("roll:%d.%d,",temp/10,temp%10);//通过串口1输出roll
		
		temp=yaw*10;                           //赋temp为yaw
		if(temp<0)								//对数据正负判断，判断为负时
		{
			temp=-temp;						    //对负数据取反
		}
		else                                    //判断为正时
		{
		}
		sprintf((char *)tmp_buf,"yaw:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,48,(uint8_t *)tmp_buf,16);
		printf("yaw:%d.%d,",temp/10,temp%10);//通过串口1输出yaw	
		printf("gyrox:%d,gyroy:%d,gyroz:%d,aacx:%d,aacy:%d,aacz:%d\r\n",gyrox,gyroy,gyroz,aacx,aacy,aacz);//上报角速度数据，角加速度数据�
		mpu6050.flag = 0;									//采集成功标志位设置为无效
	}
	else ;														//防卡死
}


void SYS_Init(void)
{
	MAP_WDT_A_holdTimer(); //关闭开门狗
	delay_init();	    							 //延时函数初始化	  
	uart_init(115200);	 	                         //串口初始化为115200
	LED_Init();		                               	 //初始化与LED连接的硬件接口
	//OLED_Init();									 //OLED初始化
	MPU_Init();	                                     //初始化MPU6050
	while(mpu_dmp_init())                            //初始化mpu_dmp库
 	{
		OLED_ShowString(0,0,"Failed",16);			 //显示字符串
		//OLED_Refresh();	                             //刷新显存
		LED_R_Tog();delay_ms(50);LED_R_Tog();     //LED闪烁指示
		printf("Initialization failed！\r\n");		//串口初始化失败上报
//		OLED_ShowString(0,0,"      ",16);          //显示字符串
		//OLED_Refresh();								//刷新显存
	}
	printf("Initialization successed！\r\n");		//串口初始化成功上报
//	OLED_ShowString(0,0,"  OK!",16);				//显示字符串
	//OLED_Refresh();									//刷新显存
	delay_ms(999);									//延时初界面显示
	mpu6050.flag = 0;                               //采集成功标志位初始化
	mpu6050.speed = 0;								//上报速度初始化
}

int main(void) 
{	
	SYS_Init();//系统初始化总函数
	while(1)   //主循环
	{
		MPU_Read();    //MPU6050数据读取
		DATA_Report(); //MPU6050数据上报
	}
}
