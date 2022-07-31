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

char tmp_buf[33];			//×Ö·û´®Êı×é
float pitch,roll,yaw; 		//Å·À­½Ç:¸©Ñö½Ç£¬Æ«º½½Ç£¬¹ö×ª½Ç
short aacx,aacy,aacz;		//¼ÓËÙ¶È´«¸ĞÆ÷Ô­Ê¼Êı¾İ  angular acceleration
short gyrox,gyroy,gyroz;	//ÍÓÂİÒÇÔ­Ê¼Êı¾İ  gyroscope
short temp;					//ÎÂ¶È

struct MPU6050				//MPU6050½á¹¹Ìå
{
	uint8_t flag;				//²É¼¯³É¹¦±êÖ¾Î»
	uint8_t speed;				//ÉÏ±¨ËÙ¶È
}mpu6050;					//Î¨Ò»½á¹¹Ìå±äÁ¿

void MPU_Read(void);		//MPU6050Êı¾İ¶ÁÈ¡º¯Êı	
void DATA_Report(void);		//MPU6050Êı¾İÉÏ±¨	

void MPU_Read(void)
{
	
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//dmp´¦ÀíµÃµ½Êı¾İ£¬¶Ô·µ»ØÖµ½øĞĞÅĞ¶Ï
	{ 
		temp=MPU_Get_Temperature();	                //µÃµ½ÎÂ¶ÈÖµ
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//µÃµ½¼ÓËÙ¶È´«¸ĞÆ÷Êı¾İ
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//µÃµ½ÍÓÂİÒÇÊı¾İ
		mpu6050.speed++;                            //ÉÏ±¨ËÙ¶È×Ô¼Ó
		if(mpu6050.speed == 4)						//ÉÏ±¨ËÙ¶ÈãĞÖµÉèÖÃ
		{
			mpu6050.flag = 1;						//²É¼¯³É¹¦±êÖ¾Î»ÉèÖÃÎªÓĞĞ§
			mpu6050.speed = 0;						//ÉÏ±¨ËÙ¶È¹éÁã
		}	
	}
	else 											//²É¼¯²»³É¹¦										
	{
		mpu6050.flag = 0;							//²É¼¯³É¹¦±êÖ¾Î»ÉèÖÃÎªÎŞĞ§
	}	
}

void DATA_Report(void)
{
	if(mpu6050.flag == 1)						//²É¼¯³É¹¦Ê±
	{ 
		if(temp<0)								//¶ÔÊı¾İÕı¸ºÅĞ¶Ï£¬ÅĞ¶ÏÎª¸ºÊ±
		{
			temp=-temp;							//¶Ô¸ºÊı¾İÈ¡·´
		}
		else                                    //ÅĞ¶ÏÎªÕıÊ±
		{
		}
		printf("temp:%d.%d,",temp/100,temp%10); //Í¨¹ı´®¿Ú1Êä³öÎÂ¶È
		
		temp=pitch*10;							 //¸³tempÎªpitch
		if(temp<0)								//¶ÔÊı¾İÕı¸ºÅĞ¶Ï£¬ÅĞ¶ÏÎª¸ºÊ±
		{
			temp=-temp;						    //¶Ô¸ºÊı¾İÈ¡·´		
		}
		else                                    //ÅĞ¶ÏÎªÕıÊ± 
		{
		}
		sprintf((char *)tmp_buf,"Pitch:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,16,(uint8_t *)tmp_buf,16);
		printf("pitch:%d.%d,",temp/10,temp%10); //Í¨¹ı´®¿Ú1Êä³öpitch	
		
		temp=roll*10;                            //¸³tempÎªroll
		if(temp<0)								//¶ÔÊı¾İÕı¸ºÅĞ¶Ï£¬ÅĞ¶ÏÎª¸ºÊ±
		{
			temp=-temp;						    //¶Ô¸ºÊı¾İÈ¡·´	
		}
		else                                    //ÅĞ¶ÏÎªÕıÊ±
		{
		}
		sprintf((char *)tmp_buf,"roll:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,32,(uint8_t *)tmp_buf,16);
		printf("roll:%d.%d,",temp/10,temp%10);//Í¨¹ı´®¿Ú1Êä³öroll
		
		temp=yaw*10;                           //¸³tempÎªyaw
		if(temp<0)								//¶ÔÊı¾İÕı¸ºÅĞ¶Ï£¬ÅĞ¶ÏÎª¸ºÊ±
		{
			temp=-temp;						    //¶Ô¸ºÊı¾İÈ¡·´
		}
		else                                    //ÅĞ¶ÏÎªÕıÊ±
		{
		}
		sprintf((char *)tmp_buf,"yaw:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,48,(uint8_t *)tmp_buf,16);
		printf("yaw:%d.%d,",temp/10,temp%10);//Í¨¹ı´®¿Ú1Êä³öyaw	
		printf("gyrox:%d,gyroy:%d,gyroz:%d,aacx:%d,aacy:%d,aacz:%d\r\n",gyrox,gyroy,gyroz,aacx,aacy,aacz);//ÉÏ±¨½ÇËÙ¶ÈÊı¾İ£¬½Ç¼ÓËÙ¶ÈÊı¾İ¸
		mpu6050.flag = 0;									//²É¼¯³É¹¦±êÖ¾Î»ÉèÖÃÎªÎŞĞ§
	}
	else ;														//·À¿¨ËÀ
}


void SYS_Init(void)
{
	MAP_WDT_A_holdTimer(); //¹Ø±Õ¿ªÃÅ¹·
	delay_init();	    							 //ÑÓÊ±º¯Êı³õÊ¼»¯	  
	uart_init(115200);	 	                         //´®¿Ú³õÊ¼»¯Îª115200
	LED_Init();		                               	 //³õÊ¼»¯ÓëLEDÁ¬½ÓµÄÓ²¼ş½Ó¿Ú
	//OLED_Init();									 //OLED³õÊ¼»¯
	MPU_Init();	                                     //³õÊ¼»¯MPU6050
	while(mpu_dmp_init())                            //³õÊ¼»¯mpu_dmp¿â
 	{
		OLED_ShowString(0,0,"Failed",16);			 //ÏÔÊ¾×Ö·û´®
		//OLED_Refresh();	                             //Ë¢ĞÂÏÔ´æ
		LED_R_Tog();delay_ms(50);LED_R_Tog();     //LEDÉÁË¸Ö¸Ê¾
		printf("Initialization failed£¡\r\n");		//´®¿Ú³õÊ¼»¯Ê§°ÜÉÏ±¨
//		OLED_ShowString(0,0,"      ",16);          //ÏÔÊ¾×Ö·û´®
		//OLED_Refresh();								//Ë¢ĞÂÏÔ´æ
	}
	printf("Initialization successed£¡\r\n");		//´®¿Ú³õÊ¼»¯³É¹¦ÉÏ±¨
//	OLED_ShowString(0,0,"  OK!",16);				//ÏÔÊ¾×Ö·û´®
	//OLED_Refresh();									//Ë¢ĞÂÏÔ´æ
	delay_ms(999);									//ÑÓÊ±³õ½çÃæÏÔÊ¾
	mpu6050.flag = 0;                               //²É¼¯³É¹¦±êÖ¾Î»³õÊ¼»¯
	mpu6050.speed = 0;								//ÉÏ±¨ËÙ¶È³õÊ¼»¯
}

int main(void) 
{	
	SYS_Init();//ÏµÍ³³õÊ¼»¯×Üº¯Êı
	while(1)   //Ö÷Ñ­»·
	{
		MPU_Read();    //MPU6050Êı¾İ¶ÁÈ¡
		DATA_Report(); //MPU6050Êı¾İÉÏ±¨
	}
}
