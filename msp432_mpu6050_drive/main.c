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

char tmp_buf[33];			//�ַ�������
float pitch,roll,yaw; 		//ŷ����:�����ǣ�ƫ���ǣ���ת��
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����  angular acceleration
short gyrox,gyroy,gyroz;	//������ԭʼ����  gyroscope
short temp;					//�¶�

struct MPU6050				//MPU6050�ṹ��
{
	uint8_t flag;				//�ɼ��ɹ���־λ
	uint8_t speed;				//�ϱ��ٶ�
}mpu6050;					//Ψһ�ṹ�����

void MPU_Read(void);		//MPU6050���ݶ�ȡ����	
void DATA_Report(void);		//MPU6050�����ϱ�	

void MPU_Read(void)
{
	
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//dmp����õ����ݣ��Է���ֵ�����ж�
	{ 
		temp=MPU_Get_Temperature();	                //�õ��¶�ֵ
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
		mpu6050.speed++;                            //�ϱ��ٶ��Լ�
		if(mpu6050.speed == 4)						//�ϱ��ٶ���ֵ����
		{
			mpu6050.flag = 1;						//�ɼ��ɹ���־λ����Ϊ��Ч
			mpu6050.speed = 0;						//�ϱ��ٶȹ���
		}	
	}
	else 											//�ɼ����ɹ�										
	{
		mpu6050.flag = 0;							//�ɼ��ɹ���־λ����Ϊ��Ч
	}	
}

void DATA_Report(void)
{
	if(mpu6050.flag == 1)						//�ɼ��ɹ�ʱ
	{ 
		if(temp<0)								//�����������жϣ��ж�Ϊ��ʱ
		{
			temp=-temp;							//�Ը�����ȡ��
		}
		else                                    //�ж�Ϊ��ʱ
		{
		}
		printf("temp:%d.%d,",temp/100,temp%10); //ͨ������1����¶�
		
		temp=pitch*10;							 //��tempΪpitch
		if(temp<0)								//�����������жϣ��ж�Ϊ��ʱ
		{
			temp=-temp;						    //�Ը�����ȡ��		
		}
		else                                    //�ж�Ϊ��ʱ 
		{
		}
		sprintf((char *)tmp_buf,"Pitch:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,16,(uint8_t *)tmp_buf,16);
		printf("pitch:%d.%d,",temp/10,temp%10); //ͨ������1���pitch	
		
		temp=roll*10;                            //��tempΪroll
		if(temp<0)								//�����������жϣ��ж�Ϊ��ʱ
		{
			temp=-temp;						    //�Ը�����ȡ��	
		}
		else                                    //�ж�Ϊ��ʱ
		{
		}
		sprintf((char *)tmp_buf,"roll:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,32,(uint8_t *)tmp_buf,16);
		printf("roll:%d.%d,",temp/10,temp%10);//ͨ������1���roll
		
		temp=yaw*10;                           //��tempΪyaw
		if(temp<0)								//�����������жϣ��ж�Ϊ��ʱ
		{
			temp=-temp;						    //�Ը�����ȡ��
		}
		else                                    //�ж�Ϊ��ʱ
		{
		}
		sprintf((char *)tmp_buf,"yaw:%d.%d",temp/10,temp%10);
		OLED_ShowString(0,48,(uint8_t *)tmp_buf,16);
		printf("yaw:%d.%d,",temp/10,temp%10);//ͨ������1���yaw	
		printf("gyrox:%d,gyroy:%d,gyroz:%d,aacx:%d,aacy:%d,aacz:%d\r\n",gyrox,gyroy,gyroz,aacx,aacy,aacz);//�ϱ����ٶ����ݣ��Ǽ��ٶ����ݸ
		mpu6050.flag = 0;									//�ɼ��ɹ���־λ����Ϊ��Ч
	}
	else ;														//������
}


void SYS_Init(void)
{
	MAP_WDT_A_holdTimer(); //�رտ��Ź�
	delay_init();	    							 //��ʱ������ʼ��	  
	uart_init(115200);	 	                         //���ڳ�ʼ��Ϊ115200
	LED_Init();		                               	 //��ʼ����LED���ӵ�Ӳ���ӿ�
	//OLED_Init();									 //OLED��ʼ��
	MPU_Init();	                                     //��ʼ��MPU6050
	while(mpu_dmp_init())                            //��ʼ��mpu_dmp��
 	{
		OLED_ShowString(0,0,"Failed",16);			 //��ʾ�ַ���
		//OLED_Refresh();	                             //ˢ���Դ�
		LED_R_Tog();delay_ms(50);LED_R_Tog();     //LED��˸ָʾ
		printf("Initialization failed��\r\n");		//���ڳ�ʼ��ʧ���ϱ�
//		OLED_ShowString(0,0,"      ",16);          //��ʾ�ַ���
		//OLED_Refresh();								//ˢ���Դ�
	}
	printf("Initialization successed��\r\n");		//���ڳ�ʼ���ɹ��ϱ�
//	OLED_ShowString(0,0,"  OK!",16);				//��ʾ�ַ���
	//OLED_Refresh();									//ˢ���Դ�
	delay_ms(999);									//��ʱ��������ʾ
	mpu6050.flag = 0;                               //�ɼ��ɹ���־λ��ʼ��
	mpu6050.speed = 0;								//�ϱ��ٶȳ�ʼ��
}

int main(void) 
{	
	SYS_Init();//ϵͳ��ʼ���ܺ���
	while(1)   //��ѭ��
	{
		MPU_Read();    //MPU6050���ݶ�ȡ
		DATA_Report(); //MPU6050�����ϱ�
	}
}
