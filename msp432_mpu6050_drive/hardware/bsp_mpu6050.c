#include "bsp_mpu6050.h"
#include "delay.h"
#include  "string.h"
#include "stdio.h"



//IO��������	 
#define IIC_SCL_High()  GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN5) //SCL_High
#define IIC_SCL_Low()   GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN5) //SCL_Low
#define IIC_SDA_High()  GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN4) //SDA_High
#define IIC_SDA_Low()   GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN4) //SDA_Low
#define READ_SDA        GPIO_getInputPinValue(GPIO_PORT_P6,GPIO_PIN4)  //����SDA 
#define  SCL_PORT  GPIO_PORT_P6   // �ӣ�SCL��
#define  SCL_PIN   GPIO_PIN5      // �ӣ�SCL��
#define  SDA_PORT  GPIO_PORT_P6   // �ӣ�SDA��
#define  SDA_PIN   GPIO_PIN4   // �ӣ�SDA��


//��ʼ��IIC
void IIC_Init(void)
{			
  GPIO_setAsOutputPin(SCL_PORT,SCL_PIN ); //CLK         
  GPIO_setAsOutputPin(SDA_PORT,SDA_PIN);//DIN
	IIC_SCL_High();
	IIC_SDA_High();
}
//����IIC��ʼ�ź�
void IIC_Start(void)//SDA 10 SCL 010
{
	SDA_OUT();     //sda�����
	IIC_SCL_High();
	IIC_SDA_High();
	delay_us(4);
 	IIC_SDA_Low();//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_Low();//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)//SDA 01 SCL 01
{
	SDA_OUT();//sda�����
	IIC_SCL_Low();//STOP:when CLK is high DATA change form low to high
    IIC_SDA_Low();
 	delay_us(4);
    IIC_SCL_High();
	IIC_SDA_High();//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)//
{
	uint8_t cy;
    SDA_IN();      //SDA����Ϊ����
    IIC_SCL_High();delay_us(10);
	IIC_SDA_High();delay_us(10);
    if(READ_SDA)
    {
        cy=1;
        IIC_SCL_Low();
        return cy; 
    }      
    else
    {
        cy=0;
    }             
	IIC_SCL_Low();//ʱ�����0
	return cy;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
    IIC_SCL_Low();
	SDA_OUT();
    IIC_SDA_Low();
	delay_us(2);
    IIC_SCL_High();
    delay_us(2);
	IIC_SCL_Low();
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL_Low();
	SDA_OUT();
    IIC_SDA_High();
	delay_us(2);
    IIC_SCL_High();
    delay_us(2);
	IIC_SCL_Low();
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 
    IIC_SCL_Low();//����ʱ�ӿ�ʼ���ݴ���
    delay_us(2);
    for(t=0;t<8;t++)
    {       
        if(txd&0x80)
        {
            IIC_SDA_High();delay_us(2); 
        }  
        else
        {
            IIC_SDA_Low();delay_us(2);  
        }
        txd<<=1;
		IIC_SCL_High();
		delay_us(4); 
        IIC_SCL_Low();
        delay_us(2); 
    }
        delay_us(2);
    
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
    uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
		IIC_SCL_Low();
        delay_us(2);
        IIC_SCL_High();
        receive<<=1;
        if(READ_SDA)
            receive++;
        delay_us(2);
    }
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

uint8_t MPU_Init(void)
{ 
	uint8_t res;
    
	IIC_Init();//��ʼ��IIC����
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);//����MPU6050
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);//��ʼ��MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0x07);
    MPU_Write_Byte(MPU_CFG_REG,0x06);
    MPU_Write_Byte(MPU_GYRO_CFG_REG,0x18);
    MPU_Write_Byte(MPU_ACCEL_CFG_REG,0x01);
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(0X75);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if(IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte(MPU_WRITE);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte(MPU_READ);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte(MPU_WRITE);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();	
        delay_ms(100);
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	if(IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
    IIC_Start(); 
	IIC_Send_Byte(MPU_WRITE);//����������ַ+д����	
	IIC_Wait_Ack();      
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();
    IIC_Start();
	IIC_Send_Byte(MPU_READ);//����������ַ+������	
    IIC_Wait_Ack(); 
	res=IIC_Read_Byte(0);//��ȡ����,����nACK 
    IIC_Stop();			//����һ��ֹͣ���� 
	return res;		
}
