
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"

#include "hal_lcd.h"

#include "hal_uart.h"
#include "hal_i2c.h"


#include "gatt.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "simpleGATTprofile.h"


#include "sbp_MPU9250App.h"
#include "math.h"


int16 AHRVal[10];

static uint8 buffer[30];

static uint8 AK8963_Afresh = 0x11;     
static uint8 MPU9250_Reset = 0x80;
static uint8 MPU9250_WakeUp = 0x00;
static uint8 MPU9250_DisableINT = 0x00;
static uint8 MPU9250_DisableI2CMaster = 0x00;
static uint8 MPU9250_DisableFIFO = 0x00;
static uint8 MPU9250_BypassMode = 0x82;
static uint8 MPU9250_PWRSet01 = 0x01;
static uint8 MPU9250_PWRSet02 = 0x00;
static uint8 AK8963_Reset = 0x01;

void delay_ms(unsigned int ms)
{                         
    unsigned int a;
    while(ms)
    {
        a=1800;
        while(a--);
        ms--;
    }
}

static void IIC_Init(void)
{
    HalI2CInit(MPU9250_ADDR, i2cClock_533KHZ);
}

/*******************************************************************************/

/*******************************************************************************
* @param       addr - which register to write
* @param       pBuf - pointer to buffer containing data to be written
* @param       nBytes - number of bytes to write
*
* @return      TRUE if successful write
*/
static bool MPU_WriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
    uint8 i;
    uint8 *p = buffer;

    /* Copy address and data to local buffer for burst write */
    *p++ = addr;
    for (i = 0; i < nBytes; i++)
    {
      *p++ = *pBuf++;
    }
    nBytes++;

    /* Send address and data */
    i = HalI2CWrite(nBytes, buffer);
    
    if ( i!= nBytes)
      return 0;

    return (i == nBytes);
} 
/********************************************************************************
 * @param       addr - which register to read
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - numbver of bytes to read
 *
 * @return      TRUE if the required number of bytes are reveived
 ******************************************************************************/
static bool MPU_ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{ 
    uint8 i = 0;

    /* Send address we're reading from */
    if (HalI2CWrite(1,&addr) == 1)
    {
      /* Now read data */
      i = HalI2CRead(nBytes,pBuf);
    }

    return i == nBytes;      
}


//����MPU9250�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8 MPU_Set_Gyro_Fsr(uint8 fsr)
{
    uint8 WriteData = fsr << 3;
    HalI2CInit(MPU9250_ADDR, i2cClock_533KHZ);
    
    return MPU_WriteReg(MPU_GYRO_CFG_REG,&WriteData, 1);//���������������̷�Χ  
}

//����MPU9250���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8 MPU_Set_Accel_Fsr(uint8 fsr)
{
    uint8 WriteData = fsr << 3;
    HalI2CInit(MPU9250_ADDR, i2cClock_533KHZ);
    
    return MPU_WriteReg(MPU_ACCEL_CFG_REG,&WriteData, 1);//���ü��ٶȴ����������̷�Χ  
}

//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8 MPU_Set_LPF(uint16 lpf)
{
    uint8 data = 0;
    
    HalI2CInit(MPU9250_ADDR, i2cClock_533KHZ);
    
    if(lpf >= 188)
      data = 1;
    else if(lpf >= 98)
      data = 2;
    else if(lpf >= 42)
      data = 3;
    else if(lpf >= 20)
      data = 4;
    else if(lpf >= 10)
      data = 5;
    else 
      data = 6; 
    
    return MPU_WriteReg(MPU_CFG_REG,&data, 1);//�������ֵ�ͨ�˲���  
}

//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8 MPU_Set_Rate(uint16 rate)
{
    uint8 data;
    
    HalI2CInit(MPU9250_ADDR, i2cClock_533KHZ);
    
    if(rate > 1000) 
      rate = 1000;
    if(rate < 4) 
      rate = 4;
    
    data = 1000 / rate - 1;
    
    MPU_WriteReg(MPU_SAMPLE_RATE_REG,&data, 1);	//�������ֵ�ͨ�˲���
    
    return MPU_Set_LPF(rate / 2);	//�Զ�����LPFΪ�����ʵ�һ��
}


//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
float MPU_Get_Temperature(void)
{
    uint8 buf[2]; 
    int16 raw;
    float temp;
    
    HalI2CInit(MPU9250_ADDR, i2cClock_533KHZ);
    
    MPU_ReadReg(MPU_TEMP_OUTH_REG,buf, 2); 
    
    raw = ((int16)buf[0] << 8) | buf[1];  
    temp = 21 + ((double)raw / 333.87);  
    
    return temp;
}

//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8 MPU_Get_Gyroscope(int16 *gx,int16 *gy,int16 *gz)
{
    uint8 buf[6],res; 
    
    HalI2CInit(MPU9250_ADDR, i2cClock_533KHZ);
    
    res = MPU_ReadReg(MPU_GYRO_XOUTH_REG,buf, 6);
    
    if(res)
    {
        *gx = ((uint16)buf[0] << 8) | buf[1];  
        *gy = ((uint16)buf[2] << 8) | buf[3];  
        *gz = ((uint16)buf[4] << 8) | buf[5];
    } 	
    
    return res;
}

//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8 MPU_Get_Accelerometer(int16 *ax,int16 *ay,int16 *az)
{
    uint8 buf[6],res;  
    
    HalI2CInit(MPU9250_ADDR, i2cClock_533KHZ);
    
    res = MPU_ReadReg(MPU_ACCEL_XOUTH_REG,buf, 6);
    
    if(res)
    {
        *ax = ((uint16)buf[0] << 8) | buf[1];  
        *ay = ((uint16)buf[2] << 8) | buf[3];  
        *az = ((uint16)buf[4] << 8) | buf[5];
    } 	
    
    return res;
}

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
// ����,�������
uint8 MPU_Get_Magnetometer(int16 *mx,int16 *my,int16 *mz)
{
    uint8 buf[6],res;  
    
    HalI2CInit(AK8963_ADDR, i2cClock_533KHZ);
    
    res = MPU_ReadReg(MAG_XOUT_L,buf, 6);
    
    if(res)
    {
        *mx = ((uint16)buf[1] << 8) | buf[0];  
        *my = ((uint16)buf[3] << 8) | buf[2];  
        *mz = ((uint16)buf[5] << 8) | buf[4];
    } 	
    
    MPU_WriteReg(MAG_CNTL1,&AK8963_Afresh, 1); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    
    return res;
}

/*******************************************************************************/

uint8 MPU9250_Init(void)
{
    uint8 Re = 0, Ree = 0;
////////    uint8 ReturnVal = 100;
    
    //��ʼ��IIC����
    IIC_Init();
    
    //��λMPU9250
    MPU_WriteReg(MPU_PWR_MGMT1_REG,&MPU9250_Reset, 1);
    
    delay_ms(100);  //��ʱ100ms
    
    //����MPU9250
    MPU_WriteReg(MPU_PWR_MGMT1_REG,&MPU9250_WakeUp, 1);
    
    MPU_Set_Gyro_Fsr(3);	//�����Ǵ�����,��2000dps
    MPU_Set_Accel_Fsr(0);       //���ٶȴ�����,��2g
    MPU_Set_Rate(50);		//���ò�����50Hz
    
    //�ر������ж�
    MPU_WriteReg(MPU_INT_EN_REG,&MPU9250_DisableINT, 1);
    
    //I2C��ģʽ�ر�
    MPU_WriteReg(MPU_USER_CTRL_REG,&MPU9250_DisableI2CMaster, 1);
    
    //�ر�FIFO
    MPU_WriteReg(MPU_FIFO_EN_REG,&MPU9250_DisableFIFO, 1);
    
    //INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������    
    MPU_WriteReg(MPU_INTBP_CFG_REG,&MPU9250_BypassMode, 1);
    
    //��ȡMPU6500��ID
    MPU_ReadReg(MPU_DEVICE_ID_REG, &Re, 1);
    
    
    //����ID��ȷ
//    if(Re == MPU6500_ID)
      {
          //����CLKSEL,PLL X��Ϊ�ο�
          MPU_WriteReg(MPU_PWR_MGMT1_REG,&MPU9250_PWRSet01, 1);
          
          //���ٶ��������Ƕ�����
          MPU_WriteReg(MPU_PWR_MGMT2_REG,&MPU9250_PWRSet02, 1);
          
          //���ò�����Ϊ50Hz  
          MPU_Set_Rate(50); 
      }
//    else  return 1;
    
    HalI2CInit(AK8963_ADDR, i2cClock_533KHZ);
 
    //��ȡAK8963 ID 
    MPU_ReadReg(MAG_WIA, &Ree, 1);  
    
    if(Ree == AK8963_ID)
      {
          //��λAK8963
          MPU_WriteReg(MAG_CNTL2,&AK8963_Reset, 1);	
          
          delay_ms(50);
          
          //����AK8963Ϊ���β���ģʽ
          MPU_WriteReg(MAG_CNTL1,&AK8963_Afresh, 1);
      }
//    else  return 1;
    
    delay_ms(1);  // Debugging.............

    return 0;
}

/********************************************************************************/

int16* MEMS_Get_AHRSVal(void)
{
//    float pitch,roll,yaw; 	        //ŷ����
    int16 aacex,aacey,aacez;	        //���ٶȴ�����ԭʼ����
    int16 gyrox,gyroy,gyroz;           //������ԭʼ���� 
    int16 magnx, magny, magnz;
    float temperature;		        //�¶�
    
    temperature = MPU_Get_Temperature();	        //�õ��¶�ֵ
    delay_ms(1);
    MPU_Get_Accelerometer(&aacex,&aacey,&aacez);	//�õ����ٶȴ���������
    delay_ms(1);
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	        //�õ�����������
    delay_ms(1);
    MPU_Get_Magnetometer(&magnx, &magny, &magnz);
    
    AHRVal[0] = aacex;
    AHRVal[1] = aacey;
    AHRVal[2] = aacez;
    AHRVal[3] = gyrox;
    AHRVal[4] = gyroy;
    AHRVal[5] = gyroz;
    AHRVal[6] = magnx;
    AHRVal[7] = magny;
    AHRVal[8] = magnz;
    AHRVal[9] = (int8)temperature;
    
    return AHRVal;
}



/*******************************************************************************/
