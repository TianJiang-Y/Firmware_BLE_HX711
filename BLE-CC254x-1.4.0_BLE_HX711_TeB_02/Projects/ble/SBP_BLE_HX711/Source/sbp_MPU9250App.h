
#ifndef _SBP_MPU9250APP_H_
#define _SBP_MPU9250APP_H_

#ifdef __cplusplus
extern "C"
{
#endif
  
  
//I2C Pins of CC2541 Setting to GPIO
  /* MEMS Sensor SDA is at pins3 */
////////#define I2CWRAPPER_PORT                I2CWC
////////#define I2CGPIO_PORT                   I2CIO
////////#define I2CPIN_ENABLE_GPIO             BV(7)
////////#define SDA_BIT                        BV(0)
////////#define SDA_PULLUP                     BV(2)
////////#define SDA                            BV(0)
////////
////////#define SCL_BIT                        BV(1)
////////#define SCL_PULLUP                     BV(3)
////////#define SCL                            BV(1)
////////// clr 0 == SDA pin input enable
////////#define SDA_INPUT()         st( I2CWC &= ~(SDA_BIT); )
////////// set 1 == SDA pin output enable
////////#define SDA_OUTPUT()        st( I2CWC |= (SDA_BIT); )
////////  
////////#define SDA_HIGH()          st(I2CIO |= SDA;)     // Sets SDA line
////////#define SDA_LOW()           st(I2CIO &= ~(SDA);)  // Clears SDA line
////////#define SCL_HIGH()          st(I2CIO |= SCL;)     // Sets SCL line
////////#define SCL_LOW()           st(I2CIO &= ~(SCL);)  // Clears SCL line

/***************************************************************************/

#define MPU9250_ADDR            0X68    //MPU6500������IIC��ַ
#define MPU6500_ID	        0X71    //MPU6500������ID

//MPU9250�ڲ���װ��һ��AK8963������,��ַ��ID����:
#define AK8963_ADDR		0X0C	//AK8963��I2C��ַ
#define AK8963_ID		0X48	//AK8963������ID


//AK8963���ڲ��Ĵ���
#define MAG_WIA			0x00	//AK8963������ID�Ĵ�����ַ
#define MAG_CNTL1          	0X0A    
#define MAG_CNTL2            	0X0B

#define MAG_XOUT_L		0X03	
#define MAG_XOUT_H		0X04
#define MAG_YOUT_L		0X05
#define MAG_YOUT_H		0X06
#define MAG_ZOUT_L		0X07
#define MAG_ZOUT_H		0X08

//MPU6500���ڲ��Ĵ���
#define MPU_SELF_TESTX_REG		0X0D	//�Լ�Ĵ���X
#define MPU_SELF_TESTY_REG		0X0E	//�Լ�Ĵ���Y
#define MPU_SELF_TESTZ_REG		0X0F	//�Լ�Ĵ���Z
#define MPU_SELF_TESTA_REG		0X10	//�Լ�Ĵ���A
#define MPU_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define MPU_CFG_REG			0X1A	//���üĴ���
#define MPU_GYRO_CFG_REG		0X1B	//���������üĴ���
#define MPU_ACCEL_CFG_REG		0X1C	//���ٶȼ����üĴ���
#define MPU_MOTION_DET_REG		0X1F	//�˶���ֵⷧ���üĴ���
#define MPU_FIFO_EN_REG			0X23	//FIFOʹ�ܼĴ���
#define MPU_I2CMST_CTRL_REG		0X24	//IIC�������ƼĴ���
#define MPU_I2CSLV0_ADDR_REG	        0X25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU_I2CSLV0_REG			0X26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU_I2CSLV0_CTRL_REG	        0X27	//IIC�ӻ�0���ƼĴ���
#define MPU_I2CSLV1_ADDR_REG	        0X28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU_I2CSLV1_REG			0X29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU_I2CSLV1_CTRL_REG	        0X2A	//IIC�ӻ�1���ƼĴ���
#define MPU_I2CSLV2_ADDR_REG	        0X2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU_I2CSLV2_REG			0X2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU_I2CSLV2_CTRL_REG	        0X2D	//IIC�ӻ�2���ƼĴ���
#define MPU_I2CSLV3_ADDR_REG	        0X2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU_I2CSLV3_REG			0X2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU_I2CSLV3_CTRL_REG	        0X30	//IIC�ӻ�3���ƼĴ���
#define MPU_I2CSLV4_ADDR_REG	        0X31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU_I2CSLV4_REG			0X32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU_I2CSLV4_DO_REG		0X33	//IIC�ӻ�4д���ݼĴ���
#define MPU_I2CSLV4_CTRL_REG	        0X34	//IIC�ӻ�4���ƼĴ���
#define MPU_I2CSLV4_DI_REG		0X35	//IIC�ӻ�4�����ݼĴ���

#define MPU_I2CMST_STA_REG		0X36	//IIC����״̬�Ĵ���
#define MPU_INTBP_CFG_REG		0X37	//�ж�/��·���üĴ���
#define MPU_INT_EN_REG			0X38	//�ж�ʹ�ܼĴ���
#define MPU_INT_STA_REG			0X3A	//�ж�״̬�Ĵ���

#define MPU_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU_TEMP_OUTH_REG		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU_TEMP_OUTL_REG		0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU_I2CSLV0_DO_REG		0X63	//IIC�ӻ�0���ݼĴ���
#define MPU_I2CSLV1_DO_REG		0X64	//IIC�ӻ�1���ݼĴ���
#define MPU_I2CSLV2_DO_REG		0X65	//IIC�ӻ�2���ݼĴ���
#define MPU_I2CSLV3_DO_REG		0X66	//IIC�ӻ�3���ݼĴ���

#define MPU_I2CMST_DELAY_REG	        0X67	//IIC������ʱ����Ĵ���
#define MPU_SIGPATH_RST_REG		0X68	//�ź�ͨ����λ�Ĵ���
#define MPU_MDETECT_CTRL_REG	        0X69	//�˶������ƼĴ���
#define MPU_USER_CTRL_REG		0X6A	//�û����ƼĴ���
#define MPU_PWR_MGMT1_REG		0X6B	//��Դ����Ĵ���1
#define MPU_PWR_MGMT2_REG		0X6C	//��Դ����Ĵ���2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO�����Ĵ����߰�λ
#define MPU_FIFO_CNTL_REG		0X73	//FIFO�����Ĵ����Ͱ�λ
#define MPU_FIFO_RW_REG			0X74	//FIFO��д�Ĵ���
#define MPU_DEVICE_ID_REG		0X75	//����ID�Ĵ���
  
  
//////static void IIC_Init(void);
//////static bool GPIO_SDAD_ReadBit(void);
//////static void IIC_Start(void);
//////static void IIC_Send_Byte(uint8 data);
//////static uint8 IIC_Read_Byte(uint8 ack);
//////static uint8 IIC_Wait_Ack();
//////static void IIC_Ack(void);
//////static void IIC_NAck(void);
//////static void IIC_Stop(void);

uint8 MPU_Write_Len(uint8 addr,uint8 reg,uint8 len,uint8 *buf);
uint8 MPU_Read_Len(uint8 addr,uint8 reg,uint8 len,uint8 *buf);
uint8 MPU_Write_Byte(uint8 addr,uint8 reg,uint8 data);
uint8 MPU_Read_Byte(uint8 addr,uint8 reg);

uint8 MPU9250_Init(void);

uint8 MPU_Set_Gyro_Fsr(uint8 fsr);
uint8 MPU_Set_Accel_Fsr(uint8 fsr);
uint8 MPU_Set_LPF(uint16 lpf);
uint8 MPU_Set_Rate(uint16 rate);

float MPU_Get_Temperature(void);
uint8 MPU_Get_Gyroscope(int16 *gx,int16 *gy,int16 *gz);
uint8 MPU_Get_Accelerometer(int16 *ax,int16 *ay,int16 *az);
uint8 MPU_Get_Magnetometer(int16 *mx,int16 *my,int16 *mz);

//void MPU_Set_Gyro_Fsr(uint8 fsr);
//void MPU_Set_Accel_Fsr(uint8 fsr);
//void MPU_Set_LPF(uint16 lpf);
//void MPU_Set_Rate(uint16 rate);
//
//void MPU_Get_Temperature(float *temp);
//void MPU_Get_Gyroscope(int16 *gx,int16 *gy,int16 *gz);
//void MPU_Get_Accelerometer(int16 *ax,int16 *ay,int16 *az);
//void MPU_Get_Magnetometer(int16 *mx,int16 *my,int16 *mz);
//
//void MPU_ReadData(uint8 addr,uint8 reg,uint8 len,uint8 *buf);
//void MPU_WriteReg(uint8 addr,uint8 reg,uint8 data);

int16 *MEMS_Get_AHRSVal(void);

  
#ifdef __cplusplus
}
#endif

#endif
