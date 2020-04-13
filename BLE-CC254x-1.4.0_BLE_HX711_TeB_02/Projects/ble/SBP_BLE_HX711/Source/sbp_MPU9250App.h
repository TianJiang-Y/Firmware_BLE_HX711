
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

#define MPU9250_ADDR            0X68    //MPU6500的器件IIC地址
#define MPU6500_ID	        0X71    //MPU6500的器件ID

//MPU9250内部封装了一个AK8963磁力计,地址和ID如下:
#define AK8963_ADDR		0X0C	//AK8963的I2C地址
#define AK8963_ID		0X48	//AK8963的器件ID


//AK8963的内部寄存器
#define MAG_WIA			0x00	//AK8963的器件ID寄存器地址
#define MAG_CNTL1          	0X0A    
#define MAG_CNTL2            	0X0B

#define MAG_XOUT_L		0X03	
#define MAG_XOUT_H		0X04
#define MAG_YOUT_L		0X05
#define MAG_YOUT_H		0X06
#define MAG_ZOUT_L		0X07
#define MAG_ZOUT_H		0X08

//MPU6500的内部寄存器
#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG			0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	        0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	        0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	        0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	        0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	        0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	        0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	        0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	        0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	        0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	        0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	        0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	        0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
  
  
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
