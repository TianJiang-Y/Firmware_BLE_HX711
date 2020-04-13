
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


#include "sbp_BreathSensorApp.h"

static bool GPIO_ReadBit();
static void IIC_Start(void);
static void IIC_Send_Byte(uint8 data);
static uint8 IIC_Read_Byte(uint8 ack);
static uint8 IIC_Wait_Ack(void);
static void IIC_Ack(void);
static void IIC_NAck(void);
static void IIC_Stop(void);
//static void BBS_RightSensWakeUp(void);
//uint8 WR_Reg(uint16 reg, uint8 buf);

static bool GPIO_ReadBit()
{
    uint8 bitVal;
    
    if(rSDA) // Read bit, save it in bit
        bitVal=1;
    else
        bitVal=0;
    
    return bitVal;
}

static void IIC_Start(void)
{
    // Start Condition
    rSDA_OUTPUT();
    rSDA_HIGH();
    rSCL_HIGH();
    Delay_us(TCYBUF);
    rSDA_LOW();
    Delay_us(TCYBUF);
    rSCL_LOW();
}

static void IIC_Send_Byte(uint8 data)
{
    uint8 i;

    rSDA_OUTPUT();
    rSCL_LOW();
    Delay_us(TCYBUF);

    for (i = 0; i < 8; i++)
    {              
        if (((data & 0x80) >> 7) & 0x01)
        {
            rSDA_HIGH();
        }
        else
        {
            rSDA_LOW();
        }

        data <<= 1; 	     

        rSCL_HIGH();
        Delay_us(TCYBUF);
        rSCL_LOW();
        Delay_us(TCYBUF);
    }	 
}

static uint8 IIC_Read_Byte(uint8 ack)
{
    uint8 i, receive = 0;
    uint8 bit;

    rSDA_INPUT(); // SDA-input
    Delay_us(TCYBUF);

    for (i = 0; i < 8; i++)
    { 
        rSCL_LOW();
        Delay_us(TCYBUF);
        rSCL_HIGH();

        receive <<= 1;
        bit = GPIO_ReadBit();

        if (bit) 
            receive++;
    }	  				 

    if (!ack) 
        IIC_NAck();
    else 
        IIC_Ack(); 

    return receive;
}

static uint8 IIC_Wait_Ack()
{
    uint8 bit;
    uint8 ucErrTime = 0;

    rSDA_INPUT(); // SDA-input
    rSCL_HIGH();
    Delay_us(TCYBUF);
    bit = GPIO_ReadBit();

    while (bit)
    {
        ucErrTime++;

        if(ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        } 

        Delay_us(TCYBUF);
        bit = GPIO_ReadBit();
    }

    rSCL_LOW();

    return 0;  
}

static void IIC_Ack(void)
{
    rSCL_LOW();
    rSDA_OUTPUT();
    Delay_us(TCYBUF);
    rSDA_LOW();
    Delay_us(TCYBUF);
    rSCL_HIGH();
    Delay_us(TCYBUF);
    rSCL_LOW();
}

static void IIC_NAck(void)
{
    rSCL_LOW();
    rSDA_OUTPUT();
    Delay_us(TCYBUF);
    rSDA_HIGH();
    Delay_us(TCYBUF);
    rSCL_HIGH();
    Delay_us(TCYBUF);
    rSCL_LOW();
}

static void IIC_Stop(void)
{
    rSDA_OUTPUT();
    rSCL_HIGH();
    Delay_us(TCYBUF);
    rSDA_LOW();
    Delay_us(TCYBUF);
    rSDA_HIGH();
}

//uint8 WR_Reg(uint16 reg, uint8 buf)
//{
//	uint8 ret = 0;
//	IIC_Start();	
// 	IIC_Send_Byte(WRITE_ADDR);
//	IIC_Wait_Ack(); 	 										  		   
//	IIC_Send_Byte(reg & 0XFF);
//	IIC_Wait_Ack();  
//
//	IIC_Send_Byte(buf); 
//	ret = IIC_Wait_Ack();
//
//      IIC_Stop();
//	return ret; 
//}

uint16 BreathRightTemp_Capture(void)
{
    uint16 TempVal;
    uint8 SlaveAddress = MLX_SLAVEADDRESS << 1; //Contains device address
    uint8 MLX_Command = MLX_RAM_ACCESS | MLX_RAM_TOBJ1;
    uint8 DataL, DataH, PEC;
    uint8 Pecreg; //Calculated PEC byte storage
    uint8 arr[6]; //Buffer for the sent bytes
    uint16 TBuf = 0;

//    do{
        IIC_Start();	
        IIC_Send_Byte(SlaveAddress);
        IIC_Wait_Ack(); 	 										  		   
        IIC_Send_Byte(MLX_Command);
        IIC_Wait_Ack();  
        IIC_Start();  	 	   
        IIC_Send_Byte(SlaveAddress | 0x01);
        IIC_Wait_Ack();

        DataL = IIC_Read_Byte(0); //Read low data,master must send ACK
        DataH = IIC_Read_Byte(0); //Read high data,master must send ACK
        PEC = IIC_Read_Byte(1);   //Read PEC byte, master must send NACK

        IIC_Stop();
        
        arr[5] = (SlaveAddress);
        arr[4] = MLX_Command;
        arr[3] = (SlaveAddress | 0x01);
        arr[2] = DataL;
        arr[1] = DataH;
        arr[0] = 0;

        Pecreg = PEC_calculate(arr, 6); //Calculate CRC
        
//      }while(Pecreg != PEC); //If received and calculated CRC are equal go out from do-while{}
    
    TBuf = (((DataH & 0x7F) << 8) | (DataL & 0xFF));

//    HalLcdWriteStringValue( "RightVal= ", TBuf, 16,  HAL_LCD_LINE_6 );
        
    TempVal = Calc_TempValue(TBuf);
	
    return TempVal;
}
