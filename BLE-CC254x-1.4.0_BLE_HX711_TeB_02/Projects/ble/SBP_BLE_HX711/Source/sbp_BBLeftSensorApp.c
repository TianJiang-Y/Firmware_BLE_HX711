
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

static bool GPIO_SDAD_ReadBit();
static void IIC2_Start(void);
static void IIC2_Send_Byte(uint8 data);
static uint8 IIC2_Read_Byte(uint8 ack);
static uint8 IIC2_Wait_Ack();
static void IIC2_Ack(void);
static void IIC2_NAck(void);
static void IIC2_Stop(void);
static void BBS_LeftSensInit(void);
static void BBS_RightSensInit(void);

//static uint8 BBS_EEPROMInitial[2] = {0x00, 0x00};   //Erase (write 0) EEPROM
//static uint8 BBS_EEPROMTomax[2] = {0x12, 0x41};     //60.01 c
//static uint8 BBS_EEPROMTomin[2] = {0x10, 0x01};     //50 ms non sleep and 1.15 seconds sleep
/******************************************************************************/
void Delay_ms(unsigned int ms)
{                         
    unsigned int a;
    while(ms)
    {
        a=1800;
        while(a--);
        ms--;
    }
}


void Delay_us(unsigned int times) // 3us
{
  while(times--)
  {
     asm("nop");
  }
}

/*******************************************************************************/
//      Both Sensor Set Up
/*******************************************************************************/

static void BBS_LeftSensInit(void)
{
    LEFTSENS_POW_SEL &= ~(LEFTSENS_POW_BIT);    /* Set pin function to GPIO */
    LEFTSENS_POW_DIR |= LEFTSENS_POW_BIT;       /* Set pin direction to Output */
    
    LeSDA_SEL &= ~(LeSDA_BIT); // set SDA pin P1.2 function to GPIO
    LeSDA_OUTPUT();           // set SDA pin P1.2 direction to output

    LeSCL_SEL &= ~(LeSCL_BIT); // set SCL pin P1.1 function to GPIO
    LeSCL_DIR |= LeSCL_BIT;    // set always SCL pin P0.7 direction to output
}

static void BBS_RightSensInit(void)
{
    RIGHTSENS_POW_SEL &= ~(RIGHTSENS_POW_BIT);    /* Set pin function to GPIO */
    RIGHTSENS_POW_DIR |= RIGHTSENS_POW_BIT;       /* Set pin direction to Output */

    rSDA_SEL &= ~(rSDA_BIT); // set SDA pin P0.6 function to GPIO
    rSDA_OUTPUT();           // set SDA pin P0.6 direction to output

    rSCL_SEL &= ~(rSCL_BIT); // set SCL pin P0.7 function to GPIO
    rSCL_DIR |= rSCL_BIT;    // set always SCL pin P0.7 direction to output
}

void BBS_BothSensorTurnOn(void)
{
    BBS_LeftSensInit();  
    BBS_RightSensInit();
    
    // Sensor POR
    LEFTSENS_POWER = POWER_OFF;
    Delay_ms(2);
    RIGHTSENS_POWER = POWER_OFF;
    Delay_ms(10);   // Ensure POR signal
    LEFTSENS_POWER = POWER_ON;    
    Delay_ms(2);
    RIGHTSENS_POWER = POWER_ON;
    Delay_ms(200);  // Tvalid = 0.25s, No below 200ms
    
    //Left Sensor SMBus setting-up
    LeSDA_HIGH();
    LeSCL_HIGH(); 
      
    //Right Sensor SMBus setting-up
    rSDA_HIGH();       //The bus is in idle state
    rSCL_HIGH();       //SDA and SCL are in high level from pull up resitors
}

void BBS_BothSensorTurnOff(void)
{
    LEFTSENS_POWER = POWER_OFF;
    Delay_ms(3);
    RIGHTSENS_POWER = POWER_OFF;
}

/***********************************************************************************/
static bool GPIO_SDAD_ReadBit()
{
    uint8 bitVal;
    
    if(LeSDA) // Read bit, save it in bit
        bitVal=1;
    else
        bitVal=0;
    
    return bitVal;
}

static void IIC2_Start(void)
{
    // Start Condition
    LeSDA_OUTPUT();
    LeSDA_HIGH();
    LeSCL_HIGH();
    Delay_us(TCYBUF);
    LeSDA_LOW();
    Delay_us(TCYBUF);
    LeSCL_LOW();
}

static void IIC2_Send_Byte(uint8 data)
{
    uint8 i;

    LeSDA_OUTPUT();
    LeSCL_LOW();
    Delay_us(TCYBUF);

    for (i = 0; i < 8; i++)
    {              
        if (((data & 0x80) >> 7) & 0x01)
        {
            LeSDA_HIGH();
        }
        else
        {
            LeSDA_LOW();
        }

        data <<= 1; 	     

        LeSCL_HIGH();
        Delay_us(TCYBUF);
        LeSCL_LOW();
        Delay_us(TCYBUF);
    }	 
}

static uint8 IIC2_Read_Byte(uint8 ack)
{
    uint8 i, receive = 0;
    uint8 bit;

    LeSDA_INPUT(); // SDA-input
    Delay_us(TCYBUF);

    for (i = 0; i < 8; i++)
    { 
        LeSCL_LOW();
        Delay_us(TCYBUF);
        LeSCL_HIGH();

        receive <<= 1;
        bit = GPIO_SDAD_ReadBit();

        if (bit) 
            receive++;
    }

    if (!ack) 
        IIC2_NAck();
    else 
        IIC2_Ack(); 

    return receive;
}

static uint8 IIC2_Wait_Ack()
{
    uint8 bit;
    uint8 ucErrTime = 0;

    LeSDA_INPUT(); // SDA-input
    LeSCL_HIGH();
    Delay_us(TCYBUF);
    bit = GPIO_SDAD_ReadBit();

    while (bit)
    {
        ucErrTime++;

        if(ucErrTime > 250)
        {
            IIC2_Stop();
            return 1;
        } 

        Delay_us(TCYBUF);
        bit = GPIO_SDAD_ReadBit();
    }

    LeSCL_LOW();

    return 0;  
}

static void IIC2_Ack(void)
{
    LeSCL_LOW();
    LeSDA_OUTPUT();
    Delay_us(TCYBUF);
    LeSDA_LOW();
    Delay_us(TCYBUF);
    LeSCL_HIGH();
    Delay_us(TCYBUF);
    LeSCL_LOW();
}

static void IIC2_NAck(void)
{
    LeSCL_LOW();
    LeSDA_OUTPUT();
    Delay_us(TCYBUF);
    LeSDA_HIGH();
    Delay_us(TCYBUF);
    LeSCL_HIGH();
    Delay_us(TCYBUF);
    LeSCL_LOW();
}

static void IIC2_Stop(void)
{
    LeSDA_OUTPUT();
    LeSCL_HIGH();
    Delay_us(TCYBUF);
    LeSDA_LOW();
    Delay_us(TCYBUF);
    LeSDA_HIGH();
}

//----------------------------------------------------------------------------------------------------------------------------------------//
//CALCULATE THE PEC PACKET
//----------------------------------------------------------------------------------------------------------------------------------------//
uint8 PEC_calculate(uint8 pec[],int n)
{
    uint8 crc[6];
    uint8 Bitposition=47;
    uint8 shift;
    uint8 i;
    uint8 j;
    uint8 temp;

    do
    {
      crc[5]=0; //Load CRC value 0x000000000107
      crc[4]=0;
      crc[3]=0;
      crc[2]=0;
      crc[1]=0x01;
      crc[0]=0x07;

      Bitposition = 47; //Set maximum bit position at 47
      shift=0;        //Find first 1 in the transmitted bytes
      i=5; //Set highest index (package byte index)
      j=0; //Byte bit index, from lowest

      while((pec[i] & (0x80 >> j)) == 0 && (i > 0))
      {
         Bitposition--;
         if(j<7)
          { 
              j++;
          }
         else 
          {
              j = 0x00;
              i--;
          }
      }//the position of highest "1" bit in Bitposition is calculated

      shift = Bitposition - 8; //Get shift value for CRC value
	
      while(shift)
      {
          for(i = 5; i < 0xFF; i--)
            {
               if((crc[i-1] & 0x80) && (i > 0)) //Check if the MSB of the byte lower is "1"
                {             //Yes - current byte + 1
                    temp = 1; //No - current byte + 0
                }             //So that "1" can shift between bytes
               else 
                {
                    temp = 0;
                }

              crc[i] <<= 1;
              crc[i] += temp;
            }

          shift--;
      }
      //Exclusive OR between pec and crc
      for(i = 0; i <= 5; i++) 
        {
            pec[i] ^= crc[i]; 
        }
      
    }while(Bitposition > 8);

    return pec[0];
}
	
//-------------------------------------------------
//Name: Calc_TempValue           
//Temperature Value is T=(Value)*0.02-273.15
// Tmin = 0'C, Tmax = 60'C
//-------------------------------------------------
uint16 Calc_TempValue(uint16 Value)
{
   uint32 T;
   uint16 A;
   
   T = Value * 2;
   A = T - 27315;
   return A;
}

/*******************************************************************************/
uint16 BreathLeftTemp_Capture(void)
{
    uint16 TempVal2;
    uint16 TBuf2 = 0;
    uint8 PEC2 = 0;
    uint8 DataL2;
    uint8 DataH2; //Data packets from MLX90614
    uint8 Pecreg2; //Calculated PEC byte storage
    uint8 arr2[6]; //Buffer for the sent bytes
    uint8 SlaveAddress2 = MLX_SLAVEADDRESS << 1; //Contains device address
    uint8 MLX_Command2 = MLX_RAM_ACCESS | MLX_RAM_TOBJ1;

//    do{
        IIC2_Start();	
        IIC2_Send_Byte(SlaveAddress2);
        IIC2_Wait_Ack(); 	 										  		   
        IIC2_Send_Byte(MLX_Command2);
        IIC2_Wait_Ack();  
        IIC2_Start();  	 	   
        IIC2_Send_Byte(SlaveAddress2 | 0x01);
        IIC2_Wait_Ack();

        DataL2 = IIC2_Read_Byte(0); //Read low data,master must send ACK
        DataH2 = IIC2_Read_Byte(0); //Read high data,master must send ACK
        PEC2 = IIC2_Read_Byte(1);   //Read PEC byte, master must send NACK

        IIC2_Stop();
        
        arr2[5] = (SlaveAddress2);
        arr2[4] = MLX_Command2;
        arr2[3] = (SlaveAddress2 | 0x01);
        arr2[2] = DataL2;
        arr2[1] = DataH2;
        arr2[0] = 0;

        Pecreg2 = PEC_calculate(arr2, 6); //Calculate CRC
        
//      }while(Pecreg2 != PEC2);
       
    TBuf2 = (((DataH2 & 0x7F) << 8) | (DataL2 & 0xFF));
//    HalLcdWriteStringValue( "LeftVal = ", TBuf, 16,  HAL_LCD_LINE_6 );
    
    TempVal2 = Calc_TempValue(TBuf2);
  
    return TempVal2;
}

/*******************************************************************************/
