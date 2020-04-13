
#ifndef _SBP_BREATHSENSORAPP_H_
#define _SBP_BREATHSENSORAPP_H_

#ifdef __cplusplus
extern "C"
{
#endif

  /* Left Sensor Power is at P1.0 */
#define LEFTSENS_POW_PORT              P1
#define LEFTSENS_POW_BIT               BV(0)
#define LEFTSENS_POW_SEL               P1SEL
#define LEFTSENS_POW_DIR               P1DIR

#define LEFTSENS_POWER                 P1_0
//#define LEFTSENS_POW_POLARITY          ACTIVE_HIGH
  
//SMBus control signals For Left Sensor
  /* SDA --> P1.2 */
#define LeSDA_PORT                      P1
#define LeSDA_BIT                       BV(2)
#define LeSDA_SEL                       P1SEL
#define LeSDA_DIR                       P1DIR

#define LeSDA                           P1_2
  /* SCL --> P1.1 */
#define LeSCL_PORT                      P1
#define LeSCL_BIT                       BV(1)
#define LeSCL_SEL                       P1SEL
#define LeSCL_DIR                       P1DIR

#define LeSCL                           P1_1
/************************************************************/
#define LeSDA_INPUT()     LeSDA_DIR &= ~(LeSDA_BIT);  // clr 0 == input
#define LeSDA_OUTPUT()    LeSDA_DIR |= LeSDA_BIT;     // set 1 == output
  
#define LeSDA_HIGH()      LeSDA = 1;  // Sets SDA line
#define LeSDA_LOW()       LeSDA = 0;  // Clears SDA line
#define LeSCL_HIGH()      LeSCL = 1;  // Sets SCL line
#define LeSCL_LOW()       LeSCL = 0;  // Clears SCL line
/***********************************************************************/  
  /* Right Sensor Power is at P0.5 */
#define RIGHTSENS_POW_PORT             P0
#define RIGHTSENS_POW_BIT              BV(5)
#define RIGHTSENS_POW_SEL              P0SEL
#define RIGHTSENS_POW_DIR              P0DIR

#define RIGHTSENS_POWER                P0_5
//#define RIGHTSENS_POW_POLARITY         ACTIVE_HIGH
  
//SMBus control signals For Right Sensor
  /* SDA --> P0.6 */
#define rSDA_PORT                      P0
#define rSDA_BIT                       BV(6)
#define rSDA_SEL                       P0SEL
#define rSDA_DIR                       P0DIR

#define rSDA                           P0_6
  /* SCL --> P0.7 */
#define rSCL_PORT                      P0
#define rSCL_BIT                       BV(7)
#define rSCL_SEL                       P0SEL
#define rSCL_DIR                       P0DIR

#define rSCL                           P0_7
/************************************************************/  
#define rSDA_INPUT()     rSDA_DIR &= ~(rSDA_BIT);  // clr 0 == input
#define rSDA_OUTPUT()    rSDA_DIR |= rSDA_BIT;     // set 1 == output
  
#define rSDA_HIGH()      rSDA = 1;  // Sets SDA line
#define rSDA_LOW()       rSDA = 0;  // Clears SDA line
#define rSCL_HIGH()      rSCL = 1;  // Sets SCL line
#define rSCL_LOW()       rSCL = 0;  // Clears SCL line
/************************************************************/  
#define TCYBUF                           13
#define TCYBUFH                          20
/*****************************************************************/
#define POWER_ON                         1
#define POWER_OFF                        0

#define MLX_SLAVEADDRESS                 0x5A
  
#define MLX_RAM_ACCESS                   0x00
#define MLX_EEPROM_ACCESS                0x20
#define MLX_READ_FLAGS                   0xF0
#define MLX_ENTER_SLEEP_MODE             0xFF

#define MLX_RAM_AMBIENTSEN               0x03
#define MLX_RAM_IRSEN1                   0x04
#define MLX_RAM_IRSEN2                   0x05
#define MLX_RAM_TA                       0x06
#define MLX_RAM_TOBJ1                    0x07
#define MLX_RAM_TOBJ2                    0x08
  
#define MLX_EEPROM_TOBJMAX               0x00
#define MLX_EEPROM_TOBJMIN               0x01
#define MLX_EEPROM_PWCTRL                0x02
#define MLX_EEPROM_TARANGE               0x03
#define MLX_EEPROM_EMICORRCOE            0x04
#define MLX_EEPROM_CONFIGREG1            0x05
#define MLX_EEPROM_SMBUSADDR             0x0E
#define MLX_EEPROM_IDNUMBER1             0x1C
#define MLX_EEPROM_IDNUMBER2             0x1D
#define MLX_EEPROM_IDNUMBER3             0x1E
#define MLX_EEPROM_IDNUMBER4             0x1F
  
#define MLX_EEPROM_INIT                  0x00  
  
/* Register length == 16bit == 2 bytes */
#define MLX_REG_LEN                      2
  
void Delay_ms(unsigned int ms);
void Delay_us(unsigned int times);

uint16 BreathLeftTemp_Capture(void);

uint16 BreathRightTemp_Capture(void);

uint8 PEC_calculate(uint8 pec[],int n);
uint16 Calc_TempValue(uint16 Value);

void BBS_BothSensorTurnOn(void);
void BBS_BothSensorTurnOff(void);


#ifdef __cplusplus
}
#endif

#endif
