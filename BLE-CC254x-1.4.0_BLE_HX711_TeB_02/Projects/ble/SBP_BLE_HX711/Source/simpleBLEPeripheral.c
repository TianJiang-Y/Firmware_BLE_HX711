/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

//#if defined( CC2540_MINIDK )
//  #include "simplekeys.h"
//#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "Npi.h"
#include "stdio.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SBP_HX711_MEAS_START                   20  
   
#define SBP_BBS_MEAS_SWITCH_DELAY             300

#define SBP_AHRS_MEAS_DELAY                   100
   
#define SBP_TERMINATE_RETURN_DELAY            10

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD               5000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          480 // 300ms, Default: 160

// Limited discoverable mode advertises for 30.72s, and then stops
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     50 //6 //80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800 //6 //800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          200 // 3s

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         8 //3//6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x07,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'G',   // 'S'
  'r',   // 'i'
  'a',   // 'm'
  'm',   // 'p'
  's',   // 'l'
  's',   // 'e'
//  '7',   // 'B'
//  '1',   // 'L'
//  '1',   // 'E'
//  '_',   // 'P'
//  'S',   // 'e'
//  'C',   // 'r'
//  'A',   // 'i'
//  'L',   // 'p'
//  'E',   // 'h'
//  ' ',   // 'r'
//  'T',   // 'a'
//  'e',   // 'l'
//  's',   // 'l'
//  't',   // 'l'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "BLE Gramss";  

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask();
static void simpleProfileChangeCB( uint8 paramID );

static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif 

/***********************************************************************
*   my Define
************************************************************************/
#define HPAPP_THRESHOLD             48  // HandPhone Threshold Value

#define idle_mode 0
#define read_mode 1

uint8 ScaleOk = 0;
uint32 val = 0;

uint8 ScaleConState = 0;
uint8 state = idle_mode;

uint8 ADdata[3];
uint16 HX711_MCnt;
uint32 ScaleValue;

uint16 Raw_data_0 = 0;
uint32 Raw_data = 0;

uint32 reading = 0;
uint32 mass = 0;

float y1 = 10.0;
float x1 = 0L;
float x0 = 0L;
float avg_size = 10.0;

uint8 m[15] = {'M', 'a', 's', 's', '=' , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8 w[11] = {'S', 'c', 'a', 'l', 'e', ' ', 'W', 'r','o','n','g'};

uint8 CaptureHX711_Init(void);
void Delay_ms(unsigned int ms);
static void MeasValue_SendToSmart( uint32 AD_data );
static uint32 HX711Value_Capture(void);
static uint32 HX711Value_ScaleCalc(void);

static void HX711Scale_Calibration(void);
static uint8 Uint24to_Uint8BBSArray( uint32 value);
/*********************************************************************
 * PROFILE CALLBACKS
 ********************************************************************/

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 **********************************************************************/
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;

  // Callback Function 
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    // Our "BLE Breath Sensor" device doesn't start advertising until ..........
    uint8 initial_advertising_enable = FALSE;  // This is Broadcasting Function

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }


  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );
  
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
  HalLcdWriteString( "BLE HX711_Scale Test", HAL_LCD_LINE_1 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 **********************************************************************/
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
//--------------------------------------------------------------------------------//
  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );
    
    // Start Breathing Sensor measurement.......
    if(( gapProfileState != GAPROLE_CONNECTED ) &( gapProfileState != GAPROLE_ADVERTISING))
    {
      uint8 turnOnAdv = TRUE;
      
      // HX711 Turn On.....
      ScaleConState = CaptureHX711_Init();
      
      if(ScaleConState > 0)
        {
            HX711Scale_Calibration();
            ScaleOk = 1;
        }
      else
        {
            #if (defined HAL_LCD) && (HAL_LCD == TRUE)
                HalLcdWriteString( "ScaleConn Wrong...",  HAL_LCD_LINE_5 );
            #endif 
                
            ScaleOk = 0;
        }

      // Turn on advertising....
      GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &turnOnAdv);
      
      osal_start_timerEx(simpleBLEPeripheral_TaskID, 
                         SBP_HX711_READ_EVT, 
                         SBP_HX711_MEAS_START);
    }

    return ( events ^ SBP_START_DEVICE_EVT );
  }
//--------------------------------------------------------------------------------//
  if ( events & SBP_PERIODIC_EVT )
  {
    // Perform periodic application task
    performPeriodicTask(); // Temp data Send to Smartlock

    return (events ^ SBP_PERIODIC_EVT);
  }
//--------------------------------------------------------------------------------// 
  if ( events & SBP_HX711_READ_EVT )
  {
     HX711_MCnt++;
     
     ScaleValue = 0;
      
     if(ScaleOk)
     {
        ScaleValue = HX711Value_ScaleCalc();
     
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Scale measuring...",  HAL_LCD_LINE_5 );
  //       HalLcdWriteStringValue("MeasVal= ", (uint16)ScaleValue, 10, HAL_LCD_LINE_6);
          HalLcdWriteStringValue("Meas-Cnt= ", HX711_MCnt, 10, HAL_LCD_LINE_8);
        #endif
       
        MeasValue_SendToSmart(ScaleValue);
     }
     else
     {
        ScaleConState = CaptureHX711_Init();
        
        if(ScaleConState > 0)
        {
            HX711Scale_Calibration();
            ScaleOk = 1;
        }
        else        
            MeasValue_SendToSmart(0xFFFF);
     }
     
    // Restart timer
    if ( SBP_BBS_MEAS_SWITCH_DELAY )
    {
        osal_start_timerEx( simpleBLEPeripheral_TaskID,
                            SBP_HX711_READ_EVT,
                            SBP_BBS_MEAS_SWITCH_DELAY );
    }

    return (events ^ SBP_HX711_READ_EVT);
  }

//--------------------------------------------------------------------------------//  
  if ( events & SBP_TERMINATE_CONNECTION_EVT )
  {
    uint8 turnOffAdv = FALSE;
    // Turn off advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOffAdv );
    
    return (events ^ SBP_TERMINATE_CONNECTION_EVT);
  }
//--------------------------------------------------------------------------------// 
    
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 **********************************************************************/
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

  default:
    // do nothing
    break;
  }
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 **********************************************************************/
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_6 )
  {
      if(( gapProfileState != GAPROLE_CONNECTED ) &
         ( gapProfileState != GAPROLE_ADVERTISING))
        {
          uint8 turnOnAdv = TRUE;
          
          // HX711 Turn On.....
//          CaptureHX711_Init();
          
          // Turn on advertising....
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &turnOnAdv);
          
          osal_start_timerEx(simpleBLEPeripheral_TaskID, 
                             SBP_HX711_READ_EVT, 
                             SBP_AHRS_MEAS_DELAY);
        }
      
//      else if(( gapProfileState == GAPROLE_CONNECTED ) &
//              ( gapProfileState == GAPROLE_ADVERTISING ))
//        {
//          // Both Sensor Turn Off.....
//          BBS_BothSensorTurnOff();
//          // Terminate Connection....
//          GAPRole_TerminateConnection();
//          // Go Back Turn Off Advertising State....
//          osal_start_timerEx(simpleBLEPeripheral_TaskID, 
//                             SBP_TERMINATE_CONNECTION_EVT, 
//                             SBP_TERMINATE_RETURN_DELAY ); // 10ms
//        }
  }
}

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


static uint32 HX711Value_Capture(void)
{
  uint8 i, cnt;
  uint8 receivedata_H, receivedata_M ,receivedata_L;
  uint16 cnt_timer = 0x0000;
  
  if(state & BV(0))       //read_mode
      {
          P0IEN &= ~BV(0);  //P00 interrupt disable
          IRCON &= ~BV(5); 
          P0IFG &= ~BV(0);  //P00 interrupt flag reset

          for (i=0; i<10; i++) { };           
          //create clock
          P0_1 = ~P0_1;
                  
          for (cnt = 0; cnt < 16; cnt++)
          {
              //delay
               for (i=0; i<10; i++) { };
              
              //create clock
              P0_1 = ~P0_1;
              if (P0_1 == 0)
              {  receivedata_H = receivedata_H << 1 | P0_0; }
          }
          for (cnt = 0; cnt < 16; cnt++)
          {
              for (i=0; i<10; i++) { };
              
              //create clock
              P0_1 = ~P0_1;
              if (P0_1 == 0)
              {  receivedata_M = receivedata_M << 1 | P0_0; }
          }
          for (cnt = 0; cnt < 16; cnt++)
          {
              for (i=0; i<10; i++) { };
              
              //create clock
              P0_1 = ~P0_1;
              if (P0_1 == 0)
              {  receivedata_L = receivedata_L << 1 | P0_0; }
          }

          for (i=0; i<10; i++) { };
              //create clock
              P0_1 = ~P0_1;

           ADdata[0] = receivedata_H & 0xFF;
           ADdata[1] = receivedata_M & 0xFF;
           ADdata[2] = receivedata_L & 0xFF;

           Raw_data_0 = (ADdata[1] << 8) | ADdata[2];

           Raw_data = (((uint32)ADdata[0] << 16) | Raw_data_0);

          /////////////////////////////////////////////////////////
          HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
          HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
          /////////////////////////////////////////////////////////

  //        HalLcdWriteStringValue( "High   Byte:", receivedata_H, 16, HAL_LCD_LINE_4 );      
  //        HalLcdWriteStringValue( "Middle Byte:", receivedata_M, 16, HAL_LCD_LINE_5 );      
  //        HalLcdWriteStringValue( "Low    Byte:", receivedata_L, 16, HAL_LCD_LINE_6 );      

          //Send measured value to Smartphone
  //        MeasValue_SendToSmart( ADdata );
               
          receivedata_H = 0;
          receivedata_M = 0;
          receivedata_L = 0;
          
          //Return to idle_mode
           state = idle_mode;
           P0IEN |= BV(0);       //P00 interrupt enable
           P0IFG &= ~BV(0);  //P00 interrupt flag reset
      }
      
      else            //idle_mode
      {
        PICTL |= BV(0);       //P0 falling edge interrupt
        P0IEN |= BV(0);       //P00 interrupt enable
        cnt_timer++;
        if ( cnt_timer > 0x2000 )
        {
            /////////////////////////////////////////////////////////
            HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
            HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
            /////////////////////////////////////////////////////////   
        }
        if(P0IFG & 0x01 )
        {
            cnt_timer = 0;
            P0IEN &= ~BV(0);  //P00 interrupt disable
            IRCON = 0;
            P0IFG = 0;  //P00 interrupt flag reset
            state = read_mode;
        }
      }
  
  return Raw_data;
}

uint8 CaptureHX711_Init(void)
{
    uint8 i;
    val = 0;
    x0 = 0;
    
    P0SEL &= 0xFC; //P00,P01 General purpose port
//    P0SEL &= ~(BV(1) | BV(0)); //P00,P01 General purpose port
    P0DIR &= 0xFE;      //P00 input port
    P0DIR |= BV(1);     //P01 output port   
    P0_1 = 0;
    PICTL |= BV(0);       //P0 falling edge interrupt
    P0IEN |= BV(0);       //P00 interrupt enable
    P0IFG = 0;
    
    Delay_ms(1000);
    
    P0_1 = 1;
    for (i=0; i<10; i++) { };
    P0_1 = 0;

    if(P0_0 == 1)
    {
       Delay_ms(100);
       
       if(P0_0 == 1)
         return 0;
    }
    
    return 1;
}


static void HX711Scale_Calibration(void)
{
  uint8 ii = 0, jj = 0;
  x0 = 0;
  Delay_ms(500);
  
  for(ii = 0; ii < (uint8)(avg_size); ii++)
  {
    Delay_ms(15);
    x0 += HX711Value_Capture();
  }
  
  x0 /= avg_size;
  
  #if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "Scale Calibrating...",  HAL_LCD_LINE_5 );
  #endif 
    
    ii = 1;
    
    while(1)
    {
      if(HX711Value_Capture() < (x0 + 10000))
      {
      }
      else
      {
        ii++;
        Delay_ms(1000);
        
        for(jj = 0; jj < (uint8)(avg_size); jj++)
        {
          x1 += HX711Value_Capture();
        }
        
        x1 /= (long)(avg_size);
        break;
      }
    }
    
  #if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "Calibration Success!",  HAL_LCD_LINE_5 );
  #endif 
}

static uint32 HX711Value_ScaleCalc(void)
{
     uint8 k;
     reading = 0;
     mass = 0;
     uint32 temp = 0;
     float ratio_1, ratio_2, ratio;
     
     for(k = 0; k < (uint8)(avg_size); k++)
        reading += HX711Value_Capture();
     
     reading /= avg_size;
     
     ratio_1 = (float)(reading - x0);
     ratio_2 = (float)(x1 - x0);
     ratio = ratio_1 / ratio_2;
     mass = (uint32)(y1 * ratio);
     
     temp = mass - 10;
     if(temp > 10000)
       temp = 0;
     
     HalLcdWriteStringValue("W = ", (uint16)temp, 10, HAL_LCD_LINE_7);
     
     return (uint32)(temp);
}

static void MeasValue_SendToSmart(uint32 AD_data)
{
    uint8 ADValArrlen = 0; // Array number

    static attHandleValueNoti_t pReport;
    
    if(AD_data == 0xFFFF)
    {
        ADValArrlen = 11;
        pReport.len = ADValArrlen;
        pReport.handle = 0x002E;        // UART App Handle
        osal_memcpy(pReport.value, w, ADValArrlen);
        GATT_Notification( 0, &pReport, FALSE );   
    }
    else
    {
        ADValArrlen = Uint24to_Uint8BBSArray(AD_data);

        pReport.len = ADValArrlen;
        pReport.handle = 0x002E;        // UART App Handle
        osal_memcpy(pReport.value, m, ADValArrlen);
        GATT_Notification( 0, &pReport, FALSE );
    }
}

/*********************************************************************/
static uint8 Uint24to_Uint8BBSArray( uint32 value)
{
    uint32 A, B, C, D, E, F, G, H, I, J, K, L;
    uint32 val = 0;
    
    val = value;
    
    if(val >= 1000000)
    {
        A = val / 1000000;     // upper the decimal place
        B = val % 1000000; // below the decimal place
        C = B / 100000;
        D = B % 100000;
        E = D / 10000;
        F = D % 10000;
        G = F / 1000;
        H = F % 1000;
        I = H / 100;
        J = H % 100;
        K = J / 10;
        L = J % 10;
        
        if(A == 0)
          m[5] = ' ';
        else
          m[5] = A + HPAPP_THRESHOLD;

        m[6] = C + HPAPP_THRESHOLD;
        m[7] = E + HPAPP_THRESHOLD;
        m[8] = G + HPAPP_THRESHOLD;
        m[9] = I + HPAPP_THRESHOLD;
        m[10] = K + HPAPP_THRESHOLD;
        m[11] = L + HPAPP_THRESHOLD;
        
        return 12;
    }
    
    else if(val >= 100000)
    {
        A = val / 100000;     // upper the decimal place
        B = val % 100000; // below the decimal place
        C = B / 10000;
        D = B % 10000;
        E = D / 1000;
        F = D % 1000;
        G = F / 100;
        H = F % 100;
        I = H / 10;
        J = H % 10;
        
        if(A == 0)
          m[5] = ' ';
        else
          m[5] = A + HPAPP_THRESHOLD;

        m[6] = C + HPAPP_THRESHOLD;
        m[7] = E + HPAPP_THRESHOLD;
        m[8] = G + HPAPP_THRESHOLD;
        m[9] = I + HPAPP_THRESHOLD;
        m[10] = J + HPAPP_THRESHOLD;
        
        return 11;
    }
    
    else if(val >= 10000)
    {
        A = val / 10000;     // upper the decimal place
        B = val % 10000; // below the decimal place
        C = B / 100;
        D = B % 100;
        
        if(A == 0)
          m[5] = ' ';
        else
          m[5] = A + HPAPP_THRESHOLD;

        if(C >= 10)
        {
            m[6] = (C / 10) + HPAPP_THRESHOLD;
            C = C % 10;
            m[7] = C  + HPAPP_THRESHOLD;
        }

        else
        {
            m[6] = HPAPP_THRESHOLD;
            m[7] = C + HPAPP_THRESHOLD;
        }
        
        if(D >= 10)
        {
            m[8] = (D / 10) + HPAPP_THRESHOLD;
            D = D % 10;
            m[9] = D  + HPAPP_THRESHOLD;
        }

        else
        {
            m[8] = HPAPP_THRESHOLD;
            m[9] = D + HPAPP_THRESHOLD;
        }
        
        return 10;
    }
    
    else if(val >= 1000)
    {
      A = val / 100;
      C = val - A * 100;
      
      m[5] = (A / 10) + HPAPP_THRESHOLD; // UART App threshold = 48
      A = A % 10;
      m[6] = A + HPAPP_THRESHOLD;

     if(C >= 10)
      {
          m[7] = (C / 10) + HPAPP_THRESHOLD;
          C = C % 10;
          m[8] = C  + HPAPP_THRESHOLD;
      }

     else
      {
          m[7] = HPAPP_THRESHOLD;
          m[8] = C + HPAPP_THRESHOLD;
      }
      
      return 9;
    }
    
    else if (val >= 100)
    {
        A = val / 100;
        C = val - A * 100;
        
        m[5] = A + HPAPP_THRESHOLD;
      
       if(C >= 10)
        {
            m[6] = (C / 10) + HPAPP_THRESHOLD;
            C = C % 10;
            m[7] = C  + HPAPP_THRESHOLD;
        }

       else
        {
            m[6] = HPAPP_THRESHOLD;
            m[7] = C + HPAPP_THRESHOLD;
        }
      
      return 8;
    }
    
    else if (val >= 10)
    {
        m[5] = (val / 10) + HPAPP_THRESHOLD;
        val = val % 10;
        m[6] = val  + HPAPP_THRESHOLD;
        
        return 7;
    }

    else
    {
        m[5] = val + HPAPP_THRESHOLD;
        
        return 6;
    }
}

//static void MeaState_SendToHandPhone(void)
//{
//    uint8 DisplayArrlen = 20; // Array number
//    static attHandleValueNoti_t pReport;
//   
//    if(LeftSensMCnt == RightSensMCnt + 1)
//      {
//        for(uint8 i = 0; i < 20; i++)
//          mm[i] = '>';
//      }
//    else if(LeftSensMCnt == RightSensMCnt)
//      {
//        for(uint8 i = 0; i < 20; i++)
//          mm[i] = '=';
//      }
//    else if(MEMSensCnt == RightSensMCnt)
//      {
//        for(uint8 i = 0; i < 20; i++)
//          mm[i] = '#';
//      }
//    
//    pReport.len = DisplayArrlen;
//    pReport.handle = 0x002E;        // UART App Handle
//    osal_memcpy(pReport.value, mm, DisplayArrlen);
//    GATT_Notification( 0, &pReport, FALSE );
//}
//
//static void BBSMeasValue_SendToHandPhone( uint16 TempData )
//{
////    uint16 tmpTempValue = 0;
////    uint8 TempVallen = 2;    // 2bytes
//    uint8 TempValArrlen = 15; // Array number
//    static attHandleValueNoti_t pReport;
//
////    osal_memcpy(&tmpTempValue, &TempData, TempVallen);
////    HalLcdWriteStringValue("TempSending... ", tmpTempValue, 10, HAL_LCD_LINE_8 );
//   
//    Uint16to_Uint8BBSArray(TempData);
//    
//    if ( LeftSensMCnt == RightSensMCnt + 1)
//      {
//          m[5] = '1';
//      }
//    else
//      {
//          m[5] = '2';
//      }
//    
//    pReport.len = TempValArrlen;
//    pReport.handle = 0x002E;        // UART App Handle
//    osal_memcpy(pReport.value, m, TempValArrlen);
//    GATT_Notification( 0, &pReport, FALSE );
//}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 **********************************************************************/
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif 
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif 
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
//          HalLcdWriteString( "Key S1 = Notify CHAR6", HAL_LCD_LINE_8 );
        #endif 
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif 
      }
      break;      
    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif 
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif 
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif 
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
  uint8 valueToCopy;
  uint8 stat;
  uint8 returnBytes; 

  // Call to retrieve the value of the third characteristic in the profile
  stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy, &returnBytes);

  if( stat == SUCCESS )
  {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
  }
}
/*********************************************************************/

/*********************************************************************/
//static uint8 compareChars(char *a, char *b, uint8 length)
//{
//  int i;
//  for(i = 0; i < length; i++)
//  {
//    if(a[i] != b[i]) 
//      return 1; // Different
//  }
//  return 0; // equal
//}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 ********************************************************************/

//static bool SmartLockConnEnable = FALSE;

static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue[20] = {0};
  uint8 returnBytes; 

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue, &returnBytes);

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue[0]), 10,  HAL_LCD_LINE_5 );
      #endif

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue, &returnBytes);

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue[0]), 16,  HAL_LCD_LINE_5 );
      #endif

      break;

    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 **********************************************************************/
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif 
/*********************************************************************/
