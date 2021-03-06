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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "hal_drivers.h"
#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#include "battservice.h"
//#include "simplekeys.h"


#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
#include "oad.h"
#include "oad_target.h"
#include "hal_flash.h"
#endif
#include "SerialApp.h"
#include "BattApp.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// How often to perform periodic event  
#define SBP_PERIODIC_EVT_MAX_TIME                   ((uint32)12*60*60*1000) //12 hour
//#define SBP_PERIODIC_EVT_MAX_TIME                   ((uint32)10*60*1000) //10min hour
// How often to perform periodic event  
#define SBP_PERIODIC_EVT_PERIOD                   600000 //10 min
//#define SBP_PERIODIC_EVT_PERIOD                     60000 //1min
// Battery measurement period in ms
#define DEFAULT_BATT_PERIOD                   3000
// 4 notifications are sent every 7ms, based on an OSAL timer
#define SBP_BURST_EVT_PERIOD                      7
//  charge evt period
#define SBP_CHARGE_EVT_PERIOD                      1000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          1600 
// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 40=50ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     40

// Maximum connection interval (units of 1.25ms, 40=50ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     40

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF
// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6 

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 g_MeasPeriodMode = MEAS_PERIODMEA_OFF;
uint8 g_MeasPeriodStatus= MEAS_PERIODMEA_FINISH;
uint8 g_Chargestatus= UNCHARGING;
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

gaprole_States_t gapProfileState = GAPROLE_INIT;

static uint32 g_MeasPeriod = SBP_PERIODIC_EVT_PERIOD;   // Task ID for internal task/event processing
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0c,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x50,   // 'P'
  0x4d,   // 'M'
  0x32,   // '2'
  0x2e,   // '.'
  0x35,   // '5'
  0x54,   // 'T'
  0x45,   // 'E'
  0x53,   // 'S'
  0x54,   // 'T'
  0x45,   // 'E'
  0x52,   // 'R'

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
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID)

};

// GAP GATT Attributes
static uint8 attDeviceName[] = "PM2.5_TESTER";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );
static void SimpleBLEBattCB(uint8 event);
static void PM25TesterBattPeriodicTask( void );
static void PM25TesterNotifyHistroy();
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
static void PM25TesterChargePeriodicTask();

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



/*********************************************************************
 * PROFILE CALLBACKS
 */

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
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;
  SerialApp_Init();
  HalBattInit();
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

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
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof(attDeviceName)+1, attDeviceName );

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
    //uint32 passkey = 780525; // passkey "1"
    uint32 passkey = 1; // passkey "1"
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
  // Setup Battery Characteristic Values
  {
    uint8 critical = DEFAULT_BATT_CRITICAL_LEVEL;
    Batt_SetParameter( BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8 ), &critical );
  }
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  Batt_AddService( );
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif
  // Register for Battery service callback;
  Batt_Register ( SimpleBLEBattCB );
  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 0;
    uint8 charValue2[SIMPLEPROFILE_CHAR2_LEN] = {0};
    uint8 charValue3 = SBP_PERIODIC_EVT_PERIOD/60000;
    uint8 charValue4[SIMPLEPROFILE_CHAR4_LEN] = {0};
    uint8 charValue5 = 0;
    uint8 charValue6 = MEAS_PERIODMEA_OFF;
    uint8 charValue7 = UNKNOWSERSORTYPE;
    uint8 charValue8[SIMPLEPROFILE_CHAR8_LEN] = {0};
    uint8 charValue9 = UNCHARGING;
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, sizeof ( uint8 ), &charValue5 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, sizeof ( uint8 ), &charValue6 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, sizeof ( uint8 ), &charValue7 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR8, SIMPLEPROFILE_CHAR8_LEN, charValue8 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR9, sizeof ( uint8 ), &charValue9 );
  }


  //SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  // Register for all key events - This app will handle all key events
 RegisterForKeys( simpleBLEPeripheral_TaskID );
#endif
#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted ��MCU���е�ʱ�򽵵���Ƶ�����͹��ģ����ǻ�Ӱ�쵽DMA������������
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
 // power management enbale
  HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);//Power off pm2.5
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
*/
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
    
    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );           
        // Start Bond Manager
        VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );
        //HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_MINUS_23_DBM);
        SensorPowerOff();
        return ( events ^ SBP_START_DEVICE_EVT );
    }
    
    if ( events & SBP_PERIODIC_EVT )
    {
        // Restart timer
        if ( (g_MeasPeriod>0) &&(MEAS_PERIODMEA_ON==g_MeasPeriodMode))
        {
            osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, g_MeasPeriod);
            // Perform periodic application task
            performPeriodicTask();
        }
        
        return (events ^ SBP_PERIODIC_EVT);
    }
    if ( events & BATT_PERIODIC_EVT )
    {
      // Perform periodic battery task
        PM25TesterBattPeriodicTask();
        
        return (events ^ BATT_PERIODIC_EVT);
    }  
       // Send Data
    if ( events & SBP_BURST_EVT )
  {
    // Restart timer
    if ( SBP_BURST_EVT_PERIOD )
    {
      //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_BURST_EVT, SBP_BURST_EVT_PERIOD );
    }

   PM25TesterNotifyHistroy();

    //burstData[0] = !burstData[0];
    return (events ^ SBP_BURST_EVT);
  } 
         // Send Data
  if ( events & SBP_CHARGE_EVT )
  {
      
      PM25TesterChargePeriodicTask();
      
      //burstData[0] = !burstData[0];
      return (events ^ SBP_CHARGE_EVT);
  }  
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
 */
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
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  VOID shift;  // Intentionally unreferenced parameter
  //charge on
  //start to monitor the battery level  and stwich to charge the baterry
  if ( keys & HAL_POWER_CHARGE )
  {
      HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
      //The I2C configuration and state is not retained in power modes PM2 and PM3. It must be reconfigured after coming out of sleep mode
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_CHARGE_EVT, SBP_CHARGE_EVT_PERIOD );

  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  //SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
            static uint8 charValue1 = 0;

static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
    static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER#if !defined( CC2540_MINIDK )
    VOID gapProfileState;     // added to prevent compiler warning with
    // "CC2540 Slave" configurations
    gapProfileState = newState;

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
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        }
        break;
        
        case GAPROLE_ADVERTISING:
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
            HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
            charValue1++;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );   
        }
        break;
        
        case GAPROLE_CONNECTED:
        {
            uint8  newValue = 1;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1,  sizeof ( uint8 ), &newValue);
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
            HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
            SensorPowerOn();
            g_PM25WorkCnt=0;
            //The I2C configuration and state is not retained in power modes PM2 and PM3. It must be reconfigured after coming out of sleep mode
            HalBattInit();
            osal_start_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
#ifdef PLUS_BROADCASTER
            // Only turn advertising on for this state when we first connect
            // otherwise, when we go from connected_advertising back to this state
            // we will be turning advertising back on.
            if ( first_conn_flag == 0 ) 
            {
                uint8 adv_enabled_status = 1;
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
                first_conn_flag = 1;
            }
#endif // PLUS_BROADCASTER
        }
        break;
        
        case GAPROLE_CONNECTED_ADV:
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
            HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
            uint8 charValue1 = 0;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
            osal_stop_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT);
            SensorPowerOff();
        }
        break;    
        case GAPROLE_WAITING:
        {
            uint8 charValue1 = 0;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );

           SensorPowerOff();

            
            osal_stop_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT);
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
            HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
            //          osal_pwrmgr_task_state(2,PWRMGR_CONSERVE);
            //          osal_pwrmgr_device(PWRMGR_BATTERY);
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        }
        break;
        
        case GAPROLE_WAITING_AFTER_TIMEOUT:
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
            HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
            uint8 charValue1 = 0;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
            osal_stop_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT);
            SensorPowerOff();
#ifdef PLUS_BROADCASTER
            // Reset flag for next connection.
            first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
        }
        break;
        
        case GAPROLE_ERROR:
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
            HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
            uint8 charValue1 = 0;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
            osal_stop_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT);
            SensorPowerOff();
            HAL_SYSTEM_RESET();
        }
        break;
        
        default:
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
            HalLcdWriteString( "",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
            uint8 charValue1 = 0;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
            osal_stop_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT);
            SensorPowerOff();
        }
        break;
        
    }  
    
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
    if(g_PM25PeriodCnt == (SBP_PERIODIC_EVT_MAX_TIME/g_MeasPeriod))
    {
        osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
        g_MeasPeriodStatus= MEAS_PERIODMEA_FINISH;
        g_MeasPeriodMode = MEAS_PERIODMEA_OFF;      
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, sizeof ( uint8 ), &g_MeasPeriodMode );
    }
    else
    {
        SensorPowerOn();
        g_PM25WorkCnt=0;
        g_MeasPeriodStatus= MEAS_PERIODMEAING;
    } 
}
/*********************************************************************
 * @fn      heartRateBattPeriodicTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void PM25TesterBattPeriodicTask( void )
{
    if (gapProfileState == GAPROLE_CONNECTED)
    {
        // perform battery level check
        HalBattInit();
        Batt_MeasLevel();
        // Restart timer
        osal_start_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
    }
    else if(g_Chargestatus == CHARGING)
    {
        osal_stop_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT);
    }
    
}
/*********************************************************************
 * @fn      PM25TesterChargePeriodicTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void PM25TesterChargePeriodicTask( void )
{  

    if(HAL_PUSH_BUTTON2())//����������Ϊ1������Ϊ�û��Ͽ�����ߣ��Ͽ��������
    {
        //The I2C configuration and state is not retained in power modes PM2 and PM3. It must be reconfigured after coming out of sleep mode
        osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_CHARGE_EVT);
        g_Chargestatus = UNCHARGING;
        SensorPowerOff();
    }
    else//�������������Ϊ1������Ϊ�û�������磬�������
    {
        g_Chargestatus= CHARGING;
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_CHARGE_EVT, SBP_CHARGE_EVT_PERIOD );
    }  
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR9, sizeof ( uint8 ), &g_Chargestatus );
}
/*********************************************************************
 * @fn      SimpleBLEBattCB
 *
 * @brief   Callback function for battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void SimpleBLEBattCB(uint8 event)
{
  if (event == BATT_LEVEL_NOTI_ENABLED)
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
    } 
  }
  else if (event == BATT_LEVEL_NOTI_DISABLED)
  {
    // stop periodic measurement
    osal_stop_timerEx( simpleBLEPeripheral_TaskID, BATT_PERIODIC_EVT );
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
      if(newValue!=HAL_LED_MODE_OFF)
      {
         HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);//Power on pm2.5
         HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);//Power on pm2.5
      }
      else
      {
         HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);//Power off pm2.5
         HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);//Power off pm2.5
      }
      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

      case SIMPLEPROFILE_CHAR3://Set Periodic Time
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );
      osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
      if(newValue!=0)
      {
          osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, g_MeasPeriod);
          g_MeasPeriod = (uint32)newValue*60*1000;
      }
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      
      break;
      case SIMPLEPROFILE_CHAR5: //notify histoty data on char4 
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, &newValue );
      if(newValue!=0)
      {
         //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_BURST_EVT, SBP_BURST_EVT_PERIOD );
         osal_set_event( simpleBLEPeripheral_TaskID, SBP_BURST_EVT );
      }  
      else
      {
         //osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_BURST_EVT);
      }
      break;
      case SIMPLEPROFILE_CHAR6:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, &newValue );
      if(newValue!=MEAS_PERIODMEA_OFF)
      {
          g_MeasPeriodMode = MEAS_PERIODMEA_ON;//Turn on period meas
          // Set timer for first periodic event
          osal_set_event( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT );
          // osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, g_MeasPeriod);
          g_PM25PeriodCnt = 0;
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR9, sizeof ( uint8 ), &g_PM25PeriodCnt);
      }
      else
      {
          g_MeasPeriodMode = MEAS_PERIODMEA_OFF;//Turn off period meas
          g_MeasPeriodStatus= MEAS_PERIODMEA_FINISH;
          // Set timer for first periodic event
          osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
      }
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      
      break;
#if defined FEATURE_OAD
      case SIMPLEPROFILE_CHAR10:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR10, &newValue );
      if(newValue==OAD_FLAG)
      {
          GAPRole_TerminateConnection();           
          uint16 crc[2] = { 0x0000, 0xFFFF };
          uint16 addr = OAD_IMG_R_PAGE * ((uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)) + OAD_IMG_CRC_OSET / HAL_FLASH_WORD_SIZE;
          HalFlashWrite(addr, (uint8 *)crc, 1);
          //asm("LJMP 0x0830");   
          HAL_SYSTEM_RESET();
      }     
      break;
#endif
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
 */
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
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
/*
 * Send Data
 */
static void PM25TesterNotifyHistroy()
{  
    uint16 i = 0;
    uint8 newValue = 0;
    uint16 datalen = (g_PM25PeriodCnt*2);
    while(datalen>=SIMPLEPROFILE_CHAR4_LEN)
    {
        simpleProfile_NofifyChar4Data(PM25PeriodBuffer+i*SIMPLEPROFILE_CHAR4_LEN,SIMPLEPROFILE_CHAR4_LEN);
        i++;
        datalen = datalen - SIMPLEPROFILE_CHAR4_LEN;
    }
    if(datalen>0)
    {
        simpleProfile_NofifyChar4Data(PM25PeriodBuffer+i*SIMPLEPROFILE_CHAR4_LEN,datalen);
    }
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, sizeof(uint8), &newValue );
}
void SensorPowerOn()
{
    osal_pwrmgr_task_state(Hal_TaskID,PWRMGR_HOLD);
    osal_pwrmgr_device(PWRMGR_ALWAYS_ON);
    HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );
    HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);//set on pm2.5
    HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);//Power on pm2.5
}
void SensorPowerOff()
{
    if((MEAS_PERIODMEA_FINISH==g_MeasPeriodStatus)&&(GAPROLE_CONNECTED!=gapProfileState))
    {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);//set off pm2.5
 
        HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);//Power off pm2.5
        
        osal_pwrmgr_task_state(Hal_TaskID,PWRMGR_CONSERVE);
        osal_pwrmgr_device(PWRMGR_BATTERY);
        HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
    }
}
/*********************************************************************
*********************************************************************/
