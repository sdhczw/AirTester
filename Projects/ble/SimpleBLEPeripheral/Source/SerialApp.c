#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "peripheral.h"
#include "OnBoard.h"
#include "hal_uart.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "SerialApp.h"
#include "simpleBLEPeripheral.h"
#include "simpleGATTprofile.h"
//static uint8 sendMsgTo_TaskID;
char HR01DataPrefix[] = {0x42,0x4d};
uint8 PM25PeriodBuffer[200] = {0};
uint16 g_PM25PeriodCnt = 0;
uint16 g_PM25WorkCnt = 0;
uint16 g_PM25WorkTime = PM25WORKTIME;
/*
uart初始化代码，配置串口的波特率、流控制等
*/
void HKPM01InitTransport()
{
  halUARTCfg_t uartConfig;
  
  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HKPM01_UART_BR;//波特率
  uartConfig.flowControl          = SBP_UART_FC;//流控制
  uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;//流控制阈值，当开启flowControl时，该设置有效
  uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE;//uart接收缓冲区大小
  uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE;//uart发送缓冲区大小
  uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = SBP_UART_INT_ENABLE;//是否开启中断
  uartConfig.callBackFunc         = HKPM01AppCallback;//uart接收回调函数，在该函数中读取可用uart数据

  // start UART
  // Note: Assumes no issue opening UART port.
  (void)HalUARTOpen( HAL_UART_PORT_0, &uartConfig );

//  
//              HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);//Turn on pm2.5
//            HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);//power on pm2.5
//            osal_pwrmgr_task_state(2,PWRMGR_HOLD);
//            osal_pwrmgr_device(PWRMGR_ALWAYS_ON);
  return;
}
/*
串口设备初始化，
必须在使用串口打印之前调用该函数进行uart初始化
*/
void SerialApp_Init(void)
{
  //调用uart初始化代码
  //serialAppInitTransport();
    SensorPowerOff();
  HKPM01InitTransport();
  //SensorPowerOn();
  //  SensorPowerOff();
  //记录任务函数的taskID，备用
  //sendMsgTo_TaskID = taskID;
}

//*****************************************************************************
//
//! Calculates an 8-bit checksum
//!
//! \param pui8Data is a pointer to an array of 8-bit data of size ui32Size.
//! \param ui32Size is the size of the array that will run through the checksum
//! algorithm.
//!
//! This function simply calculates an 8-bit checksum on the data passed in.
//!
//! \return Returns the calculated checksum.
//
//*****************************************************************************
uint32 CheckSum(const uint8 *pui8Data, uint32 ui32Size)
{
    uint32 ui32CheckSum;

    //
    // Initialize the checksum to zero.
    //
    ui32CheckSum = 0;

    //
    // Add up all the bytes, do not do anything for an overflow.
    //
    while(ui32Size--)
    {
        ui32CheckSum += *pui8Data++;
    }

    //
    // Return the caculated check sum.
    //
    return(ui32CheckSum & 0xffff);
}

/*
uart接收回调函数
当我们通过pc向开发板发送数据时，会调用该函数来接收
*/
void HKPM01AppCallback(uint8 port, uint8 event)
{
    static uint8 HR01Data[HR01MAXDATALEN];
    static uint8  HR01MatchNum = 0;
    static uint8 cur_type=PKT_UNKNOWN;
    static uint8 HR01Dataindex =0;
    static uint16 HR01DataLen =HR01MAXDATALEN;
    //  uint16 HR01Value = 0;
    uint32 ret =0;
    uint8 ch =0;
    uint32 ui32CheckSum = 0;
    ret= HalUARTRead(port,&ch,1);
    if(ret!=1)
    {
        return;
    }
    switch (cur_type)
    {
        case PKT_UNKNOWN:
        {  
            
            /**************** detect packet type ***************/
            //support more ATcmd prefix analysis
            /*case 1:AT#*/
            if (HR01DataPrefix[HR01MatchNum] ==ch)
            {         
                HR01MatchNum++;
            }
            else
            {         
                HR01MatchNum = 0;
            }
            
            if (HR01MatchNum ==sizeof(HR01DataPrefix))   //match case 3:arm  data
            {   
                
                cur_type = PKT_HR01DATA;           //match case 1: AT#                         
                HR01MatchNum = 0;
                osal_memcpy(HR01Data,HR01DataPrefix,sizeof(HR01DataPrefix));
                HR01Dataindex = HR01Dataindex+2;
                return;
            }           
        }
        break;
        case PKT_HR01DATA:
        {  
            HR01Data[HR01Dataindex++]= ch;
            if(HR01Dataindex==HR01_DATALENOFFSET)
            {
                HR01DataLen = ch;
            }
            else if(HR01Dataindex==(HR01_DATALENOFFSET +1))
            {
                HR01DataLen = ((HR01DataLen<<8) + ch + HR01_PREFIXLEN);
            } 
            else if(HR01Dataindex==HR01DataLen)
            {
                
                uint8 newValue = 0xff;
                if(PM1003_DATALEN==HR01DataLen)
                {
                    newValue = PM25_PM1003TYPE;
                }
                else if(HKPM01_DATALEN==HR01DataLen)
                {
                    newValue = PM25_HKPM01TYPE;
                }
                SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7,  sizeof ( uint8 ), &newValue);              
                HR01Dataindex=0;
                cur_type=PKT_UNKNOWN;               
                ui32CheckSum = (HR01Data[HR01DataLen-2]<<8) + HR01Data[HR01DataLen-1];
                if(CheckSum(HR01Data, HR01DataLen-2) != (ui32CheckSum & 0xffff))
                {
                    break;
                }
                
                //HR01Data[7]= Onboard_rand()%500; 
                
                SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, HR01Data+4); 
                if((g_MeasPeriodMode ==MEAS_PERIODMEA_ON)&&(g_PM25WorkCnt++>=g_PM25WorkTime)
                   &&(g_MeasPeriodStatus==MEAS_PERIODMEAING))
                {
                    
                    g_PM25WorkCnt=0;                  
                    //to do store pm2.5data
                    PM25PeriodBuffer[g_PM25PeriodCnt*2] = HR01Data[6]; 
                    PM25PeriodBuffer[g_PM25PeriodCnt*2+1] = HR01Data[7]; ; 
                    g_PM25PeriodCnt++;
                    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR9, sizeof ( uint8 ), &g_PM25PeriodCnt);
                    g_MeasPeriodStatus= MEAS_PERIODMEA_FINISH;
                    if(gapProfileState!=GAPROLE_CONNECTED)
                    {
                        uint8 charValue1 = 0;
                        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
                        SensorPowerOff();
                    }
                    
                    HR01DataLen = HR01MAXDATALEN;
                }
                
            }
            
        }
        break; 
        default:
        break;
        
    }
}
