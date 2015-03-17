#ifndef _SERIAL_APP_H_
#define _SERIAL_APP_H_

#ifdef __cplusplus
extern "C"
{
#endif
  
typedef enum {
    PKT_UNKNOWN,
    PKT_HR01DATA
} PKT_TYPE;
#define SBP_UART_PORT                  HAL_UART_PORT_0
//#define SBP_UART_FC                    TRUE
#define SBP_UART_FC                    FALSE
#define SBP_UART_FC_THRESHOLD          48
#define SBP_UART_RX_BUF_SIZE           128
#define SBP_UART_TX_BUF_SIZE           64
#define SBP_UART_IDLE_TIMEOUT          6
#define SBP_UART_INT_ENABLE            TRUE
#define SBP_UART_BR                     HAL_UART_BR_115200 
#define HKPM01_UART_BR                     HAL_UART_BR_9600
#define HKPM01_UART_PORT   HAL_UART_PORT_1
#define HR01MAXDATALEN 32
#define PM25WORKTIME 35 // measure 20 times
#define HR01_DATALENOFFSET 3
#define HR01_PREFIXLEN 4
#define PM1003_DATALEN 32
#define HKPM01_DATALEN 24
#define PM25_PM1003TYPE 1
#define PM25_HKPM01TYPE 0
#define UNKNOWSERSORTYPE 0xff
extern uint16 g_PM25WorkCnt;
extern uint16 g_PM25PeriodCnt;
extern uint8 PM25PeriodBuffer[200];
// Serial Port Related
void SerialApp_Init(void);
void sbpSerialAppCallback(uint8 port, uint8 event);
void HKPM01AppCallback(uint8 port, uint8 event);
void serialAppInitTransport();
void sbpSerialAppWrite(uint8 *pBuffer, uint16 length);
void SerialPrintString(uint8 str[]);
void SerialPrintValue(char *title, uint16 value, uint8 format);

#ifdef __cplusplus
}
#endif

#endif
