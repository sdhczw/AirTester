/**************************************************************************************************'
  Filename:       BattApp.c
  Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
  Revision:       $Revision: 33575 $

  Description:    Driver for the MAX17040 and MAX17041


  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/* ------------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/
#include "BattApp.h"
#include "hal_i2c.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/

// Sensor I2C address
#define MAX17040_SLAVE_ADDRESS      0x36
#define MAX17040_MEM_ADDRESS        0x04

#define S_REG_LEN                  2
#define MAX17040_DATA_LEN                   2
/* ------------------------------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
static bool HalBattReadData(uint8 *pBuf,uint8 nBytes);
static bool HalBattWriteCmd(uint8 cmd);

/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 buf[MAX17040_DATA_LEN];
/**************************************************************************************************
* @fn          HalBattInit
*
* @brief       Initialise the humidity sensor driver
*
* @return      none
**************************************************************************************************/
void HalBattInit(void)
{
  //Set up I2C that is used to communicate with SHT21
  HalI2CInit(MAX17040_SLAVE_ADDRESS,i2cClock_123KHZ);
}

/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/
bool HalBattExecMeasurement(uint16 *pBuf)
{
    bool success;
    /* Send address we're reading from */
    HalBattInit();
    do
    {
        //  success = HalBattReadData(buf, MAX17040_DATA_LEN); 
        success = HalBattWriteCmd(MAX17040_MEM_ADDRESS);
        
        // Start for humidity read
        if (!success)
        {
            break;
            
        }
        HalBattInit();
        success = HalBattReadData(buf, MAX17040_DATA_LEN);
        pBuf[0] = buf[0];
    }while(0);
    return success;
}


/**************************************************************************************************
* @fn          halBattWriteCmd
*
* @brief       Write a command to the humidity sensor
*
* @param       cmd - command to write
*
* @return      TRUE if the command has been transmitted successfully
**************************************************************************************************/
static bool HalBattWriteCmd(uint8 cmd)
{
  /* Send command */
  return HalI2CWrite(1,&cmd) == 1;
}


/**************************************************************************************************
* @fn          HalBattReadData
*
* @brief       This function implements the I2C protocol to read from the SHT21.
*
* @param       pBuf - pointer to buffer to place data
*
* @param       nBytes - number of bytes to read
*
* @return      TRUE if the required number of bytes are received
**************************************************************************************************/
bool HalBattReadData(uint8 *pBuf, uint8 nBytes)
{
  /* Read data */
  return HalI2CRead(nBytes,pBuf ) == nBytes;
}
/*********************************************************************
*********************************************************************/

