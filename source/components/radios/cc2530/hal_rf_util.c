/*********************************************************************
	Filename:       hal_rf_util.c

	Description:    Support for anergy detection applications.

*********************************************************************/


/*********************************************************************
* INCLUDES
*/

#include "hal_board.h"
#include "hal_assert.h"
#include "hal_rf_util.h"
#include "hal_rf.h"


/*********************************************************************
* CONSTANTS
*/


/*********************************************************************
* MACROS
*/

/*********************************************************************
* TYPEDEFS
*/


/*********************************************************************
* GLOBAL VARIABLES
*/



/*********************************************************************
* FUNCTIONS
*/

/***********************************************************************************
* @fn          halSampleED
*
* @brief      Sample Energy Detect
*
* @param      uint8 channel - channel between 11 and 26
*             uint16 sampleTime - sample time in us
*            
* @return     int8 - sampled RSSI value      
*/
int8 halSampleED(uint8 channel, uint16 sampleTime)
{
  int8 rssi=0;
  
  // Set channel
  halRfSetChannel(channel);
  
  // Set RX on
  halRfReceiveOn();
  while (!RSSISTAT);
  
  // Enable energy scan mode, using peak signal strength
  FRMCTRL0 |= 0x10;
  
  // Spend sampleTime us accumulating the peak RSSI value
  halMcuWaitUs(sampleTime);
  rssi = RSSI;
  
  // Exit the current channel
  halRfReceiveOff();
  // Disable ED scan mode
  FRMCTRL0 &= ~0x10;
  
  return rssi;
}
// 返回Received Signal Strength Indication接收的信号强度指示
int8 halRSSI()
{
  int8 rssi=0;
  //halRfReceiveOff(); 
  // Set RX on
  //halRfReceiveOn();
  while (!RSSISTAT);//RSSI_VALID 等待RSSI值有效
  
  // Enable energy scan mode, using peak signal strength
  FRMCTRL0 |= 0x10;
  
  // Spend sampleTime us accumulating the peak RSSI value
  halMcuWaitUs(1000);//us，我不确定这里应该延迟多久，可能不需要能量检测的延时
  
  rssi = RSSI;
  // Exit the current channel
  //halRfReceiveOff(); 
  
  // Disable ED scan mode
  FRMCTRL0 &= ~0x10;
  return rssi;
}

/***********************************************************************************
* @fn          halSetRxScanMode
*
* @brief       Set chip in RX scanning mode
*
* @param       none 
*            
*
* @return     none
*/
void halSetRxScanMode(void)
{
  // Infinite RX mode (disables symbol search)
  FRMCTRL0 |=  0x0C;   // 1100,FRMCTRL0.RX_MODE = 11b 
}
void halSetRxNormalMode(void)
{
  // Infinite RX mode (disables symbol search)
  FRMCTRL0 &=  ~0x0C;   // FRMCTRL0.RX_MODE = 00b 
}