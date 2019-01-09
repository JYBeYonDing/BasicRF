/***********************************************************************************

  Filename:       hal_rf.c

  Description:    CC2530 radio interface.

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "hal_board.h"
#include "hal_mcu.h"
#include "hal_int.h"
#include "hal_assert.h"
#include "hal_rf.h"
#include "util.h"
#include "hal_rf_util.h"


/***********************************************************************************
* CONSTANTS AND DEFINES
*/

// Chip revision
#define REV_A                      0x01
#define CHIPREVISION              REV_A

// CC2530 RSSI Offset
#define RSSI_OFFSET                               73
#define RSSI_OFFSET_LNA_HIGHGAIN                  79
#define RSSI_OFFSET_LNA_LOWGAIN                   67

// Various radio settings
#define AUTO_ACK                   0x20
#define AUTO_CRC                   0x40

// TXPOWER values
#if CC2530_PG1
#define CC2530_TXPOWER_MIN_3_DBM   0x88 
#define CC2530_TXPOWER_0_DBM       0x32
#define CC2530_TXPOWER_4_DBM       0xF7
#else
#define CC2530_TXPOWER_MIN_3_DBM   0xB5 
#define CC2530_TXPOWER_0_DBM       0xD5
#define CC2530_TXPOWER_4_DBM       0xF5
#endif

// RF interrupt flags
#define IRQ_TXDONE                 0x02
#define IRQ_RXPKTDONE              0x40

// FSMSTAT1 (0x6193) Radio Status Register
#define SAMPLED_CCA                0x08

// Selected strobes
#define ISRXON()                st(RFST = 0xE3;)
#define ISTXON()                st(RFST = 0xE9;)
#define ISTXONCCA()             st(RFST = 0xEA;)
#define ISSAMPLECCA()           st(RFST = 0xEB;)
#define ISRFOFF()               st(RFST = 0xEF;)
#define ISFLUSHRX()             st(RFST = 0xEC;)
#define ISFLUSHTX()             st(RFST = 0xEE;)

// CC2590-CC2591 support
#if INCLUDE_PA==2591

// Support for PA/LNA
#define HAL_PA_LNA_INIT()

// Select CC2591 RX high gain mode 
#define HAL_PA_LNA_RX_HGM() st( uint8 i; P0_7 = 1; for (i=0; i<8; i++) asm("NOP"); )

// Select CC2591 RX low gain mode
#define HAL_PA_LNA_RX_LGM() st( uint8 i; P0_7 = 0; for (i=0; i<8; i++) asm("NOP"); )

// TX power lookup index
#define HAL_RF_TXPOWER_0_DBM          0
#define HAL_RF_TXPOWER_13_DBM         1
#define HAL_RF_TXPOWER_16_DBM         2
#define HAL_RF_TXPOWER_18_DBM         3
#define HAL_RF_TXPOWER_20_DBM         4

// TX power values
#define CC2530_91_TXPOWER_0_DBM       0x25
#define CC2530_91_TXPOWER_13_DBM      0x85
#define CC2530_91_TXPOWER_16_DBM      0xA5
#define CC2530_91_TXPOWER_18_DBM      0xC5
#define CC2530_91_TXPOWER_20_DBM      0xE5

#else // dummy macros when not using CC2591

#define HAL_PA_LNA_INIT()
#define HAL_PA_LNA_RX_LGM()
#define HAL_PA_LNA_RX_HGM()

#define HAL_RF_TXPOWER_MIN_3_DBM  0
#define HAL_RF_TXPOWER_0_DBM      1
#define HAL_RF_TXPOWER_4_DBM	  2

#endif

/***********************************************************************************
* GLOBAL DATA
*/
#if INCLUDE_PA==2591
static const menuItem_t pPowerSettings[] =
{
  "0dBm", HAL_RF_TXPOWER_0_DBM,
  "13dBm", HAL_RF_TXPOWER_13_DBM,
  "16dBm", HAL_RF_TXPOWER_16_DBM,
  "18dBm", HAL_RF_TXPOWER_18_DBM,
  "20dBm", HAL_RF_TXPOWER_20_DBM
};
#else
static const menuItem_t pPowerSettings[] =
{
  "-3dBm", HAL_RF_TXPOWER_MIN_3_DBM,
  "0dBm", HAL_RF_TXPOWER_0_DBM,
  "4dBm", HAL_RF_TXPOWER_4_DBM
};
#endif

const menu_t powerMenu =
{
  pPowerSettings,
  N_ITEMS(pPowerSettings)
};

/***********************************************************************************
* LOCAL DATA
*/
#ifndef MRFI
static ISR_FUNC_PTR pfISR= NULL;
#endif
static uint8 rssiOffset = RSSI_OFFSET;

/***********************************************************************************
* LOCAL FUNCTIONS
*/
static void halPaLnaInit(void);

/***********************************************************************************
* GLOBAL FUNCTIONS
*/

/***********************************************************************************
* @fn      halRfInit
*
* @brief   Power up, sets default tuning settings, enables autoack, enables random
*          generator.
*
* @param   none
*
* @return  SUCCESS always (for interface compatibility)
*/
uint8 halRfInit(void)
{
    // Enable auto ack and auto crc //p259
    FRMCTRL0 |= (AUTO_ACK | AUTO_CRC);

    // Recommended RX settings
    TXFILTCFG = 0x09;
    AGCCTRL1 = 0x15;
    FSCAL1 = 0x00;

    // Enable random generator -> Not implemented yet


    // Enable RX interrupt
    halRfEnableRxInterrupt();

    return SUCCESS;
}



/***********************************************************************************
* @fn      halRfGetChipId
*
* @brief   Get chip id
*
* @param   none
*
* @return  uint8 - result
*/
uint8 halRfGetChipId(void)
{
    return CHIPID;
}


/***********************************************************************************
* @fn      halRfGetChipVer
*
* @brief   Get chip version
*
* @param   none
*
* @return  uint8 - result
*/
uint8 halRfGetChipVer(void)
{
    // return major revision (4 upper bits)
    return CHVER>>4;
}

/***********************************************************************************
* @fn      halRfGetRandomByte
*
* @brief   Return random byte
*
* @param   none
*
* @return  uint8 - random byte
*/
uint8 halRfGetRandomByte(void)
{
    // Not yet implemented
    HAL_ASSERT(FALSE);
    return 0;
}


/***********************************************************************************
* @fn      halRfGetRssiOffset
*
* @brief   Return RSSI Offset
*
* @param   none
*
* @return  uint8 - RSSI offset
*/
uint8 halRfGetRssiOffset(void)
{
    return rssiOffset;
}

/***********************************************************************************
* @fn      halRfSetChannel
*
* @brief   Set RF channel in the 2.4GHz band. The Channel must be in the range 11-26,
*          11= 2005 MHz, channel spacing 5 MHz.
*
* @param   channel - logical channel number
*
* @return  none
*/
void halRfSetChannel(uint8 channel)
{
    FREQCTRL = (MIN_CHANNEL + (channel - MIN_CHANNEL) * CHANNEL_SPACING);
}


/***********************************************************************************
* @fn      halRfSetShortAddr
*
* @brief   Write short address to chip
*
* @param   none
*
* @return  none
*/
void halRfSetShortAddr(uint16 shortAddr)
{
    SHORT_ADDR0 = LO_UINT16(shortAddr);
    SHORT_ADDR1 = HI_UINT16(shortAddr);
}


/***********************************************************************************
* @fn      halRfSetPanId
*
* @brief   Write PAN Id to chip
*
* @param   none
*
* @return  none
*/
void halRfSetPanId(uint16 panId)
{
    PAN_ID0 = LO_UINT16(panId);
    PAN_ID1 = HI_UINT16(panId);
}


/***********************************************************************************
* @fn      halRfSetPower
*
* @brief   Set TX output power
*
* @param   uint8 power - power level: TXPOWER_MIN_4_DBM, TXPOWER_0_DBM,
*                        TXPOWER_4_DBM
*
* @return  uint8 - SUCCESS or FAILED
*/
uint8 halRfSetTxPower(uint8 power)
{
    uint8 n;

    switch(power)
    {
#if INCLUDE_PA==2591
    case HAL_RF_TXPOWER_0_DBM: n = CC2530_91_TXPOWER_0_DBM; break;
    case HAL_RF_TXPOWER_13_DBM: n = CC2530_91_TXPOWER_13_DBM; break;
    case HAL_RF_TXPOWER_16_DBM: n = CC2530_91_TXPOWER_16_DBM; break;
    case HAL_RF_TXPOWER_18_DBM: n = CC2530_91_TXPOWER_18_DBM; break;
    case HAL_RF_TXPOWER_20_DBM: n = CC2530_91_TXPOWER_20_DBM; break;
#else
    case HAL_RF_TXPOWER_MIN_3_DBM: n = CC2530_TXPOWER_MIN_3_DBM; break;
    case HAL_RF_TXPOWER_0_DBM: n = CC2530_TXPOWER_0_DBM; break;
    case HAL_RF_TXPOWER_4_DBM: n = CC2530_TXPOWER_4_DBM; break;
#endif
    default:
        return FAILED;
    }

    // Set TX power
    TXPOWER = n;

    return SUCCESS;
}


/***********************************************************************************
* @fn      halRfSetGain
*
* @brief   Set gain mode - only applicable for units with CC2590/91.
*
* @param   uint8 - gain mode
*
* @return  none
*/
void halRfSetGain(uint8 gainMode)
{
    if (gainMode==HAL_RF_GAIN_LOW) {
        HAL_PA_LNA_RX_LGM();
        rssiOffset = RSSI_OFFSET_LNA_LOWGAIN;
    } else {
        HAL_PA_LNA_RX_HGM();
        rssiOffset = RSSI_OFFSET_LNA_HIGHGAIN;
    }
}

/***********************************************************************************
* @fn      halRfWriteTxBuf
*
* @brief   Write to TX buffer
*
* @param   uint8* pData - buffer to write
*          uint8 length - number of bytes
*
* @return  none
*/
void halRfWriteTxBuf(uint8* pData, uint8 length)
{
    uint8 i;

    ISFLUSHTX();          // Making sure that the TX FIFO is empty.

    RFIRQF1 = ~IRQ_TXDONE;   // Clear TX done interrupt

    // Insert data
    for(i=0;i<length;i++){
        RFD = pData[i];
    }
}


/***********************************************************************************
* @fn      halRfAppendTxBuf
*
* @brief   Write to TX buffer
*
* @param   uint8* pData - buffer to write
*          uint8 length - number of bytes
*
* @return  none
*/
void halRfAppendTxBuf(uint8* pData, uint8 length)
{
    uint8 i;

    // Insert data
    for(i=0;i<length;i++){
        RFD = pData[i];
    }
}


/***********************************************************************************
* @fn      halRfReadRxBuf
*
* @brief   Read RX buffer
*
* @param   uint8* pData - data buffer. This must be allocated by caller.
*          uint8 length - number of bytes
*
* @return  none
*/
void halRfReadRxBuf(uint8* pData, uint8 length)
{
    // Read data
    while (length>0) {
        *pData++= RFD;
        length--;
    }
}


/***********************************************************************************
* @fn      halRfReadMemory
*
* @brief   Read RF device memory
*
* @param   uint16 addr - memory address
*          uint8* pData - data buffer. This must be allocated by caller.
*          uint8 length - number of bytes
*
* @return  Number of bytes read
*/
uint8 halRfReadMemory(uint16 addr, uint8* pData, uint8 length)
{
    return 0;
}


/***********************************************************************************
* @fn      halRfWriteMemory
*
* @brief   Write RF device memory
*
* @param   uint16 addr - memory address
*          uint8* pData - data buffer. This must be allocated by caller.
*          uint8 length - number of bytes
*
* @return  Number of bytes written
*/
uint8 halRfWriteMemory(uint16 addr, uint8* pData, uint8 length)
{
    return 0;
}

/***********************************************************************************
* @fn      halRfTransmit
*
* @brief   Transmit frame with NO Clear Channel Assessment.
*
* @param   none
*
* @return  uint8 - SUCCESS or FAILED
*/
uint8 halRfTransmit(void)
{
    uint8 status;

    ISTXON(); // Sending

    // Waiting for transmission to finish
    while(!(RFIRQF1 & IRQ_TXDONE) );

    RFIRQF1 = ~IRQ_TXDONE;
    status= SUCCESS;

    return status;
}

/***********************************************************************************
* @fn      halRfTransmitOnCCA
*
* @brief   Transmit frame with Clear Channel Assessment.
*
* @param   none
*
* @return  uint8 - SUCCESS or FAILED
*/
uint8 halRfTransmitOnCCA(void)
{
    uint8 status;
    uint8 CCA = 0;
    int8 ppRssi = 0;

    uint8 TIMER2_PERF = 0;

    //rssiOffset= halRfGetRssiOffset();
    CCACTRL0=0X10;//CCA_THR CCA阈值
    //CCACTRL1 = 0X1A; // CCA_MODE 默认
    // Set chip in RX scan mode
    halSetRxScanMode();
    
    while(!CCA){
        ppRssi = halRSSI();
    
    
        ISSAMPLECCA();
        //ISTXONCCA();
        //ISRXON();
        
        CCA = FSMSTAT1 & SAMPLED_CCA;
    }

    //设置timer2
    T2IE=1;
    T2MSEL = 0X22;//overflow period and timer period
    
    T2M0 = 0Xff;// 这个目前没用到
    T2M1 = 0Xff;
    T2MOVF0 = 0xff; //
    T2MOVF1 = 0x02;
    T2MOVF2 = 0x00;
    
    T2IRQM |= 0X9;//TIMER2_PERM
    T2CTRL &= ~0x02;// 关闭同步,测试，正常应该开启
    T2CTRL |= 0X01;//start timer2
    // set a timer to backoff
    while(TRUE){
        ISSAMPLECCA();
        //ISTXONCCA();
        //ISRXON();
    
        CCA = FSMSTAT1 & SAMPLED_CCA;
        
        if( 0 == CCA ){
            // stop timer if channel is busy.
          //if(T2CTRL & 0X01){//如果目前是打开的则关闭
            T2CTRL &= ~0X01;//关闭timer2
          //}
            
        }else{
          
          //if(!(T2CTRL & 0X01)){//如果目前是关闭的则打开
            T2CTRL |= 0X01;//开启timer2，但是不知道这里开启是接着计数还是重新从0开始
          //}
            
        }
        if(T2IRQF & 0x08){ //如果时间到，退出
            T2IRQF &= ~0X08;
            T2CTRL &= ~0X01;//关闭timer2
            break;
        }
    }
    
    halSetRxNormalMode();//前面设置为了halSetRxScanMode();这里恢复，不然一直发送不出去，但是我不知道接收模式为什么有影响
    ISTXONCCA();

    // Waiting for transmission to finish
    while(!(RFIRQF1 & IRQ_TXDONE) );

    RFIRQF1 = ~IRQ_TXDONE;
    status= SUCCESS;

    return status;
}


/***********************************************************************************
* @fn      halRfReceiveOn
*
* @brief   Turn receiver on
*
* @param   none
*
* @return  none
*/
void halRfReceiveOn(void)
{
    ISFLUSHRX();     // Making sure that the TX FIFO is empty.
    ISRXON();
}

/***********************************************************************************
* @fn      halRfReceiveOff
*
* @brief   Turn receiver off
*
* @param   none
*
* @return  none
*/
void halRfReceiveOff(void)
{
    ISRFOFF();
    ISFLUSHRX();    // Making sure that the TX FIFO is empty.
}


#ifndef MRFI
/***********************************************************************************
* @fn      halRfDisableRxInterrupt
*
* @brief   Clear and disable RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfDisableRxInterrupt(void)
{
    // disable RXPKTDONE interrupt
    RFIRQM0 &= ~BV(6);
    // disable general RF interrupts
    IEN2 &= ~BV(0);
}


/***********************************************************************************
* @fn      halRfEnableRxInterrupt
*
* @brief   Enable RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfEnableRxInterrupt(void)
{
    // enable RXPKTDONE interrupt // have received a packet
    RFIRQM0 |= BV(6);
    // enable general RF interrupts
    IEN2 |= BV(0);
}


/***********************************************************************************
* @fn      halRfRxInterruptConfig
*
* @brief   Configure RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfRxInterruptConfig(ISR_FUNC_PTR pf)
{
    uint8 x;
    HAL_INT_LOCK(x);
    pfISR= pf;
    HAL_INT_UNLOCK(x);
}

#endif

/***********************************************************************************
* @fn      halRfWaitTransceiverReady
*
* @brief   Wait until the transciever is ready (SFD inactive).
*
* @param   none
*
* @return  none
*/
void halRfWaitTransceiverReady(void)
{
    // Wait for SFD not active and TX_Active not active
    while (FSMSTAT1 & (BV(1) | BV(5) ));
}

#ifndef MRFI
/************************************************************************************
* @fn          rfIsr
*
* @brief       Interrupt service routine that handles RFPKTDONE interrupt.
*
* @param       none
*
* @return      none
*/
HAL_ISR_FUNCTION( rfIsr, RF_VECTOR )
{
    uint8 x;

    HAL_INT_LOCK(x);

    if( RFIRQF0 & IRQ_RXPKTDONE )
    {
        if(pfISR){
            (*pfISR)();                 // Execute the custom ISR
        }
        S1CON= 0;                   // Clear general RF interrupt flag
        RFIRQF0&= ~IRQ_RXPKTDONE;   // Clear RXPKTDONE interrupt
    }
    HAL_INT_UNLOCK(x);
}
#endif

/***********************************************************************************
* LOCAL FUNCTIONS
*/
/***********************************************************************************
* LOCAL FUNCTIONS
*/
static void halPaLnaInit(void)
{
#if INCLUDE_PA==2591
    // Initialize CC2591 to RX high gain mode
    static uint8 fFirst= TRUE;

    if(fFirst) {
        AGCCTRL1  = 0x15;
        FSCAL1 = 0x0; 
        RFC_OBS_CTRL0 = 0x68;
        RFC_OBS_CTRL1 = 0x6A;
        OBSSEL1 = 0xFB;
        OBSSEL4 = 0xFC;
        P0DIR |= 0x80;
        halRfSetGain(HAL_RF_GAIN_HIGH);
    }

#else // do nothing
#endif
}


/***********************************************************************************
Copyright 2007 Texas Instruments Incorporated. All rights reserved.

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
PROVIDED 锟斤拷锟斤拷锟斤拷锟斤拷锟紸S IS锟斤拷锟斤拷锟斤拷锟斤拷锟?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
***********************************************************************************/
