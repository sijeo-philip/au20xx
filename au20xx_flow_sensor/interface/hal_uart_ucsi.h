/****************************************************************************
* Title                 :   HAL_UART_UCSI
* Filename              :   hal_uart_ucsi.h
* Author                :   Sijeo Philip
* Origin Date           :   04/28/2015
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6XX
* Notes                 :   This module is used to interface with the UART peripheral
*
* THIS SOFTWARE IS PROVIDED BY UNISEM ELECTRONICS "AS IS" AND ANY EXPRESSED
* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL UNISEM ELECTRONICS OR ITS CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/


/*************** INTERFACE CHANGE LIST **************************************
*
*    Date    Version   Author         Description
*  04/29/15   1.0.0   Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file hal_uart_ucsi.h
 *  @brief This module is to communicate and use the UART peripheral
 *
 *  This is the header file for the definition of APIs to be used by application
 *  for using UART peripheral
 */

#ifndef HAL_UART_UCSI_H_
#define HAL_UART_UCSI_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include "common.h"

#include "usci_uart_config.h"
#include "msp430.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define UART_CTRL_WORD_REG0(peri)        UC##peri##CTLW0   /**<< Value can be A0, A1, base
                                                                based on the MCU */
#define UART_CTRL_WORD_REG1(peri)        UC##peri##CTLW1

#define UART_BAUD_RATE_REG(peri)         UC##peri##BRW

#define UART_MOD_CTRL_REG(peri)          UC##peri##MCTLW

#define UART_STATW_REG(peri)             UC##peri##STATW

#define UART_RX_BUF(peri)                UC##peri##RXBUF

#define UART_TX_BUF(peri)                UC##peri##TXBUF

#define UART_BAUD_CTRL_REG(peri)         UC##peri##ABCTL      /**<<Used when Auto Baud Detection is used */

#define UART_IRDA_CTRL_REG(peri)         UC##peri##IRCTL      /**<< Used in IRDA mode */

#define UART_INT_ENABLE_REG(peri)        UC##peri##IE

#define UART_TX_BYTE_INT_FLG(peri)   ((UC##peri##IFG & 0x0008)?1:0)

#define UART_START_INT_FLG(peri)     ((UC##peri##IFG & 0x0004)?1:0)

#define UART_TX_END_INT_FLG(peri)    ((UC##peri##IFG & 0x0002)?1:0)

#define UART_RX_END_INT_FLG(peri)    ((UC##peri##IFG & 0x0001)?1:0)

/******************************************************************************
* Configuration Constants
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/

/**
 * @brief  This Macro is used to transmit the Address bit in the next
 *         Transmit in UART
 */
#define UART_TX_ADDR_BIT(peri)  \
    do{   \
        UC##peri##CTLW0 |= 0x0004; \
    }while(0)

/**
 * @brief This Macro is used to transmit Break in the next cycle in UART
 */
#define UART_TX_BREAK_BIT(peri)  \
    do{  \
        UC##peri##CTLW0 |= 0x0002; \
    }while(0)


/**
 * @brief This Macro is used to Enable Software Reset for UART
 */
#define UART_ENABLE_SW_RST(peri)  \
    do{  \
        UC##peri##CTLW0 |=  0x0001;\
    }while(0)


/**
 * @brief This MACRO is used to Disable Software Reset for UART
 */
#define UART_DISABLE_SW_RST(peri)  \
    do{   \
        UC##peri##CTLW0 &= ~(0x0001);\
    }while(0)

/**
 * @brief This MACRO is used to clear the UART RX Interrupt flag
 */
#define UART_CLR_RXIFG(peri) \
    do{   \
        UC##peri##IFG &= ~(0x0001); \
    }while(0)


/**
 * @brief This Macro is used to Clear the UART TX Interrupt Flag
 */
#define UART_CLR_TXIFG(peri) \
    do{ \
        UC##peri##IFG &= ~(0x0002); \
    }while(0)

/**
 * @brief This Macro is used to Clear the Start Bit Receive Interrupt
 */
#define UART_CLR_STTIFG(peri) \
    do{ \
        UC##peri##IFG &= ~(0x0004); \
    }while(0)

/**
 * @brief This Macro is used to Clear the Transmit Complete Interrupt Flag
 * with Stop Bit
 */
#define UART_CLR_UCTXCPTIFG(peri)  \
    do {\
        UC##peri##IFG &= ~(0x0008); \
    }while(0)
/******************************************************************************
* Typedefs
*******************************************************************************/

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void uart_init(void);
void uart_send(uint8_t const * data,  uint16_t bytes);
uint16_t uart_read(uint8_t* data);

#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*File_H_*/

/*** End of File **************************************************************/
