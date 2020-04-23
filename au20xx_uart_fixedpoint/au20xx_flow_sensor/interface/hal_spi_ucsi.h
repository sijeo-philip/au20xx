/****************************************************************************
* Title                 :   hal_ucsi
* Filename              :   hal_ucsi.h
* Author                :   Sijeo Philip
* Origin Date           :   17/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to check status
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
*  17/03/20   1.0.0    Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file hal_spi_ucsi.h
 *  @brief This module is used to initialize read and write using SPI peripheral
 *
 *  This is the header file for the definition of SPI initializing and SPI operations
 */

#ifndef HAL_SPI_UCSI_H_
#define HAL_SPI_UCSI_H_

/********************************************
 * This defines the Peripheral of SPI used
 * <A0, B0, => This determines the Value of
 * UCSI_A, or UCSI_B used for SPI Peripheral
 */



/******************************************************************************
* Includes
*******************************************************************************/
#include "usci_spi_config.h"
#include "msp430.h"
#include "hal_gpio.h"


#define ENABLE_SPI_CS 	CLEAR_GPIO_OUTPUT(P1, BIT5)
#define DISABLE_SPI_CS	SET_GPIO_OUTPUT(P1, BIT5)
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

#define SPI_RX_BUF(peri)		UC##peri##RXBUF
#define SPI_TX_BUF(peri)		UC##peri##TXBUF
#define SPI_CTL_REG(peri)		UC##peri##CTLW0
#define SPI_BAUD_REG(peri)		UC##peri##BRW
#define SPI_STAT_REG(peri)		UC##peri##STATW
#define SPI_INT_REG(peri)		UC##peri##IE

/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/
#define CHECK_SPI_BUSY(peri) ((UC##peri##STATW & 0x0001)?1:0)

#define CHECK_SPI_OVERRUN(peri) ((UC##peri##STATW & 0x0020)?1:0)

#define CHECK_SPI_FRAME_ERR(peri) ((UC##peri##STATW & 0x0040)?1:0)

#define SPI_RX_INT_FLAG(peri)   ((UC##peri##IFG & 0x0001)?1:0)

#define CLR_SPI_RX_INT_FLAG(peri) (UC##peri##IFG &= ~(0x0001))

#define CLR_SPI_TX_INT_FLAG(peri)  (UC##peri##IFG &= ~(0x0002))

#define SPI_TX_INT_FLAG(peri)   ((UC##peri##IFG & 0x0002)?1:0)

#define SPI_IV_REG(peri)	(UC##peri##IV)

#define ENABLE_SPI_SETTING(peri) \
	do{				\
		UC##peri##CTLW0 |= 0x0001; \
	}while(0)

#define DISABLE_SPI_SETTING(peri)  \
	do{				\
		UC##peri##CTLW0 &= 0xFFFE; \
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

void spi_init(void);
void spi_read(void * const p_data, uint16_t data_count);
void spi_write(void * p_data, uint16_t data_count);

#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*HAL_SPI_UCSI_H_*/

/*** End of File **************************************************************/
