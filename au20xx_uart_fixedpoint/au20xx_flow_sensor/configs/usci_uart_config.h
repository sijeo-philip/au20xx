/****************************************************************************
* Title                 :   usci_uart_config
* Filename              :   usci_uart_config.h
* Author                :   Sijeo Philip
* Origin Date           :   30/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is to configure the UART as per application
*
* THIS SOFTWARE IS PROVIDED BY UNISEM ELECTRONICS "AS IS" AND ANY EXPRESSED
* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL BENINGO ENGINEERING OR ITS CONTRIBUTORS BE LIABLE FOR ANY
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
*  30/03/20   1.0.0    Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file usci_uart_config.h
 *  @brief This module is used to configure the uart peripheral as
 *  per the needs of the application
 *
 *  This is the header file for the definition of configuration parameters
 *  of UART peripherals
 */
//TODO: UPDATE MACRO BELOW
#ifndef USCI_UART_CONFIG_H_
#define USCI_UART_CONFIG_H_

/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/
/*
 * @brief To Enable/Disable Parity Bit, Parity is Generated
 * while Transmit i.e UCAxTXD and expected during UCAxRXD
 * <0x0000=>    Disable Parity
 * <0x8000=>    Enable Parity
 *
 */

#ifndef CONF_UCACTLW0_UCPEN
#define CONF_UCACTLW0_UCPEN     0
#endif

/*
 * @brief To Select the Parity Check of the Data
 * This is not used when Parity is disabled
 * <0x0000=>    Odd Parity
 * <0x4000=>    Even Parity
 *
 */
#ifndef CONF_UCACTLW0_UCPAR
#define CONF_UCACTLW0_UCPAR     0
#endif

/*
 * @brief Controls the Direction of Receive and Transmit
 * Shift register
 * <0x0000=>    LSB First
 * <0x2000=>    MSB First
 *
 */
#ifndef CONF_UCACTLW0_UCMSB
#define CONF_UCACTLW0_UCMSB     0
#endif



/*
 * @brief To Select 7Bit or 8Bit Character length
 * <0x0000=>    8-bit Data
 * <0x1000=>    7-Bit Data
 *
 */
#ifndef CONF_UCACTLW0_UC7BIT
#define CONF_UCACTLW0_UC7BIT    0
#endif

/*
 * @brief To Select Stop Bit , Numbe of Stop bits
 * <0x0000=>    One Stop Bit
 * <0x0800=>    Two Stop Bit
 *
 */
#ifndef CONF_UCACTLW0_UCSPB
#define CONF_UCACTLW0_UCSPB    0
#endif

/*
 * @brief To Select the Mode of UART Operation, This
 * Selects th Asynchronous mode when USYNC = 0
 * <0x0000=>    UART MODE
 * <0x0200=>    IDLE Line Multiprocessor Mode
 * <0x0400=>    Address-bit multiprocessor mode
 * <0x0600=>    UART-Mode with automatic baud-rate detection
 */
#ifndef CONF_UCACTLW0_UCMODE
#define CONF_UCACTLW0_UCMODE    0
#endif
/*
 * @brief  To Enable Disable Synchronous Mode
 * <0x0000=>    Disable Synchronous Mode
 * <0x0100=>    Enable Synchronous Mode
 *
 */
#ifndef CONF_UCACTLW0_UCSYNC
#define CONF_UCACTLW0_UCSYNC    0
#endif

/*
 * @brief To Select Clock Source for UART Peripheral
 * <0x0000=>    UCLK is selected
 * <0x0040=>    ACLK is selected
 * <0x0080=>    SMCLK is selected
 * <0x00C0=>    SMCLK is selected
 */
#ifndef CONF_UCACTLW0_UCSSEL
#define CONF_UCACTLW0_UCSSEL    0x0080
#endif

/*
 * @brief On Setting this bit Interrupt is set when a
 * erroneous character is received
 * <0x0000=>    Erroneous Character rejected and UCRXIFG is not set
 * <0x0020=>    Erroneous Characters received set UCRXIFG
 *
 */
#ifndef CONF_UCACTLW0_UCRXEIE
#define CONF_UCACTLW0_UCRXEIE    0
#endif

/*
 * @brief On setting this bit Interrupt on Break Character is
 * Enabled/Disabled
 * <0x0000=>    Received Break Character do not set UCRXIFG
 * <0x0010=>    Received Break Character sets the UCRXIFG
 *
 */
#ifndef CONF_UCACTLW0_UCBRKIE
#define CONF_UCACTLW0_UCBRKIE    0
#endif

/*
 * @brief This bits Puts the UART peripheral to Sleep Mode
 * <0x0000=>    Not Dormant. All Received characters set UCRXIFG
 * <0x0008=>    Dormant. Only characters that are preceded by an
 *              Idle line or with address bit set the UCRXIFG.
 *              In UART mode with automatic baud-rate detection, only
 *              the combination fo break and synch field sets
 *              UCRXIFG
 */
#ifndef CONF_UCACTLW0_UCDORM
#define CONF_UCACTLW0_UCDORM    0
#endif

/*
 * @brief This bits set the Deglitch time for UART
 * <0x0000=>    Approximately 2ns
 * <0x0001=>    Approximately 50ns
 * <0x0002=>    Approximately 100ns
 * <0x0003=>    Approximately 200ns
 *
 */
#ifndef CONF_UCACTLW1_UCGLIT
#define CONF_UCACTLW1_UCGLIT    0
#endif

/*
 * @brief This is to set the Baud Rate clock of the UART peripheral
 * <0x0000 to 0xFFFF=>    Clock Pre-Scaler setting for Baud Rate
 *                        Generator
 */
#ifndef CONF_UCABRW_UCABRW
#define CONF_UCABRW_UCABRW    0x0006    /* << This is for 9600 Baud rate  as per table 3-5 */
#endif

/*
 * @brief This byte is used to set Second Modulation Stage
 * for Baud Rate
 * <0x0000 to 0xFF00=>    Second Modulation Stage Select
 *                        These bits hold a free modulation pattern
 *                        for BITCLK
 */
#ifndef CONF_UCAMCTLW_UCBRS
#define CONF_UCAMCTLW_UCBRS    0x2000    /* << This is for 9600 Baud rate  as per table 30-5 */
#endif


/*
 * @brief This is to set First Modulation STage for the Baud Rate
 * <0x0000 to 0xFFFF=>    Clock Pre-Scaler setting for Baud Rate
 *                        Generator
 */
#ifndef CONF_UCAMCTLW_UCBRF
#define CONF_UCAMCTLW_UCBRF    0x0080    /* << This is for 9600 Baud rate  as per table 3-5 */
#endif

/*
 * @brief This Bit is to Enable/Disable OverSampling Mode
 * <0x0000=>    Disables the OverSampling
 * <0x0001=>    Enables the OverSampling
 */
#ifndef CONF_UCAMCTLW_UCOS16
#define CONF_UCAMCTLW_UCOS16    0x0001    /* << This is for 9600 Baud rate  as per table 3-5 */
#endif


/*
 * @brief This bit is to enable Listen for Loopback facility
 * <0x0000=>    Disables the Listen
 * <0x0080=>    Enables the Listen. UCAxTXD is internally fed back to the
 *              receiver
 */
#ifndef CONF_UCASTATW_UCLISTEN
#define CONF_UCASTATW_UCLISTEN    0x0000
#endif

/******************************************************************************************
 * This Setting to be used if Auto Baud Rate Detection is Enabled
 */


/*
 * @brief This bit is used to set the Delimiter length if the Auto Baud rate
 * Detection is enabled
 * <0x0000=>     1 bit Time
 * <0x0010=>     2 bit Time
 * <0x0020=>     3 bit Time
 * <0x0030=>     4 bit Time
 */
#ifndef CONF_UCAABCTL_UCDELIM
#define CONF_UCAABCTL_UCDELIM    0x0000
#endif


/**
 * @brief This bit is used to enable/disable Automatic baud-rate detect
 * <0x0000=>     Baud-rate detection disabled. Length of break and synch field is not measured
 * <0x0001=>     Baud-rate detection is enabled. Length of break and synch field is measured
 *               and baud rate settings are changed accordingly
 */
#ifndef CONF_UCAABCTL_UCABDEN
#define CONF_UCAABCTL_UCABDEN    0x0000
#endif


/****
 * INTERRUPTS
 */
/*
 * @brief This Bit is to Enable/Disable Receive Interrupt
 * <0x0000=>    Disables the Interrupt
 * <0x0001=>    Enables the  Interrupt
 */
#ifndef CONF_UCAIE_UCRXIE
#define CONF_UCAIE_UCRXIE    0x0001
#endif

/*
 * @brief This Bit is to Enable/Disable Transmit Interrupt
 * <0x0000=>    Disables the Interrupt
 * <0x0002=>    Enables the  Interrupt
 */
#ifndef CONF_UCAIE_UCTXIE
#define CONF_UCAIE_UCTXIE    0x0000
#endif



/*
 * @brief This Bit is to Enable/Disable Start Bit Interrupt
 * <0x0000=>    Disables the Interrupt
 * <0x0004=>    Enables the  Interrupt
 */
#ifndef CONF_UCAIE_UCSTTIE
#define CONF_UCAIE_UCSTTIE    0x0000
#endif



/*
 * @brief This Bit is to Enable/Disable Transmit Complete Interrupt
 * <0x0000=>    Disables the Interrupt
 * <0x0008=>    Enables the  Interrupt
 */
#ifndef CONF_UCAIE_UCTXCPTIE
#define CONF_UCAIE_UCTXCPTIE    0x0000
#endif


/******************************************************************************
* Macros
*******************************************************************************/



/******************************************************************************
* Typedefs
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/


//TODO: UPDATE COMMENT BELOW
#endif /*USCI_UART_CONFIG_H_*/

/*** End of File **************************************************************/
