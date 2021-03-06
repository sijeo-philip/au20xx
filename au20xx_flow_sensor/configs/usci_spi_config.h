/****************************************************************************
* Title                 :   usci_spi_config
* Filename              :   usci_spi_config.h
* Author                :   Sijeo Philip
* Origin Date           :   17/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   this module is used to configure the USCI for SPI peripheral
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
*  17/03/20   1.0.0   Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file usci_config.h
 *  @brief This module to configure the usci for spi mode
 *
 *  This is the header file for the definition of properties of usci peripheral
 */
//TODO: UPDATE MACRO BELOW
#ifndef USCI_SPI_CONFIG_H_
#define USCI_SPI_CONFIG_H_

/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/
/*******************************************
 * This bit is used to enable the SPI peripheral
 * <0=> Disables the Peripheral
 * <1=> Enables the Peripheral
 */
#ifndef CONF_SPI_ENABLE
#define CONF_SPI_ENABLE 	1
#endif





/******************************************
 * This bit is used to configure STE Mode in Master Mode (This bit is ignored
 * in slave or 3-Wire Mode.
 * <0x0000=>  	STE pin is used prevent conflicts with other masters (default)
 * <0x0002=>	STE pin is used to generate the enable signal for a 4
 *          	wire slave
 *
 */
#ifndef CONF_UCCTLW0_UCSTEM
#define CONF_UCCTLW0_UCSTEM		0
#endif



/******************************************
 * This bit is used to configure eUSCI clock source.
 * <0x0000=>  	UCxCLK in slave mode. Do not use in master mode(default)
 * <0x0040=>	ACLK in master mode
 * <0x0080=>    SMCLK in master mode
 * <0x00C0=>    SMCLK in master mode
 */
#ifndef CONF_UCCTLW0_UCSSEL
#define CONF_UCCTLW0_UCSSEL		0x0040
#endif


/******************************************
 * This bit is used enable the synchronous mode
 * <0x0000=>  	Asynchronous Mode (default)
 * <0x0100=>	Synchronous Mode
 *
 */
#ifndef CONF_UCCTLW0_UCSYNC
#define CONF_UCCTLW0_UCSYNC		0x0100
#endif


/******************************************
 * This bit is used to select the SPI eUSCI mode
 * <0x0000=>  	3-pin SPI Mode(default)
 * <0x0200=>	4-pin SPI with UCxSTE active high
 * <0x0400=>    4-pin SPI with UCxSTE active low
 * <0x0600=>    Reserved
 */
#ifndef CONF_UCCTLW0_UCMODE
#define CONF_UCCTLW0_UCMODE		0x0000
#endif


/******************************************
 * This bit is used to select master mode
 * <0x0000=>  	Slave Mode (default)
 * <0x0800=>	Master Mode
 *
 */
#ifndef CONF_UCCTLW0_UCMST
#define CONF_UCCTLW0_UCMST		0x0800
#endif


/******************************************
 * This bit is used to select character length
 * <0x0000=>  	8-bit Data (default)
 * <0x1000=>	7-bit Data
 *
 */
#ifndef CONF_UCCTLW0_UC7BIT
#define CONF_UCCTLW0_UC7BIT		0x0000
#endif

/******************************************
 * This bit is used to select MSB/LSB First
 * <0x0000=>  	LSB First (default)
 * <0x2000=>	MSB First
 *
 */
#ifndef CONF_UCCTLW0_UCMSB
#define CONF_UCCTLW0_UCMSB		0x2000
#endif


/******************************************
 * This bit is used to select Clock Polarity
 * <0x0000=>  	Inactive State is low (default)
 * <0x4000=>	Inactive State is high
 *
 */
#ifndef CONF_UCCTLW0_UCCKPL
#define CONF_UCCTLW0_UCCKPL		0x4000
#endif


/******************************************
 * This bit is used to select Clock Phase
 * <0x0000=>  	 Data is changed in first UCLK edge and captured on
 * 				 the following edge(default)
 * <0x8000=>	Data is captured on the first UCLK edge and changed
 * 				on the following edge
 */
#ifndef CONF_UCCTLW0_UCCKPH
#define CONF_UCCTLW0_UCCKPH		0x0000
#endif


/******************************************
 * This bit is used to select Bit Rate of SPI Clock
 * <0x0000 - 0xFFFF=>  	(if O then Bit Clock = Clock Frequency)
 * Fbit_clock = Fbr_clock/CONF_UCAXCLTW0_UCAxBRW
 */
#ifndef CONF_UCBRW_UCBRW
#define CONF_UCBRW_UCBRW		0x0001
#endif


/******************************************
 * This bit is used to enable the Loop Back for SPI
 * <0x0000=> 	Listen Disabled (default)
 * <0x0080=>	Enabled. The Transmitter output is internally fed back
 * 				to the receiver
 */
#ifndef CONF_UCSTATW_UCLISTEN
#define CONF_UCSTATW_UCLISTEN		0x0000
#endif



/******************************************
 * This bit is used to enable Receive Interrupt
 * <0x0000=> 	Interrupt Disabled(default)
 * <0x0001=>	Interrupt Enabled
 */
#ifndef CONF_UCIE_UCRXIE
#define CONF_UCIE_UCRXIE		0x0000
#endif

/******************************************
 * This bit is used to enable Transmit Interrupt
 * <0x0000=> 	Interrupt Disabled(default)
 * <0x0002=>	Interrupt Enabled
 */
#ifndef CONF_UCIE_UCTXIE
#define CONF_UCIE_UCTXIE		0x0000
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

#endif /*USCI_SPI_CONFIG_H_*/

/*** End of File **************************************************************/
