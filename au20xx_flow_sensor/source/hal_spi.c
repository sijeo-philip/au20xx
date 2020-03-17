/*******************************************************************************
* Title                 :   hal_spi
* Filename              :   hal_spi.c
* Author                :   Sijeo Philip
* Origin Date           :   17/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to initializing the SPI and Operatiosn
*
* THIS SOFTWARE IS PROVIDED BY UNISEM ELECTRONICS  "AS IS" AND ANY EXPRESSED
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
*******************************************************************************/

/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author         Description
*  17/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file hal_spi.c
 *  @brief This is the source file for initializing the SPI peripherals
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>				/* For portable types */

//TODO: UPDATE MY INCLUDE
#include "hal_spi_ucsi.h"				/* For SPI routines */

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/

/******************************************************************************
* Function : spi_init()
*//**
* \b Description:
*
* This function is used to initialize the spi peripheral as per the configurations
* set in the file.
*
* PRE-CONDITION: Configuration to be done in ucsi_spi_config file
* PRE-CONDITION: The Oscillator should be initialized
* PRE-CONDITION: The GPIO should be intialized
*
* POST-CONDITION: SPI Peripheral will be configured and ready to use
*
* @return 		None
*
* \b Example Example:
* @code
*
*   system_init()
*   gpio_init()
* 	spi_init()
* @endcode
*
* @see system_init()
* @see gpio_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 17/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void spi_init( void )
{
	ENABLE_SPI_SETTING(B0);
	SPI_CTL_REG(B0) |= CONF_UCCTLW0_UCSTEM | CONF_UCCTLW0_UCSSEL | CONF_UCCTLW0_UCSYNC |
			       CONF_UCCTLW0_UCMODE | CONF_UCCTLW0_UCMST | CONF_UCCTLW0_UC7BIT |
				   CONF_UCCTLW0_UCMSB  | CONF_UCCTLW0_UCCKPL | CONF_UCCTLW0_UCCKPH ;

	SPI_BAUD_REG(B0) = CONF_UCBRW_UCBRW ;

	SPI_STAT_REG(B0) |= CONF_UCSTATW_UCLISTEN;

	SPI_INT_REG(B0)  |= CONF_UCIE_UCTXIE | CONF_UCIE_UCRXIE;

	DISABLE_SPI_SETTING(B0);

}

/*************** END OF FUNCTIONS ***************************************************************************/