/*******************************************************************************
* Title                 :   hal_gpio
* Filename              :   hal_gpio.c
* Author                :   Sijeo Philip
* Origin Date           :   16/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to initailize the GPIO as per the application
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
*******************************************************************************/

/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author         Description
*  16/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file hal_gpio.c
 *  @brief This is the source file for initializing the GPIO for the applicaiton
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>				/* For portable types */

//TODO: UPDATE MY INCLUDE
#include "hal_gpio.h"				/* For GPIO Macros*/

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
* Function : gpio_init()
*//**
* \b Description:
*
* This function is used to initialize the gpios for the application
*
* PRE-CONDITION: Clock should be set and enabled
*
* POST-CONDITION: The IOs will be set as input and output
*
* @return 		None
*
* \b Example Example:
* @code
*
*
* 	gpio_init();
* @endcode
*
* @see Dio_Init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 17/03/2020 </td><td> 0.5.0            </td><td> SP       </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void gpio_init( void )
{
	PM5CTL0 	=	ENABLE_PINS;			/* << Enables the use of inputs and outputs */

	SET_GPIO_PRI_FUN(P1, BIT4);             /* << P1.4 ==> SPI Clock Signal */
	SET_GPIO_PRI_FUN(P1, BIT6); 			/* << P1.6 ==> SPI Slave IN / Master Out (MOSI) */
	SET_GPIO_PRI_FUN(P1, BIT7);				/* << P1.7 ==> SPI Slave OUT / Master In (MISO) */

	SET_GPIO_OUTPUT(P2, BIT0);				/* << CS to start HI to avoid possible glitches */
	SET_GPIO_DIR_OUT(P2, BIT0);				/* << Make CS pin an output */

	SET_GPIO_PRI_FUN(PJ, BIT4|BIT5);		/* Enable some clock pins for SPIs synch operation */

	CLEAR_GPIO_OUTPUT(P1, BIT3);              /* << P1.3==> SENS_EN to high */
	SET_GPIO_DIR_OUT(P1, BIT3);            /* << Make SENS_EN to output */
}

/*************** END OF FUNCTIONS ***************************************************************************/
