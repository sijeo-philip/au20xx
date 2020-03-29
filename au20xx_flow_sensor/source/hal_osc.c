/*******************************************************************************
* Title                 :   hal_osc
* Filename              :   hal_osc.c
* Author                :   Sijeo Philip
* Origin Date           :   15/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This is used to intialize and use the oscillators for peripherals and core in use
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
*   15/03/20   1.0.0   Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file hal_osc.c
 *  @brief This is the source file for initializing the clocks for the Core and
 *         the peripherals in use
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>				/* For portable types */
#include <msp430.h>

#include "hal_osc.h"				/* For intializing the clock */
#include "common.h"

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
* Function : system_clock_init()
*//**
* \b Description:
*
* This function is used to initialize the clocks to be used for the core and the
* peripherals in use for the application.
*
* PRE-CONDITION: Configurations in osc_config file should be set
*
* POST-CONDITION: The clocks for the Core and peripherals will be set as per the
* 				  configurations in the corresponding config file
*
* @return 		None
*
* \b Example Example:
* @code
*
* 	system_clock_init();
* @endcode
*
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 15/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void system_clock_init( void )
{
    CSCTL0_H = CSKEY >> 8; 					//**<< Unlock CS Register
    CSCTL1 = CONF_CSCTL1_DCOFSEL|CONF_CSCTL1_DCORSEL;   			 // **<< Set Clock source for MCU
    CSCTL2 = CONF_CSCTL2_SELA | CONF_CSCTL2_SELS | CONF_CSCTL2_SELM;  // **<< Set Source for MCLK, SMCLK, ACLK
    CSCTL3 = CONF_CSCTL3_DIVA | CONF_CSCTL3_DIVS | CONF_CSCTL3_DIVM;  // **<< Set Clock Division for MCLK, SMCLK, ACLK
    CSCTL4 = CONF_CSCTL4_LFXTOFF | CONF_CSCTL4_SMCLKOFF | 			  // **<< Enabling/Disabling the Clock Sources
    		 CONF_CSCTL4_VLOOFF | CONF_CSCTL4_LFXTBYPASS |
			 CONF_CSCTL4_LFXTDRIVE | CONF_CSCTL4_HFXTOFF |
			 CONF_CSCTL4_HFFREQ | CONF_CSCTL4_HFXTBYPASS | CONF_CSCTL4_HFXTDRIVE;
    CSCTL6 = CONF_CSCTL6_ACLKREQEN | CONF_CSCTL6_MCLKREQEN |		// ** << Conditional Requests for Each clock source
    		 CONF_CSCTL6_SMCLKREQEN | CONF_CSCTL6_MODCLKREQEN;
#if 0
    do
    {
    	CSCTL5 = CONF_CSCTL5_LFXTOFFG | CONF_CSCTL5_HFXTOFFG |
    			CONF_CSCTL5_ENSTFCNT1 | CONF_CSCTL5_ENSTFCNT2;
    	SFRIFG1 = SFRIFG1 & 0xFFFD;                               //** << Waiting and clearing fault flags of clock src
    	delay_us(1);
    }while( SFRIFG1 & OFIFG);
#endif
    CSCTL0_H = 0;							  						// ** << Lock CS Register
 }


/*************** END OF FUNCTIONS ***************************************************************************/
