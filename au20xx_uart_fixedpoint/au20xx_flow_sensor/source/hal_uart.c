/*******************************************************************************
* Title                 :   hal_uart
* Filename              :   hal_uart.c
* Author                :   Sijeo Philip
* Origin Date           :   04/29/2015
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FRXX
* Notes                 :   This module is used to access the UART peripheral
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
*  04/29/15   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file hal_uart.c
 *  @brief This is the source file for initializing and communicating using
 *         UART Peripheral
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "hal_uart_ucsi.h"

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
* Function : uart_init()
*//**
* \b Description:
*
* This function is used to initialize the UART peripheral
*
* PRE-CONDITION: The clock should be configured for the system
* PRE-CONDITION: The GPIOs should be configured
*
* POST-CONDITION: The UART will be configured as per the configuration header
*                 and ready to use.
*
* @return       None
*
* \b Example Example:
* @code
*    uart_init(DioConfig);
* @endcode
*
* @see  aura_hw_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 29/04/2015 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void uart_init( void )
{
  UART_ENABLE_SW_RST(A0);
  UART_CTRL_WORD_REG0(A0) |= CONF_UCACTLW0_UCPEN | CONF_UCACTLW0_UCPAR | CONF_UCACTLW0_UCMSB |
                             CONF_UCACTLW0_UC7BIT| CONF_UCACTLW0_UCSPB | CONF_UCACTLW0_UCMODE|
                             CONF_UCACTLW0_UCSYNC| CONF_UCACTLW0_UCSSEL| CONF_UCACTLW0_UCRXEIE |
                             CONF_UCACTLW0_UCBRKIE | CONF_UCACTLW0_UCDORM ;
  UART_CTRL_WORD_REG1(A0) |= CONF_UCACTLW1_UCGLIT;
  UART_BAUD_RATE_REG(A0) |= CONF_UCABRW_UCABRW;
  UART_MOD_CTRL_REG(A0) |= CONF_UCAMCTLW_UCBRS | CONF_UCAMCTLW_UCBRF | CONF_UCAMCTLW_UCOS16;
  UART_STATW_REG(A0) |= CONF_UCASTATW_UCLISTEN;
  UART_INT_ENABLE_REG(A0) |= CONF_UCAIE_UCRXIE | CONF_UCAIE_UCTXIE | CONF_UCAIE_UCSTTIE |
                             CONF_UCAIE_UCTXCPTIE;
  UART_DISABLE_SW_RST(A0);
}

/******************************************************************************
* Function : UART_ISR()
*//**
* \b Description:
*
* This function is UART INTERRUPT HANDLER FOR A0 peripheral
*
*
* PRE-CONDITION: The clock should be configured for the system
* PRE-CONDITION: The GPIOs should be configured
* PRE-CONDIITON: UART interrupt should be enabled
*
* POST-CONDITION: This ISR will be triggered on Event of Enabled Interrupt
*
* @return       None
*
* \b Example Example:
* @code
*    __interrupt void UART_ISR(void)
* @endcode
*
* @see  uart_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 29/04/2015 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
#pragma vector = USCI_A0_VECTOR
__interrupt void UART_ISR(void)
{
    if(UART_RX_END_INT_FLG(A0))
    {
        UART_CLR_RXIFG(A0);
    }
}
/*************** END OF FUNCTIONS ***************************************************************************/
