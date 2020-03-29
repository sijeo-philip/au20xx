/*******************************************************************************
* Title                 :   main
* Filename              :   main.c
* Author                :   Sijeo Philip
* Origin Date           :   29/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This is main application entry point
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
*  29/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file main.c
 *  @brief This is the source file of the application Entry Point
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>             /* For portable types */
#include <stdbool.h>

//TODO: UPDATE MY INCLUDE
#include "common.h"             /* For All routines like string manipulation and other utilities*/
#include "hal_adc.h"            /* For all ADC routines */
#include "hal_gpio.h"           /* For GPIO routines */
#include "hal_osc.h"            /* For Oscillator and clocks */
#include "hal_refa.h"           /* For reference voltage generator */
#include "hal_spi_ucsi.h"       /* For SPI Peripheral */
#include "hal_timerA.h"         /* For TimerA */
#include "hal_fram.h"           /* For FRAM */
#include "au20xx_api.h"         /* For Au20xx chip */
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
* Function : aura_hw_init()
*//**
* \b Description:
*
* This function is used to initialize all the peripherals of the controller
*  as per configurations set in the respective *_config.h file
*
* PRE-CONDITION: System Power On
*
* POST-CONDITION: All the peripherals will be initialized and ready to use.
*
* @return       None
*
* \b Example Example:
* @code
*   aura_hw_init()
* @endcode
*
* @see timera_init()
* @see refa_init()
* @see adc_init()
* @see gpio_init()
* @see system_clock_init()
*
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 29/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void aura_hw_init ( void )
{
    system_clock_init();

}



/**
 * ENTRY POINT
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
   aura_hw_init();

    for(;;){}


}


/*************** END OF FUNCTIONS ***************************************************************************/
