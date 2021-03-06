/*******************************************************************************
* Title                 :   hal_refa
* Filename              :   hal_refa.c
* Author                :   Sijeo Philip
* Origin Date           :   26/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to initialize the REFA Module of MCU
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
*  26/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file hal_refa.c
 *  @brief This is the source file for initializing the REFA module this
 *  should be modified as per the application needs.
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>             /* For portable types */
#include <stdbool.h>            /* For boolean types */
//TODO: UPDATE MY INCLUDE
#include "hal_refa.h"             /* For set and check registers and flags
                                   of REFA module */

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

/******************************************************************
 * Inline Functions
 ******************************************************************/

/**
 * @brief This is one time trigger for Band gap buffer
 *
 */
inline void __one_time_trigger_ref_bandgap( void )
 {
     REFCTL0 |= 0x0080;
 }


 /**
  * @brief This is one time trigger for Reference Generator
  */
inline void __one_time_trigger_ref_gen( void )
{
    REFCTL0 |= 0x0040;
}


/**
 * @brief This set the reference voltage of the REFA
 */

inline bool __set_refa_voltage( ref_voltage_t voltage_setting )
{
    if ( IS_REFGEN_BUSY() )
        return false;
    else
    REFCTL0 |= voltage_setting;
    return true;
}

/**
 * @brief Disable the Temperature Sensor
 */
inline bool __set_disable_temp_sensor( void )
{
    if ( IS_REFGEN_BUSY() )
        return false;
    else
       REFCTL0 |= 0x0008;
    return true;
}

/**
 * @brief This this to enable Reference Output Buffer
 * On devices with an ADC10_A, this bit must be written
 * with 0
 */
inline bool __enable_reference_output( void )
{
    if ( IS_REFGEN_BUSY() )
        return false;
    else
        REFCTL0 |= 0x0002;
    return true;
}


/**
 * @brief This is to enable the Reference of REFA
 *
 */
inline bool __enable_reference( void )
{
    if ( IS_REFGEN_BUSY() )
        return false;
    else
        REFCTL0 |= 0x0001;
    return true;
}


/******************************************************************************
* Function : refa_init()
*//**
* \b Description:
*
* This function is used to initialize the REFA module as per the application needs.
*
* PRE-CONDITION: The clock of the system should be initailized
*
* POST-CONDITION: The REFA module will be initalized and ready to use
*                 for other peripherals
*
* @return       None
*
* \b Example Example:
* @code
* refa_init()
* @endcode
*
* @see system_clock_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 25/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void refa_init( void )
{
   __set_refa_voltage(REF_1_2V);  /** <<SET the Reference Voltage to 1.2V */
   __enable_reference();          /** <<Enable Internal Reference */
   while ( !IS_REFGEN_READY() ) {} /** << Wait till the Reference Generator is Ready */

}

/*************** END OF FUNCTIONS ***************************************************************************/



