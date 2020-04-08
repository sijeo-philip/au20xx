/*******************************************************************************
* Title                 :   common
* Filename              :   common.c
* Author                :   Sijeo Philip
* Origin Date           :   20/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used for common functions accross the application
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
*  20/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file common.c
 *  @brief This is the source file for common functions such as delay
 *  and other string manipulations functions
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>             /* For portable types */

//TODO: UPDATE MY INCLUDE
#include "common.h"             /* For common functions such as delays */
#include "hal_fram.h"           /* For top variable address locations */
#include "hal_timerA.h"
#include "au20xx_api.h"
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
* Function : delay_us()
*//**
* \b Description:
*
* This function is used to generate delay in micro seconds
*
*
* PRE-CONDITION: Clocks should be initialized for 1MHz Source
*
* POST-CONDITION: Delay in Microsecond is generated;
*
* @return       A pointer to the configuration table.
*
* \b Example Example:
* @code
*
*   delay_us(100)
* @endcode
*
* @see system_clock_init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 20/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void delay_us( uint16_t microseconds )
{
   static uint16_t i =0;
   static uint16_t j = 0;
   for ( i = 0; i < microseconds; i ++ )
       for (j = 0; j <2400; j ++)
               __no_operation();
}


/******************************************************************************
* Function : get_top_variables
*//**
* \b Description:
*
* This function is used to read all the top variables from the memory store in a
* struct of type top_variables_t
*
* PRE-CONDITION: Clocks should be initialized for 1MHz Source
*
* POST-CONDITION: The data from FRAM is read
*
* @param[out]   values of top variables in a structure
*
* @return       None
*
* \b Example Example:
* @code
*  top_variables_t topVariable;
*  get_top_variable (&topVariable)
*
* @endcode
*
* @see system_clock_init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 30/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
bool get_top_variables(top_variables_t * topVariable)
{
    static bool retVal = false;
#if DEBUG == 1
    topVariable->samplesPerTemp = 100;
    topVariable->sensEnTime = 3;
    topVariable->sensEnTime_us = 384;
    topVariable->lastRotCount =0;
    topVariable->sampleTime = 20;
    topVariable->cd1_corr_slope = 1.054;
    topVariable->cd1_corr_slope = 2.0034;


#else

    retVal =  fram_read (&topVariable->samplesPerTemp , 4, SAMPLES_PER_TEMP_READ_ADDRESS);
    if( false == retVal )
        return retVal;
    set_samples_per_temperature_read( topVariable->samplesPerTemp);

    retVal =  fram_read(&topVariable->sensEnTime, 2, SAMPLE_TIME_ADD);
    if( false == retVal )
        return retVal;

    if( topVariable->sensEnTime > 3 )
    { topVariable->sensEnTime = 3 ; topVariable->sensEnTime_us = 384; }
    else
    {
        switch(topVariable->sensEnTime)
        {
        case 0:
            topVariable->sensEnTime_us = 48;
        break;
        case 1:
            topVariable->sensEnTime_us = 96;
        break;
        case 2:
            topVariable->sensEnTime_us = 192;
        break;
        case 3:
            topVariable->sensEnTime_us = 384;
        break;
        default:
            topVariable->sensEnTime_us = 384;
        break;
        }
    }
    retVal = fram_read(&topVariable->lastRotCount, 2, LAST_ROT_COUNT_ADD);
    if( false == retVal )
        return retVal;
    retVal = fram_read(&topVariable->sampleTime, 2, SAMPLE_TIME_ADD);
    if( false == retVal )
        return retVal;
    if ( (topVariable->sampleTime < 2) || (topVariable->sampleTime > 512))
        topVariable->sampleTime = 20;
    timerA_load_time(topVariable->sampleTime);

    retVal = fram_read(&topVariable->cd1_corr_slope, 4, CD1_CORR_SLOPE_ADD);
    if( false == retVal )
        return retVal;
    retVal = fram_read(&topVariable->cd2_corr_slope, 4, CD2_CORR_SLOPE_ADD);
    if( false == retVal )
        return retVal;
    au20xx_read_reg(SNS1_OFF0_REG, &topVariables->sns1_off0);
    au20xx_read_reg(SNS1_OFF1_REG, &topVariables->sns1_off0);
    au20xx_read_reg(SNS2_OFF0_REG, &topVariables->sns2_off0);
    au20xx_read_reg(SNS2_OFF1_REG, &topVariables->sns2_off1);

#endif
  return retVal;
}

/******************************************************************************
* Function : absolute
*//**
* \b Description:
*
* This function returns absolute value of the input
*
* PRE-CONDITION: Clocks should be initialized for 1MHz Source
*
* POST-CONDITION: absolute value of the input is returned
*
* @param[in]   variable whose absolute value has to be returned.
*
* @return       absolute value of the input variable.
*
* \b Example Example:
* @code
*  int8_t value;
* value = absolute (value)
*
* @endcode
*
* @see system_clock_init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 30/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
int8_t absolute(int8_t value)
{
    if ( value < 0 )
        return (-1 * value);
    else
        return value;
}

/******************************************************************************
* Function : configure_au20xx
*//**
* \b Description:
*
* This writes to all the registers of the au20xx
*
* PRE-CONDITION: Clocks should be initialized for 1MHz Source
*
* POST-CONDITION: absolute value of the input is returned
*
* @param[in]   variable whose absolute value has to be returned.
*
* @return       absolute value of the input variable.
*
* \b Example Example:
* @code
*  top_variable_t system_settings
*  configure_au20xx(&system_settings)
*
* @endcode
*
* @see system_clock_init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 8/04/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/

void configure_au20xx(top_variables_t * topVariables)
{

    au20xx_write_reg(SNS1_OFF0_REG, topVariables->sns1_off0);
    au20xx_write_reg(SNS1_OFF1_REG, topVariables->sns1_off0);
    au20xx_write_reg(SNS2_OFF0_REG, topVariables->sns2_off0);
    au20xx_write_reg(SNS2_OFF1_REG, topVariables->sns2_off1);

    au20xx_write_reg(INTF_CFG_REG, (0x78|topVariables->sensEnTime));

}

/*************** END OF FUNCTIONS ***************************************************************************/
