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
#include <string.h>             /* For String types */
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
uint16_t sensEn_timer_delay = 0;
static bool sensEn_delay_flag = false;
char data[5];


extern uint32_t volatile samplesPerTempReading;
/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/

/******************************************************************************
* Function : sensEn_delay_us()
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
*   sensEn_delay_us()
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
void sensEn_delay_us( void )
{
     timerA1_load_time( 0 );
    timerA1_load_time(sensEn_timer_delay);
    sensEn_delay_flag = true;
    SNS_EN_HIGH;
    while ( true == sensEn_delay_flag){}
    timerA1_load_time(0);
}

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
*   delay_us(300)
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
void delay_us(uint16_t microseconds )
{
    static uint16_t i=0;
    for (i = 0; i < microseconds; i++)
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
    topVariable->samplesPerTemp = 0;
    samplesPerTempReading = topVariable->samplesPerTemp;
    topVariable->sensEnTime = 3;
    topVariable->sensEnTime_us = 384;
    sensEn_timer_delay = 380 * 4;
    topVariable->lastRotCount =0;
    topVariable->sampleTime = 50;
    topVariable->cd1_corr_slope = 2.004;
    topVariable->cd2_corr_slope = 1.802;
    topVariable->tempInit = 30;

#endif

#if FPGA_CONNECT == 0
    fram_read(&topVariable->sns1_off0, 1, SNS1_OFF0);
    fram_read(&topVariable->sns1_off1, 1, SNS1_OFF1);
    fram_read(&topVariable->sns2_off0, 1, SNS2_OFF0);
    fram_read(&topVariable->sns2_off1, 1, SNS2_OFF1);
    fram_read(&topVariable->tempInit, 1, INIT_TEMP_ADD);
#endif
#if DEBUG == 0

    retVal =  fram_read (&topVariable->samplesPerTemp , 4, SAMPLES_PER_TEMP_READ_ADDRESS);
    if( false == retVal )
        return retVal;
    set_samples_per_temperature_read( topVariable->samplesPerTemp);

    retVal =  fram_read(&topVariable->sensEnTime, 2, SENS_EN_TIME_ADD);
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
            sensEn_timer_delay = 42 * 4;
        break;
        case 1:
            topVariable->sensEnTime_us = 96;
            sensEn_timer_delay = 90 * 4;
        break;
        case 2:
            topVariable->sensEnTime_us = 192;
            sensEn_timer_delay = 184* 4;
        break;
        case 3:
            topVariable->sensEnTime_us = 384;
            sensEn_timer_delay = 380* 4;

        break;
        default:
            topVariable->sensEnTime_us = 384;
            sensEn_timer_delay = 380 * 4;
        break;
        }
    }
    retVal = fram_read(&topVariable->lastRotCount, 2, LAST_ROT_COUNT_ADD);
    if( false == retVal )
        return retVal;
    retVal = fram_read(&topVariable->sampleTime, 2, AU20xx_READ_TIME_ADD);
    if( false == retVal )
        return retVal;
    if ( (topVariable->sampleTime < 2) || (topVariable->sampleTime > 512))
        topVariable->sampleTime = 20;
    timerA0_load_time(topVariable->sampleTime);

    retVal = fram_read(&topVariable->cd1_corr_slope, 4, CD1_CORR_SLOPE_ADD);
    if( false == retVal )
        return retVal;
    retVal = fram_read(&topVariable->cd2_corr_slope, 4, CD2_CORR_SLOPE_ADD);
    if( false == retVal )
        return retVal;




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
_iq24 absolute(_iq24 value)
{
    if ( value < 0 )
        return _IQ24mpy(_IQ24(-1.0) , value);
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
    au20xx_chip_reset();
#if 0
    au20xx_write_reg(SNS1_OFF0_REG, topVariables->sns1_off0);
    au20xx_write_reg(SNS1_OFF1_REG, topVariables->sns1_off1);
    au20xx_write_reg(SNS2_OFF0_REG, topVariables->sns2_off0);
    au20xx_write_reg(SNS2_OFF1_REG, topVariables->sns2_off1);
#endif
    au20xx_write_reg(INTF_CFG_REG, (0x78|topVariables->sensEnTime));

}


void set_au20xx_regs(top_variables_t * topVariables)
{
    au20xx_chip_reset();
    au20xx_write_reg(SNS1_OFF0_REG, topVariables->sns1_off0);
    au20xx_write_reg(SNS1_OFF1_REG, topVariables->sns1_off1);
    au20xx_write_reg(SNS2_OFF0_REG, topVariables->sns2_off0);
    au20xx_write_reg(SNS2_OFF1_REG, topVariables->sns2_off1);

    au20xx_write_reg(INTF_CFG_REG, (0x78|topVariables->sensEnTime));

}



/******************************************************************************
* Function : average_by_4
*//**
* \b Description:
*
* This function returns average value of the array of 4 long unsigned element
*
* PRE-CONDITION: Clocks should be initialized for 1MHz Source
*
* POST-CONDITION: Average of 4 elements are calculated and returned
*
* @param[in]   address to the array of 4 elements
*
* @return       average of 4 elements
*
* \b Example Example:
* @code
*  uint32_t value[5];
*  uint32_t avg_value;
* avg_value = absolute (value)
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
uint32_t average_by_4(uint32_t * address_of_array, int* slope_of_slope)
{
    static uint8_t i = 0;
    static uint32_t average_value;
    int slope2, slope1;
    slope1 = 0 ; slope2 = 0;
    average_value = 0;
    for (i = 0; i < 4; i++ )
    {
        average_value += address_of_array[i];
    }
    average_value = average_value >> 2;

    slope1 = address_of_array[1] - address_of_array[0];
    slope2 = address_of_array[3] - address_of_array[2];

    *slope_of_slope = slope2 - slope1;
    return average_value;
}






/**********************************
 * TIMERA1 ISR
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_ISR ( void )
{
     sensEn_delay_flag = false;
     AU20xx_CAP_COMP_REG(1) = 0;  /** << This is to stop the timer by loading 0
                                         to the timer register */
     SNS_EN_LOW;
  }


/******************************************************************************
* Function : strcpymarker()
*//**
* \b Description:
*
* This function is extract the string in between the startMarker and end Marker
* from source string and store the the same in the destination array
*
* PRE-CONDITION: None
*
* POST-CONDITION: The string between the start Marker and End Marker in the
*                 source string is copied to the destination string.
*
* @param[in]    Address of the Source String
* @param[out]   Address of the Destination String
* @param[in]    The starting delimiter of the string
* @param[in]    The Ending delimiter of the String

* @return       Address of the next location to End delimiter
* \b Example Example:
* @code
*  uint8_t SrcString[50] = "sijeo@hotmail.com";
*   uint8_t destString[20];
*  uint8_t startMarker = '@';
* uint8_t EndMarker = '.';
*   strcpymarker(SrcString, destString, startMarker, EndMarker);
*  /// The string hotmail will be stored in the dstString
* @endcode
*
*
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 20/05/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
static char * strcpymarker(char * src, char * dest, uint8_t startMarker, uint8_t endMarker)
{

        char *s, *d;
        uint16_t bytes=0;
        while(*src != startMarker)
        {
            src++;
            if(*src == '\0')
            return 0;
        }
        s = src;
        src++;
        s++;
        while(*src != endMarker)
        {
            src++;
            if(*src == '\0')
            return 0;
        }
        d = src;
        while(s!=d)
        {
            *dest = *s;
            dest++;
            s++;
            bytes++;
        }

        //dest++;
        *dest = '\0';

        return d;
}

/******************************************************************************
* Function : convert_string_to_integer()
*//**
* \b Description:
*
* This function is used to convert string to integer value
*
* PRE-CONDITION: None
*
* POST-CONDITION: The string been passed to function is returned as unsigned integer
*
* @return       Unsigned Integer up to 255

* @param[in]    address of the String to be converted to number
*
* \b Example Example:
* @code
*  uint8_t strNumber[2] = "22"
*  uint8_t number = 0;
*   number = convert_string_to_integer(strNumber);
* @endcode
*
*
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 21/05/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
uint16_t convert_string_to_integer(char *const number)
{
    uint16_t dec = 0, i;
    uint32_t len = 0;
    len = strlen(number);
    for( i = 0; i < len; i++ )
    dec = dec * 10 + (number[i] - '0');

    return dec;
}


/******************************************************************************
* Function : bool process_uart_data(char*, top_variables_t*)
*//**
* \b Description:
*
* This function is used to parse the data received from the UART and set top variables as
*
* PRE-CONDITION: None
*
* POST-CONDITION: The character array which is passed as input is parsed and stored in the
* top variable structure
*
* @param[in]    Address of the Source String
* @param[out]   Address of the top variable structure which is been updated
* @return       True if it is successfully stored else false
* \b Example Example:
* @code
*  top_variable_t system_settings;
*   bool retVal;
*   retVal = process_uart_data(&sourceString, &system_settings);
*  @endcode
*
*
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 20/05/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
bool process_uart_data(char* rx_data, top_variables_t* topVariables, bool *calibration_flag)
{
    char* temp_ptr;

    temp_ptr = rx_data;
    memcpy(data, '\0', 5);
    temp_ptr = strcpymarker(temp_ptr, data, '#', ',');
    if (temp_ptr == 0)
        return false;
    else
        topVariables->samplesPerTemp = convert_string_to_integer(data);
    if(topVariables->samplesPerTemp > 512)
        topVariables->samplesPerTemp = 100; /* if the user tries to set value greater than 512 the value is set to*/
                                            /* 100 by default */
    memcpy(data, '\0', 5);
    temp_ptr = strcpymarker(temp_ptr, data, ',', ',');
    if(temp_ptr == 0)
        return false;
    else
        topVariables->sensEnTime = convert_string_to_integer(data);
    if( topVariables->sensEnTime > 3)
        topVariables->sensEnTime = 3;  /* if the user tries to set value greater than 3 the value is set to */
                                       /* 3 by default */
    memcpy(data, '\0', 5);
    temp_ptr = strcpymarker(temp_ptr, data, ',',',');
    if(temp_ptr == 0)
        return false;
    else
        topVariables->sampleTime = convert_string_to_integer(data);
    if(topVariables->sampleTime > 512)
        topVariables->sampleTime = 50;  /* if the user tries to set value of Sampling time greater than 512ms */
                                        /* it is reset to 50ms by default */
    memcpy(data, '\0', 5);
    temp_ptr = strcpymarker(temp_ptr, data, ',',',');
    if (temp_ptr == 0)
        return false;
    else
        topVariables->calibSampleTime = convert_string_to_integer(data);
    if(topVariables->calibSampleTime > 512)
        topVariables->calibSampleTime  =250;  /* if the user tries to set value of calibration sampling greater than*/
                                              /* 512ms then the value is reset to 250ms by default */
    memcpy(data, '\0', 5);
    temp_ptr = strcpymarker(temp_ptr, data, ',', '#');
    if (temp_ptr == 0)
        return false;
    else
    {
        if (convert_string_to_integer(data) == 1)
            *calibration_flag = true;
        else *calibration_flag = false;

    }
    return true;
}



/*************** END OF FUNCTIONS ***************************************************************************/
