/*******************************************************************************
* Title                 :   hal_adc
* Filename              :   hal_adc.c
* Author                :   Sijeo Philip
* Origin Date           :   25/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to initialize the ADC and read sensor
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
*  25/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file hal_adc.c
 *  @brief This is the source file for initializing the ADC and
 *  reading the temperature sensor.
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>             /* For portable types */
#include <stdbool.h>
//TODO: UPDATE MY INCLUDE
#include "hal_adc.h"             /* For ADC routines */
#include "common.h"
#include "au20xx_api.h"         /* For AU20xx IC routines */


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
static uint16_t volatile tempValue;
static int volatile degC;
static uint16_t volatile voltValue;
static float degC_per_bit;

bool volatile temperatureReadFlag = false;
extern bool readTemperatureFlag;
/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
// TODO: UPDATE AND COPY THESE FOR EACH NON_TRIVIAL FUNCTION
/******************************************************************************
* Function : adc_init()
*//**
* \b Description:
*
* This function is used to initialize the ADC as per the application
*
* PRE-CONDITION: Initialize the system clock
* PRE-CONDITION: Initialize the refa module
*
* POST-CONDITION: ADC is configured as per the parameters configured in the adc_config.h file
*
* @return       None
*
* \b Example Example:
* @code
*
*   adc_init();
* @endcode
*
* @see system_clock_init()
* @see refa_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 25/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*
*******************************************************************************/
//TODO: Call refa_init() before calling the below function
void adc_init( void )
{
   ADC_CONV_DISABLE;            /**<< The ADC conversion is stopped */
   ADC12CTL1 |= CONF_ADC12CTL1_ADC12SHS | CONF_ADC12CTL1_ADC12DIV | CONF_ADC12CTL1_ADC12PDIV |
                CONF_ADC12CTL1_ADC12SSEL | CONF_ADC12CTL1_ADC12CONSEQ | CONF_ADC12CTL1_ADC12SHP;

   ADC12CTL2 |= CONF_ADC12CTL2_ADC12RES | CONF_ADC12CTL2_ADC12DF | CONF_ADC12CTL2_ADC12PWRMD;

   ADC12CTL3 |= CONF_ADC12CTL3_ADC12ICH3MAP | CONF_ADC12CTL3_ADC12ICH2MAP | CONF_ADC12CTL3_ADC12ICH1MAP |
                CONF_ADC12CTL3_ADC12ICH0MAP | CONF_ADC12CTL3_ADC12TCMAP | CONF_ADC12CTL3_ADC12BATMAP |
                CONF_ADC12CTL3_ADC12CSTARTADD ;

   ADC12CTL0 |= CONF_ADC12CTL0_ADC12ON;

   ADC12CTL0 |= CONF_ADC12CTL0_ADC12SHT1 | CONF_ADC12CTL0_ADC12SHT0 | CONF_ADC12CTL0_ADC12MSC;

   ADC_MEMORY_CTL(0) |= COMPARATOR_WIN_DISABLE | ADC_SINGLE_ENDED | VREFP_VEREFB_VREFN_AVSS |
                        ADC_NOT_END_OF_SEQUENCE | ADC_A30_CHANNEL_SINGLE;
   ADC_MEMORY_CTL(1) |= COMPARATOR_WIN_DISABLE | ADC_SINGLE_ENDED | VREFP_AVCC_VREFN_AVSS |
                        ADC_END_OF_SEQUENCE | ADC_A31_CHANNEL_SINGLE;

   __adc_enable_memory_interrupt(0);
   __adc_enable_memory_interrupt(1);

   degC_per_bit = (CALADC_12V_85C - CALADC_12V_30C);
   ADC_CONV_ENABLE;
   __start_adc_conv();

}

/******************************************************************************
* Function : ADC12_ISR()
*//**
* \b Description:
*
* This is Interrupt Vecort Routine for Interrupt Caused due to ADC Interrupts Enabled
*
* PRE-CONDITION: Initialize the system clock
* PRE-CONDITION: Initialize the refa module
** PRE-CONDITION: ADC is configured as per the parameters configured in the adc_config.h file
*
* @return       None
*
* \b Example Example:
* @code
*
*
* @endcode
*
* @see system_clock_init()
* @see refa_init()
* @see adc_init()
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

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
  static uint16_t vectorNumber = 0;

  vectorNumber = ADC12IV & 0x0FFF;
  switch (vectorNumber)
  {
  case 0: break;
  case 2: break;
  case 4: break;
  case 6: break;
  case 8: break;
  case 10: break;
  case 12:
      tempValue = ((ADC_CONV_MEMORY(0) - CALADC_12V_30C) * (85 - 30));
      degC = (float)((long)tempValue)/degC_per_bit + 150.0;
      //TO DO: SAVE Temperature to Holding Register
      ADC12IFGR0 &= 0xFFFE;
      //TO DO: Go to low power mode if needed
  break;
  case 14:
          voltValue = 33 * (ADC_CONV_MEMORY(1) / 2048 );
          if ((voltValue > 13 ) && ( voltValue < 38 ))
          {
              //TO DO: Save to Holding Register
          }
          ADC12IFGR0 &= 0xFFFD;

    temperatureReadFlag = true;   /** << This flag is set to true once new values are read from
                                        temperature sensor and Battery ADC */

    ADC_CONV_DISABLE;
    //TO DO: Can go to low power mode if needed
   break;
  case 16: break;
  case 18: break;
  case 20: break;
  case 22: break;
  case 24: break;
  case 26: break;
  case 28: break;
  case 30: break;
  case 32: break;
  case 34: break;
  case 36: break;
  case 38: break;
  case 40: break;
  case 42: break;
  case 44: break;
  case 46: break;
  case 48: break;
  case 50: break;
  case 52: break;
  case 54: break;
  case 56: break;
  case 58: break;
  case 60: break;
  case 62: break;
  case 64: break;
  case 66: break;
  case 68: break;
  case 70: break;
  case 72: break;
  case 74: break;
  case 76: break;
  default: break;
  }

}

/******************************************************************************
* Function : read_temp_sensor()
*//**
* \b Description:
*
* This function is used to initialize the ADC as per the application
*
* PRE-CONDITION: Initialize the system clock
* PRE-CONDITION: Initialize the refa module
* PRE-CONDITION :  ADC is configured and running
*
* POST-CONDITION: new temperature value is read.
*
* @param[out]   degrees     new temperature value is read
*
* @return       bool       returns true if the new value is read else return false
*
* \b Example Example:
* @code
*
*  int temperature;
*  bool ret;
*  ret = read_temp_sensor(&temperature);
* @endcode
*
* @see system_clock_init()
* @see refa_init()
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

bool read_temp_sensor( void * degrees )
{
#if FPGA_CONNECT == 1 && CALIBRATION_TEST_EN == 0
    static bool retVal;
    retVal = temperatureReadFlag;
    if (true == temperatureReadFlag)
    {
      au20xx_read_reg( TEMP_REG_ADD, degrees);
      temperatureReadFlag = false;
    }
    return retVal;
#elif FPGA_CONNECT == 0 || CONSTANT_TEMP == 0 || CALIBRATION_TEST_EN ==1
    static bool retVal;
    int * degCel = (int*) degrees;
    retVal = temperatureReadFlag;

    *degCel = degC;
    if (true == temperatureReadFlag)
        temperatureReadFlag = false;
    return retVal;
#endif
}
/*************** END OF FUNCTIONS ***************************************************************************/




