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
uint8_t reg_data=0;
top_variables_t system_settings;
static int8_t cd1_value;
static int8_t cd2_value;
static int8_t cd1_previous_value=0;
static int8_t cd2_previous_value=0;
static uint8_t volatile valid_data = 0;
static bool volatile normal_operation_mode = true;
static int8_t const tempInit = 25;
static int currTempValue = 0;
static int volatile tempNorm = 0;
static int8_t volatile delta_x =0;
static int8_t volatile delta_y = 0;
int8_t volatile delta_x_abs = 0;
int8_t volatile delta_y_abs = 0;
static int8_t volatile x0 =0;
static int8_t volatile y0 = 0;
static int8_t volatile delta_r = 0;
float volatile delta_XC;
float volatile delta_YC;
float volatile delta_previous_XC = 0;
float volatile delta_previous_YC = 0;
long volatile cd_rot_direction_x = 0;

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

    system_clock_init();        /**<< Set the Frequency of the system */
    gpio_init();                /**<< Initialize the GPIOs for primary/Tertiary/Secondary or as IOs */
    spi_init();                 /**<< Initialize the SPI peripheral */
    timerA_init();
    get_top_variables(&system_settings);
    timerA_load_time(system_settings.sampleTime);
#if DEBUG == 1
    au20xx_calibrate(&system_settings);
#endif
    configure_au20xx(&system_settings);
    refa_init();
    adc_init();
    au20xx_read_reg(INTF_CFG_REG, &reg_data);
   _BIS_SR(GIE);
}



/**
 * ENTRY POINT
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
   aura_hw_init();
   /*TODO : Check for the IO pin after Power ON to work in normal measurment mode
     or Calibration Mode, wherein the UART will be active and ready to recieve the
     Top Variable settings and store the same in the respective FRAM locations */
   while(!read_temp_sensor( &currTempValue )) {} /** << After ADC Initialization the First Temperature
                                                        Value is read*/
   for(;;){

       if(true == normal_operation_mode)
       {
          if ( true == sensENFlag)
          {
              configure_au20xx(&system_settings);
              valid_data = 0;
              SNS_EN_HIGH;
              delay_us(system_settings.sensEnTime_us);
              while( 0 == valid_data )
              {
                au20xx_read_reg( SNS_VALID_REG, (void*)&valid_data);
                delay_us(2);
              }
              SNS_EN_LOW;
              sensENFlag = false;
             if(valid_data)
              {
                 valid_data =0;
                 au20xx_read_reg(SNS1_OUT_Q8_REG, &cd1_value);
                 au20xx_read_reg(SNS2_OUT_Q8_REG, &cd2_value);
                 /** Temperature Compensate the Values */
                 tempNorm = currTempValue - tempInit;
                 cd1_value = (int8_t)(cd1_value - (system_settings.cd1_corr_slope * tempNorm));
                 cd2_value = (int8_t)(cd2_value - (system_settings.cd2_corr_slope * tempNorm));
                 if ((absolute(cd1_value - cd1_previous_value)) <=1)
                     cd1_value = cd1_previous_value;
                 if ((absolute(cd2_value - cd2_previous_value)) <=1)
                     cd2_value = cd2_previous_value;
                 delta_x = cd1_value - x0;
                 delta_y = cd2_value - y0;
                 delta_x_abs = absolute(delta_x);
                 delta_y_abs = absolute(delta_y);
                 if((delta_x == 0) && (delta_y == 0))
                    delta_r = 1;
                 else
                 delta_r = delta_x_abs + delta_y_abs;
                 delta_XC = (2*delta_x)/delta_r;           // We can convert to integer based on the decimal places.
                 delta_YC = (2*delta_y)/delta_r;           // We can convert to integer based on the decimal places.
                 x0 = (int8_t)(cd1_value - delta_XC);
                 y0 = (int8_t)(cd2_value - delta_YC);
                 if (( delta_XC >=0 ) && ( delta_previous_XC < 0 ))
                 {
                     if(delta_YC < 0)
                         cd_rot_direction_x = cd_rot_direction_x + 1;
                     else
                         cd_rot_direction_x = cd_rot_direction_x - 1;
                     // TODO: Need to understand when to reset the value... (Ask Nigesh or Sandeep)
                 }

              }
              delta_previous_XC = delta_XC;
              delta_previous_YC = delta_YC;
              cd1_previous_value = cd1_value;
              cd2_previous_value = cd2_value;
          }
          if ( true == temperatureReadFlag )
          {
              read_temp_sensor(&currTempValue);
          }
       }
       else
       {
          //TODO: Write for Calibration Mode of operation
       }
       //TODO : Get to Low Power Mode.. On timer Interrupt switch to Active Mode.
       //TODO : Feed the Watchdog. Mechanism
   }


}


/*************** END OF FUNCTIONS ***************************************************************************/
