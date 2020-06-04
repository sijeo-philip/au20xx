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
#include "hal_uart_ucsi.h"      /* For UART */

#include "IQmathLib.h"
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

static char uart_top_variable[UART_BUFF_SIZE];
static uint16_t uart_byte_count = 0;

top_variables_t system_settings;
static bool volatile normal_operation_mode = true;

uint8_t i_temp;            /** << For loop counter for Calibration Mode average filtering */
uint16_t avg_temp_value;
int8_t currTempValue;       /** << Variable used for calculating Initial Temperature during Calibration */
 bool calibration_done_flag = false;

extern uint16_t sensEn_timer_delay;
extern int8_t sum_of_tempNorm[10];
bool start_calibration_flag = false;


SYSTEM_VAR_DEF(au20xx_chip_variables);
CALIB_VAR_DEF(au20xx_calib_variables);

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
   int i = 0;
    system_clock_init();        /**<< Set the Frequency of the system */
    gpio_init();                /**<< Initialize the GPIOs for primary/Tertiary/Secondary or as IOs */
    spi_init();                 /**<< Initialize the SPI peripheral */
    timerA_init();
    _BIS_SR(GIE);
    //TODO : an GPIO has to read of HIGH or LOW , if HIGH normal_operation flag is to
    //       be set to true else false
    get_top_variables(&system_settings);

#if 0
    au20xx_calibrate(&system_settings);
#endif
#if FPGA_CONNECT == 0 && CALIBRATION_TEST_EN == 0
    set_au20xx_regs(&system_settings);
#endif
for (i = 0; i < 8; i++ )
{
    sum_of_tempNorm[i] = system_settings.tempInit;
}
#if FPGA_CONNECT == 0
    refa_init();
    adc_init();
#if DEBUG == 1
    au20xx_read_reg(INTF_CFG_REG, &reg_data);
    au20xx_read_reg(SNS1_OFF0_REG, &offset_reg_values[0]);
    au20xx_read_reg(SNS1_OFF1_REG, &offset_reg_values[1]);
    au20xx_read_reg(SNS2_OFF0_REG, &offset_reg_values[2]);
    au20xx_read_reg(SNS2_OFF1_REG, &offset_reg_values[3]);
#endif
#endif

    timerA0_load_time(system_settings.sampleTime);

#if CALIBRATION_TEST_EN == 1
    normal_operation_mode = false;
    uart_init();
    refa_init();
    adc_init();
    uart_send("OK", 2);
#endif



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
#if CONSTANT_TEMP == 1
   au20xx_chip_variables.currTempValue = 30;
   system_settings.tempInit = 30;
#endif
   for(;;){

       if(true == normal_operation_mode)
       {
           if ( true == sensENFlag)
            {
              sensENFlag = false;
               process_sensor_data(&au20xx_chip_variables, &system_settings);  /*<< Returns false if valid bit is not high */
               SLEEP;
            }
        }
  else
       {

           timerA0_load_time(10);

          //TODO: Write for Calibration Mode of operation


           uart_byte_count = uart_read(uart_top_variable);
           if ((uart_byte_count > 0) && (start_calibration_flag == false))
           {
               if('#' == uart_top_variable[uart_byte_count-1])
               {
                    //if(process_data(uart_top_variable);)
                   //TODO : process data function to be written to parse the data received and store in FRAM
                   // if the data is OK it will return 'true' else 'false'
                   if(process_uart_data(uart_top_variable, &system_settings, &start_calibration_flag))
                   {
                       uart_send("OK", 2);
                       fram_write(&system_settings.samplesPerTemp, 4, SAMPLES_PER_TEMP_READ_ADDRESS);
                       fram_write(&system_settings.sampleTime, 2, AU20xx_READ_TIME_ADD);
                       fram_write(&system_settings.sensEnTime, 1, SENS_EN_TIME_ADD);
                       switch(system_settings.sensEnTime)
                        {
                          case 0:
                            system_settings.sensEnTime_us = 48;
                            sensEn_timer_delay = 42 * 4;
                          break;
                          case 1:
                             system_settings.sensEnTime_us = 96;
                             sensEn_timer_delay = 90 * 4;
                          break;
                          case 2:
                             system_settings.sensEnTime_us = 192;
                             sensEn_timer_delay = 184 * 4;
                          break;
                          case 3:
                             system_settings.sensEnTime_us = 380;   /*previous working value 384*/
                             sensEn_timer_delay = 380*4;              /*previous working value 380 */
                           break;
                           default:
                              system_settings.sensEnTime_us = 380;  /* previous working value 384 */
                              sensEn_timer_delay = 380 * 4;              /* previous working value 380 */
                            break;
                          }
                       fram_write(&system_settings.calibSampleTime, 2, CALIB_SAMPLE_TIME_ADD);
                       timerA0_load_time(system_settings.calibSampleTime);
                       configure_au20xx(&system_settings);
                   }
                   else
                       uart_send("ERROR", 5);
               }
               else
                   uart_send("ERROR", 5);
           }

           if (( true == sensENFlag) && (false == calibration_done_flag)&&(start_calibration_flag == true))
           {
              sensENFlag = false;
              if( (au20xx_calib_variables.total_cd1_sample_count & au20xx_calib_variables.total_cd2_sample_count) != 16)
              {
                  calibrate_au20xx(&au20xx_calib_variables);
              }
              else
              {
                  au20xx_calib_variables.avg_max_cd1_value = au20xx_calib_variables.avg_max_cd1_value >> 3;
                  au20xx_calib_variables.avg_max_cd2_value = au20xx_calib_variables.avg_max_cd2_value >> 3;
                  au20xx_calib_variables.avg_min_cd1_value = au20xx_calib_variables.avg_min_cd1_value >> 3;
                  au20xx_calib_variables.avg_min_cd2_value = au20xx_calib_variables.avg_min_cd2_value >> 3;
                  if ((((au20xx_calib_variables.avg_max_cd1_value - au20xx_calib_variables.avg_min_cd1_value) < 15 ) || (( au20xx_calib_variables.avg_max_cd2_value - au20xx_calib_variables.avg_min_cd2_value ) < 15)) && (calibration_done_flag == false))
                  {
                      uart_send("CALIB NOT OK", 12);
                      calibration_done_flag = true;
                   }
                    else if (calibration_done_flag == false)
                   {

                       au20xx_calib_variables.cd1_offset_value = au20xx_calib_variables.cd1_offset_value >> 4;
                       au20xx_calib_variables.cd2_offset_value = au20xx_calib_variables.cd2_offset_value >> 4;
                       system_settings.sns1_off0 = (uint8_t)au20xx_calib_variables.cd1_offset_value;
                       system_settings.sns1_off1 = (uint8_t)(au20xx_calib_variables.cd1_offset_value >> 8);
                       system_settings.sns2_off0 = (uint8_t)au20xx_calib_variables.cd2_offset_value;
                       system_settings.sns2_off1 = (uint8_t)(au20xx_calib_variables.cd2_offset_value >> 8);
                       for(i_temp=0; i_temp<8; i_temp++)
                         {
                           __start_adc_conv();
                           while(!read_temp_sensor(&currTempValue)){}
                           avg_temp_value += currTempValue;
                           avg_temp_value = avg_temp_value >> 1;
                          }
                        currTempValue = avg_temp_value;
                        fram_write(&currTempValue, 1, INIT_TEMP_ADD);
                        fram_write(&system_settings.sns1_off0, 1, SNS1_OFF0);
                        fram_write(&system_settings.sns1_off1, 1, SNS1_OFF1);
                        fram_write(&system_settings.sns2_off0, 1, SNS2_OFF0);
                        fram_write(&system_settings.sns2_off1, 1, SNS2_OFF1);
                        calibration_done_flag = true;
                        start_calibration_flag = false;
                        uart_send("CALIB DONE", 10);
                 }

              }   /**<< 16 Samples are collected of CD1 and CD2 */


           }/** << End of Calibration routine */






              }  /** << Calibration Mode */

           //TODO : Call configure_au20xx function only if calibration is done by this process
           /***
            * In this process the Calibration process is run and the offset value is written to fram so that
            * on Reset it is read from FRAM and written to OFFSET registers of the chip by calling configure_au20xx()
            * function ... Should not call calibrate_au20xx() function once calibration is done through this
            * routine .
            */
       }
       //TODO : Get to Low Power Mode.. On timer Interrupt switch to Active Mode.
       //TODO : Feed the Watchdog. Mechanism
   }




/*************** END OF FUNCTIONS ***************************************************************************/

