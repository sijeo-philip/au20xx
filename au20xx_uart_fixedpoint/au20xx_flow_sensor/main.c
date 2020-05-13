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
#include "au20xx_api.h"         /* For Au20xx chip */
#include "IQmathLib.h"
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
uint8_t offset_reg_values[5];
static uint8_t uart_top_variable[UART_BUFF_SIZE];
static uint16_t uart_byte_count = 0;

top_variables_t system_settings;
static _iq24 cd1_value;
static _iq24 cd2_value;
_iq24 cd1_value_corr=0;
_iq24 cd2_value_corr=0;
_iq24 cd1_previous_value=0;
_iq24 cd2_previous_value=0;
static uint8_t volatile valid_data = 0;
static bool volatile normal_operation_mode = true;
static int8_t const tempInit = 30;
static int8_t currTempValue = 0;
static int8_t volatile tempNorm = 0;
static _iq24 volatile delta_x =0;
static _iq24 volatile delta_y = 0;
_iq24 volatile delta_x_abs = 0;
_iq24 volatile delta_y_abs = 0;
static _iq24 volatile x0 =0;
static _iq24 volatile y0 = 0;
static _iq24 volatile delta_r = 0;
_iq24 volatile delta_XC;
_iq24 volatile delta_YC;
_iq24 volatile delta_previous_XC = 0;
_iq24 volatile delta_previous_YC = 0;
long volatile cd_rot_direction_x = 0;
_iq24 cd1_corr_slope;
_iq24 cd2_corr_slope;
_iq24 q24_tempNorm;

_iq24 one = _IQ24(1);
_iq24 two = _IQ24(2);

_iq24 pi = _IQ24(3.14149);
_iq24 two_pi = _IQ24(6.2832);
_iq24 neg_pi = _IQ24(-3.14149);
_iq24 minus_one = _IQ24(-1.0);
_iq24 pi_by_2 = _IQ24(1.570796);

_iq24 ang = 0;             /**<< Calculated Arc Tangent of delta_YC and delta_XC */
_iq24 previous_ang;        /**<< Calculated Angle in previous cycle to determine Step change */
_iq24 ang_step;            /** << Resultant Angular Step Change */
_iq24 acc_ang;             /** << Accumulation of calculated angle to determine rotation */
_iq24 rotation;            /** << Variable to check if the angle of pi is crossed */


uint16_t cd1_value_q16 = 0;
uint16_t cd2_value_q16 = 0;
uint16_t count;

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
    _BIS_SR(GIE);
    gpio_init();                /**<< Initialize the GPIOs for primary/Tertiary/Secondary or as IOs */
    spi_init();                 /**<< Initialize the SPI peripheral */
    timerA_init();
    //TODO : an GPIO has to read of HIGH or LOW , if HIGH normal_operation flag is to
    //       be set to true else false
    get_top_variables(&system_settings);

#if EN_CALIBRATE == 1
    au20xx_calibrate(&system_settings);
#endif
#if FPGA_CONNECT == 0
    configure_au20xx(&system_settings);
#endif

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
    cd1_corr_slope = _IQ24(system_settings.cd1_corr_slope);
    cd2_corr_slope = _IQ24(system_settings.cd2_corr_slope);
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
   currTempValue = 30;
#elif FPGA_CONNECT == 0
   while(!read_temp_sensor( &currTempValue )) {} /** << After ADC Initialization the First Temperature
                                                  Value is read*/
#endif
   for(;;){


       if(true == normal_operation_mode)
       {
          if ( true == sensENFlag)
          {
              sensENFlag = false;
              valid_data = 0;
              sensEn_delay_us();
              while(0 == valid_data)
               au20xx_read_reg( SNS_VALID_REG, (void*)&valid_data);
               //delay_us(2);

             if(valid_data)
              {
                 valid_data =0;
#if 0
                 au20xx_read_reg( SNS1_OUT_Q16_LSB_REG, &offset_reg_values[0]);
                 au20xx_read_reg( SNS1_OUT_Q16_MSB_REG, &offset_reg_values[1]);
                 au20xx_read_reg( SNS2_OUT_Q16_LSB_REG, &offset_reg_values[2]);
                 au20xx_read_reg( SNS2_OUT_Q16_MSB_REG, &offset_reg_values[3]);

                                 /** Temperature Compensate the Values */
                 cd1_value_q16 = offset_reg_values[1] << 8 | offset_reg_values[0];
                 cd2_value_q16 = offset_reg_values[3] << 8 | offset_reg_values[2];

                 cd1_value = (cd1_value_q16 );  //CD1_OFFSET need to be subtracted
                 cd2_value = (cd2_value_q16 );  //CD2_OFFSET need to be subtracted
                 cd1_value = cd1_value << 24;
                 cd2_value = cd2_value << 24;
#endif
#if 0
                 if ( i <= 150)
                 {
                     cd1_values_array[i] = cd1_value;
                     cd2_values_array[i] = cd2_value;
                     sensEn_delay_us();
                     i++;
                     if( i >= 150)
                         i =0;
                 }

              }
          }
#endif

#if FPGA_CONNECT ==1
                 au20xx_read_reg( SNS1_OUT_Q16_LSB_REG, &offset_reg_values[0]);
                 au20xx_read_reg( SNS1_OUT_Q16_MSB_REG, &offset_reg_values[1]);
                 au20xx_read_reg( SNS2_OUT_Q16_LSB_REG, &offset_reg_values[2]);
                 au20xx_read_reg( SNS2_OUT_Q16_MSB_REG, &offset_reg_values[3]);

                /** Temperature Compensate the Values */
                 cd1_value_q16 = offset_reg_values[1] << 8 | offset_reg_values[0];
                 cd2_value_q16 = offset_reg_values[3] << 8 | offset_reg_values[2];

                 cd1_value = _IQ24(cd1_value_q16 );  //CD1_OFFSET need to be subtracted
                 cd2_value = _IQ24(cd2_value_q16 );  //CD2_OFFSET need to be subtracted
#endif
#if CONSTANT_TEMP == 0
                 if ( true == temperatureReadFlag )
                 {
                     read_temp_sensor(&currTempValue);
                 }
#endif
#if 1

                 tempNorm = currTempValue - tempInit;
                 q24_tempNorm = _IQ24(tempNorm);
                 cd1_value_corr = _IQ24mpy(cd1_corr_slope , q24_tempNorm);
                 cd1_value_corr = (cd1_value - cd1_value_corr);
                 cd2_value_corr = _IQ24mpy(cd2_corr_slope , q24_tempNorm);
                 cd2_value_corr = (cd2_value - cd2_value_corr);
                 if (((absolute(cd1_value_corr - cd1_previous_value)) <= one))
                     cd1_value_corr = cd1_previous_value;
                 if (((absolute(cd2_value_corr - cd2_previous_value)) <= one))
                     cd2_value_corr = cd2_previous_value;

                 delta_x = cd1_value_corr - x0;
                 delta_y = cd2_value_corr - y0;
                 delta_x_abs = absolute(delta_x);
                 delta_y_abs = absolute(delta_y);
                 if((delta_x == 0) && (delta_y == 0))
                    delta_r = one;
                 else
                 delta_r = delta_x_abs + delta_y_abs;
                 delta_x = _IQ24mpy(two, delta_x);
                 delta_y = _IQ24mpy(two, delta_y);
                 delta_XC = _IQ24div(delta_x, delta_r);           // We can convert to integer based on the decimal places.
                 delta_YC = _IQ24div(delta_y, delta_r);           // We can convert to integer based on the decimal places.

                 ang = _IQ24atan2(delta_YC, delta_XC);
                 if ((ang <= 0 ) && (previous_ang > pi_by_2))
                     cd_rot_direction_x = cd_rot_direction_x + 1;
                 else
                  if((ang > pi_by_2) && (previous_ang <= 0))
                      cd_rot_direction_x = cd_rot_direction_x - 1;

               previous_ang = ang;
               x0 = (cd1_value_corr - delta_XC);
               y0 = (cd2_value_corr - delta_YC);
              delta_previous_XC = delta_XC;
              delta_previous_YC = delta_YC;
              cd1_previous_value = cd1_value_corr;
              cd2_previous_value = cd2_value_corr;

   }
          }
#endif



       }
       else
       {
           uart_init();
           timerA0_load_time(2);
          //TODO: Write for Calibration Mode of operation
           uart_byte_count = uart_read(uart_top_variable);
           if (uart_byte_count > 0)
           {
               if('#' == uart_top_variable[uart_byte_count-1])
               {
                    //if(process_data(uart_top_variable);)
                   //TODO : process data function to be written to parse the data received and store in FRAM
                   // if the data is OK it will return 'true' else 'false'
                      uart_send("OK", 2);

               }
               else
                   uart_send("ERROR", 5);
           }

           if (( true == sensENFlag) && (false == calibration_done_flag))
           {
              sensENFlag = false;
              valid_data = 0;
              sensEn_delay_us();
              while(0 == valid_data)
              au20xx_read_reg( SNS_VALID_REG, (void*)&valid_data);
              if(valid_data)
              {
                valid_data =0;
                au20xx_read_reg( SNS1_OUT_Q16_LSB_REG, &offset_reg_values[0]);
                au20xx_read_reg( SNS1_OUT_Q16_MSB_REG, &offset_reg_values[1]);
                au20xx_read_reg( SNS2_OUT_Q16_LSB_REG, &offset_reg_values[2]);
                au20xx_read_reg( SNS2_OUT_Q16_MSB_REG, &offset_reg_values[3]);

                        /** Temperature Compensate the Values */
                cd1_value_q16 = offset_reg_values[1] << 8 | offset_reg_values[0];
                cd2_value_q16 = offset_reg_values[3] << 8 | offset_reg_values[2];

                if (avg_filter_sample_count < 4)
                {
                 sum_cd1_value += cd1_value_q16;
                 sum_cd2_value += cd2_value_q16;
                 avg_filter_sample_count ++ ;
                 if(avg_filter_sample_count > 3)
                 {
                     avg_filter_sample_count = 0;
                     avg_cd1_value += sum_cd1_value>>2;
                     avg_cd2_value += sum_cd2_value>>2;
                     if (avg_cd1_value > max_cd1_value)
                     {
                        max_cd1_value = avg_cd1_value;
                        max_sample_count_1 = 0;
                     }
                     else
                     {
                        max_sample_count_1++;
                        if(max_sample_count_1 >4 )
                         {
                           cd1_offset_value += max_cd1_value;
                           total_cd1_sample_count++;
                           max_sample_count_1 = 0;
                          }
                      }
                      if(avg_cd1_value < min_cd1_value )
                      {
                        min_cd1_value = avg_cd1_value;
                        min_sample_count_1 = 0;
                       }
                       else
                       {
                          min_sample_count_1++;
                          if(min_sample_count_1 > 4)
                           {
                             cd1_offset_value += min_cd1_value;
                             total_cd1_sample_count++;
                             min_sample_count_1 = 0;
                            }
                        }
                       if(avg_cd2_value > max_cd2_value )
                        {
                           max_cd2_value = avg_cd2_value;
                           max_sample_count_2 = 0;
                         }
                        else
                        {
                          max_sample_count_2++;
                          if( max_sample_count_2 > 4)
                           {
                              cd1_offset_value += max_cd2_value;
                              total_cd2_sample_count++;
                              max_sample_count_2 = 0;
                            }
                         }
                        if(avg_cd2_value < min_cd2_value )
                         {
                            min_cd2_value = avg_cd2_value;
                            min_sample_count_2 = 0;
                          }
                          else
                          {
                             min_sample_count_2++;
                             if(min_sample_count_2 > 4)
                             {
                               cd2_offset_value += min_cd2_value;
                               total_cd2_sample_count++;
                               min_sample_count_2 = 0;
                              }
                           }
                      }
                   } /* if (avg_filter_sample_count < 4) */

              }  /* If (valid_data) */
              if( (total_cd1_sample_count & total_cd2_sample_count) == 16)
              {
                   if (((max_cd1_value - min_cd1_value) < 15 ) || (( max_cd2_value - min_cd2_value ) < 15))
                   {
                        uart_send("CALIB NOT OK", 12);
                        total_cd1_sample_count = 0;
                        total_cd2_sample_count = 0;
                        sum_cd1_value = 0;
                        sum_cd2_value = 0;
                        avg_cd1_value = 0;
                        avg_cd2_value = 0;
                        cd1_offset_value = 0;
                        cd2_offset_value = 0;
                        max_cd1_value = 0;
                        min_cd1_value = 0;
                        max_cd2_value = 0;
                        min_cd2_value = 0;
                        calibration_done_flag = false;
                   }
                   else
                   {
                        uart_send("CALIB DONE", 10);
                        cd1_offset_value = cd1_offset_value >> 4;
                        cd2_offset_value = cd2_offset_value >> 4;
                        system_settings.sns1_off0 = (uint8_t)cd1_offset_value;
                        system_settings.sns1_off1 = (uint8_t)(cd1_offset_value >> 8);
                        system_settings.sns2_off0 = (uint8_t)cd2_offset_value;
                        system_settings.sns2_off1 = (uint8_t)(cd2_offset_value >> 8);
                        fram_write(&system_settings.sns1_off0, 1, SNS1_OFF0);
                        fram_write(&system_settings.sns1_off1, 1, SNS1_OFF1);
                        fram_write(&system_settings.sns2_off0, 1, SNS2_OFF0);
                        fram_write(&system_settings.sns2_off1, 1, SNS2_OFF1);
                        calibration_done_flag = true;
                   }

              }
           }
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


}


/*************** END OF FUNCTIONS ***************************************************************************/

