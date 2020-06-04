/*******************************************************************************
* Title                 :   au20xx_api
* Filename              :   au20xx_api.c
* Author                :   Sijeo Philip
* Origin Date           :   20/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to AU20xx Chip Operations
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
/** @file au20xx_api.c
 *  @brief This is the source file for au20xx chip operations
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>             /* For portable types */

//TODO: UPDATE MY INCLUDE
#include "au20xx_api.h"             /* For au20xx api */
#include "hal_spi_ucsi.h"           /* For SPI types */
#include "hal_fram.h"               /* For FRAM types */
#include "common.h"
#include "hal_adc.h"                /* For Temperature Reading */

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define READ_AU20xx         0x00
#define WRITE_AU20xx        0x40
#define CMD_EXEC_AU20xx     0x80
#define RESET_AU20xx        0xC0

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static uint8_t volatile valid_data = 0;
static int8_t volatile tempNorm = 0;

uint16_t cd1_value_q16 = 0;
uint16_t cd2_value_q16 = 0;

uint8_t offset_reg_values[5];
int8_t sum_of_tempNorm[10];

uint8_t temp_count;        /** <<For Loop counter for normal temperature average filtering */


_iq24 q24_tempNorm;
_iq24 cd1_value_corr=0;
_iq24 cd2_value_corr=0;
static _iq24 volatile delta_x = 0;
static _iq24 volatile delta_y = 0;
_iq24 volatile delta_x_abs = 0;
_iq24 volatile delta_y_abs = 0;
static _iq24 volatile delta_r = 0;
_iq24 volatile delta_XC;
_iq24 volatile delta_YC;

_iq24 ang = 0;             /**<< Calculated Arc Tangent of delta_YC and delta_XC */

_iq24 ang_step;            /** << Resultant Angular Step Change */
_iq24 acc_ang;             /** << Accumulation of calculated angle to determine rotation */
_iq24 rotation;            /** << Variable to check if the angle of pi is crossed */

/** Constants   */
_iq24 one = _IQ24(1);
_iq24 two = _IQ24(2);
_iq24 pi = _IQ24(3.14149);
_iq24 two_pi = _IQ24(6.2832);
_iq24 neg_pi = _IQ24(-3.14149);
_iq24 minus_one = _IQ24(-1.0);
_iq24 pi_by_2 = _IQ24(2);

extern bool readTemperatureFlag;

/***
 *
 * VARIABLES used for Calibration Routine
 *
 */

uint16_t avg_filter_sample_count = 0;
uint16_t max_sample_count_1 = 0;
uint16_t min_sample_count_1 = 0;
uint16_t max_sample_count_2 = 0;
uint16_t min_sample_count_2 = 0;

uint32_t sample_count =0;
uint32_t sum_cd1_value[5];
uint32_t sum_cd2_value[5];
uint32_t avg_cd1_value = 0;
uint32_t avg_cd2_value = 0;
uint32_t max_cd1_value = 0;
uint32_t max_cd2_value = 0;
uint32_t min_cd1_value = 0;
uint32_t min_cd2_value = 0;


static bool get_minima1_flag = false;
static bool get_minima2_flag = false;

int up_down_curve1;
int up_down_curve2;


uint16_t count;
uint16_t max_cd1_sample_count;
uint16_t max_cd2_sample_count;
uint16_t min_cd1_sample_count;
uint16_t min_cd2_sample_count;


/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/

/******************************************************************************
* Function : au20xx_chip_reset()
*//**
* \b Description:
*
* This function sends reset command to au20xx chip for reset of the chip the chip
* registers are reset to the default value on reset of the chip
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_chip_reset();
* @endcode
*
* @see Dio_Init
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
void au20xx_chip_reset( void )
{
 static uint8_t reset_command = 0;
 reset_command = RESET_AU20xx;
 spi_write(&reset_command, 1);
 delay_us(3);
 DISABLE_SPI_CS;
}


/******************************************************************************
* Function : au20xx_send_command()
*//**
* \b Description:
*
* This function is used to send sub command to the au20xx chip
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[in]    sub_command     byte of subcommand to be send to the au20xx chip
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_send_command(0x20);
* @endcode
*
* @see Dio_Init
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
void au20xx_send_command(uint8_t sub_command)
{
    static uint8_t command = 0;
    command = CMD_EXEC_AU20xx | sub_command ;
    spi_write(&command, 1);
    delay_us(3);
    DISABLE_SPI_CS;
}

/******************************************************************************
* Function : au20xx_write_reg();
*//**
* \b Description:
*
* This function is used to write to the au20xx chip as per register address given
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[in]    reg_addr     Address of the register to which the data has to be written
* @param[in]    data         Data to be written to the register
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_write_reg(0x01, 0x05);
* @endcode
*
* @see Dio_Init
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
void au20xx_write_reg(uint8_t reg_addr, uint8_t data)
{
  static uint16_t register_data = 0;
  register_data = data<<8|(WRITE_AU20xx)| (reg_addr);
  ENABLE_SPI_CS;                                 /* Pull the Chip Select line LO to select the Chip */
  delay_us(5);
  spi_write(&register_data, 2);
  delay_us(5);
  DISABLE_SPI_CS;
 }

/******************************************************************************
* Function : au20xx_read_reg();
*//**
* \b Description:
*
* This function is used to read from the au20xx chip as per register address given
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[in]    reg_addr     Address of the register to which the data has to be written
* @param[out]    data         Data to be read from the register
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_read_reg(0x01, 0x05);
* @endcode
*
* @see spi_read()
* @see spi_write()
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
void au20xx_read_reg( uint8_t reg_addr, void * const data)
{
  static uint16_t register_data = 0;
  register_data = READ_AU20xx | reg_addr;
  spi_write(&register_data, 1);
  delay_us(3);
  spi_read ( (void*)data, 1 );
  delay_us(3);
  DISABLE_SPI_CS;

}



/******************************************************************************
* Function : process_sensor_data()
*//**
* \b Description:
*
* This function is used to read from the au20xx chip and process the data for
* counting the number of rotations
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[in]     top_variable_t*  This variable is used to input the system set values for calculations
* @param[out]    au20xx_var_t*    This variable is used to store or output the calculated rotaitons
*
* @return        true if valid data is read high from the chip
*
* \b Example Example:
* @code
*  SYSTEM_VARIABLE_DEF(variable);
*  top_variable_t system_settings
*  process_sensor_data(&variable, &system_settings);
* @endcode
*
* @see spi_read()
* @see spi_write()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 01/06/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*
*******************************************************************************/
bool process_sensor_data(au20xx_var_t* au20xx_variables, top_variables_t* top_variables)
{
 bool retVal = false;

               valid_data = 0;
               sensEn_delay_us();                /** << Sens Enable Pulse duration in the function */
               au20xx_read_reg( SNS_VALID_REG, (void*)&valid_data);
                //delay_us(2);

              if(valid_data)
               {
                  valid_data =0;
                  retVal =true;
 #if 1
                  au20xx_read_reg( SNS1_OUT_Q16_LSB_REG, &offset_reg_values[0]);
                  au20xx_read_reg( SNS1_OUT_Q16_MSB_REG, &offset_reg_values[1]);
                  au20xx_read_reg( SNS2_OUT_Q16_LSB_REG, &offset_reg_values[2]);
                  au20xx_read_reg( SNS2_OUT_Q16_MSB_REG, &offset_reg_values[3]);

                                  /** Temperature Compensate the Values */
                  cd1_value_q16 = offset_reg_values[1] << 8 | offset_reg_values[0];
                  cd2_value_q16 = offset_reg_values[3] << 8 | offset_reg_values[2];

                  au20xx_variables->cd1_value = _IQ24(cd1_value_q16 );  //CD1_OFFSET need to be subtracted
                  au20xx_variables->cd2_value = _IQ24(cd2_value_q16 );  //CD2_OFFSET need to be subtracted

 #endif


 #if CONSTANT_TEMP == 0
                  if ( true == readTemperatureFlag )
                  {
                      while(!read_temp_sensor(&au20xx_variables->currTempValue)){}
                      readTemperatureFlag = false;  /** << This flag is set to true when ADC conversion is initiated
                                                              on timer expire */
                      sum_of_tempNorm[temp_count] = au20xx_variables->currTempValue;
                      temp_count++;
                      if(temp_count > 7)
                         temp_count = 0;
                      tempNorm = average_by_8(sum_of_tempNorm);
                      tempNorm = tempNorm - top_variables->tempInit;
                  }
#elif   CONSTANT_TEMP ==1
                 tempNorm = au20xx_variables->currTempValue - top_variables->tempInit;

#endif
                  q24_tempNorm = _IQ24(tempNorm);
                  cd1_value_corr = _IQ24mpy(au20xx_variables->cd1_corr_slope , q24_tempNorm);
                  cd1_value_corr = (au20xx_variables->cd1_value - cd1_value_corr);
                  cd2_value_corr = _IQ24mpy(au20xx_variables->cd2_corr_slope , q24_tempNorm);
                  cd2_value_corr = (au20xx_variables->cd2_value - cd2_value_corr);
                  if (((absolute(cd1_value_corr - au20xx_variables->cd1_previous_value)) <= one))
                      cd1_value_corr = au20xx_variables->cd1_previous_value;
                  if (((absolute(cd2_value_corr - au20xx_variables->cd2_previous_value)) <= one))
                      cd2_value_corr = au20xx_variables->cd2_previous_value;

                  delta_x = cd1_value_corr - au20xx_variables->x0;
                  delta_y = cd2_value_corr - au20xx_variables->y0;
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
                  if ((ang <= 0 ) && (au20xx_variables->previous_ang > two))
                      au20xx_variables->cd_rot_direction_x = au20xx_variables->cd_rot_direction_x + 1;
                  else
                   if((ang > two) && (au20xx_variables->previous_ang <= 0))
                       au20xx_variables->cd_rot_direction_x = au20xx_variables->cd_rot_direction_x - 1;

                au20xx_variables->previous_ang = ang;
                au20xx_variables->x0 = (cd1_value_corr - delta_XC);
                au20xx_variables->y0 = (cd2_value_corr - delta_YC);
                au20xx_variables->delta_previous_XC = delta_XC;
                au20xx_variables->delta_previous_YC = delta_YC;
                au20xx_variables->cd1_previous_value = cd1_value_corr;
                au20xx_variables->cd2_previous_value = cd2_value_corr;


        }
              else
                  retVal = false;
           return retVal;
}

/******************************************************************************
* Function : calibrate_au20xx()
*//**
* \b Description:
*
* This function is used to calibrate the au20xx chip offset values and written to the NonVolatile
* memory for further use in Normal operations
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[out]     top_variable_t*  This variable is used to store the offset values post calibration
*
*
* @return        true if valid data is read high from the chip
*
* \b Example Example:
* @code
*  top_variables_t * system_settings;
*  calibrate_au20xx( &system_settings);
* @endcode
*
* @see spi_read()
* @see spi_write()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 01/06/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*
*******************************************************************************/
bool calibrate_au20xx(au20xx_calib_var_t* calibVariables)
{
    bool retVal = false;

    valid_data = 0;
    sensEn_delay_us();
    au20xx_read_reg( SNS_VALID_REG, (void*)&valid_data);
    if(valid_data)
     {
       retVal = true;
       sample_count++;
       valid_data =0;
       au20xx_read_reg( SNS1_OUT_Q16_LSB_REG, &offset_reg_values[0]);
       au20xx_read_reg( SNS1_OUT_Q16_MSB_REG, &offset_reg_values[1]);
       au20xx_read_reg( SNS2_OUT_Q16_LSB_REG, &offset_reg_values[2]);
       au20xx_read_reg( SNS2_OUT_Q16_MSB_REG, &offset_reg_values[3]);

                            /** Temperature Compensate the Values */
       cd1_value_q16 = offset_reg_values[1] << 8 | offset_reg_values[0];
       cd2_value_q16 = offset_reg_values[3] << 8 | offset_reg_values[2];

       sum_cd1_value[avg_filter_sample_count] = cd1_value_q16;
       sum_cd2_value[avg_filter_sample_count] = cd2_value_q16;
       avg_cd1_value = average_by_4(sum_cd1_value, &up_down_curve1);
       avg_cd2_value = average_by_4(sum_cd2_value , &up_down_curve2);
       avg_filter_sample_count ++ ;


        if(calibVariables->total_cd1_sample_count < CALIB_CD1_MIN_MAX_SAMPLES)
          {
            if ( (avg_cd1_value > max_cd1_value) && ( get_minima1_flag == false) )
              {
                 max_cd1_value = avg_cd1_value;
                 max_sample_count_1 = 0;
                  //min_sample_count_1 = 0;
                }
             else if ( (get_minima1_flag == false))  //(up_down_curve1 <= 0) &&
              {
                 max_sample_count_1++;
                 if(max_sample_count_1 > 16 )   /**<< Need to set MACRO for calibration */
                  {
                     calibVariables->cd1_offset_value += max_cd1_value;
                     calibVariables->avg_max_cd1_value += max_cd1_value;
                     min_cd1_value = max_cd1_value;
                     calibVariables->total_cd1_sample_count++;
                     max_sample_count_1 = 0;
                     max_cd1_sample_count++;
                     get_minima1_flag = true;

                   }
               }
              if (( avg_cd1_value < min_cd1_value )&& (get_minima1_flag == true))  // && (up_down_curve1 > 0)
               {
                  min_cd1_value = avg_cd1_value;
                  min_sample_count_1 = 0;
                //max_sample_count_1 = 0;
                }
                else if ((get_minima1_flag == true)) //( up_down_curve1 > 0) &&
                 {
                   min_sample_count_1++;
                   if(min_sample_count_1 > 16)
                     {
                        calibVariables->cd1_offset_value += min_cd1_value;
                        calibVariables->avg_min_cd1_value += min_cd1_value;
                         max_cd1_value = min_cd1_value;
                         calibVariables->total_cd1_sample_count++;
                         min_sample_count_1 = 0;
                         min_cd1_sample_count++;
                         get_minima1_flag = false;
                      }
                  }
               }
               if(calibVariables->total_cd2_sample_count < CALIB_CD2_MIN_MAX_SAMPLES)
                {
                  if ((avg_cd2_value > max_cd2_value )&& (get_minima2_flag == false)) // && (up_down_curve2 <=0)
                   {
                     max_cd2_value = avg_cd2_value;
                     max_sample_count_2 = 0;
                     //min_sample_count_2 = 0;
                    }
                   else if ( (get_minima2_flag == false))  //(up_down_curve2 <= 0) &&
                    {
                      max_sample_count_2++;
                      if( max_sample_count_2 > 16)
                      {
                         calibVariables->cd2_offset_value += max_cd2_value;
                         calibVariables->avg_max_cd2_value += max_cd2_value;
                         min_cd2_value = max_cd2_value;
                         calibVariables->total_cd2_sample_count++;
                         max_sample_count_2 = 0;
                         max_cd2_sample_count++;
                         get_minima2_flag = true;
                       }
                     }
                     if ((avg_cd2_value < min_cd2_value ) && (get_minima2_flag == true))  //&& (up_down_curve2 > 0)
                       {
                         min_cd2_value = avg_cd2_value;
                         min_sample_count_2 = 0;
                         //max_sample_count_2 = 0;
                        }
                     else if ( (get_minima2_flag == true)) //(up_down_curve2 > 0) &&
                       {
                          min_sample_count_2++;
                          if(min_sample_count_2 > 16)
                           {
                             calibVariables->cd2_offset_value += min_cd2_value;
                             calibVariables->avg_min_cd2_value += min_cd2_value;
                             max_cd2_value = min_cd2_value;
                             calibVariables->total_cd2_sample_count++;
                              min_sample_count_2 = 0;
                             min_cd2_sample_count++;
                             get_minima2_flag = false;
                             }
                         }
                     }
                     if(avg_filter_sample_count > 3)
                     {
                         avg_filter_sample_count = 0;
                     }

                  }  /* If (valid_data) */
    else
        return false;

    return retVal;
}

/*************** END OF FUNCTIONS ***************************************************************************/
