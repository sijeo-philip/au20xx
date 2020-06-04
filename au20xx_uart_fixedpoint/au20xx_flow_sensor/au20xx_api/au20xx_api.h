/****************************************************************************
* Title                 :   au20xx_api
* Filename              :   au20xx_api.h
* Author                :   Sijeo Philip
* Origin Date           :   20/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to read and write to au20xx chip
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
*****************************************************************************/
/*************** INTERFACE CHANGE LIST **************************************
*
*    Date    Version   Author         Description
*  20/03/20   1.0.0    Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file au20xx_api.h
 *  @brief This module is used to interface with the au20xx chip
 *
 *  This is the header file for the definition of functions used to read and write
 *  au20xx chip
 */
//TODO: UPDATE MACRO BELOW
#ifndef AU20xx_API_H_
#define AU20xx_API_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "common.h"
#include <stdint.h>
#include "hal_gpio.h"
#include <stdbool.h>
#include "IQmathLib.h"
#include "common.h"
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define DEV_ID_REG              0x00
#define VER_ID_REG              0x01
#define INTF_CFG_REG            0x05
#define SNS1_OFF0_REG           0x06       /** << LSB of the offset to be added to SENSOR 1 code */
#define SNS1_OFF1_REG           0x07       /** << MSB of the offset to be added to SENSOR 1 code */
#define SNS2_OFF0_REG           0x08       /** << LSB of the offset to be added to SENSOR 2 code */
#define SNS2_OFF1_REG           0x09       /** << MSB of the offset to be added to SENSOR 2 code */
#define SNS_VALID_REG           0x0A       /** << When Set the Sensor output registers are valid */
#define SNS1_OUT_Q8_REG         0x0B       /** << Sensor 1 output read back register, Soft decision
                                           quantized to 8 bits */
#define SNS2_OUT_Q8_REG         0x0C       /** << Sensor 2 output read back register, Soft decision
                                                  quantized to 8 bits */
#define SNS1_OUT_Q16_LSB_REG    0x0D       /** << Sensor 1 output read back register, Soft decision
                                                   quantized to 16 bits */
#define SNS1_OUT_Q16_MSB_REG    0x0E       /** << Sensor 1 output read back register, Soft decision
                                                  quantized to 16 bits */
#define SNS2_OUT_Q16_LSB_REG    0x0F       /** << Sensor 2 output read back register, Soft decision
                                                  quantized to 16 bits */
#define SNS2_OUT_Q16_MSB_REG    0x10       /** << Sensor 2 output read back register, Soft decision
                                                  quantized to 16 bits */
#if FPGA_CONNECT == 1
#define TEMP_REG_ADD            0x2B
#define CD1_OFFSET              5733
#define CD2_OFFSET              5703
#endif

#define SNS_EN_HIGH             SET_GPIO_OUTPUT(P1, BIT3)
#define SNS_EN_LOW              CLEAR_GPIO_OUTPUT(P1, BIT3)



/******************************************************************************
* Configuration Constants
*******************************************************************************/


#ifndef CD1_CORR_SLOPE
#define CD1_CORR_SLOPE          2.004
#endif

#ifndef CD2_CORR_SLOPE
#define CD2_CORR_SLOPE          1.802
#endif

#ifndef CALIB_CD1_MIN_MAX_SAMPLES
#define CALIB_CD1_MIN_MAX_SAMPLES   16
#endif

#ifndef CALIB_CD2_MIN_MAX_SAMPLES
#define CALIB_CD2_MIN_MAX_SAMPLES   16
#endif


/******************************************************************************
* Typedefs
*******************************************************************************/
struct au20xx_var
{
    _iq24   cd1_corr_slope;
    _iq24   cd2_corr_slope;
    int8_t  currTempValue;
    _iq24   cd1_value;
    _iq24   cd2_value;
    _iq24   cd1_previous_value;
    _iq24   cd2_previous_value;
    _iq24   x0;
    _iq24   y0;
    _iq24   delta_previous_XC;
    _iq24   delta_previous_YC;
    _iq24   previous_ang;        /**<< Calculated Angle in previous cycle to determine Step change */
    long    cd_rot_direction_x;
    long    cd_rot_direction_x_tens;
};

typedef struct au20xx_var au20xx_var_t;

struct au20xx_calib_var
{
    uint16_t total_cd1_sample_count;
    uint16_t total_cd2_sample_count;
    uint32_t avg_max_cd1_value;
    uint32_t avg_max_cd2_value;
    uint32_t avg_min_cd1_value;
    uint32_t avg_min_cd2_value;
    uint32_t cd1_offset_value;
    uint32_t cd2_offset_value;
};

typedef struct au20xx_calib_var au20xx_calib_var_t;
/******************************************************************************
* Macros
*******************************************************************************/
#define SYSTEM_VAR_DEF(variable) \
    au20xx_var_t variable

#define CALIB_VAR_DEF(variable) \
    au20xx_calib_var_t variable


/******************************************************************************
* Variables
******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void au20xx_chip_reset(void);
void au20xx_send_command(uint8_t sub_command);
void au20xx_write_reg(uint8_t reg_addr, uint8_t data);
void au20xx_read_reg( uint8_t reg_addr, void * const data);
bool process_sensor_data(au20xx_var_t* , top_variables_t* );
bool calibrate_au20xx(au20xx_calib_var_t*);

#ifdef __cplusplus
} // extern "C"
#endif


#endif /*AU20xx_API_H_*/

/*** End of File **************************************************************/
