/****************************************************************************
* Title                 :   common
* Filename              :   common.h
* Author                :   Sijeo Philip
* Origin Date           :   20/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used for common functionality used accross the application
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
/** @file common.h
 *  @brief This module is for common functionalities such as delay or string
 *  manipulation routines etc..
 *
 *  This is the header file for the definition of common functions such as delays
 */
//TODO: UPDATE MACRO BELOW
#ifndef COMMON_H_
#define COMMON_H_


/******************************************************************************
* Includes
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
extern bool volatile sensENFlag;
extern bool volatile temperatureReadFlag;
/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/
#ifndef DEBUG
#define DEBUG       1
#endif


/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct
{
    uint32_t samplesPerTemp;        /**<< This variable stores the value of when to read
                                          Temperature from ADC */
    uint16_t sensEnTime;            /**<< This variable stores the duration for which the
                                          SENSEN should be asserted 0 - 4 i.e. Accuracy setting*/
    uint16_t lastRotCount;          /**<< This variable stores the last Rotation Count
                                          before Power Down */
    uint16_t sampleTime;            /**<< This variable stores the Sampling Time of the
                                          system for au20xx chip Measurement Delay */
    uint16_t sensEnTime_us;          /**<< Accuracy Setting Delay in microseconds */

    float cd1_corr_slope;           /** <<Slope for temperature correction for cd1 reading */

    float cd2_corr_slope;           /** <<Slope for temperature correction for cd2 reading */

    uint8_t sns1_off0;              /** <<LSB of Sensor 1 offset */

    uint8_t sns1_off1;              /** <<MSB of Sensor1 offset */

    uint8_t sns2_off0;              /** <<LSB of Sensor 2 offset */

    uint8_t sns2_off1;              /** <<MSB of Sensor 2 offset */
}top_variables_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void sensEn_delay_us( void );
void delay_us(uint16_t microseconds );
bool get_top_variables(top_variables_t*);
bool set_top_variables(top_variables_t*);
int8_t absolute(int8_t);
void configure_au20xx(top_variables_t * topVariables);
#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*File_H_*/

/*** End of File **************************************************************/
