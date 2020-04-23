/****************************************************************************
* Title                 :   hal_adc
* Filename              :   hal_adc.h
* Author                :   Sijeo Philip
* Origin Date           :   26/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to initialize the ADC and APIs for ADC
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
*  26/03/20   1.0.0    Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file hal_adc.h
 *  @brief This module is used to initialize and use the ADC module
 *
 *  This is the header file for the definition to initialize ADC and
 *  read from the ADC after conversion
 */
//TODO: UPDATE MACRO BELOW
#ifndef HAL_ADC_H_
#define HAL_ADC_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "msp430.h"
#include <stdint.h>
#include "adc_config.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

// See device datasheet for TLV table memory mapping
#define CALADC_12V_30C  *((unsigned int *)0x1A1A)       // Temperature Sensor Calibration-30 C
#define CALADC_12V_85C  *((unsigned int *)0x1A1C)       // Temperature Sensor Calibration-85 C

/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
 * Inline Functions
 ******************************************************************************/
/**
 * This function to be called while conversion should be start i.e when
 * the Conversions are Software Triggered
 * ADC12SC and ADC12ENC can be set together with one instruction the
 * Start conversion bit is reset automatically
 */
inline void __start_adc_conv( void )
{
    ADC12CTL0 |= 0x0003;
}

/******************************************************************************
* Typedefs
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void adc_init( void );
bool read_temp_sensor( void * degrees );

#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*HAL_ADC_H_*/

/*** End of File **************************************************************/
