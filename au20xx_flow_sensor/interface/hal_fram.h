/****************************************************************************
* Title                 :   hal_fram
* Filename              :   hal_fram.h
* Author                :   Sijeo Philip
* Origin Date           :   29/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to read and write data to FRAM
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
*  29/03/20   1.0.0    Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file hal_fram.h
 *  @brief This module is used to read and write to FRAM module
 *
 *  This is the header file for the definition to read and write to FRAM
 *  so that it can be retrieved on Power Cycle.
 */
//TODO: UPDATE MACRO BELOW
#ifndef HAL_FRAM_H_
#define HAL_FRAM_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "msp430.h"
#include <stdint.h>
#include "fram_config.h"
#include <stdbool.h>


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define FRAM_START_ADDRESS    (uint8_t*)0x001800
#define FRAM_END_ADDRESS      (uint8_t*)0x0019FF

#define SAMPLES_PER_TEMP_READ_ADDRESS           (uint8_t*)0x001800    /**<< Address of FRAM where
                                                                    samplePerTempReading is stored */
#define AU20xx_READ_TIME_ADD                    (uin8_t*)0x001804    /**<< Address of FRAM where the
                                                                       au20xx read interval is stored */
#define LAST_ROT_COUNT_ADD                      (uint8_t*)0x001806   /**<< Last Rotation count before
                                                                       Power down. */
#define SAMPLE_TIME_ADD                         (uint8_t*)0x001808   /**<< Sampling Time Setting */

#define CD1_CORR_SLOPE_ADD                      (uint8_t*)0x00180A   /**<< CD1_correction Slope (float) */

#define CD2_CORR_SLOPE_ADD                      (uint8_t*)0x00180E   /** << CD2_Correction Slope (float) */
/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/



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

void fram_init ( void );
bool fram_write ( void * data, uint16_t bytes , uint8_t * p_fram);
bool fram_read ( void * data, uint16_t bytes, uint8_t * p_fram);


#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*File_H_*/

/*** End of File **************************************************************/
