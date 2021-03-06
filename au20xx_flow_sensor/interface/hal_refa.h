/****************************************************************************
* Title                 :   hal_refa
* Filename              :   hal_refa.h
* Author                :   Sijeo Philip
* Origin Date           :   25/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to intialize the REFA module
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
/** @file hal_refa.h
 *  @brief This module is used to initalize the REFA module
 *
 *  This is the header file for the definition of function to intialize the
 *  REFA module of MCU, The inializing routine should be modified as per the
 *  application
 */
//TODO: UPDATE MACRO BELOW
#ifndef HAL_REFA_H_
#define HAL_REFA_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "refa_config.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/



/******************************************************************************
* Typedefs
*******************************************************************************/
typedef enum {REF_1_2V=0, REF_2_0V=0x0010, REF_2_5V=0x0020 }ref_voltage_t;
/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void refa_init(void);

#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*HAL_REFA_H_*/

/*** End of File **************************************************************/
