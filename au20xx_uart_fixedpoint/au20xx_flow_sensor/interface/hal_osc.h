/****************************************************************************
* Title                 :   HAL_OSC
* Filename              :   hal_osc.h
* Author                :   Sijeo Philip
* Origin Date           :   15/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This Module is used to initialize and use the oscillator
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
*  15/03/20   1.0.0   Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file hal_osc.h
 *  @brief This module is used to configure the osc and use in project
 * 
 *  This is the header file for the definition of oscillator initilizaitions
 *  and clock source for all the peripherals used in MSP430
 */
//TODO: UPDATE MACRO BELOW
#ifndef HAL_OSC_H_
#define HAL_OSC_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "osc_config.h"

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


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void system_clock_init(void);

#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*HAL_OSC_H_*/

/*** End of File **************************************************************/
