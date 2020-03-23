/****************************************************************************
* Title                 :   hal_timerA
* Filename              :   hal_timerA.h
* Author                :   Sijeo Philip
* Origin Date           :   23/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to intialize, start and stop the timer
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
*  23/03/20   1.0.0    Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file hal_timerA.h
 *  @brief This module is to start, stop and reload the timer
 *
 *  This is the header file for the definition of timer_init, start and stop
 *  timer
 */
//TODO: UPDATE MACRO BELOW
#ifndef HAL_TIMERA_H_
#define HAL_TIMERA_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "timerA_config.h"
#include <stdint.h>



/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/
/**
 * @brief The MACRO should be used after setting the
 * input clock divider for the Timer.
 */

#define TIMERA_CLR(instance) \
    do{         \
        TA##instance##CTL |= 0x0004; \
    }while(0)

/**
 * @brief Timer used for setting the Measurement Interval
 */
#define AU20xx_TIMER_CTRL_REG(instance)  TA##instance##CTL

/**
 * @brief Timer Capture/Compare Control Register 0
 */
#define AU20xx_CAP_COMP_CTRL_REG(instance) TA##instance##CCTL0

/**
 * @brief Timer Capture/Compare Register 0
 *
 */
#define AU20xx_CAP_COMP_REG(instance)    TA##instance##CCR0

/**
 * @brief Timer Expansion 0 Register
 *
 */
#define AU20xx_TIMER_EXP_REG(instance)    TA##instance##EX0
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

void timerA_init(void);
void timerA_load_time(uint16_t);

#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*File_H_*/

/*** End of File **************************************************************/
