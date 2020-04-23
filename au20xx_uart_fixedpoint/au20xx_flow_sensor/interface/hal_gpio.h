/****************************************************************************
* Title                 :   hal_gpio
* Filename              :   hal_gpio.h
* Author                :   Sijeo Philip
* Origin Date           :   16/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to configure and set up the GPIO of device
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
*  16/03/20   1.0.0    Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file hal_gpio.h
 *  @brief This module is used to configure the GPIO as per the application
 *
 *  This is the header file for the definition of macros and functions to set
 *  the GPIO pin functions
 */
//TODO: UPDATE MACRO BELOW
#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "msp430.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
/**
 * This constant is used to enable ghe inputs and outputs GPIO
 */
#define ENABLE_PINS 			0xFFFE

/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/

/*********************************
 *  This is used to select the alternative primary functions of GPIO
 *  @param[in] port should be P1, P2, P3, etc...
 *  @param[in] pin should be values such as BIT1, BIT2, etc..
 *  \b  Example Example:
 *  @code
 *  SET_GPIO_PRI_FUN(P1, BIT4)    // to select primary function in pin P1.4
 *  @endcode
 */

#define SET_GPIO_PRI_FUN(port, pin) \
	do{ 						    \
		port##SEL0 = port##SEL0 | pin;\
		port##SEL1 = port##SEL1 & (~pin);\
	}while(0)


/*********************************
 *  This is used to select the alternative secondary functions of GPIO
 *  @param[in] port should be P1, P2, P3, etc...
 *  @param[in] pin should be values such as BIT1, BIT2, etc..
 *  \b  Example Example:
 *  @code
 *  SET_GPIO_SEC_FUN(P1, BIT4)    // to select secondary function in pin P1.4
 *  @endcode
 */

#define SET_GPIO_SEC_FUN(port, pin) \
	do{								\
		port##SEL1 = port##SEL1 | pin;\
		port##SEL0 = port##SEL0 & (~pin);\
	}while)(0)


/*********************************
 *  This is used to select the alternative tertary functions of GPIO
 *  @param[in] port should be P1, P2, P3, etc...
 *  @param[in] pin should be values such as BIT1, BIT2, etc..
 *  \b  Example Example:
 *  @code
 *  SET_GPIO_TER_FUN(P1, BIT4)    // to select tertary function in pin P1.4
 *  @endcode
 */

#define SET_GPIO_TER_FUN(port, pin) \
	do{								\
		port##SEL1 = port##SEL1 | pin;\
		port##SEL0 = port##SEL0 | pin;\
	}while(0)


/*********************************
 *  This is used to set the GPIO to HIGH if the port is set as output
 *  @param[in] port should be P1, P2, P3, etc...
 *  @param[in] pin should be values such as BIT1, BIT2, etc..
 *  \b  Example Example:
 *  @code
 *  SET_GPIO_OUTPUT(P1, BIT4)    // to set the output
 *  @endcode
 */

#define SET_GPIO_OUTPUT(port, pin) \
	do{								\
		port##OUT = port##OUT | pin;  \
	}while(0)

/*********************************
 *  This is used to set the GPIO to LOW if the port is set as output
 *  @param[in] port should be P1, P2, P3, etc...
 *  @param[in] pin should be values such as BIT1, BIT2, etc..
 *  \b  Example Example:
 *  @code
 *  CLEAR_GPIO_OUTPUT(P1, BIT4)    // to set the output
 *  @endcode
 */

#define CLEAR_GPIO_OUTPUT(port, pin) \
	do {  \
		 port##OUT = port##OUT & (~pin);\
	}while(0)


/*********************************
 *  This is used to toggle the GPIOif the port is set as output
 *  @param[in] port should be P1, P2, P3, etc...
 *  @param[in] pin should be values such as BIT1, BIT2, etc..
 *  \b  Example Example:
 *  @code
 *  CLEAR_GPIO_OUTPUT(P1, BIT4)    // to set the output
 *  @endcode
 */
#define TOGGLE_GPIO_OUTPUT(port, pin) \
    do { \
        port##OUT = port##OUT ^ pin; \
    }while(0)


/*********************************
 *  This is used to read the GPIO which is set as input
 *  @param[in] port should be P1, P2, P3, etc...
 *  @param[in] pin should be values such as BIT1, BIT2, etc..
 *  \b  Example Example:
 *  @code
 *  uint8_t inputval;
 *  inputval = READ_GPIO_INPUT(P1, BIT4)    // read the input
 *  @endcode
 */



#define READ_GPIO_INPUT(port, pin)  ((port##IN&pin)?1:0)

/************************************
 * This is used to set the direction of the GPIO to output
 *  @param[in] port should be P1, P2, P3, etc...
 *  @param[in] pin should be values such as BIT1, BIT2, etc..
 *  \b  Example Example:
 *  @code
 *  SET_GPIO_DIR_OUT(port, pin)
 *  @endcode
 *
 */
#define SET_GPIO_DIR_OUT(port, pin) \
	do{		\
		port##DIR = port##DIR | pin; \
	}while(0)

/******
 * @brief This Enable the Pull up resistor on PIN
 * ENABLE_PULL_UP(port, pin)
 */
#define ENABLE_PULL_UP(port, pin) \
    do{ \
        port##OUT = port##OUT | pin;\
        port##REN = port##REN | pin; \
    }while(0)


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

void gpio_init(void);

#ifdef __cplusplus
} // extern "C"
#endif

//TODO: UPDATE COMMENT BELOW
#endif /*HAL_GPIO_H_*/

/*** End of File **************************************************************/
