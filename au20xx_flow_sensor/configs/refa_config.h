/****************************************************************************
* Title                 :   refa_config
* Filename              :   refa_config.h
* Author                :   Sijeo Philip
* Origin Date           :   25/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to configure the Reference module of MSP430
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
*  24/03/20   1.0.0    Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file refa_config.h
 *  @brief This module is to configure the REference module of the MSP430
 *  device
 *
 *  This is the header file for the definition of configuration parameters
 *  for the MSP430 device
 */
#ifndef REFA_CONFIG_H_
#define REFA_CONFIG_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "msp430.h"
#include <stdint.h>


/******************************************************************************
* Typedefs
*******************************************************************************/


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
 * @brief Checks if the Buffered Bandgap voltage is ready to be used
 *
 */
#define IS_REFBG_READY()       ((REFCTL0 & 0x2000)?1:0)

 /**
  * @brief Checks if Variable Reference Voltage is Ready
  */
#define  IS_REFGEN_READY()      ((REFCTL0 & 0x1000)?1:0)


 /**
  * @brief Read BandGap mode if 1 its Sampled mode and Static
  * mode otherwise
  */
#define READ_BANDGAP_MODE()     ((REFCTL0 & 0x0800)?1:0)

/**
 * @brief Check if reference Generator is busy
 */
#define IS_REFGEN_BUSY()        ((REFCTL0 &0x0400)?1:0)

/**
 * @brief Check if Reference Bandgap is active
 */
#define IS_REF_BANDGAP_ACTIVE() ((REFCTL0 & 0x0200)?1:0)

/**
 * @brief Check if Reference Generator is Active
 *
 */
#define IS_REF_GEN_ACTIVE()     ((REFCTL0 & 0x0100)?1:0)


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/

//TODO: UPDATE COMMENT BELOW
#endif /*REFA_CONFIG_H_*/

/*** End of File **************************************************************/
