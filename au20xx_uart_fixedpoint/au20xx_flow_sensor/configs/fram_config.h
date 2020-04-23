/****************************************************************************
* Title                 :   fram_config
* Filename              :   fram_config.h
* Author                :   Sijeo Philip
* Origin Date           :   29/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to configure the FRAM Module of MCU
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
/** @file fram_config.h
 *  @brief This module is used to configure the FRAM Module in MCU
 *
 *  This is the header file for the definition of configurations of FRAM
 *  Module of MCU
 */
//TODO: UPDATE MACRO BELOW
#ifndef FRAM_CONFIG_H_
#define FRAM_CONFIG_H_

/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/
/**
 * @brief This sets the Numbe of Wait states for the FRAM Operations
 * <0x0000> to <0x0070> Waits States to be inserted
 * 0x000 is the default value.
 */
#ifndef CONF_FRCTL0_NWAITS
#define CONF_FRCTL0_NWAITS      0x0000
#endif


/**
 * @brief This bit Enable/Disable Power Up Clear reset if FRAM uncorrectable
 * bit error detected The bits UBDRSTEN and UBDIE are mutual exclusive
 * <0x0000=>   PUC not initiated on uncorrectable bit detection flag (default)
 * <0x0080=>   PUC initiated on uncorrectable bit detection flag.
 */
#ifndef CONF_GCCTL0_UBDRSTEN
#define CONF_GCCTL0_UBDRSTEN    0x0000
#endif


/**
 * @brief Enable/Disable NMI Event if uncorrectable bit error is
 * detected
 * <0x0000=> Uncorrectable bit detection interrupt disabled (default)
 * <0x0040=> Uncorrectable bit detection Interrupt Enabled
 */
#ifndef CONF_GCCTL0_UBDIE
#define CONF_GCCTL0_UBDIE       0x0000
#endif

/**
 * @brief Enable/Disable NMI Event if correctable bit error is
 * detected
 * <0x0000=> correctable bit detection interrupt disabled (default)
 * <0x0020=> correctable bit detection Interrupt Enabled
 */
#ifndef CONF_GCCTL0_CBDIE
#define CONF_GCCTL0_CBDIE       0x0000
#endif

/**
 * @brief Enable/Disable FRAM Power Supply
 * detected
 * <0x0000=> FRAM Power supply is disabled
 * <0x0004=> FRAM Power Supply is enabled (default )
 */
#ifndef CONF_GCCTL0_FRPWR
#define CONF_GCCTL0_FRPWR       0x0004
#endif

/**
 * @brief Enable/Disable FRAM auto power up after LPM
 * detected
 * <0x0000=> FRAM Startup is delayed to the first FRAM access after LPM exit
 * <0x0002=> FRAM is powered up instantly with LPM exit (default )
 */
#ifndef CONF_GCCTL0_FRLPMPWR
#define CONF_GCCTL0_FRLPMPWR       0x0002
#endif

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


//TODO: UPDATE COMMENT BELOW
#endif /*FRAM_CONFIG_H_*/

/*** End of File **************************************************************/
