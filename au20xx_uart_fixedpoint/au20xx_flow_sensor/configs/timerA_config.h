/****************************************************************************
* Title                 :   timerA_config
* Filename              :   timerA_config.h
* Author                :   Sijeo Philip
* Origin Date           :   23/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to configure Timers for the application
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
/** @file timerA_config.h
 *  @brief This module is used to configure the TIMERA 0/1/2.. instance
 *
 *  This is the header file for the configuration of TIMER A instance 0
 *  Below configurations should be repeated for further instances of TIMERA
 */
//TODO: UPDATE MACRO BELOW
#ifndef TIMERA_CONFIG_H_
#define TIMERA_CONFIG_H_

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
 * @brief This bit is used to enable and disable
 * TimerA0 Interrupt
 * <0x0000=> Disable the Timer Interrupt
 * <0x0002=> Enable the Timer Interrupt
 */

#ifndef CONF_TACTL0_TAIE
#define CONF_TACTL0_TAIE    0x0000
#endif

/**
 * @brief This bit is used to set the MODE control
 * <0x0000=> Stop Mode
 * <0x0010=> Up Mode: Timer counts up to TAxCCR0
 * <0x0020=> Continuous Mode : Timer counts up to 0xFFFF
 * <0x0030=> Up/Down Mode : Timer Counts up to TAxCCR0 then
 *           down to 0x000h
 */
#ifndef CONF_TACTL0_MC
#define CONF_TACTL0_MC      0x0010
#endif

/**
 * @brief This bits are used to select the divider for input
 * clock
 * <0x0000=>  Divide by 1
 * <0x0040=>  Divide by 2
 * <0x0080=>  Divide by 4
 * <0x00C0=>  Divide by 8
 */
#ifndef CONF_TACTL0_ID
#define CONF_TACTL0_ID  0x0080
#endif

/**
 * @brief This bits are used to Select the Clock source
 * <0x0000=>  TA0CLK
 * <0x0100=>  ACLK is selected as clock Source
 * <0x0200=>  SMCLK is selected as clock Source
 * <0x0300=>  INCLK is selected as clock source
 */
#ifndef CONF_TACTL0_TASSEL
#define CONF_TACTL0_TASSEL  0x0200
#endif

/**
 * The settings below are used for Capture Compare Register
 * 0 as Capture compare Register is used in this application
 * This Configurations have to be repeated in case of other
 * Capture/Compare registers
 */

/**
 * @brief This Controls the Output in output mode 0
 * <0x0000=> The output is low
 * <0x0004=> The output is set high
 */
#ifndef CONF_TACCTL0_OUT
#define CONF_TACCTL0_OUT      0x0000
#endif

/**
 * @brief Capture/Compare Interrupt Enable
 * <0x0010=> Enable the Interrupt
 * <0x0000=> Disable the Interrupt
 */
#ifndef CONF_TACCTL0_CCIE
#define CONF_TACCTL0_CCIE     0x0010
#endif


/**
 * @brief This bit sets the Output Mode Mode 2, 3, 6 and 7
 * are not useful for TAxCCR0 because EQUx = EQU0
 * <0x0000=> OUT bit Value
 * <0x0020=> Set
 * <0x0040=> Toggle/reset
 * <0x0060=> Set/reset
 * <0x0080=> Toggle
 * <0x00A0=> Reset
 * <0x00C0=> Toggle/set
 * <0x00E0=> Reset/Set
 */
#ifndef CONF_TACCTL0_OUTMOD
#define CONF_TACCTL0_OUTMOD  0x0000
#endif

/**
 * @brief This bit sets the enables/disables Capture Mode
 * <0x0000=> Disables Capture Mode
 * <0x0100=> Enables Capture Mode.
 */
#ifndef CONF_TACCTL0_CAP
#define CONF_TACCTL0_CAP    0x0000
#endif

/**
 * @brief This bit is used to synchronize the capture input
 * signal with timer clock
 * <0x0000=> Asynchronous Capture
 * <0x0800=> Synchronous Capture
 */
#ifndef CONF_TACCTL0_SCS
#define CONF_TACCTL0_SCS    0x0000
#endif

/**
 * @brief Capture/Compare input select This bit selects the
 * TAxCCR0 input signal.
 * <0x0000=> CCIxA is selected
 * <0x1000=> CCIxB is selected
 * <0x2000=> GND is selected
 * <0x3000=> VCC is selected
 */
#ifndef CONF_TACCTL0_CCIS
#define CONF_TACCTL0_CCIS       0x0000
#endif

/**
 * @brief Capture Mode is Selected using this
 * <0x0000=> No Capture
 * <0x4000=> Capture on rising edge
 * <0x8000=> Capture on falling edge
 * <0xC000=> Capture on both rising and falling edge
 */
#ifndef CONF_TACCTL0_CM
#define CONF_TACCTL0_CM     0x0000
#endif

/**
 * @brief These bits are Input Divider Expansion. These bits
 * along with ID bits selects the divider for the input clock
 * <0x0000=>        Divide by 1
 * <0x0001=>        Divide by 2
 * <0x0002=>        Divide by 3
 * <0x0003=>        Divide by 4
 * <0x0004=>        Divide by 5
 * <0x0005=>        Divide by 6
 * <0x0006=>        Divide by 7
 * <0x0007=>        Divide by 8
 */
#ifndef CONF_TAEX0_TAIDEX
#define CONF_TAEX0_TAIDEX       0x0004
#endif

/******************************************************************************
 *                      TIMER A1 CONFIGURATIONS
 ******************************************************************************/
/******************************************************************************
* Configuration Constants
*******************************************************************************/
/**
 * @brief This bit is used to enable and disable
 * TimerA0 Interrupt
 * <0x0000=> Disable the Timer Interrupt
 * <0x0002=> Enable the Timer Interrupt
 */

#ifndef CONF_TACTL1_TAIE
#define CONF_TACTL1_TAIE    0x0000
#endif

/**
 * @brief This bit is used to set the MODE control
 * <0x0000=> Stop Mode
 * <0x0010=> Up Mode: Timer counts up to TAxCCR0
 * <0x0020=> Continuous Mode : Timer counts up to 0xFFFF
 * <0x0030=> Up/Down Mode : Timer Counts up to TAxCCR0 then
 *           down to 0x000h
 */
#ifndef CONF_TACTL1_MC
#define CONF_TACTL1_MC      0x0010
#endif

/**
 * @brief This bits are used to select the divider for input
 * clock
 * <0x0000=>  Divide by 1
 * <0x0040=>  Divide by 2
 * <0x0080=>  Divide by 4
 * <0x00C0=>  Divide by 8
 */
#ifndef CONF_TACTL1_ID
#define CONF_TACTL1_ID  0x0000
#endif

/**
 * @brief This bits are used to Select the Clock source
 * <0x0000=>  TA0CLK
 * <0x0100=>  ACLK is selected as clock Source
 * <0x0200=>  SMCLK is selected as clock Source
 * <0x0300=>  INCLK is selected as clock source
 */
#ifndef CONF_TACTL1_TASSEL
#define CONF_TACTL1_TASSEL  0x0200
#endif

/**
 * The settings below are used for Capture Compare Register
 * 0 as Capture compare Register is used in this application
 * This Configurations have to be repeated in case of other
 * Capture/Compare registers
 */

/**
 * @brief This Controls the Output in output mode 0
 * <0x0000=> The output is low
 * <0x0004=> The output is set high
 */
#ifndef CONF_TACCTL1_OUT
#define CONF_TACCTL1_OUT      0x0000
#endif

/**
 * @brief Capture/Compare Interrupt Enable
 * <0x0010=> Enable the Interrupt
 * <0x0000=> Disable the Interrupt
 */
#ifndef CONF_TACCTL1_CCIE
#define CONF_TACCTL1_CCIE     0x0010
#endif


/**
 * @brief This bit sets the Output Mode Mode 2, 3, 6 and 7
 * are not useful for TAxCCR0 because EQUx = EQU0
 * <0x0000=> OUT bit Value
 * <0x0020=> Set
 * <0x0040=> Toggle/reset
 * <0x0060=> Set/reset
 * <0x0080=> Toggle
 * <0x00A0=> Reset
 * <0x00C0=> Toggle/set
 * <0x00E0=> Reset/Set
 */
#ifndef CONF_TACCTL1_OUTMOD
#define CONF_TACCTL1_OUTMOD  0x0000
#endif

/**
 * @brief This bit sets the enables/disables Capture Mode
 * <0x0000=> Disables Capture Mode
 * <0x0100=> Enables Capture Mode.
 */
#ifndef CONF_TACCTL1_CAP
#define CONF_TACCTL1_CAP    0x0000
#endif

/**
 * @brief This bit is used to synchronize the capture input
 * signal with timer clock
 * <0x0000=> Asynchronous Capture
 * <0x0800=> Synchronous Capture
 */
#ifndef CONF_TACCTL1_SCS
#define CONF_TACCTL1_SCS    0x0000
#endif

/**
 * @brief Capture/Compare input select This bit selects the
 * TAxCCR0 input signal.
 * <0x0000=> CCIxA is selected
 * <0x1000=> CCIxB is selected
 * <0x2000=> GND is selected
 * <0x3000=> VCC is selected
 */
#ifndef CONF_TACCTL1_CCIS
#define CONF_TACCTL1_CCIS       0x0000
#endif

/**
 * @brief Capture Mode is Selected using this
 * <0x0000=> No Capture
 * <0x4000=> Capture on rising edge
 * <0x8000=> Capture on falling edge
 * <0xC000=> Capture on both rising and falling edge
 */
#ifndef CONF_TACCTL1_CM
#define CONF_TACCTL1_CM     0x0000
#endif

/**
 * @brief These bits are Input Divider Expansion. These bits
 * along with ID bits selects the divider for the input clock
 * <0x0000=>        Divide by 1
 * <0x0001=>        Divide by 2
 * <0x0002=>        Divide by 3
 * <0x0003=>        Divide by 4
 * <0x0004=>        Divide by 5
 * <0x0005=>        Divide by 6
 * <0x0006=>        Divide by 7
 * <0x0007=>        Divide by 8
 */
#ifndef CONF_TAEX1_TAIDEX
#define CONF_TAEX1_TAIDEX       0x0000
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
#endif /*TIMERA_CONFIG_H_*/

/*** End of File **************************************************************/
