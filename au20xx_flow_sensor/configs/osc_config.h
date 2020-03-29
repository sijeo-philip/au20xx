/***************************************************************************
* Title                 :   osc_config
* Filename              :   osc_config.h
* Author                :   Sijeo Philip
* Origin Date           :   15/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This is configuration file to set the clock settings
*
* THIS SOFTWARE IS PROVIDED BY  UNISEM ELECTRONICS "AS IS" AND ANY EXPRESSED
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
*  15/03/2020  1.0.0   Sijeo Philip   Interface Created.
*
*****************************************************************************/
/** @file osc_config.h
 *  @brief This module is used to configure the clocks for core and peripherals
 *
 *  This is the header file for the configurations of clocks
 */
//TODO: UPDATE MACRO BELOW
#ifndef OSC_CONFIG_H_
#define OSC_CONFIG_H_

/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocess or Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/
/*******************************************
 * This is used to set the DCO frequecy for the MCU
 * << 0x0000  ==> For 1MHz for DCORSEL =0 and DCORSEL =1
 * << 0x0002  ==> For 2.67MHz for DCORSEL = 0 and 5.33MHz for DCORSEL = 1
 * << 0x0004  ==> for 3.5MHz for DCORSEL = 0 and 7MHz for DCORSEL = 1
 * << 0x0006  ==> for 4 MHz for DCORSEL = 0 and 8MHz for DCORSEL = 1
 * << 0x0008  ==> for 5.33MHz for DCORSEL = 0 and 16MHz for DCORSEL =1
 * << 0x000A  ==> for 7MHz for DCORSEL = 0 and 21MHz for DCORSEL = 1
 * << 0x000C  ==> for 8MHz for DCORSEL = 0 and 24MHz for DCORSEL  =1
 * << 0x000E  ==> defaults to 8MHz DCORSEL=0 and defaults to 24MHz for DCORSEL =1
 * **************************************************/
#ifndef CONF_CSCTL1_DCOFSEL
#define CONF_CSCTL1_DCOFSEL 0x0000
#endif

/************************************
 * This is to set to 0 in case of low frequency device and
 * << 0x0000  ==> For low frequency Devices
 * << 0x0040  ==> For High Frequency Devices
 *
 */
#ifndef CONF_CSCTL1_DCORSEL
#define CONF_CSCTL1_DCORSEL 0x0000
#endif

/**********************************************
 * This is to select the  MCLK Source
 * << 0x0000  ==> LFXTCLK when LFXT available otherwise VLOCLK
 * << 0x0001  ==> VLOCLK
 * << 0x0002  ==> LFMODCLK
 * << 0x0003  ==> DCOCLK
 * << 0x0004  ==> MODCLK
 * << 0x0005  ==> HFXTCLK when HFXT available, otherwise DCOCLK
 * << 0x0006  ==> Defaults to HFXTCLK  (Not Recommended to use)
 * << 0x0007  ==> Defaults to HFXTCLK  (Not Recommended to use)
 */
#ifndef CONF_CSCTL2_SELM
#define CONF_CSCTL2_SELM  0x0003
#endif


/**********************************************
 * This is to select the  SMCLK Source
 * << 0x0000  ==> LFXTCLK when LFXT available otherwise VLOCLK
 * << 0x0010  ==> VLOCLK
 * << 0x0020  ==> LFMODCLK
 * << 0x0030  ==> DCOCLK
 * << 0x0040  ==> MODCLK
 * << 0x0050  ==> HFXTCLK when HFXT available, otherwise DCOCLK
 * << 0x0060  ==> Defaults to HFXTCLK  (Not Recommended to use)
 * << 0x0070  ==> Defaults to HFXTCLK  (Not Recommended to use)
 */
#ifndef CONF_CSCTL2_SELS
#define CONF_CSCTL2_SELS  0x0030
#endif

/**********************************************
 * This is to select the  ACLK Source
 * << 0x0000  ==> LFXTCLK when LFXT available otherwise VLOCLK
 * << 0x0100  ==> VLOCLK
 * << 0x0200  ==> LFMODCLK
 * << 0x0300  ==> DCOCLK
 * << 0x0400  ==> MODCLK
 * << 0x0500  ==> HFXTCLK when HFXT available, otherwise DCOCLK
 * << 0x0600  ==> Defaults to HFXTCLK  (Not Recommended to use)
 * << 0x0700  ==> Defaults to HFXTCLK  (Not Recommended to use)
 */
#ifndef CONF_CSCTL2_SELA
#define CONF_CSCTL2_SELA  0x0000
#endif


/*************************************************
 * This Setting set the clock Division of ACLK
 * << 0x0000  ==> /1
 * << 0x0100  ==> /2
 * << 0x0200  ==> /4
 * << 0x0300  ==> /8
 * << 0x0400  ==> /16
 * << 0x0500  ==> /32
 * << 0x0600  ==> Defaults to /32  (Not Recommended to use)
 * << 0x0700  ==> Defaults to /32  (Not Recommended to use)
 */
#ifndef CONF_CSCTL3_DIVA
#define CONF_CSCTL3_DIVA   0x0000
#endif

/**********************************************
 * This is to select Division of  SMCLK Source
 * << 0x0000  ==> /1
 * << 0x0010  ==> /2
 * << 0x0020  ==> /4
 * << 0x0030  ==> /8
 * << 0x0040  ==> /16
 * << 0x0050  ==> /32
 * << 0x0060  ==> Defaults to /32  (Not Recommended to use)
 * << 0x0070  ==> Defaults to /32  (Not Recommended to use)
 */
#ifndef CONF_CSCTL3_DIVS
#define CONF_CSCTL3_DIVS  0x0000
#endif

/**********************************************
 * This is to select the Division for MCLK Source
 * << 0x0000  ==> /1
 * << 0x0001  ==> /2
 * << 0x0002  ==> /4
 * << 0x0003  ==> /8
 * << 0x0004  ==> /16
 * << 0x0005  ==> /32
 * << 0x0006  ==> Defaults to /32  (Not Recommended to use)
 * << 0x0007  ==> Defaults to /32  (Not Recommended to use)
 */
#ifndef CONF_CSCTL3_DIVM
#define CONF_CSCTL3_DIVM  0x0000
#endif


/***************************************************
 * This configuration switches off the LFXT
 * <0=> LXFT is on if LFXT is selected through the port selection and LFXT
 *      is not in bypass mode of operation
 * <1=> LFXT is off if it is not used as source for ACLK, MCLK, SMCLK (default)
 * *****************/
#ifndef CONF_CSCTL4_LFXTOFF
#define CONF_CSCTL4_LFXTOFF		0
#endif

/************************************************
 * This configuration is used to switch off SMCLK
 * <0x0000=> SMCLK is ON (default)
 * <0x0002=> SMCLK is OFF
 *
 */
#ifndef CONF_CSCTL4_SMCLKOFF
#define CONF_CSCTL4_SMCLKOFF   0
#endif


/************************************************
 * This configuration is used to switch off VLOOFF
 * <0x0000=> VLO is ON
 * <0x0008=> VLO is OFF if it is not used as source for ACLK,
 *           SMCLK or MCLK or if not used as source for RTC in
 *           LPM3.5 (default)
 */
#ifndef CONF_CSCTL4_VLOOFF
#define CONF_CSCTL4_VLOOFF   0x0008
#endif


/************************************************
 * This configuration is used to select bypass LFXT
 * <0x0000=> LFXT sourced from external crystal (default)
 * <0x0010=> LFXT sourced from external clock signal
 *
 */
#ifndef CONF_CSCTL4_LFXTBYPASS
#define CONF_CSCTL4_LFXTBYPASS   0x0000
#endif

/************************************************
 * This configuration is used drive strength of LFXT
 * <0x0000=> Lowest Drive strength and current consumption for LFXT oscillator
 * <0x0040=> Increased Drive Strength LFXT Oscillator
 * <0x0080=> Increased Drive Strenght LFXT Oscillator
 * <0x00C0=> Maximum Drive strenght and maximum Current Consumption
 *  		 of LFXT Oscillator (default)
 */
#ifndef CONF_CSCTL4_LFXTDRIVE
#define CONF_CSCTL4_LFXTDRIVE   0x00C0
#endif

/************************************************
 * This configuration is used to switch off HFXT
 * <0x0000=> HFXT is ON  if HFXT is selected through port selection
 *           and HFXT is not in bypass mode of operation
 * <0x0100=> HFXT is OFF if it is not used as source for ACLK, SMCLK
 *           or MCLK(default)
 */
#ifndef CONF_CSCTL4_HFXTOFF
#define CONF_CSCTL4_HFXTOFF   0x0100
#endif

/************************************************
 * This configuration is used to select the HFXT Frequency range
 * <0x0000=> 0 to 4MHz
 * <0x0400=> Greater than 4MHz to 8MHz
 * <0x0800=> Greater than 8MHz to 16MHz
 * <0x0C00=> Greater than 16MHz to 24MHz (default)
 *
 */
#ifndef CONF_CSCTL4_HFFREQ
#define CONF_CSCTL4_HFFREQ   0x0C00
#endif

/************************************************
 * This configuration is used select HFXT Bypass
 * <0x0000=> Sourced from External Crystal (default)
 * <0x1000=> Sourced from External Clock Signal
 */
#ifndef CONF_CSCTL4_HFXTBYPASS
#define CONF_CSCTL4_HFXTBYPASS   0x0000
#endif


/************************************************
 * This configuration is used drive strength of HFXT
 * <0x0000=> Lowest Drive strength and current consumption for HFXT oscillator
 * <0x4000=> Increased Drive Strength HFXT Oscillator
 * <0x8000=> Increased Drive Strenght HFXT Oscillator
 * <0xC000=> Maximum Drive strenght of HFXT Oscillator (default)
 */
#ifndef CONF_CSCTL4_HFXTDRIVE
#define CONF_CSCTL4_HFXTDRIVE   0xC000
#endif


/************************************************
 * This bit is the LFXTOFFG fault flag is set when LFXT fault occured
 * after last reset
 * <0x0000=> No fault condition occurred after last reset
 * <0x0001=> LFXT fault, fault occurred after last reset (default)
 */
#ifndef CONF_CSCTL5_LFXTOFFG
#define CONF_CSCTL5_LFXTOFFG   0x0000
#endif

/************************************************
 * This bit is the HFXTOFFG fault flag is set when HFXT fault occured
 * after last reset
 * <0x0000=> No fault condition occurred after last reset (default)
 * <0x0002=> HFXT fault, fault occurred after last reset
 */
#ifndef CONF_CSCTL5_HFXTOFFG
#define CONF_CSCTL5_HFXTOFFG   0x0000
#endif


/************************************************
 * This bit is to enable/disable startup fault counter for LFXT
 * after last reset
 * <0x0000=> Startup fault counter is disabled, counter is cleared
 * <0x0040=> Startup fault counter is enabled (default)
 */
#ifndef CONF_CSCTL5_ENSTFCNT1
#define CONF_CSCTL5_ENSTFCNT1   0x0040
#endif



/************************************************
 * This bit is to enable/disable startup fault counter for HFXT
 * after last reset
 * <0x0000=> Startup fault counter is disabled, counter is cleared
 * <0x0080=> Startup fault counter is enabled (default)
 */
#ifndef CONF_CSCTL5_ENSTFCNT2
#define CONF_CSCTL5_ENSTFCNT2   0x0080
#endif


/************************************************
 * This bit is to enable/disable conditional requests for ACLK
 * after last reset
 * <0x0000=> ACLK Conditional Requests are disabled
 * <0x0001=> ACLK Conditional Requests are enabled (default)
 */
#ifndef CONF_CSCTL6_ACLKREQEN
#define CONF_CSCTL6_ACLKREQEN   0x0001
#endif


/************************************************
 * This bit is to enable/disable conditional requests for MCLK
 * after last reset
 * <0x0000=> MCLK Conditional Requests are disabled
 * <0x0002=> MCLK Conditional Requests are enabled (default)
 */
#ifndef CONF_CSCTL6_MCLKREQEN
#define CONF_CSCTL6_MCLKREQEN   0x0002
#endif


/************************************************
 * This bit is to enable/disable conditional requests for SMCLK
 * after last reset
 * <0x0000=> SMCLK Conditional Requests are disabled
 * <0x0004=> SMCLK Conditional Requests are enabled (default)
 */
#ifndef CONF_CSCTL6_SMCLKREQEN
#define CONF_CSCTL6_SMCLKREQEN   0x0004
#endif


/************************************************
 * This bit is to enable/disable conditional requests for MODCLK
 * after last reset
 * <0x0000=> MODCLK Conditional Requests are disabled (default)
 * <0x0008=> MODCLK Conditional Requests are enabled
 */
#ifndef CONF_CSCTL6_MODCLKREQEN
#define CONF_CSCTL6_MODCLKREQEN   0x0000
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


#endif /*OSC_CONFIG_H_*/

/*** End of File **************************************************************/
