/****************************************************************************
* Title                 :   adc_config
* Filename              :   adc_config.h
* Author                :   Sijeo Philip
* Origin Date           :   23/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to set the configurations of ADC to be used in application
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
/** @file adc_config.h
 *  @brief This module is used to configure the ADC to be used in the application
 *
 *  This is the header file for the definition of configuration parameters for the
 *  ADC peripheral
 */
//TODO: UPDATE MACRO BELOW
#ifndef ADC_CONFIG_H_
#define ADC_CONFIG_H_

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
 * @brief This bits are used to set the sample and hold time for registers
 * ADC12MEM8 to ADC12MEM23 (can be modified when ADC12ENC = 0)
 * <0x0000=>    4 ADC12CLK cycles
 * <0x1000=>    8 ADC12CLK cycles
 * <0x2000=>    16 ADC12CLK cycles
 * <0x3000=>    32 ADC12CLK cycles
 * <0x4000=>    64 ADC12CLK cycles
 * <0x5000=>    96 ADC12CLK cycles
 * <0x6000=>    128 ADC12CLK cycles
 * <0x7000=>    192 ADC12CLK cycles
 * <0x8000=>    256 ADC12CLK cycles
 * <0x9000=>    384 ADC12CLK cycles
 * <0xA000=>    512 ADC12CLK cycles
 * <0xB000 to 0xF000=>    Reserved
 *
 */
#ifndef CONF_ADC12CTL0_ADC12SHT1
#define CONF_ADC12CTL0_ADC12SHT1    0x8000
#endif

/**
 * @brief This bits are used to set the sample and hold time for registers
 * ADC12MEM0 to ADC12MEM7 (can be modified when ADC12ENC = 0)
 * <0x0000=>    4 ADC12CLK cycles
 * <0x0100=>    8 ADC12CLK cycles
 * <0x0200=>    16 ADC12CLK cycles
 * <0x0300=>    32 ADC12CLK cycles
 * <0x0400=>    64 ADC12CLK cycles
 * <0x0500=>    96 ADC12CLK cycles
 * <0x0600=>    128 ADC12CLK cycles
 * <0x0700=>    192 ADC12CLK cycles
 * <0x0800=>    256 ADC12CLK cycles
 * <0x0900=>    384 ADC12CLK cycles
 * <0x0A00=>    512 ADC12CLK cycles
 * <0x0B00 to 0x0F00=>    Reserved
 *
 */
#ifndef CONF_ADC12CTL0_ADC12SHT0
#define CONF_ADC12CTL0_ADC12SHT0    0x0800
#endif

/**
 * @brief This bit is used to Enable/Disable Multiple Sample and Conversion
 * Valid for sequence or repeated modes(can be modified when ADC12ENC = 0)
 * <0x0000=> Disable the Multiple Sample and Conversion
 * <0x0080=> Enable  the Multiple Sample and Conversion
 */
#ifndef CONF_ADC12CTL0_ADC12MSC
#define CONF_ADC12CTL0_ADC12MSC     0x0000
#endif

/**
 * @brief This bit is used to ENABLE/DISABLE the ADC Peripheral
 * <0x0000=> Disable the ADC12_B
 * <0x0010=> Enable the ADC12_B
 */
#ifndef CONF_ADC12CTL0_ADC12ON
#define CONF_ADC12CTL0_ADC12ON      0x0010
#endif


/**
 * @brief This bits are used to predivide the Selected ADC Clock source
 * <0x0000=>     Predivide by 1
 * <0x2000=>     Predivide by 4
 * <0x4000=>     Predivide by 32
 * <0x6000=>     Predivide by 64
 */
#ifndef CONF_ADC12CTL1_ADC12PDIV
#define CONF_ADC12CTL1_ADC12PDIV    0x0000
#endif

/**
 * @brief This bits selects the sample and hold source
 * <0x0000=>     ADC12SC bit i.e. Software Enabled
 * <0x0400=>     Device Specific
 * <0x0800=>     Device Specific
 * <0x0C00=>     Device Specific
 * <0x1000=>     Device Specific
 * <0x1400=>     Device Specific
 * <0x1800=>     Device Specific
 * <0x1C00=>     Device Specific
 */
#ifndef CONF_ADC12CTL1_ADC12SHS
#define CONF_ADC12CTL1_ADC12SHS     0x0000
#endif

/**
 * @brief This bit selects the source of Sampling Signal (SAMPCON)
 * to be either the output of the sampling timer or the sample-
 * <0x0000=>    SAMPCON signal is sourced from the sample-input signal
 * <0x0200=>    SAMPCON signal is sourced from the sampling timer
 */
#ifndef CONF_ADC12CTL1_ADC12SHP
#define CONF_ADC12CTL1_ADC12SHP     0x0000
#endif

/**
 * @brief This bit is used to invert the Signal sample and hold
 * <0x0000=> The Sample input signal is not inverted
 * <0x0010=> The Sample input signal is inverted
 */

#ifndef CONF_ADC12CTL1_ADC12ISSH
#define CONF_ADC12CTL1_ADC12ISSH    0x0000
#endif

/**
 * @brief This bits set the Clock Divider of ADC12_B
 * <0x0000=>        Divide by 1
 * <0x0020=>        Divide by 2
 * <0x0040=>        Divide by 3
 * <0x0060=>        Divide by 4
 * <0x0080=>        Divide by 5
 * <0x00A0=>        Divide by 6
 * <0x00C0=>        Divide by 7
 * <0x00E0=>        Divide by 8
 */
#ifndef CONF_ADC12CTL1_ADC12DIV
#define CONF_ADC12CTL1_ADC12DIV     0x0000
#endif

/**
 * @brief This bits set the clock source for the ADC
 * <0x0000=>        ADC12OSC (MODOSC)
 * <0x0008=>        ACLK
 * <0x0010=>        MCLK
 * <0x0018=>        SMCLK
 */
#ifndef CONF_ADC12CTL1_ADC12SSEL
#define CONF_ADC12CTL1_ADC12SSEL        0x0000
#endif


/**
 * @brief This bits selects the sequence MODE... This mode should
 * only be set when ADC12ENC = 0 except to stop a conversion
 * immediately by ADC12CONSEQx = 00 when ADC12ENC = 1
 * <0x0000=>    Single Channel Single Conversion
 * <0x0002=>    Sequence of Channels
 * <0x0004=>    Repeat Single Channel
 * <0x0006=>    Repeat sequence of channels
 */

#ifndef CONF_ADC12CTL1_ADC12CONSEQ
#define CONF_ADC12CTL1_ADC12CONSEQ      0x0002
#endif


/**
 * @brief This bits configures the Resolution of the ADC
 * <0x0000=>    8-bit (10 clock cycle conversion time)
 * <0x0010=>    10-bit (12 clock cycle conversion time  )
 * <0x0020=>    12-bit (14 clock cycle conversion time )
 * <0x0030=>    Reserved
 */
#ifndef CONF_ADC12CTL2_ADC12RES
#define CONF_ADC12CTL2_ADC12RES     0x0020
#endif

/**
 * @brief This bit sets the data read back format. Data is always
 * stored in the binary unsigned format
 * <0x0000=>    Binary unsigned. Theoretically for ADC12DIF = 0 and
 *              12 bit mode the analog input value for -VREF is 0000h
 *              the analog input voltage +VREF results in 0x0FFFh
 * <0x0008=>    Signed Binary (2s-Complement) left aligned. Theoretically
 *              ADC12DIF = 0 and 12-bit mode, the analog input voltage
 *              -VREF results in 0x8000h and analog input voltage +VREF
 *              result in 7FF0h
 */
#ifndef CONF_ADC12CTL2_ADC12DF
#define CONF_ADC12CTL2_ADC12DF      0x0000
#endif

/**
 * @brief This bit Enables/Disables the LOW power mode of ADC for
 * ADC12CLK with 1/4 the specified maximum for ADC12PWRMD=0. This
 * bit should only be modified ADC12ENC = 0
 * <0x0000=>    Regular Power Mode where sample rate is not restricted
 * <0x0001=>    Low Power Mode enable. ADC12CLK can not be greater than
 *              1/4 the device-specific data sheet specified Maximum for
 *              ADC12PWRMD = 0
 *
 */
#ifndef CONF_ADC12CTL2_ADC12PWRMD
#define CONF_ADC12CTL2_ADC12PWRMD       0x0000
#endif


/******************************************************************************
* Macros
*******************************************************************************/
/**
 * This Macros to be used while writing settings to the ADC Register
 * Settings can be written while ADC_CONV_DISABLE
 */

#define ADC_CONV_ENABLE   \
    do{  \
        ADC12CTL0 |=   0x0002;\
      }while(0)

#define ADC_CONV_DISABLE  \
    do{     \
        ADC12CTL0 &= 0xFFFD; \
    }while(0)


/**
 * This Macro to be called while conversion should be start i.e when
 * the Conversions are Software Triggered
 * ADC12SC and ADC12ENC can be set together with one instruction the
 * Start conversion bit is reset automatically
 */
#define ADC_START_CONV  \
    do{         \
        ADC12CTL0 |= 0x0001; \
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

#endif /*ADC_CONFIG_H_*/

/*** End of File **************************************************************/
