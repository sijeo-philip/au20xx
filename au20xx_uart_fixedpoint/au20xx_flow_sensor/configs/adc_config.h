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
#include "msp430.h"
#include <stdint.h>

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define COMPARATOR_WIN_ENABLE               0x4000
#define COMPARATOR_WIN_DISABLE              0x0000
#define ADC_DIFF_MODE                       0x2000
#define ADC_SINGLE_ENDED                    0x0000
#define VREFP_AVCC_VREFN_AVSS                   0x0000    /**<<VR+ = AVCC, VR- = AVSS */
#define VREFP_VEREFB_VREFN_AVSS                 0x0100    /**<<VR+ = VREF buffered, VR- = AVSS */
#define VREFP_VEREFN_VREFN_AVSS                 0x0200    /**<<VR+ = VeREF-, VR- = AVSS */
#define VREFP_VEREFPB_VREFN_AVSS                0x0300    /**<<VR+ = VeREF+buffered, VR- = AVSS */
#define VREFP_VEREFP_VREFN_AVSS                 0x0400    /**<<VR+ = VeREF+, VR- = AVSS */
#define VREFP_AVCC_VREFN_VEREFPB                0x0500    /**<<VR+ = AVCC, VR- = VeREF+ buffered */
#define VREFP_AVCC_VREFN_VEREFP                 0x0600    /**<<VR+ = AVCC, VR- = VeREF+ */
#define VREFP_VEREFB_VREFN_VEREFP               0x0700    /**<<VR+ = VREF buffered, VR- = VeREF+ */
#define VREFP_AVCC_VREFN_VREFB                  0x0900    /**<<VR+ = AVCC, VR- = VREF buffered */
#define VREFP_VEREFP_VREFN_VREFB                0x0B00    /**<<VR+ = VeREF+, VR- = VREF buffered */
#define VREFP_AVCC_VREFN_VEREFN                 0x0C00    /**<<VR+ = AVCC, VR- = VeREF- */
#define VREFP_VREFB_VREFN_VEREFN                0x0D00    /**<<VR+ = VREF buffered, VR- = VeREF- */
#define VREFP_VEREFP_VREFN_VEREFN               0x0E00    /**<<VR+ = VeREF+, VR- = VeREF- */
#define VREFP_VEREFPB_VREFN_VEREFN              0x0F00    /**<<VR+ = VeREF+ buffered, VR- = VeREF-*/
#define ADC_END_OF_SEQUENCE                     0x0080
#define ADC_NOT_END_OF_SEQUENCE                 0x0000
#define ADC_A0_CHANNEL_SINGLE                    0x0000
#define ADC_A1_CHANNEL_SINGLE                    0x0001
#define ADC_A2_CHANNEL_SINGLE                    0x0002
#define ADC_A3_CHANNEL_SINGLE                    0x0003
#define ADC_A4_CHANNEL_SINGLE                    0x0004
#define ADC_A5_CHANNEL_SINGLE                    0x0005
#define ADC_A6_CHANNEL_SINGLE                    0x0006
#define ADC_A7_CHANNEL_SINGLE                    0x0007
#define ADC_A8_CHANNEL_SINGLE                    0x0008
#define ADC_A9_CHANNEL_SINGLE                    0x0009
#define ADC_A10_CHANNEL_SINGLE                   0x000A
#define ADC_A11_CHANNEL_SINGLE                   0x000B
#define ADC_A12_CHANNEL_SINGLE                   0x000C
#define ADC_A13_CHANNEL_SINGLE                   0x000D
#define ADC_A14_CHANNEL_SINGLE                   0x000E
#define ADC_A15_CHANNEL_SINGLE                   0x000F
#define ADC_A16_CHANNEL_SINGLE                   0x0010
#define ADC_A17_CHANNEL_SINGLE                   0x0011
#define ADC_A18_CHANNEL_SINGLE                   0x0012
#define ADC_A19_CHANNEL_SINGLE                   0x0013
#define ADC_A20_CHANNEL_SINGLE                   0x0014
#define ADC_A21_CHANNEL_SINGLE                   0x0015
#define ADC_A22_CHANNEL_SINGLE                   0x0016
#define ADC_A23_CHANNEL_SINGLE                   0x0017
#define ADC_A24_CHANNEL_SINGLE                   0x0018
#define ADC_A25_CHANNEL_SINGLE                   0x0019
#define ADC_A26_CHANNEL_SINGLE                   0x001A
#define ADC_A27_CHANNEL_SINGLE                   0x001B
#define ADC_A28_CHANNEL_SINGLE                   0x001C
#define ADC_A29_CHANNEL_SINGLE                   0x001D
#define ADC_A30_CHANNEL_SINGLE                   0x001E
#define ADC_A31_CHANNEL_SINGLE                   0x001F

#define ADC_A0_AINP_A1_AINN_DIFF                 0x0000
#define ADC_A2_AINP_A3_AINN_DIFF                 0x0002
#define ADC_A4_AINP_A5_AINN_DIFF                 0x0004
#define ADC_A6_AINP_A7_AINN_DIFF                 0x0006
#define ADC_A8_AINP_A9_AINN_DIFF                 0x0008
#define ADC_A10_AINP_A11_AINN_DIFF               0x000A
#define ADC_A12_AINP_A13_AINN_DIFF               0x000D
#define ADC_A14_AINP_A15_AINN_DIFF               0x000F
#define ADC_A16_AINP_A17_AINN_DIFF               0x0010
#define ADC_A18_AINP_A19_AINN_DIFF               0x0012
#define ADC_A20_AINP_A21_AINN_DIFF               0x0014
#define ADC_A22_AINP_A23_AINN_DIFF               0x0016
#define ADC_A24_AINP_A25_AINN_DIFF               0x0018
#define ADC_A26_AINP_A27_AINN_DIFF               0x001A
#define ADC_A28_AINP_A29_AINN_DIFF               0x001D
#define ADC_A30_AINP_A31_AINN_DIFF               0x001F

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
#define CONF_ADC12CTL0_ADC12SHT1    0x9000
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
#define CONF_ADC12CTL0_ADC12SHT0    0x0900
#endif

/**
 * @brief This bit is used to Enable/Disable Multiple Sample and Conversion
 * Valid for sequence or repeated modes(can be modified when ADC12ENC = 0)
 * <0x0000=> Disable the Multiple Sample and Conversion
 * <0x0080=> Enable  the Multiple Sample and Conversion
 */
#ifndef CONF_ADC12CTL0_ADC12MSC
#define CONF_ADC12CTL0_ADC12MSC     0x0080
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
#define CONF_ADC12CTL1_ADC12SHP     0x0200
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
#define CONF_ADC12CTL1_ADC12SSEL        0x0008
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


/**
 * @brief This bit controls the internal channel 3 selection to ADC
 * input channel A26. Can be modified only when ADC12ENC = 0
 * <0x0000=>     External Pin is Selected for ADC input channel A26
 * <0x0800=>     ADC input channel internal 3 is selected for ADC
 *               input channel A26, (see device datasheet)
 *
 */
#ifndef CONF_ADC12CTL3_ADC12ICH3MAP
#define CONF_ADC12CTL3_ADC12ICH3MAP     0x0000
#endif


/**
 * @brief This bit controls the internal channel 2 selection to ADC
 * input channel A27. Can be modified only when ADC12ENC = 0
 * <0x0000=>     External Pin is Selected for ADC input channel A27
 * <0x0400=>     ADC input channel internal 2 is selected for ADC
 *               input channel A27, (see device datasheet)
 *
 */
#ifndef CONF_ADC12CTL3_ADC12ICH2MAP
#define CONF_ADC12CTL3_ADC12ICH2MAP     0x0000
#endif

/**
 * @brief This bit controls the internal channel 1 selection to ADC
 * input channel A28. Can be modified only when ADC12ENC = 0
 * <0x0000=>     External Pin is Selected for ADC input channel A26
 * <0x0200=>     ADC input channel internal 1 is selected for ADC
 *               input channel A28, (see device datasheet)
 *
 */
#ifndef CONF_ADC12CTL3_ADC12ICH1MAP
#define CONF_ADC12CTL3_ADC12ICH1MAP     0x0000
#endif


/**
 * @brief This bit controls the internal channel 0 selection to ADC
 * input channel A29. Can be modified only when ADC12ENC = 0
 * <0x0000=>     External Pin is Selected for ADC input channel A26
 * <0x0100=>     ADC input channel internal 0 is selected for ADC
 *               input channel A29, (see device datasheet)
 *
 */
#ifndef CONF_ADC12CTL3_ADC12ICH0MAP
#define CONF_ADC12CTL3_ADC12ICH0MAP     0x0000
#endif

/**
 * @brief This bit controls Temperature Sensor ADC input channel
 * selection. Can be modified only on ADC12ENC = 0
 * <0x0000=>    External PIn is selected for ADC Input
 *              Channel A30
 * <0x0080=>    ADC internal Temperature sensor channel is selected
 *              for ADC input channel A30
 */
#ifndef CONF_ADC12CTL3_ADC12TCMAP
#define CONF_ADC12CTL3_ADC12TCMAP       0x0080
#endif

/**
 * @brief This bit Controls 1/2 AVCC ADC input channel selection.Can be
 * modified only when ADC12ENC = 0
 * <0x0000=>    External pin is selected for ADC input Channel A31
 * <0x0040=>    ADC internal 1/2 x AVCC channel is selected for ADC
 *              input channel A31
 */
#ifndef CONF_ADC12CTL3_ADC12BATMAP
#define CONF_ADC12CTL3_ADC12BATMAP      0x0040
#endif

/**
 * @brief Conversion Start Address is set on this bits this bit selects
 * conversion memory register for a single conversion or for the first
 * conversion in a sequence. The value is from 0x0000 to 0x001F
 *
 */

#ifndef CONF_ADC12CTL3_ADC12CSTARTADD
#define CONF_ADC12CTL3_ADC12CSTARTADD       0x001E
#endif

/**
 * @brief This bit enable/disable the local reference buffer ready
 * interrupt (GIE should be enabled)
 * <0x0000=>  Interrupt Disabled
 * <0x0040=>  Interrupt Enabled
 */
#ifndef CONF_ADC12IER2_ADC12RDYIE
#define CONF_ADC12IER2_ADC12RDYIE   0x0000
#endif


/**
 * @brief This bit enable/disable conversion-time-overflow
 * interrupt (GIE should be enabled)
 * <0x0000=>  Interrupt Disabled
 * <0x0020=>  Interrupt Enabled
 */
#ifndef CONF_ADC12IER2_ADC12TOVIE
#define CONF_ADC12IER2_ADC12TOVIE   0x0000
#endif


/**
 * @brief This bit enable/disable the overflow
 * interrupt (GIE should be enabled)
 * <0x0000=>  Interrupt Disabled
 * <0x0010=>  Interrupt Enabled
 */
#ifndef CONF_ADC12IER2_ADC12OVIE
#define CONF_ADC12IER2_ADC12OVIE   0x0000
#endif


/**
 * @brief This bit enable/disable upper limit of window comparator
 * interrupt (GIE should be enabled)
 * <0x0000=>  Interrupt Disabled
 * <0x0008=>  Interrupt Enabled
 */
#ifndef CONF_ADC12IER2_ADC12HIIE
#define CONF_ADC12IER2_ADC12HIIE   0x0000
#endif

/**
 * @brief This bit enable/disable lower limit of window comparator
 * interrupt (GIE should be enabled)
 * <0x0000=>  Interrupt Disabled
 * <0x0004=>  Interrupt Enabled
 */
#ifndef CONF_ADC12IER2_ADC12LOIE
#define CONF_ADC12IER2_ADC12LOIE   0x0000
#endif

/**
 * @brief This bit enable/disable result greater than lower limit and
 * lower than higher limit of window comparator nterrupt (GIE should be enabled)
 * <0x0000=>  Interrupt Disabled
 * <0x0002=>  Interrupt Enabled
 */
#ifndef CONF_ADC12IER2_ADC12INIIE
#define CONF_ADC12IER2_ADC12INIIE   0x0000
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
        ADC12CTL0 &= 0xFFFC; \
    }while(0)


/**
 * This Macro is used read the ADC Memory for the Conversion Result
 */
#define ADC_CONV_MEMORY(location)   ADC12MEM##location

/**
 * @brief This Macro selects the ACD Memory Control to be used
 */
#define ADC_MEMORY_CTL(number)      ADC12MCTL##number


/**
 * @brief this half word is for the comparator window high setting
 */
#define ADC_COMPARATOR_WINDOW_HI    ADC12HI

/**
 * @brief This Half word is the for Comparator window low setting
 */
#define ADC_COMPARATOR_WINDOW_LO   ADC12LO


/**
 * @brief This Register is used to enable the interrupts for the
 * results after ADC conversion into the respective memory
 */

inline void __adc_enable_memory_interrupt(uint8_t memory_no)
{
    if ( memory_no > 15)
        ADC12IER1 |= 1<<(memory_no - 16);
    else
        ADC12IER0 |= 1<<(memory_no);
}

/**
 * @brief Check if ADC is busy with conversion
 */
#define IS_ADC_BUSY()   ((ADC12CTL1 & 0x0001)?1:0)


/**
 * @brief check if interrupt flag from 0 to 15 mem is high
 */
#define IS_ADC_MEM_INT_0_TO_15_HIGH(memory_no)   ((ADC12IFGR0 & (1UL<<memory_no))?1:0)


/**
 * @brief check if interrupt flag from 16 to 31 is high
 */
#define IS_ADC_MEM_INT_16_31_HIGH(memory_no)  ((ADC12IFGR1 & (1UL<<(memory_no-16)))?1:0)




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
