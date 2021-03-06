/*******************************************************************************
* Title                 :   au20xx_api
* Filename              :   au20xx_api.c
* Author                :   Sijeo Philip
* Origin Date           :   20/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to AU20xx Chip Operations
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
*******************************************************************************/
/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author         Description
*  20/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file au20xx_api.c
 *  @brief This is the source file for au20xx chip operations
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>             /* For portable types */

//TODO: UPDATE MY INCLUDE
#include "au20xx_api.h"             /* For au20xx api */
#include "hal_spi_ucsi.h"           /* For SPI types */
#include "hal_fram.h"               /* For FRAM types */
#include "common.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define READ_AU20xx         0x00
#define WRITE_AU20xx        0x40
#define CMD_EXEC_AU20xx     0x80
#define RESET_AU20xx        0xC0

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/

/******************************************************************************
* Function : au20xx_chip_reset()
*//**
* \b Description:
*
* This function sends reset command to au20xx chip for reset of the chip the chip
* registers are reset to the default value on reset of the chip
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_chip_reset();
* @endcode
*
* @see Dio_Init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 20/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void au20xx_chip_reset( void )
{
 static uint8_t reset_command = 0;
 reset_command = RESET_AU20xx;
 spi_write(&reset_command, 1);
 delay_us(3);
 DISABLE_SPI_CS;
}


/******************************************************************************
* Function : au20xx_send_command()
*//**
* \b Description:
*
* This function is used to send sub command to the au20xx chip
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[in]    sub_command     byte of subcommand to be send to the au20xx chip
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_send_command(0x20);
* @endcode
*
* @see Dio_Init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 20/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void au20xx_send_command(uint8_t sub_command)
{
    static uint8_t command = 0;
    command = CMD_EXEC_AU20xx | sub_command ;
    spi_write(&command, 1);
    delay_us(3);
    DISABLE_SPI_CS;
}

/******************************************************************************
* Function : au20xx_write_reg();
*//**
* \b Description:
*
* This function is used to write to the au20xx chip as per register address given
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[in]    reg_addr     Address of the register to which the data has to be written
* @param[in]    data         Data to be written to the register
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_write_reg(0x01, 0x05);
* @endcode
*
* @see Dio_Init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 20/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void au20xx_write_reg(uint8_t reg_addr, uint8_t data)
{
  static uint16_t register_data = 0;
  register_data = data<<8|(WRITE_AU20xx)| (reg_addr);
  ENABLE_SPI_CS;                                 /* Pull the Chip Select line LO to select the Chip */
  delay_us(5);
  spi_write(&register_data, 2);
  delay_us(5);
  DISABLE_SPI_CS;
 }

/******************************************************************************
* Function : au20xx_read_reg();
*//**
* \b Description:
*
* This function is used to read from the au20xx chip as per register address given
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[in]    reg_addr     Address of the register to which the data has to be written
* @param[out]    data         Data to be read from the register
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_read_reg(0x01, 0x05);
* @endcode
*
* @see spi_read()
* @see spi_write()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 20/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void au20xx_read_reg( uint8_t reg_addr, void * const data)
{
  static uint16_t register_data = 0;
  register_data = READ_AU20xx | reg_addr;
  spi_write(&register_data, 1);
  delay_us(3);
  spi_read ( (void*)data, 1 );
  delay_us(3);
  DISABLE_SPI_CS;

}

/******************************************************************************
* Function : au20xx_calibrate();
*//**
* \b Description:
*
* This function is used to read from the au20xx chip as per register address given
*
* PRE-CONDITION: Clock should be pre-configured and enabled
* PRE-CONDITION: SPI peripheral should be pre-configured and enabled
* PRE-CONDITION: GPIO should be pre-configured and enabled
*
* POST-CONDITION: The chip will be selected and Reset command is send to the chip
*
* @param[in]     topVariable  Address of the structure where the top Variables are stored
*
* @return       None
*
* \b Example Example:
* @code
*   au20xx_read_reg(0x01, 0x05);
* @endcode
*
* @see spi_read()
* @see spi_write()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 8/04/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/

void au20xx_calibrate(top_variables_t* topVariables)
{
        static uint8_t valid_data = 0;
        au20xx_chip_reset();
        au20xx_write_reg(INTF_CFG_REG, (0x78|topVariables->sensEnTime));
        valid_data = 0;
        sensEn_delay_us();
        while( 0 == valid_data )
        {
             au20xx_read_reg( SNS_VALID_REG, (void*)&valid_data);
             delay_us(1);
        }
        valid_data = 0;
        au20xx_read_reg( SNS1_OUT_Q16_LSB_REG, &topVariables->sns1_off0);
        au20xx_read_reg( SNS1_OUT_Q16_MSB_REG, &topVariables->sns1_off1);
        au20xx_read_reg( SNS2_OUT_Q16_LSB_REG, &topVariables->sns2_off0);
        au20xx_read_reg( SNS2_OUT_Q16_MSB_REG, &topVariables->sns2_off1);

        fram_write(&topVariables->sns1_off0, 1, SNS1_OFF0);
        fram_write(&topVariables->sns1_off1, 1, SNS1_OFF1);
        fram_write(&topVariables->sns2_off0, 1, SNS2_OFF0);
        fram_write(&topVariables->sns2_off1, 1, SNS2_OFF1);

}
/*************** END OF FUNCTIONS ***************************************************************************/
