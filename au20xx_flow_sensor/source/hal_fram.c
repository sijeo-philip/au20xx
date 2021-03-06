/*******************************************************************************
* Title                 :   hal_fram
* Filename              :   hal_fram.c
* Author                :   Sijeo Philip
* Origin Date           :   29/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   The Module is used to FRAM Initialization and use
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
*  29/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file hal_fram.c
 *  @brief This is the source file for FRAM Routines
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>             /* For portable types */

//TODO: UPDATE MY INCLUDE
#include "hal_fram.h"           /* For FRAM Module  */
#include "msp430.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/


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
* Function : fram_init()
*//**
* \b Description:
*
* This function is used to initialize the fram module with the configurations set in
* the respective *_config.h file
*
* PRE-CONDITION: Clock should be set
* PRE-CONDITION: GPIO should be initialized and set
*
* POST-CONDITION: The FRAM is configured and ready to use for read and write
*
* @return     None
*
* \b Example Example:
* @code
*   fram_init();
* @endcode
*
* @see system_clock_init()
* @see gpio_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 29/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void fram_init( void )
{
    FRCTL0 |= FRCTLPW;
    FRCTL0 |= CONF_FRCTL0_NWAITS;
    GCCTL0 |= CONF_GCCTL0_UBDRSTEN | CONF_GCCTL0_UBDIE | CONF_GCCTL0_CBDIE |
              CONF_GCCTL0_FRPWR | CONF_GCCTL0_FRLPMPWR;
}

/******************************************************************************
* Function : fram_write()
*//**
* \b Description:
*
* This function is used to write to fram memory to save data
*
* PRE-CONDITION: Clock should be set
* PRE-CONDITION: GPIO should be initialized and set
*
* POST-CONDITION: The FRAM data is written to Memory
* @param[in]    Address of the data to be written to memory
* @param[in]    Number of bytes to be written
* @param[in]    Address of FRAM to which it should be written
*
* @return     None
*
* \b Example Example:
* @code
*   fram_write(&data, 2, 0x001800);
* @endcode
*
* @see system_clock_init()
* @see gpio_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 29/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/

bool fram_write ( void * data, uint16_t bytes , uint8_t * p_fram)
{
    static uint8_t* p_data ;
    static uint16_t byteCount =0;
    byteCount = bytes;
    p_data = (uint8_t*)data;
    if ((p_fram < (uint8_t*)FRAM_START_ADDRESS)|| (p_fram > (uint8_t*)FRAM_END_ADDRESS ))
        return false;
    while( byteCount > 0 )
    {
        *p_fram = *p_data;
         p_data++;
         p_fram++;
         byteCount--;
    }
    return true;
}

/******************************************************************************
* Function : fram_read()
*//**
* \b Description:
*
* This function is used to read to fram memory and save to data
*
* PRE-CONDITION: Clock should be set
* PRE-CONDITION: GPIO should be initialized and set
*
* POST-CONDITION: The FRAM data is written to Memory
* @param[in]    Address of the data to be read from memory
* @param[in]    Number of bytes to be read
* @param[in]    Address of FRAM to be read from
*
* @return     None
*
* \b Example Example:
* @code
*   fram_read(&data, 2, 0x001800);
* @endcode
*
* @see system_clock_init()
* @see gpio_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 29/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
bool fram_read ( void * data, uint16_t bytes, uint8_t * p_fram)
{
    static uint8_t* p_data;
    static uint16_t byteCount =0;
    byteCount = bytes;
    p_data = (uint8_t*)data;
    if ((p_fram < (uint8_t*)FRAM_START_ADDRESS) ||( p_fram > (uint8_t*)FRAM_END_ADDRESS ))
            return false;
    while(byteCount > 0)
    {
        *p_data++ = *p_fram++;
        byteCount--;
    }
    return true;
}

/*************** END OF FUNCTIONS ***************************************************************************/



