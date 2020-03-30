/*******************************************************************************
* Title                 :   hal_spi
* Filename              :   hal_spi.c
* Author                :   Sijeo Philip
* Origin Date           :   17/03/2020
* Version               :   1.0.0
* Compiler              :   mspx_eabi
* Target                :   MSP430FR6989
* Notes                 :   This module is used to initializing the SPI and Operatiosn
*
* THIS SOFTWARE IS PROVIDED BY UNISEM ELECTRONICS  "AS IS" AND ANY EXPRESSED
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
*  17/03/20   1.0.0    Sijeo Philip   Initial Release.
*
*******************************************************************************/
/** @file hal_spi.c
 *  @brief This is the source file for initializing the SPI peripherals
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>				/* For portable types */

//TODO: UPDATE MY INCLUDE
#include "hal_spi_ucsi.h"				/* For SPI routines */
#include "common.h"

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
 static void spi_byte_write( uint8_t data );
 static uint8_t spi_byte_read( void );
/******************************************************************************
* Function Definitions
*******************************************************************************/

/******************************************************************************
* Function : spi_init()
*//**
* \b Description:
*
* This function is used to initialize the spi peripheral as per the configurations
* set in the file.
*
* PRE-CONDITION: Configuration to be done in ucsi_spi_config file
* PRE-CONDITION: The Oscillator should be initialized
* PRE-CONDITION: The GPIO should be intialized
*
* POST-CONDITION: SPI Peripheral will be configured and ready to use
*
* @return 		None
*
* \b Example Example:
* @code
*
*   system_init()
*   gpio_init()
* 	spi_init()
* @endcode
*
* @see system_init()
* @see gpio_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 17/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void spi_init( void )
{
	ENABLE_SPI_SETTING(B0);
	SPI_CTL_REG(B0) |= CONF_UCCTLW0_UCSTEM | CONF_UCCTLW0_UCSSEL | CONF_UCCTLW0_UCSYNC |
			       CONF_UCCTLW0_UCMODE | CONF_UCCTLW0_UCMST | CONF_UCCTLW0_UC7BIT |
				   CONF_UCCTLW0_UCMSB  | CONF_UCCTLW0_UCCKPL | CONF_UCCTLW0_UCCKPH ;

	SPI_BAUD_REG(B0) = CONF_UCBRW_UCBRW ;

	SPI_STAT_REG(B0) |= CONF_UCSTATW_UCLISTEN;

	SPI_INT_REG(B0)  |= CONF_UCIE_UCTXIE | CONF_UCIE_UCRXIE;

	DISABLE_SPI_SETTING(B0);

}

/******************************************************************************
* Function : spi_byte_write()
*//**
* \b Description:
*
* This function is used to write a byte of data over spi interface
*
* PRE-CONDITION: Configuration to be done in ucsi_spi_config file
* PRE-CONDITION: The Oscillator should be initialized
* PRE-CONDITION: The GPIO should be intialized
* PRE-CONDITION: SPI should be intialized and enabled
*
* POST-CONDITION: Byte will be transferred over SPI
* @param[in] 	byte 	byte of data to be transferred over spi
*
* @return 		None
*
* \b Example Example:
* @code
*  uint8_t byte = 0x0A;
*  spi_byte_write(byte);
*
* @endcode
*
* @see system_init()
* @see gpio_init()
* @see spi_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 17/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
static void spi_byte_write( uint8_t data )
{
	while (1 == CHECK_SPI_BUSY(B0) ) {}            /* This checks if the UCBUSY bit is High, if yes then the
													 peripheral is busy reading or transmitting data over the
													 peripheral */
	SPI_TX_BUF(B0) = data;						 /* Load the Data into SPI Transmit Buffer */

	while ( 0 == SPI_TX_INT_FLAG(B0) ) {}        /* Test the TX is completed */
}


/******************************************************************************
* Function : spi_byte_read()
*//**
* \b Description:
*
* This function is used to read a byte of data over spi interface
*
* PRE-CONDITION: Configuration to be done in ucsi_spi_config file
* PRE-CONDITION: The Oscillator should be initialized
* PRE-CONDITION: The GPIO should be intialized
* PRE-CONDITION: SPI should be intialized and enabled
*
* POST-CONDITION: Byte will be read over SPI
*
* @return 		 byte of data read over SPI
*
* \b Example Example:
* @code
*  uint8_t byte = 0x00;
*  byte = spi_byte_read();
*
* @endcode
*
* @see system_init()
* @see gpio_init()
* @see spi_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 17/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
static uint8_t spi_byte_read(void)
{
	while (1 == CHECK_SPI_BUSY(B0) ) {}            /* This checks if the UCBUSY bit is High, if yes then the
														 peripheral is busy reading or transmitting data over the
														 peripheral */
	ENABLE_SPI_CS;									/* Enable the SPI Slave chip */
	SPI_TX_BUF(B0) = 0xAA;							/* Dummy Data is written over SPI to shift out the data to be read */
	while ( 0 == SPI_RX_INT_FLAG(B0) ) {}           /* Waiting to receive a byte of data in SPI Register */

	return SPI_RX_BUF(B0);
}


/******************************************************************************
* Function : spi_write()
*//**
* \b Description:
*
* This function is used to write a byte of data over spi interface
*
* PRE-CONDITION: Configuration to be done in ucsi_spi_config file
* PRE-CONDITION: The Oscillator should be initialized
* PRE-CONDITION: The GPIO should be intialized
* PRE-CONDITION: SPI should be intialized and enabled
*
* POST-CONDITION: Bytes will be transferred over SPI
*
* @param[in]	p_buff		pointer to buffer of data to be transferred over SPI
* @param[in]    byte_count	Number of bytes to be transferred
*
* @return 		None
*
* \b Example Example:
* @code
*  uint8_t byte[20] = "Hello World!";
*  spi_write(byte, 12);
*
* @endcode
*
* @see system_init()
* @see gpio_init()
* @see spi_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 17/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void spi_write ( void * p_buff, uint16_t byte_count )
{
	static uint16_t bytes;
	static uint8_t * p_data;
	p_data = (uint8_t*)p_buff;
	bytes = 0;
	ENABLE_SPI_CS;
	delay_us(10);
    while (( byte_count != bytes ) && (0 != p_buff) )
	{
		spi_byte_write(*p_data);
		bytes++;
		if (byte_count != bytes )
		    p_data++;
		delay_us(1);
	}

   /** TO DO: Disable the CS */
}


/******************************************************************************
* Function : spi_read()
*//**
* \b Description:
*
* This function is used to read bytes of data over spi interface
*
* PRE-CONDITION: Configuration to be done in ucsi_spi_config file
* PRE-CONDITION: The Oscillator should be initialized
* PRE-CONDITION: The GPIO should be intialized
* PRE-CONDITION: SPI should be intialized and enabled
*
* POST-CONDITION: Bytes will be transferred over SPI
*
* @param[in]	p_buff		pointer to buffer of data to be read over SPI
* @param[in]    byte_count	Number of bytes to be read
*
* @return 		None
*
* \b Example Example:
* @code
*  uint8_t byte[20] ;
*  spi_read(byte, 12);
*
* @endcode
*
* @see system_init()
* @see gpio_init()
* @see spi_init()
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 17/03/2020 </td><td> 0.5.0            </td><td> SP      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/
void spi_read ( void * const p_buff, uint16_t byte_count )
{
	static uint16_t bytes;
	static uint8_t * p_data;
	p_data = p_buff;
	bytes = 0;
	ENABLE_SPI_CS;
	while (( byte_count != bytes ) && (0 != p_buff) )
	{
		 p_data[bytes]  = spi_byte_read();
		 bytes++;

	}
	/** TODO: DISABLE_SPI_CS; */
}
/*************** END OF FUNCTIONS ***************************************************************************/
