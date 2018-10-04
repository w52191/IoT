/**
* \file
*
* \brief Sample project code for demonstrating Pervasive Displays 1.44", 1.9", 2", 2.6" and 2.7" EPD
*
* Copyright (c) 2012-2015 Pervasive Displays Inc. All rights reserved.
*
* \page License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel microcontroller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
**/

/**
* \brief Demonstration for toggling between two images on EPD of PDi extension board
* with Atmel SAM D21 Xplained Pro kit
*
* \par Content
* -# Include the ASF header files (through asf.h)
* -# Include Pervasive_Displays_small_EPD.h: EPD definitions
* -# Include image_data.h: image data array
*/
#include <asf.h>
#include "conf_EPD.h"
#include "image_data.h"
#include "main.h"
#include "conf_at25dfx.h"


#define STRING_EOL    "\r\n"
#define STRING_HEADER "wt is dashuaige"

/** UART module for debug. */
static struct usart_module cdc_uart_module;
	

uint8_t frame_cnt = 0; //wt
uint16_t img_cnt = 0;



uint8_t price[7];
//uint8_t price_set = 0;
uint32_t look_up_T4875[10] = {0x10000, 0x101c2, 0x10384, 0x10546, 0x10708, 0x108ca, 0x10a8c, 0x10c4e, 0x10e10, 0x10fd2};
uint32_t look_up_T2438[10] = {0x11194, 0x11206, 0x11278, 0x112ea, 0x1135c, 0x113ce, 0x11440, 0x114b2, 0x11524, 0x11596};
//! [buffers]
//! [driver_instances]
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;



/****************************************************
Function: mask_generator
Params:
1,mask_w, mask width
2,mask_h, mask height
3,mask_x, mask x start position
4,mask_y, mask y start position
5,img_w, image width
6,img_h, image height
7,arr, array to save mask
Note: The entire image that has mask in it has
the same size of the screen

Description:
The function is to generate a mask for partial updating,
any pixels outside the mask will not change.
****************************************************/
uint8_t mask_generator(uint8_t mask_w, uint8_t mask_h, uint8_t mask_x, uint8_t mask_y,
					uint8_t img_w, uint8_t img_h, unsigned char *arr){ //wt 
						
	uint16_t arr_len = img_w * img_h;
	uint8_t mask_end_x = mask_w + mask_x; //end position in each row
	uint8_t mask_end_y = mask_h + mask_y; //end position in each column
	
	if(mask_w > img_w || mask_h > img_h) return 1;
	if((mask_w+mask_x)>img_w || (mask_h+mask_y)>img_h) return 1;
	
	for(uint16_t i=0; i < arr_len;i++){
		uint8_t position_x = i % img_w;		
		uint8_t position_y = i / img_w;			
		if(position_x >= mask_x && position_x < mask_end_x){ //x in mask range
			if(position_y >= mask_y && position_y < mask_end_y){
				arr[i] = 0; //y in mask range
			}
			//else arr[i] = 0xff;
		}
		//else{
			//arr[i] = 0xFF;
		//}
	}
	return 0;
}

/****************************************************
Function: img_synthesize
Params:
1,fg_w, frontground width
2,fg_h, frontground height
3,fg_x, frontground x start position
4,fg_y, frontground y start position
5,bg_w, background width
6,bg_h, background height
7,arr, array to save new img
8,m_image, the smaller image that need to be embedded
Note: The function has the same parameters as mask_generator

****************************************************/
uint8_t img_synthesize(uint8_t fg_w, uint8_t fg_h, uint8_t fg_x, uint8_t fg_y,
					uint8_t bg_w, uint8_t bg_h, unsigned char *arr, unsigned char *m_image){ //wt 
	uint16_t arr_len = bg_w * bg_h;
	uint8_t mask_end_x = fg_w + fg_x; //end position in each row
	uint8_t mask_end_y = fg_h + fg_y; //end position in each column
	
	if(fg_w > bg_w || fg_h > bg_h) return 1;
	if((fg_w+fg_x)>bg_w || (fg_h+fg_y)>bg_h) return 1;
						
	for(uint16_t i=0; i<arr_len; i++){//generate a new image
		uint8_t position_x = i % bg_w;		
		uint8_t position_y = i / bg_w;			
		if(position_x >= fg_x && position_x < mask_end_x){ //x in mask range
			if(position_y >= fg_y && position_y < mask_end_y){
				arr[i] = (*m_image); //y in mask range
				m_image++;
			}
			//else arr[i] = 0xff;
		}
		//else{
			//arr[i] = 0xFF;
		//}
	}
	return 0;						
}

static void at25dfx_init(void)
{
	//! [config_instances]
	struct at25dfx_chip_config at25dfx_chip_config;
	struct spi_config at25dfx_spi_config;
	//! [config_instances]

	//! [spi_setup]
	//at25dfx_spi_get_config_defaults(&at25dfx_spi_config);
	//at25dfx_spi_config.mode_specific.master.baudrate = AT25DFX_CLOCK_SPEED;
	//at25dfx_spi_config.mux_setting = AT25DFX_SPI_PINMUX_SETTING;
	//at25dfx_spi_config.pinmux_pad0 = AT25DFX_SPI_PINMUX_PAD0;
	//at25dfx_spi_config.pinmux_pad1 = AT25DFX_SPI_PINMUX_PAD1;
	//at25dfx_spi_config.pinmux_pad2 = AT25DFX_SPI_PINMUX_PAD2;
	//at25dfx_spi_config.pinmux_pad3 = AT25DFX_SPI_PINMUX_PAD3;

	//spi_init(&at25dfx_spi, AT25DFX_SPI, &at25dfx_spi_config);
	//spi_enable(&at25dfx_spi);
	//spi_init(&spi_master_instance, AT25DFX_SPI, &at25dfx_spi_config);
	//spi_enable(&spi_master_instance);
	
	
	//! [spi_setup]

	//! [chip_setup]
	at25dfx_chip_config.type = AT25DFX_MEM_TYPE;
	at25dfx_chip_config.cs_pin = AT25DFX_CS;

	//at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at25dfx_chip_config);
	at25dfx_chip_init(&at25dfx_chip, &spi_master_instance, &at25dfx_chip_config);
	
	//! [chip_setup]
}


/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}

/**
* \brief The main function will toggle between two images on
* corresponding EPD depends on specified EPD size
*/
int main (void) {
	/* Initialize system clock and SAM D21 Xplained pro board */

	uint8_t dispaly_ctrl = 0;//wt
	uint16_t arr_len = 33*176;//wt
	unsigned char *temp = (unsigned char*) malloc(arr_len);//wt
	unsigned char *temp_img = (unsigned char*) malloc(arr_len);
	 /* Initialize the board. */
    system_init();
	
	configure_console();
	
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &pin_conf);
	
	uint8_t k = 0;
	
	delay_init();
	delay_ms(1000);
	/* Initialize EPD hardware */
	//spi_disable(&at25dfx_spi);
	EPD_display_init();
	at25dfx_init();
	
    system_interrupt_enable_global();
	
	at25dfx_chip_wake(&at25dfx_chip);//! [wake_chip]
	//port_pin_set_output_level(LED_0_PIN, false);
	
	
	
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {//! [check_presence]
		// Handle missing or non-responsive device
	}
	//at25dfx_chip_read_buffer(&at25dfx_chip, 0x0000, read_buffer, AT25DFX_BUFFER_SIZE);//! [read_buffer]
	at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);//! [unprotect_sector]
	at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x11000, false);//! [unprotect_sector]
	//at25dfx_chip_erase_block(&at25dfx_chip, 0x10000, AT25DFX_BLOCK_SIZE_4KB);//! [erase_block]
	//at25dfx_chip_erase_block(&at25dfx_chip, 0x11000, AT25DFX_BLOCK_SIZE_4KB);//! [erase_block]
	//at25dfx_chip_write_buffer(&at25dfx_chip, 0x10000, write_buffer, AT25DFX_BUFFER_SIZE);//! [write_buffer]
	//at25dfx_chip_write_buffer(&at25dfx_chip, look_up_T4875[8], image_array_270_no_0, 450);//! [write_buffer]
	//at25dfx_chip_write_buffer(&at25dfx_chip, look_up_T2438[8], image_array_270_no_1, 114);//! [write_buffer]
	//at25dfx_chip_write_buffer(&at25dfx_chip, look_up_T2438[9], image_array_270_no_2, 114);//! [write_buffer]
	//! [global_protect]
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
	//! [global_protect]

	//! [sleep]
	//wt at25dfx_chip_sleep(&at25dfx_chip);
	//! [sleep]
	//! [use_code]

	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
	
	while (1) {
		
		//if(frame_cnt == 22 || dispaly_ctrl == 0){//wt
		if(1){
					//wifi_data_parsing();
					
					unsigned char *ptr_temp_img;
					if(!dispaly_ctrl){//background only display one time
						for(uint16_t i=0; i < arr_len;i++){//update entire image
							temp[i] = 0x00;
						}
						//printf("Display data\r\n");
						//frame_cnt = 0;
						EPD_display_partial(EPD_270,(uint8_t *)&image_array_270_2,(uint8_t *)&image_array_270_1, (uint8_t *)temp);
						for(uint16_t i=0; i < arr_len;i++){//
							temp[i] = 0xff;
						}
						epd_spi_init();
						dispaly_ctrl++;
						delay_ms(1000);
						
						mask_generator(6, 75, 6, 91,33,176, temp);
						mask_generator(3, 38, 12, 80,33,176, temp);
						mask_generator(3, 38, 15, 80,33,176, temp);
					}
					
					//if(frame_cnt == 22){
					if(1){
						frame_cnt = 0;
						img_cnt = 0;
						/*if(dispaly_ctrl == 1) ptr_temp_img = &image_array_270_no_0[0];
						else if(dispaly_ctrl == 2) ptr_temp_img = &image_array_270_no_1[0];
						else if(dispaly_ctrl == 3) ptr_temp_img = &image_array_270_no_2[0];*/
						for(uint16_t i = 0; i < 450; i++){
							image_array_270_no_0[i] = 0;
						}
						
						at25dfx_chip_read_buffer(&at25dfx_chip, look_up_T4875[dispaly_ctrl], image_array_270_no_0, 450);
						ptr_temp_img = &image_array_270_no_0[0];
						img_synthesize(6, 75, 6, 91,33,176, temp_img, ptr_temp_img);
						
						at25dfx_chip_read_buffer(&at25dfx_chip, look_up_T2438[dispaly_ctrl], image_array_270_no_1, 114);
						ptr_temp_img = &image_array_270_no_1[0];
						img_synthesize(3, 38, 12, 80,33,176, temp_img, ptr_temp_img);
						
						at25dfx_chip_read_buffer(&at25dfx_chip, look_up_T2438[dispaly_ctrl], image_array_270_no_2, 114);
						ptr_temp_img = &image_array_270_no_2[0];
						img_synthesize(3, 38, 15, 80,33,176, temp_img, ptr_temp_img);
						
						EPD_display_partial(EPD_270,(uint8_t *)&image_array_270_1,(uint8_t *)temp_img, (uint8_t *)temp);
						epd_spi_init();
						dispaly_ctrl++;
						if(dispaly_ctrl > 9) dispaly_ctrl = 1;
						delay_ms(1000);
					}
					//data_len = 0;
					//EPD_display_from_pointer(EPD_270,(uint8_t *)&image_array_270_2,(uint8_t *)&image_array_270_1);
				}
	}
	free(temp);
	free(temp_img);
}


/**
 * \page - Quick Start Guide
 *
 * This is the quick start guide for the EPD Xplained Pro extension board made by Pervasive Displays Inc.
 * with its small size EPDs on how to setup the kit for Atmel Xplained Pro.
 * The code example in main.c provides the demo of toggling between two images from predefined image array.
 *
 * \note
 * - Released Date: 10 October, 2015.  Version: 1.0 (based on PDI's Released Date: 18 May, 2015.  Version: 2.02 for SAMD21)
 * - Compiled by Atmel Studio ver.6.0.594 with ASF ver.3.27.0
 * - PDi = Pervasive Displays Inc.(PDi) http://www.pervasivedisplays.com
 * - EPD = Electronic Paper Display (Electrophoretic Display)
 * - COG = Chip on Glass, the driver IC on EPD module
 * - COG G1 or G2: G is for generation.
 * - FPL = Front Plane Laminate which is E-Ink material film.
 *   There are Vizplex(V110, EOL already), Aurora Ma(V230) and Aurora Mb(V231) type
 * - PDi offers Aurora_Ma and Aurora_Mb material with G2 COG to the market.
 *   Some customers got our Aurora_Mb+G1 combination. The sample code is also provided.
 * - Basically, the Aurora_Mb+G1 is the replacement for Vizplex+G1.
 * - How to identify the FPL material type of your displays, please visit
 *   http://www.pervasivedisplays.com/products/label_info
 * - For driving PDi's small size EPDs, please read the "COG Driver Interface
 *   Timing" document(hereinafter COG Document) first. It explains the interface
 *   to the COG driver of EPD for a MCU based solution.
 * - COG Document no.: 4P008-00 (for Vizplex+G1)  : http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=138408
 * - COG Document no.: 4P015-00 (for Aurora_Ma+G2): http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=198794
 * - COG Document no.: 4P016-00 (for Aurora_Mb+G1): http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=220874
 * - COG Document no.: 4P018-00 (for Aurora_Mb+G2): http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=220873
 * - This project code supports EPD size: 1.44", 1.9", 2", 2.6" and 2.7"
 * - Supports Atmel Xplained PRO: SAM D21 Xplained PRO
 *
 * \section File_Explanation
 * - <b>image_data:</b>\n
 *   It defines the image arrays of each EPD size. 
 * - <b>conf_EPD.h:</b> (under [config] folder) The EPD configurations.\n
 *   -# USE_EPD_Type: define the demo size of EPD you are connecting
 *   -# Gx_Aurora_xx: define which FPL material with COG driving waveform of the EPD you're connecting
 *   -# COG_SPI_baudrate: SPI speed, G1 works in 4-12MHz, G2 works in 4-20MHz
 * - <b>Pervasive_Displays_small_EPD</b> folder:\n
 *   All of the COG driving waveforms are located in this folder. Logically developer
 *   doesn't need to change the codes in this folder in order to keep correct driving
 *   to the EPDs.\n\n
 *   <b><em>Software architecture:</em></b>\n
 *   [Application (ex. EPD Kit Tool)] <-- [COG interface (<em>EPD_interface</em>)] <--
 *   [COG driving process (<em>EPD_Gx_Aurora_Mx</em> in COG_FPL folder)] <--
 *   [Hardware Driver & GPIO (<em>EPD_hardware_driver</em>)]\n\n
 *    -# <b>EPD_hardware_driver:</b>\n
 *       Most of the COG hardware initialization, GPIO and configuration. User can implement
 *       the driver layer of EPD if some variables need to be adjusted. The provided
 *       settings and functions are Timer, SPI, PWM, temperature and EPD hardware initialization.
 *    -# <b>COG_FPL</b> folder:\n
 *       The driving process for each sub-folder represents the different display module.
 *       - <b>EPD_UpdateMethod_Gx_Aurora_Mx:</b>\n
 *         UpdateMethod: Global update or Partial update. If none, it means both.
 *         Gx: G1 or G2.
 *         Aurora_Mx: Aurora_Ma or Aurora_Mb.
 *    -# <b>EPD_interface:</b>\n
 *       The application interfaces to work with EPD.
 *
 *
 * \section Use_Case
 * -# <b>EPD_display_from_pointer</b>: Load two image data arrays from image_data.c
 *   according to predefined EPD size.
 * -# <b>EPD_display_from_flash</b>:
 *   Load stored image data from flash memory according for predefined EPD size. User
 *   must convert 1 bit bitmap image file to hex data in advance and store in flash
 *   memory. Image converting tool can be downloaded at http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=214756
 *
 * \section Steps
 * -# Ensure the EPD is connected correctly on the EPD Xplained Pro extension board
 * -# Connect the EPD Xplained Pro to the SAMD21 Xplained Pro header marked EXT1
 * -# Connect the SAM D21 Xplained Pro to computer's USB port via USB cable
 * -# Find #define Gx_Aurora_Mx in "conf_EPD.h" file. Change to the correct type of EPD you are connecting.
 * -# Change the USE_EPDXXX to the correct size.
 * -# Close the J2 jumper on board if you are connecting with 1.44" or 2"
 * -# Start debugging to program the project code to Atmel Kit. The EPD will show
 *    two images change alternately every 10 seconds (default).
 *
 *
 * * \section EPD Xplained Pro extension header pins
 * ================================================================
 * |Pin| Function       | Description                             |
 * |---|----------------|-----------------------------------------|
 * | 1 | ID             | Communication line to ID chip           |
 * | 2 | GND            | Ground                                  |
 * | 3 | Temperature    | On board temperature sensor output (ADC)|
 * | 4 | BORDER_CONTROL | Border control pin (GPIO)               |
 * | 5 | DISCHARGE      | EPD discharge when EPD power off (GPIO) |
 * | 6 | /RESET         | Reset signal. Low enable (GPIO)         |
 * | 7 | PWM            | Square wave when EPD power on (PWM)     |
 * | 8 | PANEL_ON       | COG driver power control pin (GPIO)     |
 * | 9 | BUSY           | COG busy pin (GPIO)                     |
 * |10 | FLASH_CS       | On board flash chip select (GPIO)       |
 * |11 |                |                                         |
 * |12 |                |                                         |
 * |13 |                |                                         |
 * |14 |                |                                         |
 * |15 | /EPD_CS        | Chip Select. Low enable (GPIO)          |
 * |16 | SPI_MOSI       | Serial input from host MCU to EPD       |
 * |17 | SPI_MISO       | Serial output from EPD to host MCU      |
 * |18 | SPI_CLK        | Clock for SPI                           |
 * |19 | GND            | Ground                                  |
 * |20 | VCC            | Target supply voltage                   |
 *
 *
 * \section PDi EPD displays
 * ======================================
 * | Size | PDi Model  |   FPL + COG    |
 * |------|------------|----------------|
 * | 1.44 | EK014BS011 | Aurora_Ma + G2 |
 * | 2.0  | EG020BS011 | Aurora_Ma + G2 |
 * | 2.7  | EM027BS013 | Aurora_Ma + G2 |
 * | 1.44 | EK014CS011 | Aurora_Mb + G1 |
 * | 1.9  | EB019CS011 | Aurora Mb + G1 |
 * | 2.0  | EG020CS012 | Aurora_Mb + G1 |
 * | 2.6  | EN026CS011 | Aurora Mb + G1 |
 * | 2.7  | EM027CS011 | Aurora_Mb + G1 |
 * | 1.44 | E1144CS021 | Aurora Mb + G2 |
 * | 1.9  | E1190CS021 | Aurora Mb + G2 |
 * | 2.0  | E1200CS021 | Aurora Mb + G2 |
 * | 2.6  | E1260CS021 | Aurora Mb + G2 |
 * | 2.7  | E1271CS021 | Aurora Mb + G2 |
 *
 * http://www.pervasivedisplays.com/products/label_info
 */