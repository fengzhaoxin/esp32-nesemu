// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #include <string.h>
// #include <stdio.h>
// #include "sdkconfig.h"
// #include "rom/ets_sys.h"
// #include "rom/gpio.h"
// #include "soc/gpio_reg.h"
// #include "soc/gpio_sig_map.h"
// #include "soc/gpio_struct.h"
// #include "soc/io_mux_reg.h"
// #include "soc/spi_reg.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/periph_ctrl.h"
// #include "spi_lcd.h"

// #define PIN_NUM_MISO 25
// #define PIN_NUM_MOSI 23
// #define PIN_NUM_CLK  19
// #define PIN_NUM_CS   22
// #define PIN_NUM_DC   21
// #define PIN_NUM_RST  18
// #define PIN_NUM_BCKL 5
// #define LCD_SEL_CMD()   GPIO.out_w1tc = (1 << PIN_NUM_DC) // Low to send command 
// #define LCD_SEL_DATA()  GPIO.out_w1ts = (1 << PIN_NUM_DC) // High to send data
// #define LCD_RST_SET()   GPIO.out_w1ts = (1 << PIN_NUM_RST) 
// #define LCD_RST_CLR()   GPIO.out_w1tc = (1 << PIN_NUM_RST)

// #if CONFIG_HW_INV_BL
// #define LCD_BKG_ON()    GPIO.out_w1tc = (1 << PIN_NUM_BCKL) // Backlight ON
// #define LCD_BKG_OFF()   GPIO.out_w1ts = (1 << PIN_NUM_BCKL) //Backlight OFF
// #else
// #define LCD_BKG_ON()    GPIO.out_w1ts = (1 << PIN_NUM_BCKL) // Backlight ON
// #define LCD_BKG_OFF()   GPIO.out_w1tc = (1 << PIN_NUM_BCKL) //Backlight OFF
// #endif

// #define SPI_NUM  0x3

// #define LCD_TYPE_ILI 0
// #define LCD_TYPE_ST 1


// static void spi_write_byte(const uint8_t data){
//     SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 0x7, SPI_USR_MOSI_DBITLEN_S);
//     WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), data);
//     SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
//     while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
// }

// static void LCD_WriteCommand(const uint8_t cmd)
// {
//     LCD_SEL_CMD();
//     spi_write_byte(cmd);
// }

// static void LCD_WriteData(const uint8_t data)
// {
//     LCD_SEL_DATA();
//     spi_write_byte(data);
// }

// static void  ILI9341_INITIAL ()
// {
//     LCD_BKG_ON();
//     //------------------------------------Reset Sequence-----------------------------------------//

//     LCD_RST_SET();
//     ets_delay_us(100000);                                                              

//     LCD_RST_CLR();
//     ets_delay_us(200000);                                                              

//     LCD_RST_SET();
//     ets_delay_us(200000);                                                             


// #if (CONFIG_HW_LCD_TYPE == LCD_TYPE_ILI)
//     //************* Start Initial Sequence **********//
//     LCD_WriteCommand(0xCF);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x83);
//     LCD_WriteData(0X30);

//     LCD_WriteCommand(0xED);
//     LCD_WriteData(0x64);
//     LCD_WriteData(0x03);
//     LCD_WriteData(0X12);
//     LCD_WriteData(0X81);

//     LCD_WriteCommand(0xE8);
//     LCD_WriteData(0x85);
//     LCD_WriteData(0x01); //i
//     LCD_WriteData(0x79); //i

//     LCD_WriteCommand(0xCB);
//     LCD_WriteData(0x39);
//     LCD_WriteData(0x2C);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x34);
//     LCD_WriteData(0x02);

//     LCD_WriteCommand(0xF7);
//     LCD_WriteData(0x20);

//     LCD_WriteCommand(0xEA);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x00);

//     LCD_WriteCommand(0xC0);    //Power control
//     LCD_WriteData(0x26); //i  //VRH[5:0]

//     LCD_WriteCommand(0xC1);    //Power control
//     LCD_WriteData(0x11);   //i //SAP[2:0];BT[3:0]

//     LCD_WriteCommand(0xC5);    //VCM control
//     LCD_WriteData(0x35); //i
//     LCD_WriteData(0x3E); //i

//     LCD_WriteCommand(0xC7);    //VCM control2
//     LCD_WriteData(0xBE); //i   //»òÕß B1h

//     LCD_WriteCommand(0x36);    // Memory Access Control
//     LCD_WriteData(0x28); //i //was 0x48

//     LCD_WriteCommand(0x3A);
//     LCD_WriteData(0x55);

//     LCD_WriteCommand(0xB1);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x1B); //18
    
//     LCD_WriteCommand(0xF2);    // 3Gamma Function Disable
//     LCD_WriteData(0x08);

//     LCD_WriteCommand(0x26);    //Gamma curve selected
//     LCD_WriteData(0x01);
        
//     LCD_WriteCommand(0xE0);    //Set Gamma
//     LCD_WriteData(0x1F);
//     LCD_WriteData(0x1A);
//     LCD_WriteData(0x18);
//     LCD_WriteData(0x0A);
//     LCD_WriteData(0x0F);
//     LCD_WriteData(0x06);
//     LCD_WriteData(0x45);
//     LCD_WriteData(0X87);
//     LCD_WriteData(0x32);
//     LCD_WriteData(0x0A);
//     LCD_WriteData(0x07);
//     LCD_WriteData(0x02);
//     LCD_WriteData(0x07);
//     LCD_WriteData(0x05);
//     LCD_WriteData(0x00);
 
//     LCD_WriteCommand(0XE1);    //Set Gamma
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x25);
//     LCD_WriteData(0x27);
//     LCD_WriteData(0x05);
//     LCD_WriteData(0x10);
//     LCD_WriteData(0x09);
//     LCD_WriteData(0x3A);
//     LCD_WriteData(0x78);
//     LCD_WriteData(0x4D);
//     LCD_WriteData(0x05);
//     LCD_WriteData(0x18);
//     LCD_WriteData(0x0D);
//     LCD_WriteData(0x38);
//     LCD_WriteData(0x3A);
//     LCD_WriteData(0x1F);

//     LCD_WriteCommand(0x2A);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0xEF);

//     LCD_WriteCommand(0x2B);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x01);
//     LCD_WriteData(0x3f);
//     LCD_WriteCommand(0x2C);
    
//     LCD_WriteCommand(0xB7); 
//     LCD_WriteData(0x07); 
    
//     LCD_WriteCommand(0xB6);    // Display Function Control
//     LCD_WriteData(0x0A); //8 82 27
//     LCD_WriteData(0x82);
//     LCD_WriteData(0x27);
//     LCD_WriteData(0x00);

//     //LCD_WriteCommand(0xF6); //not there
//     //LCD_WriteData(0x01);
//     //LCD_WriteData(0x30);

// #endif
// #if (CONFIG_HW_LCD_TYPE == LCD_TYPE_ST)

// //212
// //122
//     LCD_WriteCommand(0x36);
//     LCD_WriteData((1<<5)|(1<<6)); //MV 1, MX 1

//     LCD_WriteCommand(0x3A);
//     LCD_WriteData(0x55);

//     LCD_WriteCommand(0xB2);
//     LCD_WriteData(0x0c);
//     LCD_WriteData(0x0c);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x33);
//     LCD_WriteData(0x33);

//     LCD_WriteCommand(0xB7);
//     LCD_WriteData(0x35);

//     LCD_WriteCommand(0xBB);
//     LCD_WriteData(0x2B);

//     LCD_WriteCommand(0xC0);
//     LCD_WriteData(0x2C);

//     LCD_WriteCommand(0xC2);
//     LCD_WriteData(0x01);
//     LCD_WriteData(0xFF);

//     LCD_WriteCommand(0xC3);
//     LCD_WriteData(0x11);

//     LCD_WriteCommand(0xC4);
//     LCD_WriteData(0x20);

//     LCD_WriteCommand(0xC6);
//     LCD_WriteData(0x0f);

//     LCD_WriteCommand(0xD0);
//     LCD_WriteData(0xA4);
//     LCD_WriteData(0xA1);

//     LCD_WriteCommand(0xE0);
//     LCD_WriteData(0xD0);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x05);
//     LCD_WriteData(0x0E);
//     LCD_WriteData(0x15);
//     LCD_WriteData(0x0D);
//     LCD_WriteData(0x37);
//     LCD_WriteData(0x43);
//     LCD_WriteData(0x47);
//     LCD_WriteData(0x09);
//     LCD_WriteData(0x15);
//     LCD_WriteData(0x12);
//     LCD_WriteData(0x16);
//     LCD_WriteData(0x19);

//     LCD_WriteCommand(0xE1);
//     LCD_WriteData(0xD0);
//     LCD_WriteData(0x00);
//     LCD_WriteData(0x05);
//     LCD_WriteData(0x0D);
//     LCD_WriteData(0x0C);
//     LCD_WriteData(0x06);
//     LCD_WriteData(0x2D);
//     LCD_WriteData(0x44);
//     LCD_WriteData(0x40);
//     LCD_WriteData(0x0E);
//     LCD_WriteData(0x1C);
//     LCD_WriteData(0x18);
//     LCD_WriteData(0x16);
//     LCD_WriteData(0x19);

// #endif


//     LCD_WriteCommand(0x11);    //Exit Sleep
//     ets_delay_us(100000);
//     LCD_WriteCommand(0x29);    //Display on
//     ets_delay_us(100000);


// }
// //.............LCD API END----------

// static void ili_gpio_init()
// {
//     PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO21_U,2);   //DC PIN
//     PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO18_U,2);   //RESET PIN
//     PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,2);    //BKL PIN
//     WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, BIT21|BIT18|BIT5);
// }

// static void spi_master_init()
// {
//     periph_module_enable(PERIPH_VSPI_MODULE);
//     periph_module_enable(PERIPH_SPI_DMA_MODULE);

//     ets_printf("lcd spi pin mux init ...\r\n");
//     PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO19_U,2);
//     PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO23_U,2);
//     PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO22_U,2);
//     PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO25_U,2);
//     WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, BIT19|BIT23|BIT22);

//     ets_printf("lcd spi signal init\r\n");
//     gpio_matrix_in(PIN_NUM_MISO, VSPIQ_IN_IDX,0);
//     gpio_matrix_out(PIN_NUM_MOSI, VSPID_OUT_IDX,0,0);
//     gpio_matrix_out(PIN_NUM_CLK, VSPICLK_OUT_IDX,0,0);
//     gpio_matrix_out(PIN_NUM_CS, VSPICS0_OUT_IDX,0,0);
//     ets_printf("Hspi config\r\n");

//     CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_TRANS_DONE << 5);
//     SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP);
//     CLEAR_PERI_REG_MASK(SPI_PIN_REG(SPI_NUM), SPI_CK_IDLE_EDGE);
//     CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM),  SPI_CK_OUT_EDGE);
//     CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_WR_BIT_ORDER);
//     CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_RD_BIT_ORDER);
//     CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_DOUTDIN);
//     WRITE_PERI_REG(SPI_USER1_REG(SPI_NUM), 0);
//     SET_PERI_REG_BITS(SPI_CTRL2_REG(SPI_NUM), SPI_MISO_DELAY_MODE, 0, SPI_MISO_DELAY_MODE_S);
//     CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_SLAVE_MODE);
    
//     WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), (1 << SPI_CLKCNT_N_S) | (1 << SPI_CLKCNT_L_S));//40MHz
//     //WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), SPI_CLK_EQU_SYSCLK); // 80Mhz
    
//     SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP | SPI_CS_HOLD | SPI_USR_MOSI);
//     SET_PERI_REG_MASK(SPI_CTRL2_REG(SPI_NUM), ((0x4 & SPI_MISO_DELAY_NUM) << SPI_MISO_DELAY_NUM_S));
//     CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_COMMAND);
//     SET_PERI_REG_BITS(SPI_USER2_REG(SPI_NUM), SPI_USR_COMMAND_BITLEN, 0, SPI_USR_COMMAND_BITLEN_S);
//     CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_ADDR);
//     SET_PERI_REG_BITS(SPI_USER1_REG(SPI_NUM), SPI_USR_ADDR_BITLEN, 0, SPI_USR_ADDR_BITLEN_S);
//     CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MISO);
//     SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MOSI);
//     char i;
//     for (i = 0; i < 16; ++i) {
//         WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), 0);
//     }
// }

// #define U16x2toU32(m,l) ((((uint32_t)(l>>8|(l&0xFF)<<8))<<16)|(m>>8|(m&0xFF)<<8))

// extern uint16_t myPalette[];

// void ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t * data[]){
//     int x, y;
//     int i;
//     uint16_t x1, y1;
//     uint32_t xv, yv, dc;
//     uint32_t temp[16];
//     dc = (1 << PIN_NUM_DC);
    
//     for (y=0; y<height; y++) {
//         //start line
//         x1 = xs+(width-1);
//         y1 = ys+y+(height-1);
//         xv = U16x2toU32(xs,x1);
//         yv = U16x2toU32((ys+y),y1);
        
//         while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
//         GPIO.out_w1tc = dc;
//         SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
//         WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2A);
//         SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
//         while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
//         GPIO.out_w1ts = dc;
//         SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
//         WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), xv);
//         SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
//         while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
//         GPIO.out_w1tc = dc;
//         SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
//         WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2B);
//         SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
//         while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
//         GPIO.out_w1ts = dc;
//         SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
//         WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), yv);
//         SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
//         while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
//         GPIO.out_w1tc = dc;
//         SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
//         WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2C);
//         SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
//         while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        
//         x = 0;
//         GPIO.out_w1ts = dc;
//         SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);
//         while (x<width) {
//             for (i=0; i<16; i++) {
//                 if(data == NULL){
//                     temp[i] = 0;
//                     x += 2;
//                     continue;
//                 }
//                 x1 = myPalette[(unsigned char)(data[y][x])]; x++;
//                 y1 = myPalette[(unsigned char)(data[y][x])]; x++;
//                 temp[i] = U16x2toU32(x1,y1);
//             }
//             while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
//             for (i=0; i<16; i++) {
//                 WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), temp[i]);
//             }
//             SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
//         }
//     }
//     while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
// }

// void ili9341_init()
// {
//     spi_master_init();
//     ili_gpio_init();
//     ILI9341_INITIAL ();
// }





#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_lcd.h"

#define LCD_HOST    SPI2_HOST

#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK  36
#define PIN_NUM_CS   45

#define PIN_NUM_DC   4
#define PIN_NUM_RST  5
#define PIN_NUM_BCKL 6

static spi_device_handle_t spi;

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16 //each (16*2) datas will be sent together

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, {0x00, 0x83, 0X30}, 3},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    {0xE8, {0x85, 0x01, 0x79}, 3},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, {0x20}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    {0xC0, {0x26}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, {0x11}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, {0x35, 0x3E}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, {0xBE}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0x28}, 1},
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, {0x00, 0x1B}, 2},
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    /* Negative gamma correction */
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
static void send_lines(const uint16_t x0, const uint16_t x1, const uint16_t y0, const uint16_t y1, const uint16_t width, uint16_t *linedata)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=x0>>8;              //Start Col High
    trans[1].tx_data[1]=x0&0xff;              //Start Col Low
    trans[1].tx_data[2]=x1>>8;       //End Col High
    trans[1].tx_data[3]=x1&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=y0>>8;        //Start page high
    trans[3].tx_data[1]=y0&0xff;      //start page low
    trans[3].tx_data[2]=(y1)>>8;    //end page high
    trans[3].tx_data[3]=(y1)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;           //memory write
    trans[5].tx_buffer=linedata;        //finally send the line data
    trans[5].length=width*8*PARALLEL_LINES;          //Data length, in bits
    trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x=0; x<6; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}



static void send_line_finish(void)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}


static void  lcd_init ()
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL<<PIN_NUM_DC) | (1ULL<<PIN_NUM_RST) | (1ULL<<PIN_NUM_BCKL));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = true;
    gpio_config(&io_conf);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    printf("LCD ILI9341 initialization.\n");
    lcd_init_cmds = ili_init_cmds;


    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 1);
}

static void spi_master_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=SPI_MASTER_FREQ_80M,//10*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

extern uint16_t myPalette[];

void ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t * data[]){
    int x, y, t_i;
    int i;
    uint16_t t1, t2, x1, y1;
    // uint16_t *temp = heap_caps_malloc(width*PARALLEL_LINES*sizeof(uint16_t), MALLOC_CAP_DMA);
    uint16_t *temp = malloc(width*PARALLEL_LINES*sizeof(uint16_t));
    int first_sending=1;

    for (y=0; y<height; y++) {
        x1 = xs+(width-1);
        y1 = ys+y+(height-1);
        //Calculate a line.
        x = 0;
        t_i=0;
        while (x<width) {
            for (i=0; i<PARALLEL_LINES; i++) {
                if(data == NULL){
                    temp[t_i++] = 0;
                    temp[t_i++] = 0;
                    x += 2;
                    continue;
                }
                t1 = myPalette[(unsigned char)(data[y][x])]; x++;
                t2 = myPalette[(unsigned char)(data[y][x])]; x++;
                temp[t_i++] = t1;
                temp[t_i++] = t2;
            }
        }

        //Finish up the sending process of the previous line, if any
        if (first_sending!=1) send_line_finish();
        first_sending=0;
        //Send the line we currently calculated.
        send_lines(xs, x1, ys+y, y1, width, temp);
        //The line set is queued up for sending now; the actual sending happens in the
        //background. We can go on to calculate the next line set as long as we do not
        //touch line[sending_line]; the SPI sending process is still reading from that.
    }
    // heap_caps_free(temp);
    free(temp);
}

void ili9341_init()
{
    spi_master_init();
    lcd_init ();
}