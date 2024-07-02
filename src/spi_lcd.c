//Driver for the LCD on an ESP32-Wrover-Kit board
/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
// #include "esp_heap_alloc_caps.h"
#include "esp_heap_caps.h"
#include "mpumouse.h"
#include "mouse.h"

#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK  40
#define PIN_NUM_CS   39

#define PIN_NUM_DC   38
#define PIN_NUM_RST  0
#define PIN_NUM_BCKL 45		//backlight enable


/*
 The ILI9341 needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} ili_init_cmd_t;

static const ili_init_cmd_t ili_init_cmds[]={
    {0xCF, {0x00, 0x83, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    {0x36, {0x28}, 1},
    {0x3A, {0x55}, 1},
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4}, 
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

// ======ST7789============================================================
/**
 * @file ili9341.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "st7789.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void st7789_send_cmd(spi_device_handle_t spi, uint8_t cmd);
static void st7789_send_data(spi_device_handle_t spi, void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

static volatile bool spi_trans_in_progress;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void st7789_init(spi_device_handle_t spi)
{
	lcd_init_cmd_t st7789_init_cmds[]={
		{0xCF, {0x00, 0x83, 0X30}, 3},
		{0xED, {0x64, 0x03, 0X12, 0X81}, 4},
		{0xE8, {0x85, 0x01, 0x79}, 3},
		{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
		{0xF7, {0x20}, 1},
		{0xEA, {0x00, 0x00}, 2},
		{0xC0, {0x26}, 1},			/*Power control*/
		{0xC1, {0x11}, 1},			/*Power control */
		{0xC5, {0x35, 0x3E}, 2},	/*VCOM control*/
		{0xC7, {0xBE}, 1},			/*VCOM control*/
		{0x36, {0x28}, 1},			/*Memory Access Control*/
		{0x3A, {0x55}, 1},			/*Pixel Format Set*/
		{0xB1, {0x00, 0x1B}, 2},
		{0xF2, {0x08}, 1},
		{0x26, {0x01}, 1},
		{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
		{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
		{0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
		{0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
		{0x2C, {0}, 0},
		{0xB7, {0x07}, 1},
		{0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
		{0x11, {0}, 0x80},
		{0x29, {0}, 0x80},
		{0, {0}, 0xff},
	};

	//Initialize non-SPI GPIOs
	gpio_set_direction(ST7789_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(ST7789_RST, GPIO_MODE_OUTPUT);
	gpio_set_direction(ST7789_BCKL, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(ST7789_RST, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	gpio_set_level(ST7789_RST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);


	printf("ST7789 initialization.\n");


	//Send all the commands
	uint16_t cmd = 0;
	while (st7789_init_cmds[cmd].databytes!=0xff) {
		st7789_send_cmd(spi, st7789_init_cmds[cmd].cmd);
		st7789_send_data(spi, st7789_init_cmds[cmd].data, st7789_init_cmds[cmd].databytes&0x1F);
		if (st7789_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		cmd++;
	}

	///Enable backlight
	printf("Enable backlight.\n");
	gpio_set_level(ST7789_BCKL, CONFIG_DISP_BCKL_ACTIVE_LVL);
}

/**
 * @brief 发送SPI数据
 * @param data 数据指针，指向要发送的数据
 * @param length 数据长度，以字节为单位
 */
void disp_spi_send_data(spi_device_handle_t spi, uint8_t * data, uint16_t length)
{
    if (length == 0) return;           //no need to send anything

    while(spi_trans_in_progress);

    spi_transaction_t t = {
        .length = length * 8, // transaction length is in bits
        .tx_buffer = data
    };

    spi_trans_in_progress = true;
    // spi_color_sent = false;             //Mark the "lv_flush_ready" NOT needs to be called in "spi_ready"
    spi_device_queue_trans(spi, &t, portMAX_DELAY);

}

static void st7789_send_cmd(spi_device_handle_t spi, uint8_t cmd)
{
	gpio_set_level(ST7789_DC, 0);	 /*Command mode*/
	disp_spi_send_data(spi, &cmd, 1);
}

static void st7789_send_data(spi_device_handle_t spi, void * data, uint16_t length)
{
	gpio_set_level(ST7789_DC, 1);	 /*Data mode*/
	disp_spi_send_data(spi, data, length);
}

// ==========ST7789 END==========

static spi_device_handle_t spi;


//Send a command to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
void ili_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
void ili_data(spi_device_handle_t spi, const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void ili_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

//Initialize the display
void ili_init(spi_device_handle_t spi) 
{
    int cmd=0;
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    //Send all the commands
    while (ili_init_cmds[cmd].databytes!=0xff) {
        ili_cmd(spi, ili_init_cmds[cmd].cmd);
        ili_data(spi, ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
        if (ili_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 0);
}


static void send_header_start(spi_device_handle_t spi, int xpos, int ypos, int w, int h)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[5];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x=0; x<5; x++) {
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
    trans[1].tx_data[0]=xpos>>8;              //Start Col High
    trans[1].tx_data[1]=xpos;              //Start Col Low
    trans[1].tx_data[2]=(xpos+w-1)>>8;       //End Col High
    trans[1].tx_data[3]=(xpos+w-1)&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[3].tx_data[1]=ypos&0xff;      //start page low
    trans[3].tx_data[2]=(ypos+h-1)>>8;    //end page high
    trans[3].tx_data[3]=(ypos+h-1)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;           //memory write

    //Queue all transactions.
    for (x=0; x<5; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}


void send_header_cleanup(spi_device_handle_t spi) 
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 5 transactions to be done and get back the results.
    for (int x=0; x<5; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}


volatile static uint8_t *currFbPtr=NULL;
SemaphoreHandle_t dispSem = NULL;

#define NO_SIM_TRANS 5
#define MEM_PER_TRANS 1024

void IRAM_ATTR displayTask(void *arg) {
	int x, i;
	int idx=0;
	int inProgress=0;
	static uint16_t *dmamem[NO_SIM_TRANS];
	spi_transaction_t trans[NO_SIM_TRANS];
	spi_transaction_t *rtrans;

	esp_err_t ret;
	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1
	};
	spi_device_interface_config_t devcfg={
		.clock_speed_hz=26000000,               //Clock out at 40 MHz. Yes, that's heavily overclocked.
		.mode=0,                                //SPI mode 0
		.spics_io_num=PIN_NUM_CS,               //CS pin
		.queue_size=10,                          //We want to be able to queue 7 transactions at a time
		.pre_cb=ili_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
	};

	printf("*** Display task starting.\n");

	//Initialize the SPI bus
	ret=spi_bus_initialize(SPI3_HOST, &buscfg, 1);
	assert(ret==ESP_OK);
	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
	assert(ret==ESP_OK);
	//Initialize the LCD
	// ili_init(spi);
	st7789_init(spi);

	for (x=0; x<NO_SIM_TRANS; x++) {
		// updated
		// dmamem[x]=pvPortMallocCaps(MEM_PER_TRANS*2, MALLOC_CAP_DMA);
		dmamem[x]=heap_caps_malloc(MEM_PER_TRANS*2, MALLOC_CAP_DMA);
		assert(dmamem[x]);
		memset(&trans[x], 0, sizeof(spi_transaction_t));
		trans[x].length=MEM_PER_TRANS*4;
		trans[x].user=(void*)1;
		trans[x].tx_buffer=&dmamem[x];
	}

	while(1) {
		xSemaphoreTake(dispSem, portMAX_DELAY);
		uint8_t *myData=(uint8_t*)currFbPtr;

		send_header_start(spi, 0, 0, 320, 240);
		send_header_cleanup(spi);
		int xpos=0;
		for (x=0; x<320*240; x+=MEM_PER_TRANS) {
			i=0; 
			while (i<MEM_PER_TRANS) {
				for (int j=0x80; j!=0; j>>=1) {
					dmamem[idx][i++]=(*myData&j)?0:0xffff;
				}
				myData++;
				xpos+=8;
				if (xpos>=320) {
					xpos=0;
					myData+=((512-320)/8);
				}
			}
			trans[idx].length=MEM_PER_TRANS*16;
			trans[idx].user=(void*)1;
			trans[idx].tx_buffer=dmamem[idx];
			ret=spi_device_queue_trans(spi, &trans[idx], portMAX_DELAY);
			assert(ret==ESP_OK);

			idx++;
			if (idx>=NO_SIM_TRANS) idx=0;

			if (inProgress==NO_SIM_TRANS-1) {
				ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
				assert(ret==ESP_OK);
			} else {
				inProgress++;
			}
		}
		while(inProgress) {
			ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
			assert(ret==ESP_OK);
			inProgress--;
		}
	}
}



void dispDraw(uint8_t *mem) {
	int dx, dy, btn;
	currFbPtr=mem;
	xSemaphoreGive(dispSem);
	mpuMouseGetDxDyBtn(&dx, &dy, &btn);
	mouseMove(dx, dy, btn);
}


void dispInit() {
	printf("spi_lcd_init()\n");
    dispSem=xSemaphoreCreateBinary();
#if CONFIG_FREERTOS_UNICORE
	xTaskCreatePinnedToCore(&displayTask, "display", 3000, NULL, 6, NULL, 0);
#else
	xTaskCreatePinnedToCore(&displayTask, "display", 3000, NULL, 6, NULL, 1);
#endif
}
