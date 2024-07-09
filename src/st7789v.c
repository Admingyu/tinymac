#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "st7789v.h"
#include "spi_lcd.h"

#define TAG "ST7789"
#define	_DEBUG_ 0

#if 0
#ifdef CONFIG_IDF_TARGET_ESP32
#define LCD_HOST HSPI_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define LCD_HOST SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S3
#define LCD_HOST SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define LCD_HOST SPI2_HOST
#endif
#endif

#define LCD_HOST SPI3_HOST

#define SPI_DEFAULT_FREQUENCY SPI_MASTER_FREQ_20M; // 20MHz

static const int SPI_Command_Mode = 0;
static const int SPI_Data_Mode = 1;
//static const int SPI_Frequency = SPI_MASTER_FREQ_20M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_26M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_40M;
//static const int SPI_Frequency = 60000000;
//static const int SPI_Frequency = SPI_MASTER_FREQ_80M;

int clock_speed_hz = SPI_DEFAULT_FREQUENCY;

void spi_clock_speed(int speed) {
    ESP_LOGI(TAG, "SPI clock speed=%d MHz", speed/1000000);
    clock_speed_hz = speed;
}

void spi_master_init(TFT_t * dev, int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS, int16_t GPIO_DC, int16_t GPIO_RESET, int16_t GPIO_BL)
{
	esp_err_t ret;

	ESP_LOGI(TAG, "GPIO_CS=%d",GPIO_CS);
	if ( GPIO_CS >= 0 ) {
		//gpio_pad_select_gpio( GPIO_CS );
		gpio_reset_pin( GPIO_CS );
		gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_CS, 0 );
	}

	ESP_LOGI(TAG, "GPIO_DC=%d",GPIO_DC);
	//gpio_pad_select_gpio( GPIO_DC );
	gpio_reset_pin( GPIO_DC );
	gpio_set_direction( GPIO_DC, GPIO_MODE_OUTPUT );
	gpio_set_level( GPIO_DC, 0 );

	ESP_LOGI(TAG, "GPIO_RESET=%d",GPIO_RESET);
	if ( GPIO_RESET >= 0 ) {
		//gpio_pad_select_gpio( GPIO_RESET );
		gpio_reset_pin( GPIO_RESET );
		gpio_set_direction( GPIO_RESET, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_RESET, 1 );
		delayMS(100);
		gpio_set_level( GPIO_RESET, 0 );
		delayMS(100);
		gpio_set_level( GPIO_RESET, 1 );
		delayMS(100);
	}

	ESP_LOGI(TAG, "GPIO_BL=%d",GPIO_BL);
	if ( GPIO_BL >= 0 ) {
		//gpio_pad_select_gpio(GPIO_BL);
		gpio_reset_pin(GPIO_BL);
		gpio_set_direction( GPIO_BL, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_BL, 0 );
	}

	ESP_LOGI(TAG, "GPIO_MOSI=%d",GPIO_MOSI);
	ESP_LOGI(TAG, "GPIO_SCLK=%d",GPIO_SCLK);
	spi_bus_config_t buscfg = {
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_NUM_NC,
		.sclk_io_num = GPIO_SCLK,
		.quadwp_io_num = GPIO_NUM_NC,
		.quadhd_io_num = GPIO_NUM_NC,
		// .max_transfer_sz = 0,
		// .flags = 0
	};

	ret = spi_bus_initialize( LCD_HOST, &buscfg, SPI_DMA_CH_AUTO );
	ESP_LOGD(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg));
	//devcfg.clock_speed_hz = SPI_Frequency;
	devcfg.clock_speed_hz = clock_speed_hz;
	devcfg.queue_size = 10;
	devcfg.mode = 0;
	// devcfg.mode = 3;
	devcfg.pre_cb = ili_spi_pre_transfer_callback;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

	if ( GPIO_CS >= 0 ) {
		devcfg.spics_io_num = GPIO_CS;
	} else {
		devcfg.spics_io_num = -1;
	}
	
	spi_device_handle_t handle;
	ret = spi_bus_add_device( LCD_HOST, &devcfg, &handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	dev->_dc = GPIO_DC;
	dev->_bl = GPIO_BL;
	dev->_SPIHandle = handle;
}

bool spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t* Data, size_t DataLength)
{
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;
		ret = spi_device_transmit( SPIHandle, &SPITransaction );
		assert(ret==ESP_OK); 
	}
	return true;
}

bool spi_master_write_command(TFT_t * dev, uint8_t cmd)
{
	static uint8_t Byte = 0;
	Byte = cmd;
	gpio_set_level( dev->_dc, SPI_Command_Mode );
	return spi_master_write_byte( dev->_SPIHandle, &Byte, 1 );
}

bool spi_master_write_data_byte(TFT_t * dev, uint8_t data)
{
	static uint8_t Byte = 0;
	Byte = data;
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, &Byte, 1 );
}


bool spi_master_write_data_word(TFT_t * dev, uint16_t data)
{
	static uint8_t Byte[2];
	Byte[0] = (data >> 8) & 0xFF;
	Byte[1] = data & 0xFF;
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, 2);
}

bool spi_master_write_addr(TFT_t * dev, uint16_t addr1, uint16_t addr2)
{
	static uint8_t Byte[4];
	Byte[0] = (addr1 >> 8) & 0xFF;
	Byte[1] = addr1 & 0xFF;
	Byte[2] = (addr2 >> 8) & 0xFF;
	Byte[3] = addr2 & 0xFF;
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, 4);
}

bool spi_master_write_color(TFT_t * dev, uint16_t color, uint16_t size)
{
	static uint8_t Byte[1024];
	int index = 0;
	for(int i=0;i<size;i++) {
		Byte[index++] = (color >> 8) & 0xFF;
		Byte[index++] = color & 0xFF;
	}
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, size*2);
}

// Add 202001
bool spi_master_write_colors(TFT_t * dev, uint16_t * colors, uint16_t size)
{
	static uint8_t Byte[1024];
	int index = 0;
	for(int i=0;i<size;i++) {
		Byte[index++] = (colors[i] >> 8) & 0xFF;
		Byte[index++] = colors[i] & 0xFF;
	}
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, size*2);
}

void delayMS(int ms) {
	int _ms = ms + (portTICK_PERIOD_MS - 1);
	TickType_t xTicksToDelay = _ms / portTICK_PERIOD_MS;
	ESP_LOGD(TAG, "ms=%d _ms=%d portTICK_PERIOD_MS=%"PRIu32" xTicksToDelay=%"PRIu32,ms,_ms,portTICK_PERIOD_MS,xTicksToDelay);
	vTaskDelay(xTicksToDelay);
}


void lcdInit(TFT_t * dev, int width, int height, int offsetx, int offsety)
{
	dev->_width = width;
	dev->_height = height;
	dev->_offsetx = offsetx;
	dev->_offsety = offsety;
	dev->_font_direction = DIRECTION0;
	dev->_font_fill = false;
	dev->_font_underline = false;

	spi_master_write_command(dev, 0x01);	//Software Reset
	delayMS(150);

	spi_master_write_command(dev, 0x11);	//Sleep Out
	delayMS(255);
	
	spi_master_write_command(dev, 0x3A);	//Interface Pixel Format
	spi_master_write_data_byte(dev, 0x55);
	delayMS(10);
	
	spi_master_write_command(dev, 0x36);	//Memory Data Access Control
	spi_master_write_data_byte(dev, 0x00);

	spi_master_write_command(dev, 0x2A);	//Column Address Set
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0xF0);

	spi_master_write_command(dev, 0x2B);	//Row Address Set
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0xF0);

	spi_master_write_command(dev, 0x21);	//Display Inversion On
	delayMS(10);

	spi_master_write_command(dev, 0x13);	//Normal Display Mode On
	delayMS(10);

	spi_master_write_command(dev, 0x29);	//Display ON
	delayMS(255);

	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 1 );
	}

	dev->_use_frame_buffer = false;
#if CONFIG_FRAME_BUFFER
	dev->_frame_buffer = heap_caps_malloc(sizeof(uint16_t)*width*height, MALLOC_CAP_DMA);
	if (dev->_frame_buffer == NULL) {
		ESP_LOGE(TAG, "heap_caps_malloc fail");
	} else {
		ESP_LOGI(TAG, "heap_caps_malloc success");
		dev->_use_frame_buffer = true;
	}

#endif
}


// Display OFF
void lcdDisplayOff(TFT_t * dev) {
	spi_master_write_command(dev, 0x28);	// Display off
}
 
// Display ON
void lcdDisplayOn(TFT_t * dev) {
	spi_master_write_command(dev, 0x29);	// Display on
}

// Draw pixel
// x:X coordinate
// y:Y coordinate
// color:color
void lcdDrawPixel(TFT_t * dev, uint16_t x, uint16_t y, uint16_t color){
	if (x >= dev->_width) return;
	if (y >= dev->_height) return;

	if (dev->_use_frame_buffer) {
		dev->_frame_buffer[y*dev->_width+x] = color;
	} else {
		uint16_t _x = x + dev->_offsetx;
		uint16_t _y = y + dev->_offsety;

		spi_master_write_command(dev, 0x2A);	// set column(x) address
		spi_master_write_addr(dev, _x, _x);
		spi_master_write_command(dev, 0x2B);	// set Page(y) address
		spi_master_write_addr(dev, _y, _y);
		spi_master_write_command(dev, 0x2C);	// Memory Write
		//spi_master_write_data_word(dev, color);
		spi_master_write_colors(dev, &color, 1);
	}
}

// Draw line
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// color:color 
void lcdDrawLine(TFT_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	int i;
	int dx,dy;
	int sx,sy;
	int E;

	/* distance between two points */
	dx = ( x2 > x1 ) ? x2 - x1 : x1 - x2;
	dy = ( y2 > y1 ) ? y2 - y1 : y1 - y2;

	/* direction of two point */
	sx = ( x2 > x1 ) ? 1 : -1;
	sy = ( y2 > y1 ) ? 1 : -1;

	/* inclination < 1 */
	if ( dx > dy ) {
		E = -dx;
		for ( i = 0 ; i <= dx ; i++ ) {
			lcdDrawPixel(dev, x1, y1, color);
			x1 += sx;
			E += 2 * dy;
			if ( E >= 0 ) {
			y1 += sy;
			E -= 2 * dx;
		}
	}

	/* inclination >= 1 */
	} else {
		E = -dy;
		for ( i = 0 ; i <= dy ; i++ ) {
			lcdDrawPixel(dev, x1, y1, color);
			y1 += sy;
			E += 2 * dx;
			if ( E >= 0 ) {
				x1 += sx;
				E -= 2 * dy;
			}
		}
	}
}

// Backlight OFF
void lcdBacklightOff(TFT_t * dev) {
	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 0 );
	}
}

// Backlight ON
void lcdBacklightOn(TFT_t * dev) {
	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 1 );
	}
}

// Display Inversion Off
void lcdInversionOff(TFT_t * dev) {
	spi_master_write_command(dev, 0x20); // Display Inversion Off
}

// Display Inversion On
void lcdInversionOn(TFT_t * dev) {
	spi_master_write_command(dev, 0x21); // Display Inversion On
}

void lcdWrapArround(TFT_t * dev, SCROLL_TYPE_t scroll, int start, int end) {
	if (dev->_use_frame_buffer == false) return;
	
	int _width = dev->_width;
	int _height = dev->_height;
	int32_t index1;
	int32_t index2;

	if (scroll == SCROLL_RIGHT) {
		uint16_t wk[_width];
		for (int i=start;i<end;i++) {
			index1 = i * _width;
			memcpy((char *)wk, (char*)&dev->_frame_buffer[index1], _width*2);
			index2 = index1 + _width - 1;
			dev->_frame_buffer[index1] = dev->_frame_buffer[index2];
			memcpy((char *)&dev->_frame_buffer[index1+1], (char *)&wk[0], (_width-1)*2);
		}
	} else if (scroll == SCROLL_LEFT) {
		uint16_t wk[_width];
		for (int i=start;i<end;i++) {
			index1 = i * _width;
			memcpy((char *)wk, (char*)&dev->_frame_buffer[index1], _width*2);
			index2 = index1 + _width - 1;
			dev->_frame_buffer[index2] = dev->_frame_buffer[index1];
			memcpy((char *)&dev->_frame_buffer[index1], (char *)&wk[1], (_width-1)*2);
		}
	} else if (scroll == SCROLL_UP) {
		uint16_t wk;
		for (int i=start;i<=end;i++) {
			wk = dev->_frame_buffer[i];
			for (int j=0;j<_height-1;j++) {
				index1 = j * _width + i;
				index2 = (j+1) * _width + i;
				dev->_frame_buffer[index1] = dev->_frame_buffer[index2];
			}
			index2 = (_height-1) * _width + i;
			dev->_frame_buffer[index2] = wk;
		}
	} else if (scroll == SCROLL_DOWN) {
		uint16_t wk;
		for (int i=start;i<=end;i++) {
			index2 = (_height-1) * _width + i;
			wk = dev->_frame_buffer[index2];
			for (int j=_height-2;j>=0;j--) {
				index1 = j * _width + i;
				index2 = (j+1) * _width + i;
				dev->_frame_buffer[index2] = dev->_frame_buffer[index1];
			}
			dev->_frame_buffer[i] = wk;
		}
	}
}

// Draw Frame Buffer
void lcdDrawFinish(TFT_t *dev)
{
	if (dev->_use_frame_buffer == false) return;

	spi_master_write_command(dev, 0x2A); // set column(x) address
	spi_master_write_addr(dev, dev->_offsetx, dev->_offsetx+dev->_width-1);
	spi_master_write_command(dev, 0x2B); // set Page(y) address
	spi_master_write_addr(dev, dev->_offsety, dev->_offsety+dev->_height-1);
	spi_master_write_command(dev, 0x2C); // Memory Write

	//uint16_t size = dev->_width*dev->_height;
	uint32_t size = dev->_width*dev->_height;
	uint16_t *image = dev->_frame_buffer;
	while (size > 0) {
		// 1024 bytes per time.
		uint16_t bs = (size > 1024) ? 1024 : size;
		spi_master_write_colors(dev, image, bs);
		size -= bs;
		image += bs;
	}
	return;
}