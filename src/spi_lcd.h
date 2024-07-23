#include <stdint.h>
#include "driver/spi_master.h"

/*!< SPI host DMA Channel */
#define DMA_CHAN	    SPI_DMA_CH_AUTO
#define USE_HORIZONTAL  0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

#define ST7789_DC GPIO_NUM_38
#define ST7789_RST GPIO_NUM_0
#define ST7789_BCKL GPIO_NUM_45


// sda:41 sck:40 cs:39 dc:38
#define PIN_NUM_MISO GPIO_NUM_NC
#define PIN_NUM_MOSI GPIO_NUM_41
#define PIN_NUM_CLK GPIO_NUM_40
#define PIN_NUM_CS GPIO_NUM_39

#define PIN_NUM_DC ST7789_DC	 // 38
#define PIN_NUM_RST ST7789_RST	 // 0
#define PIN_NUM_BCKL ST7789_BCKL // 45		//backlight enable

#define CONFIG_SPI_HOST     SPI3_HOST

void dispInit();
// void sdlDispAudioInit();
void dispDraw(uint8_t *mem);
void ili_spi_pre_transfer_callback(spi_transaction_t *t);