#include <stdint.h>

/*!< SPI host DMA Channel */
#define DMA_CHAN	    SPI_DMA_CH_AUTO
#define USE_HORIZONTAL  0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

void dispInit();
// void sdlDispAudioInit();
void dispDraw(uint8_t *mem);