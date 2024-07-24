// Driver for the LCD on an ESP32-Wrover-Kit board
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
// updated #include "esp_heap_alloc_caps.h"
#include "esp_heap_caps.h"
#include "mpumouse.h"
#include "mouse.h"
#include "lcd.h"
#include "spi_lcd.h"

static spi_device_handle_t spi_handle;

// static spi_device_handle_t spi;
int lcd_h = 280;
int lcd_w = 240;

// This function is called (in irq context!) just before a transmission starts. It will
// set the D/C line to the value indicated in the user field.
void ili_spi_pre_transfer_callback(spi_transaction_t *t)
{
	int dc = (int)t->user;
	gpio_set_level(PIN_NUM_DC, dc);
}

/**
 * @brief 发送屏幕头信息开始
 *
 * 发送屏幕头信息开始，设置屏幕显示的起始位置、宽度和高度。
 *
 * @param spi SPI 设备句柄
 * @param xpos 屏幕显示起始 X 坐标
 * @param ypos 屏幕显示起始 Y 坐标
 * @param w 屏幕显示宽度
 * @param h 屏幕显示高度
 */
static void send_header_start(spi_device_handle_t spi, int xpos, int ypos, int w, int h)
{
	esp_err_t ret;
	int x;
	// Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
	// function is finished because the SPI driver needs access to it even while we're already calculating the next line.
	static spi_transaction_t trans[5];

	// In theory, it's better to initialize trans and data only once and hang on to the initialized
	// variables. We allocate them on the stack, so we need to re-init them each call.
	for (x = 0; x < 5; x++)
	{
		memset(&trans[x], 0, sizeof(spi_transaction_t));
		if ((x & 1) == 0)
		{
			// Even transfers are commands
			trans[x].length = 8;
			trans[x].user = (void *)0;
		}
		else
		{
			// Odd transfers are data
			trans[x].length = 8 * 4;
			trans[x].user = (void *)1;
		}
		trans[x].flags = SPI_TRANS_USE_TXDATA;
	}
	trans[0].tx_data[0] = 0x2A;					 // Column Address Set
	trans[1].tx_data[0] = xpos >> 8;			 // Start Col High
	trans[1].tx_data[1] = xpos & 0xff;					 // Start Col Low
	trans[1].tx_data[2] = (xpos + w - 1) >> 8;	 // End Col High
	trans[1].tx_data[3] = (xpos + w - 1) & 0xff; // End Col Low
	trans[2].tx_data[0] = 0x2B;					 // Page address set
	trans[3].tx_data[0] = ypos >> 8;			 // Start page high
	trans[3].tx_data[1] = ypos & 0xff;			 // start page low
	trans[3].tx_data[2] = (ypos + h - 1) >> 8;	 // end page high
	trans[3].tx_data[3] = (ypos + h - 1) & 0xff; // end page low
	trans[4].tx_data[0] = 0x2C;					 // memory write

	// Queue all transactions.
	for (x = 0; x < 5; x++)
	{
		ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
		assert(ret == ESP_OK);
	}

	// When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
	// mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
	// finish because we may as well spend the time calculating the next line. When that is done, we can call
	// send_line_finish, which will wait for the transfers to be done and check their status.
}

/**
 * @brief 发送头清理
 *
 * 清理 SPI 设备发送的头部数据。
 *
 * @param spi SPI 设备句柄
 */
void send_header_cleanup(spi_device_handle_t spi)
{
	spi_transaction_t *rtrans;
	esp_err_t ret;
	// Wait for all 5 transactions to be done and get back the results.
	for (int x = 0; x < 5; x++)
	{
		ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
		assert(ret == ESP_OK);
		// We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
	}
}

volatile static uint8_t *currFbPtr = NULL;
SemaphoreHandle_t dispSem = NULL; //  显示信号量(二进制)，用于控制显示，释放时调用xSemaphoreGive，创建时调用xSemaphoreCreateBinary

#define NO_SIM_TRANS 5 // 用来确定分配多少内存块，同时进行的最大SPI传输数量
#define MEM_PER_TRANS 1024 // 每次传输的数据量

void IRAM_ATTR displayTask(void *arg)
{
	int x, i;
	int idx = 0;
	int inProgress = 0;
	static uint16_t *dmamem[NO_SIM_TRANS];
	spi_transaction_t trans[NO_SIM_TRANS];
	spi_transaction_t *rtrans;
	esp_err_t ret;
	printf("*** Display task starting.\n");
	lcd_init(CONFIG_SPI_HOST, 26000000);

	// start emu display data
	for (x = 0; x < NO_SIM_TRANS; x++)
	{
		// updated
		// dmamem[x]=pvPortMallocCaps(MEM_PER_TRANS*2, MALLOC_CAP_DMA);
		// 为每个传输块分配内存，使用heap_caps_malloc函数，该函数确保内存分配适用于DMA传输。
		dmamem[x] = heap_caps_malloc(MEM_PER_TRANS * 2, MALLOC_CAP_DMA); // TODO：请搜索heap_caps_malloc这个函数，看下是不是应该在lcdInit里面CONFIG_FRAME_BUFFER实现的
		assert(dmamem[x]);
		memset(&trans[x], 0, sizeof(spi_transaction_t)); // 将传输描述符初始化为0。
		trans[x].length = MEM_PER_TRANS * 4;
		trans[x].user = (void *)1;
		trans[x].tx_buffer = &dmamem[x]; // 将传输缓冲区设置为分配的内存块。
	}

	while (1)
	{
		xSemaphoreTake(dispSem, portMAX_DELAY); // 拿到显示信号量
		uint8_t *myData = (uint8_t *)currFbPtr;

		send_header_start(spi_handle, 0, 0, lcd_w, lcd_h); // TODO：这是不是属于lcdInit已经做过的？
		send_header_cleanup(spi_handle); // TODO：这是不是属于lcdInit已经做过的？
		int xpos = 0;
		for (x = 0; x < lcd_w * lcd_h; x += MEM_PER_TRANS)
		{
			i = 0;
			// 将屏幕数据从myData加载到dmamem[idx]，将每个字节转换为16位颜色值。这部分内容主要是组装dmamem[idx]数据
			while (i < MEM_PER_TRANS)
			{
				// 内部的for循环for (int j = 0x80; j != 0; j >>= 1)：逐位处理字节数据，如果该位为1，则对应的像素为黑色（0），否则为白色（0xffff）。
				for (int j = 0x80; j != 0; j >>= 1)
				{
					dmamem[idx][i++] = (*myData & j) ? 0 : 0xffff;
				}
				myData++;
				xpos += 8; // 更新当前行的像素位置。
				if (xpos >= 320)
				{
					xpos = 0;
					myData += ((512 - lcd_w) / 8);
				}
			}
			trans[idx].length = MEM_PER_TRANS * 16; // 设置传输长度为MEM_PER_TRANS个像素的16位颜色值。
			trans[idx].user = (void *)1;
			trans[idx].tx_buffer = dmamem[idx]; // 将传输缓冲区设置为当前的DMA内存块
			ret = spi_device_queue_trans(spi_handle, &trans[idx], portMAX_DELAY);
			assert(ret == ESP_OK);

			idx++;
			if (idx >= NO_SIM_TRANS)
				idx = 0;

			if (inProgress == NO_SIM_TRANS - 1)
			{
				ret = spi_device_get_trans_result(spi_handle, &rtrans, portMAX_DELAY);
				assert(ret == ESP_OK);
			}
			else
			{
				inProgress++;
			}
		}
		while (inProgress)
		{
			ret = spi_device_get_trans_result(spi_handle, &rtrans, portMAX_DELAY);
			assert(ret == ESP_OK);
			inProgress--;
		}
	}
}

/**
 * @brief 绘制显示
 *
 * 将指定的内存地址中的数据绘制到显示屏上，并处理鼠标移动事件。
 *
 * @param mem 内存地址指针，指向要绘制的数据
 */
void dispDraw(uint8_t *mem)
{
	int dx, dy, btn;
	currFbPtr = mem;
	// 释放信号量，触发显示任务
	xSemaphoreGive(dispSem);
	// 获取鼠标移动信息
	mpuMouseGetDxDyBtn(&dx, &dy, &btn);
	mouseMove(dx, dy, btn);
}

/**
 * @brief 初始化显示函数
 *
 * 初始化显示相关的设置和资源，包括创建信号量和创建显示任务。
 *
 * @note 在多核环境中，使用 CONFIG_FREERTOS_UNICORE 宏来决定是否将显示任务绑定到核心 0。
 *       在非多核环境中，显示任务默认绑定到核心 1。
 */
void dispInit()
{
	printf("spi_lcd_init()\n");
	// 创建二进制信号量，用于控制显示任务和主任务之间的同步
	dispSem = xSemaphoreCreateBinary();
#if CONFIG_FREERTOS_UNICORE
	xTaskCreatePinnedToCore(&displayTask, "display", 3000, NULL, 6, NULL, 0);
#else
	xTaskCreatePinnedToCore(&displayTask, "display", 3000, NULL, 6, NULL, 1);
#endif
}
