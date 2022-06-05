/*
 * TFT_ILI9341.c
 *
 *  Created on: Jun 1, 2022
 *      Author: dzion
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "TFT_ILI9341.h"

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

SPI_HandleTypeDef *Tft_hspi;

/* Delay function */
static void ILI9341_Delay(uint32_t ms)
{
#if (ILI9341_FREERTOS == 1)
	vTaskDelay(ms / portTICK_PERIOD_MS);
#else
	HAL_Delay(ms);
#endif
}

static void ILI9341_SendToTFT(uint8_t *Byte, uint32_t Length)
 {
#if (ILI9341_OPTIMIZE_HAL_SPI == 1)
	while (Length > 0U) {
		/* Wait until TXE flag is set to send data */
		if (__HAL_SPI_GET_FLAG(Tft_hspi, SPI_FLAG_TXE)) {
			*((__IO uint8_t*) &Tft_hspi->Instance->DR) = (*Byte);
			Byte++;
			Length--;
		}
	}
	while (__HAL_SPI_GET_FLAG(Tft_hspi, SPI_FLAG_BSY) != RESET)
		;
#else
	HAL_SPI_Transmit(Tft_hspi, Byte, Length, ILI9341_SPI_TIMEOUT);
#endif
}

static void ILI9341_SendCommand(uint8_t Command)
{
	/* CS LOW */
#if (ILI9341_USE_CS == 1)
	ILI9341_CS_LOW;
#endif

	// DC to Command - to LOW
	ILI9341_DC_LOW;
	// Send to TFT 1 byte
	ILI9341_SendToTFT(&Command, 1);
	/* CS HIGH */
#if (ILI9341_USE_CS == 1)
	ILI9341_CS_HIGH;
#endif
}

static void ILI9341_SendData16(uint16_t Data)
{
#if (ILI9341_USE_CS == 1)
	ILI9341_CS_LOW;
#endif

	uint8_t tmp[2];
	tmp[0] = (Data >> 8); //(MSB FIRST)
	tmp[1] = Data & 0xFF;

	ILI9341_DC_HIGH; // Data mode
	ILI9341_SendToTFT(tmp, 2);

#if (ILI9341_USE_CS == 1)
	ILI9341_CS_HIGH;
#endif
}

static void ILI9341_SendCommandAndData(uint8_t Command, uint8_t *Data, uint32_t Length)
{
	/* CS LOW */
#if (ILI9341_USE_CS == 1)
	ILI9341_CS_LOW;
#endif

	// DC to Command - to LOW
	ILI9341_DC_LOW;
	// Send to TFT 1 byte
	ILI9341_SendToTFT(&Command, 1);

	// DC to Command - to HIGH
	ILI9341_DC_HIGH;
	// Send to TFT length byte
	ILI9341_SendToTFT(Data, Length);
	/* CS HIGH */
#if (ILI9341_USE_CS == 1)
	ILI9341_CS_HIGH;
#endif
}

static const uint8_t initcmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_MADCTL  , 1, 0x48,             // Memory Access Control
  ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_PIXFMT  , 1, 0x55,
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};

void ILI9341_Init(SPI_HandleTypeDef *hspi) {
	uint8_t cmd, x, numArgs;
	const uint8_t *addr = initcmd;
	Tft_hspi = hspi;

#if (ILI9341_OPTIMIZE_HAL_SPI == 1)
	__HAL_SPI_ENABLE(Tft_hspi);
#endif

#if (ILI9341_USE_HW_RESET == 1)
	ILI9341_RST_LOW;
	ILI9341_Delay(10);
	ILI9341_RST_HIGH;
	ILI9341_Delay(10);
#else
	ILI9341_SendCommand(ILI9341_SWRESET);
	ILI9341_Delay(150);
#endif

	while ((cmd = *(addr++)) > 0) {
		x = *(addr++);
		numArgs = (x & 0x7F);
		ILI9341_SendCommandAndData(cmd, (uint8_t*)addr, numArgs);
		addr += numArgs;
		if (x & 0x80) {
			ILI9341_Delay(150);
		}
	}
	ILI9341_SetRotation(ILI9341_ROTATION);
}

void ILI9341_SetRotation(uint8_t Rotation)
{
	if(Rotation > 3)
		return;

	switch(Rotation)
	{
	case 0:
		Rotation = (MADCTL_MX | MADCTL_BGR);
	case 1:
		Rotation = (MADCTL_MV | MADCTL_BGR);
	case 2:
		Rotation = (MADCTL_MY | MADCTL_BGR);
	case 3:
		Rotation = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
	}
	ILI9341_SendCommandAndData(ILI9341_MADCTL, &Rotation, 1);
}

void ILI9341_SetAddrWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
	uint8_t DataToTranfer[4];
	uint16_t x2 = (x1 + w - 1);
	uint16_t y2 = (y1 + h - 1);

	DataToTranfer[0] = x1 >> 8;
	DataToTranfer[1] = x1 & 0xFF;
	DataToTranfer[2] = x2 >> 8;
	DataToTranfer[3] = x2 & 0xFF;
	ILI9341_SendCommandAndData(ILI9341_CASET, DataToTranfer, 4);

	DataToTranfer[0] = y1 >> 8;
	DataToTranfer[1] = y1 & 0xFF;
	DataToTranfer[2] = y2 >> 8;
	DataToTranfer[3] = y2 & 0xFF;
	ILI9341_SendCommandAndData(ILI9341_PASET, DataToTranfer, 4);
}

void ILI9341_DrawPixel(int16_t x, int16_t y, uint16_t color) {
	uint8_t DataToTranfer[2];
	if ((x >= 0) && (x < ILI9341_TFTWIDTH) && (y >= 0) && (y < ILI9341_TFTHEIGHT)) {
		ILI9341_SetAddrWindow(x, y, 1, 1);
		DataToTranfer[0] = color >> 8;
		DataToTranfer[1] = color & 0xFF;
		ILI9341_SendCommandAndData(ILI9341_RAMWR, DataToTranfer, 2);
	}
}

void ILI9341_DrawImage(int16_t x, int16_t y, const uint8_t *img ,uint16_t w, uint16_t h) {
	if ((x >= 0) && ((x + w) <= ILI9341_TFTWIDTH) && (y >= 0) && ((y + h) <= ILI9341_TFTHEIGHT)) {
		ILI9341_SetAddrWindow(x, y, w, h);
		ILI9341_SendCommandAndData(ILI9341_RAMWR, (uint8_t*)img, (w * h * 2));
	}
}

void ILI9341_ClearDisplay(uint16_t color, int16_t x, int16_t y, uint16_t w, uint16_t h)
{
	ILI9341_SetAddrWindow(x, y, w, h);
	ILI9341_SendCommand(ILI9341_RAMWR);

#if (ILI9341_OPTIMIZE_HAL_SPI == 1)
	uint32_t Length = h * w;
#if (ILI9341_USE_CS == 1)
	ILI9341_CS_LOW;
#endif
	ILI9341_DC_HIGH; // Data mode

	while (Length > 0U) {
		if (__HAL_SPI_GET_FLAG(Tft_hspi, SPI_FLAG_TXE)) {
			*((__IO uint8_t*) &Tft_hspi->Instance->DR) = (color >> 8); //MSB color
			while (__HAL_SPI_GET_FLAG(Tft_hspi, SPI_FLAG_TXE) != SET)
				;
			*((__IO uint8_t*) &Tft_hspi->Instance->DR) = (color & 0xFF); // LSB color
			Length--;
		}
	}
	while (__HAL_SPI_GET_FLAG(Tft_hspi, SPI_FLAG_BSY) != RESET)
		;

#if (ILI9341_USE_CS == 1)
	ILI9341_CS_HIGH;
#endif
#else // ILI9341_OPTIMIZE_HAL_SPI
		for(uint32_t i = 0; i < (ILI9341_TFTWIDTH * ILI9341_TFTHEIGHT); i++)
		{
			ILI9341_SendData16(color);
		}
#endif
}
