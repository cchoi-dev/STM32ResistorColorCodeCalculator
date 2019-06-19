#ifndef _ST7735H_
#define _ST7735H_

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

// SPI-related ports
#define ST7735_SPI_HANDLE hspi1
extern SPI_HandleTypeDef ST7735_SPI_HANDLE;
extern UART_HandleTypeDef huart2;
extern osMutexId uart2MutexHandle;

// Conditionals
#define ST7735_USINGFREERTOS 1

// Display dimensions
#define ST7735_TFTWIDTH 160
#define ST7735_TFTHEIGHT 128
#define ST7735_STARTX 0
#define ST7735_STARTY 0
#define ST7735_ORIENT (ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR)

// Memory address control registers (MADCTL)
#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

// some flags for initR() :(
#define INITR_GREENTAB    0x00
#define INITR_REDTAB      0x01
#define INITR_BLACKTAB    0x02
#define INITR_18GREENTAB  INITR_GREENTAB
#define INITR_18REDTAB    INITR_REDTAB
#define INITR_18BLACKTAB  INITR_BLACKTAB
#define INITR_144GREENTAB 0x01
#define INITR_MINI160x80  0x04
#define INITR_HALLOWING   0x05

// Some register settings
#define ST7735_FRMCTR1    0xB1
#define ST7735_FRMCTR2    0xB2
#define ST7735_FRMCTR3    0xB3
#define ST7735_INVCTR     0xB4
#define ST7735_DISSET5    0xB6

#define ST7735_PWCTR1     0xC0
#define ST7735_PWCTR2     0xC1
#define ST7735_PWCTR3     0xC2
#define ST7735_PWCTR4     0xC3
#define ST7735_PWCTR5     0xC4
#define ST7735_VMCTR1     0xC5

#define ST7735_PWCTR6     0xFC

#define ST7735_GMCTRP1    0xE0
#define ST7735_GMCTRN1    0xE1

#define ST_CMD_DELAY      0x80    // special signifier for command lists

#define ST7735_NOP        0x00
#define ST7735_SWRESET    0x01
#define ST7735_RDDID      0x04
#define ST7735_RDDST      0x09

#define ST7735_SLPIN      0x10
#define ST7735_SLPOUT     0x11
#define ST7735_PTLON      0x12
#define ST7735_NORON      0x13

#define ST7735_INVOFF     0x20
#define ST7735_INVON      0x21
#define ST7735_DISPOFF    0x28
#define ST7735_DISPON     0x29
#define ST7735_CASET      0x2A
#define ST7735_RASET      0x2B
#define ST7735_RAMWR      0x2C
#define ST7735_RAMRD      0x2E

#define ST7735_PTLAR      0x30
#define ST7735_COLMOD     0x3A
#define ST7735_MADCTL     0x36

#define ST7735_RDID1      0xDA
#define ST7735_RDID2      0xDB
#define ST7735_RDID3      0xDC
#define ST7735_RDID4      0xDD


// TODO: Add preset 16-bit colors (565, BGR);
#define ST7735_RGB2BGR(rgb) ((rgb & 0xF800) >> 11)|((rgb & 0x001F) << 11) | (rgb & 0x7E0))

// Function Declarations
void ST7735Initialize();
void ST7735ExecuteDisplayInit(const uint8_t *addr);
void ST7735Select();
void ST7735Deselect();
void ST7735SetCmd();
void ST7735SetData();
void ST7735WriteCmd(uint8_t cmd);
void ST7735WriteData(uint8_t *data, uint8_t dataSize);
void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void drawPixel(uint8_t x, uint8_t y, uint16_t color);
void drawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void drawBlackBackground();
void drawWhiteBackground();
void drawCustomBackground(uint16_t color);
#endif // _ST7735H