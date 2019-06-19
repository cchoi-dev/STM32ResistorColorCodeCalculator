#include "st7735.h"

/* Adapted from Adafruit ST7735 library.
 * (https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7735.cpp)
*/

/** Display Initialization
*/
static const uint8_t
  Rcmd1[] = {                       // 7735R init, part 1 (red or green tab)
    15,                             // 15 commands in list:
    ST7735_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
      150,                          //     150 ms delay
    ST7735_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
      255,                          //     500 ms delay
    ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
      0x01, 0x2C, 0x2D,             //     Dot inversion mode
      0x01, 0x2C, 0x2D,             //     Line inversion mode
    ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
      0x07,                         //     No inversion
    ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                         //     -4.6V
      0x84,                         //     AUTO mode
    ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
      0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
      0x0A,                         //     Opamp current small
      0x00,                         //     Boost frequency
    ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
      0x8A,                         //     BCLK/2,
      0x2A,                         //     opamp current small & medium low
    ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF,  0,              // 13: Don't invert display, no args
    ST7735_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
      ST7735_ORIENT,                //     row/col addr, bottom-top, left-right refresh
    ST7735_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
      0x05 },                       //     16-bit color

  /*
  Rcmd2green[] = {                  // 7735R init, part 2 (green tab only)
    2,                              //  2 commands in list:
    ST7735_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 0
      0x00, 0x7F+0x02,              //     XEND = 127
    ST7735_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,                   //     XSTART = 0
      0x00, 0x9F+0x01 },            //     XEND = 159
  */
      
  Rcmd2red[] = {                    // 7735R init, part 2 (red tab only)
    2,                              //  2 commands in list:
    ST7735_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x9F,                   //     XEND = 127
    ST7735_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F },                 //     XEND = 159
      //0x00, 0x9F },                 //     XEND = 159

/*
  Rcmd2green144[] = {               // 7735R init, part 2 (green 1.44 tab)
    2,                              //  2 commands in list:
    ST7735_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F,                   //     XEND = 127
    ST7735_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F },                 //     XEND = 127
*/

  Rcmd3[] = {                       // 7735R init, part 3 (red or green tab)
    4,                              //  4 commands in list:
    ST7735_GMCTRP1, 16      ,       //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x02, 0x1c, 0x07, 0x12,       //     (Not entirely necessary, but provides
      0x37, 0x32, 0x29, 0x2d,       //      accurate colors)
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      ,       //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x03, 0x1d, 0x07, 0x06,       //     (Not entirely necessary, but provides
      0x2E, 0x2C, 0x29, 0x2D,       //      accurate colors)
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST7735_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
      100 };                        //     100 ms delay

      
// General ST7735 Functions
void ST7735Initialize() {
  ST7735Select();
  ST7735ExecuteDisplayInit(Rcmd1);
  ST7735ExecuteDisplayInit(Rcmd2red);
  ST7735ExecuteDisplayInit(Rcmd3);
  ST7735Deselect();
}

void ST7735ExecuteDisplayInit(const uint8_t *addr) {
  uint8_t numCommands, cmd, numArgs;
  uint16_t ms;
  
  numCommands = *addr++;            // No. of commands to send
  while (numCommands--) {
    cmd = *addr++;                  // Read the command
    ST7735WriteCmd(cmd);
    numArgs = *addr++;              // No. of arguments for command
    ms = numArgs & ST_CMD_DELAY;    // if MSB set, delay following args
    numArgs &= ~ST_CMD_DELAY;       // Mask out delay bit
    ST7735WriteData((uint8_t*)addr, numArgs);
    addr += numArgs;
    
    if (ms) {
      ms = *addr++;
      if (ms == 255) ms = 500;
      osDelay(ms); // TODO: Add conditional to do osDelay or HAL_Delay depending on FreeRTOS use
    }
  }
}

// SPI Interface Functions (4-pin serial interface)
void ST7735Select() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

void ST7735Deselect() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void ST7735SetCmd() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

void ST7735SetData() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
}

void ST7735WriteCmd(uint8_t cmd) {
  // TODO: Add code to check if CS pin on
  ST7735SetCmd();
  HAL_SPI_Transmit(&ST7735_SPI_HANDLE, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

void ST7735WriteData(uint8_t *data, uint8_t dataSize) {
  // TODO: Add code to check if CS pin on
  ST7735SetData();
  HAL_SPI_Transmit(&ST7735_SPI_HANDLE, data, dataSize, HAL_MAX_DELAY);
}

void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  x += ST7735_STARTX;
  y += ST7735_STARTY;
  // TODO: Add code to limit the values of x+w-1, y+h-1
  // First 16 bits sent are START, last 16 are END
  uint8_t sendDataX[] = {0x00, x, 0x00, x + w - 1};
  uint8_t sendDataY[] = {0x00, y, 0x00, y + h - 1};
  ST7735WriteCmd(ST7735_CASET);
  ST7735WriteData(sendDataX, sizeof(sendDataX));
  ST7735WriteCmd(ST7735_RASET);
  ST7735WriteData(sendDataY, sizeof(sendDataY));
  ST7735WriteCmd(ST7735_RAMWR);
}

void drawPixel(uint8_t x, uint8_t y, uint16_t color) {
  if (x >= ST7735_TFTWIDTH || y >= ST7735_TFTHEIGHT) {
    osMutexWait(uart2MutexHandle, 50);
    HAL_UART_Transmit(&huart2, (uint8_t*)"Draw failed!\n\r", 14, HAL_MAX_DELAY);
    osMutexRelease(uart2MutexHandle);
    return;
  }
  // Set color - segment into two separate bytes
  uint8_t sendDataColor[] = {color >> 8, color & 0xFF};
  ST7735Select();
  setAddrWindow(x, y, 1, 1);
  ST7735WriteData(sendDataColor, sizeof(sendDataColor));
  ST7735Deselect();
}

void drawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  if (x >= ST7735_TFTWIDTH || y >= ST7735_TFTHEIGHT) {
    osMutexWait(uart2MutexHandle, 50);
    HAL_UART_Transmit(&huart2, (uint8_t*)"Draw failed!\n\r", 14, HAL_MAX_DELAY);
    osMutexRelease(uart2MutexHandle);
    return;
  }
  // Set color - segment into two separate bytes
  uint8_t sendDataColor[] = {color >> 8, color & 0xFF};
  ST7735Select();
  setAddrWindow(x, y, w, h);
  for (int i = w * h; i > 0; i--) {
    ST7735WriteData(sendDataColor, sizeof(sendDataColor));
  }
  /*for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      ST7735WriteData(sendDataColor, sizeof(sendDataColor));
    }
  }*/
  ST7735Deselect();
}

void drawBlackBackground() {
  drawRectangle(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, 0x0000);
}

void drawWhiteBackground() {
  drawRectangle(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, 0xFFFF);
}

void drawCustomBackground(uint16_t color) {
  drawRectangle(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, color);
}

