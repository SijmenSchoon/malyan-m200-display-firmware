

#ifndef _HX8357_
#define _HX8357_

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif
#include <Adafruit_GFX.h>

// #define USE_3_BIT_COLORS
#define USE_16_BIT_COLORS

#define TFTWIDTH  320
#define TFTHEIGHT  480

enum {
    PORTRAIT,
    LANDSCAPE,         // Connector on top
    REVERSE_PORTRAIT,
    REVERSE_LANDSCAPE, // Connector on bottom
};


class HX8357 : public Adafruit_GFX {
private:
    void beginSPI();

    void writeCommand(uint8_t cmd);
    void writeCommandData(uint8_t cmd, const uint8_t *data,
                          uint8_t lenInBytes);
    void writeData(uint8_t data);
    void writeData(uint8_t *data, uint8_t lenInBytes);
    void writeDataRGB(uint16_t color, uint32_t repeats);

public:
  HX8357();

  void begin();

  /**
   * Turns on the display, making it start displaying the frame memory
   * contents.
   */
  void displayOn();

  /**
   * Turns off the display, making it stop displaying the frame memory
   * contents. Has no effect on the contents of the framebuffer.
   */
  void displayOff();

  /**
   * Sets the starting and ending coordinates of the address window.
   */
  void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

  /**
   * Draws a pixel at the (absolute) x and y coordinates.
   */
  void drawPixel(int16_t x, int16_t y, uint16_t color);


  void setRotation(uint8_t rotation);
};

#endif
