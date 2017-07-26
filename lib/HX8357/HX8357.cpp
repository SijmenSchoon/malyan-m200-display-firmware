#include "HX8357.h"
#include "Commands.h"

extern "C" {
#include "sdk_spi.h"
#include "spi_interface.h"
}

#define CMD 0
#define DATA 1

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

HX8357::HX8357() : Adafruit_GFX(TFTWIDTH, TFTHEIGHT) {}

void HX8357::beginSPI() {
    // Clear the 9th bit
    WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105);

    // Configure IO to SPI mode
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2);

    // Initialize the SPI settings
    SpiAttr hSpiAttr;
    hSpiAttr.bitOrder = SpiBitOrder_MSBFirst;
    hSpiAttr.speed = SpiSpeed_80MHz;
    hSpiAttr.mode = SpiMode_Master;
    hSpiAttr.subMode = SpiSubMode_0;
    SPIInit(SpiNum_HSPI, &hSpiAttr);

    //
    SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND|SPI_CS_HOLD);
    CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_DUMMY|SPI_USR_ADDR|SPI_USR_MOSI);
    WRITE_PERI_REG(SPI_USER1(HSPI), ((8&SPI_USR_COMMAND_BITLEN)<<SPI_USR_COMMAND_BITLEN_S));
}

// XXX In the original code this was repeated twice. Why?
void HX8357::begin() {
    beginSPI();
    displayOff();

    // Wakes the display from sleep mode.
    writeCommand(HX8357_SLPOUT);
    delay(120);

    // Set EQ function (seems to have something to do with timings, see p177)
    writeCommandData(HX8357_SETEQ, (const uint8_t[]){ 0x02, 0x01, 0x02, 0x01}, 4);

    // Undocumented, but sent by UI board
    writeCommandData(0xED, (const uint8_t[]){ 0x00, 0x00, 0x9A, 0x9A, 0x9B, 0x9B,
                0x00, 0x00, 0x00, 0x00, 0xAE, 0xAE, 0x01, 0xA2, 0x00 }, 15);

    // Select the DBI interface and the internal oscillation clock
    writeCommandData(HX8357_SETDISPLAY, (const uint8_t[]){ 0x00 }, 1);

    // Set the internal clock division ratio to 1/1 and the inversion mode to
    // line inversion. Leaves the rest of the values to their defaults.
    writeCommandData(HX8357_SETNORTIM, (const uint8_t[]){ 0x10 }, 1);

    // Tweak LCD panel specific settings
    writeCommandData(HX8357_SETGAMMA, (const uint8_t[]){ 0x00, 0x46, 0x12,
                0x20, 0x0C, 0x00, 0x56, 0x12, 0x67, 0x02, 0x00, 0x0C}, 12);
    writeCommandData(HX8357_SETPOWER, (const uint8_t[]){ 0x44, 0x42, 0x06 }, 3);
    writeCommandData(HX8357_SETVCOM, (const uint8_t[]){ 0x43, 0x16 }, 2);
    writeCommandData(HX8357_SETNORPOW, (const uint8_t[]){ 0x04, 0x22 }, 2);
    writeCommandData(HX8357_SETPARPOW, (const uint8_t[]){ 0x04, 0x12 }, 2);
    writeCommandData(HX8357_SETIDLPOW, (const uint8_t[]){ 0x07, 0x12 }, 2);
    writeCommandData(HX8357_SETOSC, (const uint8_t[]){ 0x0C }, 1);

    // Set the Interface Pixel Format to 16 bits per pixel.
    writeCommandData(HX8357_COLMOD, (const uint8_t[]){ 0x55 }, 1);

    // Set the address window to the full width and height.
    setAddressWindow(0, 0, 319, 480);
    delay(20);

    // Disable the tearing effect line.
    writeCommandData(HX8357_TEON, (const uint8_t[]){ 0x00 }, 1);

    // Set the brightness of the display (0x00 - 0xFF).
    writeCommandData(HX8357_WRDISBV, (const uint8_t[]){ 0x10 }, 1);

    // Enable backlight control, disable display dimming and enable the
    // brightness registers.
    writeCommandData(HX8357_WRCTRLD, (const uint8_t[]){ 0x24 }, 1);

    // Disable content adaptive brightness control.
    writeCommandData(HX8357_WRCABC, (const uint8_t[]){ 0x00 }, 1);

    // Set the minimum brightness for the adaptive brightness control (even
    // though it's disabled).
    writeCommandData(HX8357_WRCABCMB, (const uint8_t[]){ 0x00 }, 1);
}

void HX8357::displayOn() {
    writeCommand(HX8357_DISPON);
}

void HX8357::displayOff() {
    writeCommand(HX8357_DISPOFF);
    delay(10);
}

void HX8357::writeCommand(uint8_t cmd) {
    spi_lcd_9bit_write(HSPI, CMD, cmd);
}

void HX8357::writeCommandData(uint8_t cmd, const uint8_t *data, uint8_t lenInBytes) {
    writeCommand(cmd);

    // writeData(data, lenInBytes);
    for (int i = 0; i < lenInBytes; i++) {
        writeData(data[i]);
    }
}

void HX8357::writeData(uint8_t data) {
    spi_lcd_9bit_write(HSPI, DATA, data);
}

void HX8357::writeData(uint8_t *data, uint8_t lenInBytes) {
    // spi_txd(HSPI, 9, (0x01 << 8) | data);
    // spi_lcd_9bit_write(HSPI, DATA, data);

    uint32 regvalue;
    uint8 bytetemp;
    char idx = 0;

    // SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_CS_HOLD);
    // SET_PERI_REG_BITS(SPI_USER1(HSPI), SPI_USR_MOSI_BITLEN, 15, SPI_USR_MOSI_BITLEN_S);



    // WRITE_PERI_REG(SPI_USER1(HSPI), ((0&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S));
    // WRITE_PERI_REG(SPI_USER1(HSPI), ((0&SPI_USR_ADDR_BITLEN)<<SPI_USR_ADDR_BITLEN_S));

    do {
        spi_lcd_9bit_write(HSPI, DATA, data[idx]);
    } while (++idx < lenInBytes);
    //
    // // Enable MOSI
    // SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MOSI);
    //
    // // Load send buffer
    // do {
    //     bytetemp=(data[idx]>>1)|0x80;
    //
    //     regvalue= ((8&SPI_USR_COMMAND_BITLEN)<<SPI_USR_COMMAND_BITLEN_S)|((uint32)bytetemp);		//configure transmission variable,9bit transmission length and first 8 command bit
    //     regvalue|=BIT15;        //write the 9th bit
    //
    //     WRITE_PERI_REG((SPI_W0(HSPI) + (idx << 2)), regvalue);
    //     // WRITE_PERI_REG(SPI_USER2(HSPI), regvalue);
    // } while (++idx < lenInBytes);
    // // Set data send buffer length.Max data length 64 bytes.
    // SET_PERI_REG_BITS(SPI_USER1(HSPI), SPI_USR_MOSI_BITLEN, lenInBytes - 1, SPI_USR_MOSI_BITLEN_S);
    //
    // // Start send data
    // SET_PERI_REG_MASK(SPI_CMD(HSPI), SPI_USR);
    // // Wait for transmit done
    // while (!(READ_PERI_REG(SPI_SLAVE(HSPI))&SPI_TRANS_DONE));
    // CLEAR_PERI_REG_MASK(SPI_SLAVE(HSPI), SPI_TRANS_DONE);
}

void HX8357::drawPixel(int16_t x, int16_t y, uint16_t color) {
    setAddressWindow(x, y, x, y);
    writeDataRGB(color, 1);
}

void HX8357::setAddressWindow(uint16_t x0, uint16_t y0,
                              uint16_t x1, uint16_t y1) {
    const uint16_t columns[2] = { x0, x1 };
    writeCommandData(HX8357_CASET, (const uint8_t *)columns, sizeof(columns));

    const uint16_t rows[2] = { y0, y1 };
    writeCommandData(HX8357_PASET, (const uint8_t *)rows, sizeof(rows));

    writeCommand(HX83h7_RAMWR); // write to RAM
}

void HX8357::writeDataRGB(uint16_t color, uint32_t repeats){

#ifdef USE_3_BIT_COLORS
    for (; repeats > 0; repeats--) {
        writeData((uint8_t)color);
    }
#endif
#ifdef USE_16_BIT_COLORS
    uint8_t data[3] = {0};

    int r = ((color >> 11) & 0x1F);  // Extract the 5 R bits
    int g = ((color >> 5) & 0x3F);   // Extract the 6 G bits
    int b = ((color) & 0x1F);        // Extract the 5 B bits

    for (; repeats > 0; repeats--) {
        // TODO implement this
        // FF FF
        // 0000 0000 0000 0000

        //  writeData(r);
        //  writeData(g);
        //  writeData(b);
        // map(0x00, 0xFC)
        writeData(0x0);
        writeData(0x0);
        writeData(0x0);
        //  data[0] = r;
        //  data[1] = g;
        //  data[2] = b;
        //  writeData(data, 3);
    }
#endif

};

void HX8357::setRotation(uint8_t rotation) {
    writeCommand(HX8357_MADCTL);

    switch (rotation % 4) {
    case PORTRAIT:
        writeData(MADCTL_MX | MADCTL_BGR);
        _width  = TFTWIDTH;
        _height = TFTHEIGHT;
        break;

    case LANDSCAPE:
        writeData(MADCTL_MV | MADCTL_BGR);
        _width  = TFTHEIGHT;
        _height = TFTWIDTH;
        break;

    case REVERSE_PORTRAIT:
        writeData(MADCTL_MY | MADCTL_BGR);
        _width  = TFTWIDTH;
        _height = TFTHEIGHT;
        break;

    case REVERSE_LANDSCAPE:
        writeData(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
        _width  = TFTHEIGHT;
        _height = TFTWIDTH;
        break;
    }
}
