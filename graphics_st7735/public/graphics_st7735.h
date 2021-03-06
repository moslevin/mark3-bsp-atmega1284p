/*===========================================================================
     _____        _____        _____        _____
 ___|    _|__  __|_    |__  __|__   |__  __| __  |__  ______
|    \  /  | ||    \      ||     |     ||  |/ /     ||___   |
|     \/   | ||     \     ||     \     ||     \     ||___   |
|__/\__/|__|_||__|\__\  __||__|\__\  __||__|\__\  __||______|
    |_____|      |_____|      |_____|      |_____|

--[Mark3 Realtime Platform]--------------------------------------------------

Copyright (c) 2013 - 2018 m0slevin, all rights reserved.
See license.txt for more information
===========================================================================*/
/**
    @file graphics_st7735.h

    @brief Graphics driver implementation on ST7735 hardware
*/
/***************************************************
  Note:  This module is based off of 3rd party code,
     see the license below!
***************************************************
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#pragma once

#include "driver.h"
#include "draw.h"
#include "graphics.h"
#include "driver.h"

#include <avr/pgmspace.h>
#include <avr/io.h>

//---------------------------------------------------------
#define INITR_GREENTAB 0x0
#define INITR_REDTAB 0x1
#define INITR_BLACKTAB 0x2

//---------------------------------------------------------
//--[ Define this based on color of display tab ]----------
#define TAB_COLOR INITR_BLACKTAB

//---------------------------------------------------------

#define ST7735_TFTWIDTH 128
#define ST7735_TFTHEIGHT 160

#define ST7735_NOP 0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID 0x04
#define ST7735_RDDST 0x09

#define ST7735_SLPIN 0x10
#define ST7735_SLPOUT 0x11
#define ST7735_PTLON 0x12
#define ST7735_NORON 0x13

#define ST7735_INVOFF 0x20
#define ST7735_INVON 0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON 0x29
#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_RAMRD 0x2E

#define ST7735_PTLAR 0x30
#define ST7735_COLMOD 0x3A
#define ST7735_MADCTL 0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_RDID1 0xDA
#define ST7735_RDID2 0xDB
#define ST7735_RDID3 0xDC
#define ST7735_RDID4 0xDD

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

//---------------------------------------------------------
// Hard-coding register/port defines for SPEED
//---------------------------------------------------------

//--[ RESET pin ]------------------------------------------
#define TFT_RST_PORT PORTB
#define TFT_RST_PIN 0
#define TFT_RST_DIR DDRB
#define TFT_RST_OUT PORTB

//--[ Slave Select pin ]------------------------------------
#define SPI_SS_PORT PORTB
#define SPI_SS_DIR DDRB
#define SPI_SS_PIN (1 << 4)

//--[ Chip Select pin ]------------------------------------
#define TFT_CS_PORT PORTB
#define TFT_CS_PIN (1 << 2)
#define TFT_CS_DIR DDRB
#define TFT_CS_OUT PORTB

//--[ Command/Data Select pin ]----------------------------
#define TFT_CD_PORT PORTB
#define TFT_CD_PIN (1 << 0)
#define TFT_CD_DIR DDRB
#define TFT_CD_OUT PORTB

//--[ SPI Data Out pin ]-----------------------------------
#define TFT_SPI_MOSI_PORT PORTB
#define TFT_SPI_MOSI_DIR DDRB
#define TFT_SPI_MOSI_PIN (1 << 3)
#define TFT_SPI_MOSI_OUT PORTB

//--[ SPI Serial Clock pin ]-------------------------------
#define TFT_SPI_SCLK_PORT PORTB
#define TFT_SPI_SCLK_DIR DDRB
#define TFT_SPI_SCLK_PIN (1 << 5)
#define TFT_SPI_SCLK_OUT PORTB

//--[ Misc SPI Registers ]---------------------------------
#define TFT_SPI_SPDR SPDR
#define TFT_SPI_SPCR SPCR
#define TFT_SPI_SPSR SPSR
#define TFT_SPI_SPIF SPIF

//---------------------------------------------------------
//--[ Hardware/Software config ]---------------------------
#define use_HW_SPI (1)

namespace Mark3
{
//---------------------------------------------------------
class GraphicsST7735 : public GraphicsDriver
{
public:
    //---------------------------------------------------------
    virtual int    Init();
    virtual int    Open() { return 0; }
    virtual int    Close() { return 0; }
    virtual size_t Read(void* /*pu8Data_*/, size_t uBytes_) { return uBytes_; }
    virtual size_t Write(const void* /*pu8Data_*/, size_t uBytes_) { return uBytes_; }
    virtual int    Control(uint16_t /*u16EventID_*/,
                           void* /*pvDataIn_*/,
                           size_t /*uSizeIn_*/,
                           const void* /*pvDataOut_*/,
                           size_t /*uSizeOut_*/)
    {
        return 0;
    }

    //---------------------------------------------------------
    virtual void DrawPixel(DrawPoint_t* pstPoint_) { Point(pstPoint_); }
    //---------------------------------------------------------
    /*
     *   Raster operations defined using per-pixel rendering.
     *   Can be overridden in inheriting classes.
     */
    virtual void ClearScreen();
    virtual void Point(DrawPoint_t* pstPoint_);
    virtual void Line(DrawLine_t* pstLine_);
    virtual void Rectangle(DrawRectangle_t* pstRectangle_);

    //    virtual void Circle(DrawCircle_t *pstCircle_);
    virtual void Ellipse(DrawEllipse_t* pstEllipse_) {}
    virtual void Bitmap(DrawBitmap_t* pstBitmap_);
    //    virtual void Stamp(DrawStamp_t *pstStamp_);

    virtual void Move(DrawMove_t* pstMove_) {}
    //    virtual void TriangleWire(DrawPoly_t *pstPoly_);
    virtual void TriangleFill(DrawPoly_t* pstPoly_) {}
    //    virtual void Polygon(DrawPoly_t *pstPoly_);

    //    virtual void Text(DrawText_t *pstText_);
    //    virtual uint16_t TextWidth(DrawText_t *pstText_);

private:
    void CommandList(const uint8_t* pu8Data_);

    void WriteCommand(uint8_t u8X_);
    void WriteData(uint8_t u8X_);

    void SetOpWindow(DrawRectangle_t* pstRectangle_);

    void FastVLine(DrawLine_t* pstLine_);
    void FastHLine(DrawLine_t* pstLine_);

    uint8_t m_u8ColStart;
    uint8_t m_u8RowStart;
};

} // namespace Mark3
