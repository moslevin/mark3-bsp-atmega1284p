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
    @file graphics_flavr.h

    @brief Graphics driver implementation using flAVR for co-simulation
*/

#include "driver.h"
#include "draw.h"
#include "graphics.h"
#include "driver.h"

#pragma once

namespace Mark3
{
//---------------------------------------------------------
class GraphicsFlavr : public GraphicsDriver
{
public:
    //---------------------------------------------------------
    virtual int Init()
    {
        m_u16Res16X = 128;
        m_u16Res16Y = 160;
        return 0;
    }
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
    virtual void DrawPixel(DrawPoint_t* pstPoint_);

    //---------------------------------------------------------
    /*
        Raster operations defined using per-pixel rendering.
        Can be overridden in inheriting classes.
    */
    //    virtual void ClearScreen();
    virtual void Point(DrawPoint_t* pstPoint_) { DrawPixel(pstPoint_); }
    //    virtual void Line(DrawLine_t *pstLine_);
    //    virtual void Rectangle(DrawRectangle_t *pstRectangle_);

    //    virtual void Circle(DrawCircle_t *pstCircle_);
    //    virtual void Ellipse(DrawEllipse_t *pstEllipse_) {}

    //    virtual void Bitmap(DrawBitmap_t *pstBitmap_);
    //    virtual void Stamp(DrawStamp_t *pstStamp_);

    //    virtual void Move(DrawMove_t *pstMove_ ) {}
    //    virtual void TriangleWire(DrawPoly_t *pstPoly_);
    //    virtual void TriangleFill(DrawPoly_t *pstPoly_) {}
    //    virtual void Polygon(DrawPoly_t *pstPoly_);

    //    virtual void Text(DrawText_t *pstText_);
    //    virtual uint16_t TextWidth(DrawText_t *pstText_);
    void Flip(void);
};
} // namespace Mark3
