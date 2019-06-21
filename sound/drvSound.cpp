/*===========================================================================
     _____        _____        _____        _____
 ___|    _|__  __|_    |__  __|__   |__  __| __  |__  ______
|    \  /  | ||    \      ||     |     ||  |/ /     ||___   |
|     \/   | ||     \     ||     \     ||     \     ||___   |
|__/\__/|__|_||__|\__\  __||__|\__\  __||__|\__\  __||______|
    |_____|      |_____|      |_____|      |_____|

--[Mark3 Realtime Platform]--------------------------------------------------

Copyright (c) 2012 m0slevin, all rights reserved.
See license.txt for more information
===========================================================================*/
/**

    @file   drvSound.cpp

    @brief  ATMega1284p PWM Sound Driver
*/

#include "mark3cfg.h"
#include "criticalsection.h"
#include "criticalguard.h"
#include "kerneltypes.h"
#include "kerneltimer.h"
#include "threadport.h"
#include "timer.h"
#include "drvSound.h"
#include <avr/io.h>
#include <avr/interrupt.h>

namespace Mark3
{
//---------------------------------------------------------------------------
static volatile uint16_t u16BeatTimer  = 0;
static volatile uint16_t u16NoteIndex  = 0;
static volatile uint16_t u16SweepIdx   = 0;
static volatile uint16_t u16SweepSteps = 0;

static volatile uint16_t target = 0;
static volatile uint16_t count  = 0;
static volatile uint8_t  level  = 0;

Timer clTimer;

//---------------------------------------------------------------------------
ISR(TIMER0_COMPB_vect)
{
    if (++count >= target) {
        count = 0;
        if (!OCR0B) {
            OCR0B = level;
        } else {
            OCR0B = 0;
        }
    }
}

//---------------------------------------------------------------------------
static void ToneCallback(Thread* pclOwner_, void* pvData_)
{
    OCR0B = 0;
    TIMSK0 &= ~(1 << OCIE0B);
    clTimer.Stop();
}

//---------------------------------------------------------------------------
void SoundDriver::SetTone(SquareWave_t* pstWave_)
{
    clTimer.Stop();
    Open();

    {
        const auto cg = CriticalGuard{};
        target = pstWave_->u16Freq / 2;
        level  = pstWave_->u8Level;
        OCR0B  = level;
    }

    clTimer.Start(false, pstWave_->u16DurationMS, ToneCallback, 0);
}

//---------------------------------------------------------------------------
void SongCallback(Thread* pclOwner_, void* pvData_)
{
    auto* pstSong = reinterpret_cast<Song_t*>(pvData_);
    u16BeatTimer++;
    if (u16BeatTimer >= pstSong->astNotes[u16NoteIndex].u16DurationBPM32) {
        u16BeatTimer = 0;
        u16NoteIndex++;

        if (0 == pstSong->astNotes[u16NoteIndex].u16Freq) {
            u16NoteIndex = 0;
        }

        {
            const auto cg = CriticalGuard{};
            target = pstSong->astNotes[u16NoteIndex].u16Freq / 2;
            level  = pstSong->astNotes[u16NoteIndex].u8Level;
            OCR0B  = level;
        }
    }
}

//---------------------------------------------------------------------------
void SweepCallback(Thread* pstOwner_, void* pvData_)
{
    uint16_t u16Freq;
    auto*    pstSweep = reinterpret_cast<Sweep_t*>(pvData_);

    if (u16SweepIdx < u16SweepSteps) {
        u16SweepIdx++;

        if (pstSweep->bDir) {
            u16Freq = pstSweep->u16FreqStart + u16SweepIdx;
        } else {
            u16Freq = pstSweep->u16FreqStart - u16SweepIdx;
        }

        {
            const auto cg = CriticalGuard{};
            target = u16Freq / 2;
            level  = pstSweep->u8Level;
        }

        if (u16SweepIdx == u16SweepSteps) {
            OCR0B = 0;
            TIMSK0 &= ~(1 << OCIE0B);
            clTimer.Stop();
        }
    }
}
//---------------------------------------------------------------------------
void SoundDriver::StartSong(Song_t* pstSong_)
{
    uint32_t u32BeatTimeMS32 = ((60000) / (pstSong_->u8TempoBPM * 8));

    Open();

    clTimer.Stop();

    {
        const auto cg = CriticalGuard{};
        count  = 0;
        target = pstSong_->astNotes[0].u16Freq / 2;
        level  = pstSong_->astNotes[0].u8Level;
        OCR0B  = level;
    }

    clTimer.Start(true, u32BeatTimeMS32, SongCallback, pstSong_);
    u16BeatTimer = 0;
    u16NoteIndex = 0;
}

//---------------------------------------------------------------------------
void SoundDriver::StartSweep(Sweep_t* pstSweep_)
{
    clTimer.Stop();

    Open();

    u16SweepIdx   = 0;
    u16SweepSteps = (pstSweep_->u16Duration) / pstSweep_->u16Speed;

    {
        const auto cg = CriticalGuard{};
        target = pstSweep_->u16FreqStart / 2;
        level  = pstSweep_->u8Level;
        OCR0B  = level;
    }
    clTimer.Start(true, pstSweep_->u16Speed, SweepCallback, pstSweep_);
}

//---------------------------------------------------------------------------
int SoundDriver::Init()
{
    clTimer.Init();

    TCCR0A = 0;
    TCCR0A |= (1 << COM0B1)  // Non inverted mode
              | (1 << WGM01) // Fast PWM
              | (1 << WGM00) // Fast PWM (contd)
        ;
    TCCR0B = 0;
    TCCR0B |= (1 << CS00) // Div-by-1
        ;

    DDRB |= (1 << 4); // Port B4 as output (PIN 5)
    PORTB = 0;
    OCR0B = 0;
    return 0;
}

//---------------------------------------------------------------------------
int SoundDriver::Open()
{
    OCR0B = 0;
    TIMSK0 |= (1 << OCIE0B);
    return 0;
}

//---------------------------------------------------------------------------
int SoundDriver::Close()
{
    TIMSK0 &= ~(1 << OCIE0B);
    OCR0B = 0;
    return 0;
}

//---------------------------------------------------------------------------
int SoundDriver::Control(uint16_t u16Event_, void* pvDataIn_, size_t uSizeIn_, const void* pvDataOut_, size_t uSizeOut_)
{
    switch (u16Event_) {
        case SOUND_EVENT_OFF:
            clTimer.Stop();
            Close();
            break;
        case SOUND_EVENT_SQUAREWAVE: {
            auto* pstSW = reinterpret_cast<SquareWave_t*>(pvDataIn_);
            SetTone(pstSW);
            break;
        }
        case SOUND_EVENT_SONG: {
            auto* pstSong = reinterpret_cast<Song_t*>(pvDataIn_);
            StartSong(pstSong);
            break;
        }
        case SOUND_EVENT_SWEEP: {
            auto* pstSweep = reinterpret_cast<Sweep_t*>(pvDataIn_);
            StartSweep(pstSweep);
            break;
        }
        default: break;
    }
    return 0;
}

} // namespace Mark3
