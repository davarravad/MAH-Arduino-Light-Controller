/* Copyright (c) 2014 by Keith Woodard <rkwoodard ~AT~ gmail.com>
   This is a modified version of the original Arduino TLC5940 Library
   as originally distributed below.  
*/

/*  Copyright (c) 2009 by Alex Leone <acleone ~AT~ gmail.com>

    This file is part of the Arduino TLC5940 Library.

    The Arduino TLC5940 Library is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    The Arduino TLC5940 Library is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with The Arduino TLC5940 Library.  If not, see
    <http://www.gnu.org/licenses/>. */

/** \file
    Tlc5940 class functions. */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "tlc_config.h"
#include "Tlc5940.h"

/** Pulses a pin - high then low. */
#define pulse_pin(port, pin)   port |= _BV(pin); port &= ~_BV(pin)

/** Packed grayscale data, 24 bytes (16 * 12 bits) per TLC.

    Format: Lets assume we have 2 TLCs, A and B, daisy-chained with the SOUT of
    A going into the SIN of B.
    - byte 0: upper 8 bits of B.15
    - byte 1: lower 4 bits of B.15 and upper 4 bits of B.14
    - byte 2: lower 8 bits of B.0
    - ...
    - byte 24: upper 8 bits of A.15
    - byte 25: lower 4 bits of A.15 and upper 4 bits of A.14
    - ...
    - byte 47: lower 8 bits of A.0

    \note Normally packing data like this is bad practice.  But in this
          situation, shifting the data out is really fast because the format of
          the array is the same as the format of the TLC's serial interface. */
uint8_t tlc_GSData[NUM_TLCS * 24];

/** \defgroup CoreFunctions Core Libary Functions
    These function are all prefixed with "Tlc." */
/* @{ */

/** Pin i/o and Timer setup.  The grayscale register will be reset to all
    zeros, or whatever initialValue is set to and the Timers will start.
    \param initialValue = 0, optional parameter specifing the inital startup
           value */
void Tlc5940::init(uint16_t initialValue)
{
    /* Pin Setup */
    XLAT_DDR |= _BV(XLAT_PIN);
    BLANK_DDR |= _BV(BLANK_PIN);
    GSCLK_DDR |= _BV(GSCLK_PIN);
    BLANK_PORT |= _BV(BLANK_PIN); // leave blank high (until the timers start)

    tlc_shift8_init();

    setAll(initialValue);
    update();
    /* Timer Setup */

    /* Timer 2 - GSCLK */
    TCCR2A = _BV(COM2B1)      // set on BOTTOM, clear on OCR2A (non-inverting),
                              // output on OC2B
           | _BV(WGM21)       // Fast pwm with OCR2A top
           | _BV(WGM20);      // Fast pwm with OCR2A top
    TCCR2B = _BV(WGM22);      // Fast pwm with OCR2A top
    OCR2B = 16;                // duty factor (as short a pulse as possible)			// RKW 5/16 - I believe this should be 16 for ~50% duty cycle.  Was 0.
    OCR2A = 32; // see tlc_config.h														// RKW 5/16 - I believe this should be 32 (32+1 = 484.8 kHz). Was 4
    TCCR2B |= _BV(CS20);      // prescale by 8 //no prescale, (start pwm output)  		// RKW 5/16 - I believe this should be _BV(CS20) - No prescale. Was |= _BV(CS21)


    update();
}

/** Clears the grayscale data array, #tlc_GSData, but does not shift in any
    data.  This call should be followed by update() if you are turning off
    all the outputs. */
void Tlc5940::clear(void)
{
    setAll(0);
}

/** Shifts in the data from the grayscale data array, #tlc_GSData.
    If data has already been shifted in this grayscale cycle, another call to
    update() will immediately return 1 without shifting in the new data.  To
    ensure that a call to update() does shift in new data, use
    \code while(Tlc.update()); \endcode
    or
    \code while(tlc_needXLAT); \endcode
    \returns 1 if there is data waiting to be latched, 0 if data was
             successfully shifted in */
uint8_t Tlc5940::update(void)
{
    pulse_pin(SCLK_PORT, SCLK_PIN);
 
    uint8_t *p = tlc_GSData;
    while (p < tlc_GSData + NUM_TLCS * 24) {
        tlc_shift8(*p++);
        tlc_shift8(*p++);
        tlc_shift8(*p++);
    }
    return 0;
}

/** Sets channel to value in the grayscale data array, #tlc_GSData.
    \param channel (0 to #NUM_TLCS * 16 - 1).  OUT0 of the first TLC is
           channel 0, OUT0 of the next TLC is channel 16, etc.
    \param value (0-4095).  The grayscale value, 4095 is maximum.
    \see get */
void Tlc5940::set(TLC_CHANNEL_TYPE channel, uint16_t value)
{
    TLC_CHANNEL_TYPE index8 = (NUM_TLCS * 16 - 1) - channel;
    uint8_t *index12p = tlc_GSData + ((((uint16_t)index8) * 3) >> 1);
    if (index8 & 1) { // starts in the middle
                      // first 4 bits intact | 4 top bits of value
        *index12p = (*index12p & 0xF0) | (value >> 8);
                      // 8 lower bits of value
        *(++index12p) = value & 0xFF;
    } else { // starts clean
                      // 8 upper bits of value
        *(index12p++) = value >> 4;
                      // 4 lower bits of value | last 4 bits intact
        *index12p = ((uint8_t)(value << 4)) | (*index12p & 0xF);
    }
}

/** Gets the current grayscale value for a channel
    \param channel (0 to #NUM_TLCS * 16 - 1).  OUT0 of the first TLC is
           channel 0, OUT0 of the next TLC is channel 16, etc.
    \returns current grayscale value (0 - 4095) for channel
    \see set */
uint16_t Tlc5940::get(TLC_CHANNEL_TYPE channel)
{
    TLC_CHANNEL_TYPE index8 = (NUM_TLCS * 16 - 1) - channel;
    uint8_t *index12p = tlc_GSData + ((((uint16_t)index8) * 3) >> 1);
    return (index8 & 1)? // starts in the middle
            (((uint16_t)(*index12p & 15)) << 8) | // upper 4 bits
            *(index12p + 1)                       // lower 8 bits
        : // starts clean
            (((uint16_t)(*index12p)) << 4) | // upper 8 bits
            ((*(index12p + 1) & 0xF0) >> 4); // lower 4 bits
    // that's probably the ugliest ternary operator I've ever created.
}

/** Sets all channels to value.
    \param value grayscale value (0 - 4095) */
void Tlc5940::setAll(uint16_t value)
{
    uint8_t firstByte = value >> 4;
    uint8_t secondByte = (value << 4) | (value >> 8);
    uint8_t *p = tlc_GSData;
    while (p < tlc_GSData + NUM_TLCS * 24) {
        *p++ = firstByte;
        *p++ = secondByte;
        *p++ = (uint8_t)value;
    }
}

/** Initializes the SPI module to double speed (f_osc / 2) */
void tlc_shift8_init(void)
{
   SIN_DDR    |= _BV(SIN_PIN);    // SPI MOSI as output
   SCLK_DDR   |= _BV(SCLK_PIN);   // SPI SCK as output
   TLC_SS_DDR |= _BV(TLC_SS_PIN); // SPI SS as output

   SCLK_PORT &= ~_BV(SCLK_PIN);

   SPSR = _BV(SPI2X); // double speed (f_osc / 2)
   SPCR = _BV(SPE)    // enable SPI
        | _BV(MSTR);  // master mode
}

/** Shifts out a byte, MSB first */
void tlc_shift8(uint8_t byte)
{
    SPDR = byte; // starts transmission
    while (!(SPSR & _BV(SPIF)))
        ; // wait for transmission complete
}

/** Preinstantiated Tlc variable. */
Tlc5940 Tlc;

/** \defgroup ExtendedFunctions Extended Library Functions
    These functions require an include statement at the top of the sketch. */
/* @{ */ /* @} */

/** \mainpage
    The <a href="http://www.ti.com/lit/gpn/TLC5940">Texas Instruments TLC5940
    </a> is a 16-channel, constant-current sink LED driver.  Each channel has
    an individually adjustable 4096-step grayscale PWM brightness control and
    a 64-step, constant-current sink (no LED resistors needed!).  This chip
    is a current sink, so be sure to use common anode RGB LEDs.

    Check the <a href="http://code.google.com/p/tlc5940arduino/">tlc5940arduino
    project</a> on Google Code for updates.  To install, unzip the "Tlc5940"
    folder to &lt;Arduino Folder&gt;/hardware/libraries/

    &nbsp;

    \section hardwaresetup Hardware Setup
    The basic hardware setup is explained at the top of the Examples.  A good
    place to start would be the BasicUse Example.  (The examples are in
    File->Sketchbook->Examples->Library-Tlc5940).

    All the options for the library are located in tlc_config.h, including
    #NUM_TLCS, what pins to use, and the PWM period.  After changing
    tlc_config.h, be sure to delete the Tlc5940.o file in the library folder
    to save the changes.

    &nbsp;

    \section libref Library Reference
    \ref CoreFunctions "Core Functions" (see the BasicUse Example and Tlc5940):
    - \link Tlc5940::init Tlc.init(int initialValue (0-4095))\endlink - Call this is
            to setup the timers before using any other Tlc functions.
            initialValue defaults to zero (all channels off).
    - \link Tlc5940::clear Tlc.clear()\endlink - Turns off all channels
            (Needs Tlc.update())
    - \link Tlc5940::set Tlc.set(uint8_t channel (0-(NUM_TLCS * 16 - 1)),
            int value (0-4095))\endlink - sets the grayscale data for channel.
            (Needs Tlc.update())
    - \link Tlc5940::setAll Tlc.setAll(int value(0-4095))\endlink - sets all
            channels to value. (Needs Tlc.update())
    - \link Tlc5940::get uint16_t Tlc.get(uint8_t channel)\endlink - returns
            the grayscale data for channel (see set).
    - \link Tlc5940::update Tlc.update()\endlink - Sends the changes from any
            Tlc.clear's, Tlc.set's, or Tlc.setAll's.

    \ref ExtendedFunctions "Extended Functions".  These require an include
    statement at the top of the sketch to use.

    \ref ReqVPRG_ENABLED "Functions that require VPRG_ENABLED".  These
    require VPRG_ENABLED == 1 in tlc_config.h

    &nbsp;

    \section expansion Expanding the Library
    Lets say we wanted to add a function like "tlc_goCrazy(...)".  The first
    thing to do is to create a source file in the library folder,
    "tlc_my_crazy_functions.h".
    A template for this .h file is
    \code
// Explination for my crazy function extension

#ifndef TLC_MY_CRAZY_FUNCTIONS_H
#define TLC_MY_CRAZY_FUNCTIONS_H

#include "tlc_config.h"
#include "Tlc5940.h"

void tlc_goCrazy(void);

void tlc_goCrazy(void)
{
    uint16_t crazyFactor = 4000;
    Tlc.clear();
    for (uint8_t channel = 4; channel < 9; channel++) {
        Tlc.set(channel, crazyFactor);
    }
    Tlc.update();
}

#endif
 * \endcode
 * Now to use your library in a sketch, just add
 * \code
#include "tlc_my_crazy_functions.h"

// ...

tlc_goCrazy();
    \endcode
    If you would like to share your extended functions for others to use,
    email me (acleone ~AT~ gmail.com) with the file and an example and I'll
    include them in the library.

    &nbsp;

    \section bugs Contact
    If you found a bug in the library, email me so I can fix it!
    My email is acleone ~AT~ gmail.com

    &nbsp;

    \section license License - GPLv3
    Copyright (c) 2009 by Alex Leone <acleone ~AT~ gmail.com>

    This file is part of the Arduino TLC5940 Library.

    The Arduino TLC5940 Library is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    The Arduino TLC5940 Library is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with The Arduino TLC5940 Library.  If not, see
    <http://www.gnu.org/licenses/>. */

