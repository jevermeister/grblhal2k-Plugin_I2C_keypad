/*
  keypad.h - I2C keypad plugin

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _PENDANT_H_
#define _PENDANT_H_

#ifdef ARDUINO
#include "../grbl/gcode.h"
#include "../grbl/settings.h"
#else
#include "grbl/gcode.h"
#include "grbl/settings.h"
#endif

#include "keypad.h"

#if N_AXIS > 3
#define N_MACROS 5
#else
#define N_MACROS 7
#endif

#define KEYBUF_SIZE 8 // must be a power of 2
#define KEYPAD_I2CADDR 0x49
#define STATUSDATA_SIZE 256

#define JOG_START   'R'

#define MACROUP 0x18
#define MACRODOWN 0x19
#define MACROLEFT 0x1B
#define MACRORIGHT 0x1A
#define MACROLOWER  0x7D
#define MACRORAISE 0x7C
#define MACROHOME  0x8E
#define RESET  0x7F
#define UNLOCK 0x80
#define SPINON 0x81

typedef enum {
    JogMode_Fast = 0,
    JogMode_Slow,
    JogMode_Step
} jogmode_t;

typedef enum {
    JogModify_1 = 0,
    JogModify_01,
    JogModify_001
} jogmodify_t;

bool process_count_info (bool cmd_process, uint8_t * prev_count_ptr, uint8_t * count_ptr);
void prepare_status_info (uint8_t * status_ptr);
void process_keycode (char keycode);

jog_settings_t jog;

#endif
