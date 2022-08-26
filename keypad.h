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

#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#ifdef ARDUINO
#include "../grbl/gcode.h"
#include "../grbl/settings.h"
#else
#include "grbl/gcode.h"
#include "grbl/settings.h"
#endif

#include "pendant.h"

#if N_AXIS > 3
#define N_MACROS 5
#else
#define N_MACROS 7
#endif

typedef struct Machine_status_packet {
uint8_t address;
uint8_t machine_state;
uint8_t alarm;
uint8_t home_state;
uint8_t feed_override;
uint8_t spindle_override;
uint8_t spindle_stop;
int spindle_rpm;
float feed_rate;
coolant_state_t coolant_state;
uint8_t jog_mode;  //includes both modifier as well as mode
float jog_stepsize;
coord_system_id_t current_wcs;  //active WCS or MCS modal state
float x_coordinate;
float y_coordinate;
float z_coordinate;
float a_coordinate;
} Machine_status_packet;

typedef struct Pendant_count_packet {
int32_t uptime;
int8_t feed_over;
int8_t spindle_over;
int8_t rapid_over;
uint32_t buttons;
int32_t x_axis;
int32_t y_axis;
int32_t z_axis;
int32_t a_axis;
} Pendant_count_packet;

typedef void (*keycode_callback_ptr)(const char c);
typedef bool (*on_keypress_preview_ptr)(const char c, uint_fast16_t state);
typedef void (*on_jogmode_changed_ptr)(jogmode_t jogmode);
typedef void (*on_jogmodify_changed_ptr)(jogmodify_t jogmodify);

typedef struct {
    on_keypress_preview_ptr on_keypress_preview;
    on_jogmode_changed_ptr on_jogmode_changed;
    on_jogmodify_changed_ptr on_jogmodify_changed;
} keypad_t;

typedef struct {
    uint8_t port;
    char data[127];
} macro_setting_t;

typedef struct {
    macro_setting_t macro[N_MACROS];
} macro_settings_t;

extern keypad_t keypad;
extern jog_settings_t jog;

uint32_t protocol_version;

bool keypad_init (void);

#endif
