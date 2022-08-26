/*
  keypad.c - I2C keypad plugin

  Part of grblHAL

  Copyright (c) 2017-2022 Terje Io

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


#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "keypad.h"
#include "pendant.h"

#ifdef ARDUINO
#include "../i2c.h"
#include "../grbl/report.h"
#include "../grbl/override.h"
#include "../grbl/protocol.h"
#include "../grbl/nvs_buffer.h"
#include "../grbl/state_machine.h"
#else
#include "i2c.h"
#include "grbl/report.h"
#include "grbl/override.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/state_machine.h"
#include "grbl/limits.h"
#endif

static jogmode_t jogMode = JogMode_Fast;
static jogmodify_t jogModify = JogModify_1;

char charbuf[127];

static char buf[(STRLEN_COORDVALUE + 1) * N_AXIS];

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.

// BE WARNED: this function may be dangerous to use...
static char *strrepl (char *str, int c, char *str3)
{
    char tmp[30];
    char *s = strrchr(str, c);

    while(s) {
        strcpy(tmp, str3);
        strcat(tmp, s + 1);
        strcpy(s, tmp);
        s = strrchr(str, c);
    }

    return str;
}

static char *map_coord_system (coord_system_id_t id)
{
    uint8_t g5x = id + 54;

    strcpy(buf, uitoa((uint32_t)(g5x > 59 ? 59 : g5x)));
    if(g5x > 59) {
        strcat(buf, ".");
        strcat(buf, uitoa((uint32_t)(g5x - 59)));
    }

    return buf;
}

static void jog_command (char *cmd, char *to)
{
    strcat(strcpy(cmd, "$J=G91G21"), to);
}

static status_code_t disable_lock (sys_state_t state)
{
    status_code_t retval = Status_OK;

    if(state & (STATE_ALARM|STATE_ESTOP)) {

        control_signals_t control_signals = hal.control.get_state();

        // Block if self-test failed
        if(sys.alarm == Alarm_SelftestFailed)
            retval = Status_SelfTestFailed;
        // Block if e-stop is active.
        else if (control_signals.e_stop)
            retval = Status_EStop;
        // Block if safety door is ajar.
        else if (control_signals.safety_door_ajar)
            retval = Status_CheckDoor;
        // Block if safety reset is active.
        else if(control_signals.reset)
            retval = Status_Reset;
        else {
            grbl.report.feedback_message(Message_AlarmUnlock);
            state_set(STATE_IDLE);
        }
    } // Otherwise, no effect.

    return retval;
}

bool process_count_info (bool cmd_process, uint8_t * prev_count_ptr, uint8_t * count_ptr)
{    

    Pendant_count_packet * count_packet = (Pendant_count_packet*) count_ptr;
    Pendant_count_packet * previous_count_packet = (Pendant_count_packet*) prev_count_ptr;

    char command[35] = "";

    static struct {
    float x;
    float y;
    float z;
    float a;
    } deltas;

    float distance;
    float feedrate;

    if (cmd_process){
        
        //this reads the data and processes any changes


        //sprintf(charbuf, "X %d Y %d Z %d UT %d", count_packet.x_axis, count_packet.y_axis, count_packet.z_axis, count_packet.uptime);
        //report_message(charbuf, Message_Info);

        //calculate the deltas for each axis in mm
        deltas.x = ((float) (count_packet->x_axis - previous_count_packet->x_axis)) / 1000;
        deltas.y = ((float) (count_packet->y_axis - previous_count_packet->y_axis)) / 1000;
        deltas.z = ((float) (count_packet->z_axis - previous_count_packet->z_axis)) / 1000;
        #if N_AXIS > 3
        deltas.a = ((float) (count_packet->a_axis - previous_count_packet->a_axis)) / 1000;
        #endif

        //if deltas are zero, do not start jogging.
        if( deltas.x != 0 &&
            deltas.y != 0 &&
            deltas.z != 0 &&
            deltas.a != 0 ){

            //apply any configuration scaling

            //calculate feed rate so that the movement takes the sampling period to execute.
            //if the distance is longer than can be executed at max rate, limit the distance.
            /*distance=sqrt((deltas.x *deltas.x )+(deltas.y*deltas.y)+(deltas.z*deltas.z));
            feedrate = (distance/READ_COUNT_INTERVAL)*1000*60;

            jog_command(command, "X?");
            strrepl(command, '?', ftoa(deltas.x, 3));
            strcat(command, "Y?");
            strrepl(command, '?', ftoa(deltas.y, 3));
            strcat(command, "Z?");
            strrepl(command, '?', ftoa(deltas.z, 3));    

            #if N_AXIS > 3
                strcat(command, "A?");
                strrepl(command, '?', ftoa(deltas.a, 3));
                //need to ensure feed isn't zero for A only movement
            #endif

            strcat(command, "F");
            strcat(command, ftoa(feedrate, 3));

            */

            //sprintf(charbuf, "X %f Y %f Z %f UT %d KR %d JG %d SC %d", deltas.x, deltas.y, deltas.z, count_packet.uptime, keyreleased, cmd_process, strobe_counter);
            //report_message(charbuf, Message_Info);    

            //grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
            //report_message(command, Message_Info);
            //grbl.enqueue_gcode((char *)command);
        } else{
            //cmd_process = 0;
            
        }
            

        //Now handle buttons
        //if a button is active

    }//close cmd_process

    return cmd_process;
}

void prepare_status_info (uint8_t * status_ptr)
{    
    
    Machine_status_packet * status_packet = (Machine_status_packet*) status_ptr;
    
    int32_t current_position[N_AXIS]; // Copy current state of the system position variable
    float jog_modifier = 0;
    float print_position[N_AXIS];
    status_packet->a_coordinate = 0xffff;

    static uint32_t last_ms;
    uint32_t ms = hal.get_elapsed_ticks();

    if(ms < last_ms + 10) // don't spam the port
    return;
    
    memcpy(current_position, sys.position, sizeof(sys.position));

    system_convert_array_steps_to_mpos(print_position, current_position);

    uint_fast8_t idx;
    float wco[N_AXIS];
    for (idx = 0; idx < N_AXIS; idx++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[idx] = gc_get_offset(idx);
        print_position[idx] -= wco[idx];
    }  
    
    status_packet->address = 0x01;
    
    switch(jogModify){
        case JogModify_1:
            jog_modifier = 1;
        break;
        case JogModify_01:
            jog_modifier = 0.1;                    
        break;
        case JogModify_001:
            jog_modifier = 0.01;                    
        break;  
    }
    
    switch (state_get()){
        case STATE_ALARM:
            status_packet->machine_state = 1;
            break;
        case STATE_ESTOP:
            status_packet->machine_state = 1;
            break;            
        case STATE_CYCLE:
            status_packet->machine_state = 2;
            break;
        case STATE_HOLD:
            status_packet->machine_state = 3;
            break;
        case STATE_TOOL_CHANGE:
            status_packet->machine_state = 4;
            break;
        case STATE_IDLE:
            status_packet->machine_state = 5;
            break;
        case STATE_HOMING:
            status_packet->machine_state = 6;
            break;   
        case STATE_JOG:
            status_packet->machine_state = 7;
            break;                                    
        default :
            status_packet->machine_state = 254;
            break;                                                        
    }
    status_packet->coolant_state = hal.coolant.get_state();
    status_packet->feed_override = sys.override.feed_rate;
    status_packet->spindle_override = sys.override.spindle_rpm;
    status_packet->spindle_stop = sys.override.spindle_stop.value;

    // Report realtime feed speed
        if(hal.spindle.cap.variable) {
            status_packet->spindle_rpm = sys.spindle_rpm;
            if(hal.spindle.get_data)
                status_packet->spindle_rpm = hal.spindle.get_data(SpindleData_RPM)->rpm;
        } else
            status_packet->spindle_rpm = sys.spindle_rpm;

    status_packet->spindle_rpm = sys.spindle_rpm;  //rpm should be changed to actual reading
    status_packet->alarm = (uint8_t) sys.alarm;
    status_packet->home_state = (uint8_t)(sys.homing.mask & sys.homed.mask);
    status_packet->jog_mode = (uint8_t) jogMode << 4 | (uint8_t) jogModify;
    status_packet->x_coordinate = print_position[0];
    status_packet->y_coordinate = print_position[1];
    status_packet->z_coordinate = print_position[2];
    #if N_AXIS > 3
    status_packet->a_coordinate = print_position[3];
    #else
    status_packet->a_coordinate = 0xFFFFFFFF;
    #endif

    status_packet->feed_rate = st_get_realtime_rate();
    
    switch(jogMode){
        case JogMode_Slow:
        status_packet->jog_stepsize = jog.slow_speed * jog_modifier;
        break;
        case JogMode_Fast:
        status_packet->jog_stepsize = jog.fast_speed * jog_modifier;
        break;
        default:
        status_packet->jog_stepsize = jog.step_distance * jog_modifier;
        break;
    }
    
    status_packet->current_wcs = gc_state.modal.coord_system.id;    

    I2C_PendantWrite (KEYPAD_I2CADDR, status_ptr, sizeof(Machine_status_packet)); 

    last_ms = ms;   
}

void process_keycode (char keycode)
{
    char command[35] = ""; 

    spindle_state_t spindle_state;

    if(keycode) {

        switch(keycode) {

            case '?':                                    // pendant attach
                //read_protocol_version();
                //initialize_count_info();
                //pendant_attached = 1;
                break;
             case MACROUP:                                   //Macro 1 up
                //strcat(strcpy(command, "G10 L20 P0 Y"), ftoa(1.27, 5)); 
                execute_macro(0);            
                break;
             case MACRODOWN:                                   //Macro 3 down
                //strcat(strcpy(command, "G10 L20 P0 Y"), ftoa(-1.27, 5));
                execute_macro(2);              
                break;
             case MACROLEFT:                                   //Macro 2 right
                //strcat(strcpy(command, "G10 L20 P0 X"), ftoa(-1.27, 5));
                execute_macro(1); 
                break;
             case MACRORIGHT:                                   //Macro 4 left
                //strcat(strcpy(command, "G10 L20 P0 X"), ftoa(1.27, 5));
                execute_macro(3);             
                break;
             case SPINON:                                   //Macro 5 is special
                spindle_state = hal.spindle.get_state();
                if(!spindle_state.on){
                    //strcat(strcpy(command, "S"), ftoa(1500, 0));
                    //strcat(command, "M03");
                    execute_macro(4); 
                } else{
                    strcpy(command, "M05");
                }
                break;
             case MACROHOME:                                   // change WCS                
                if (gc_state.modal.coord_system.id  < N_WorkCoordinateSystems-1)
                    strcat(strcpy(command, "G"), map_coord_system(gc_state.modal.coord_system.id+1));    
                else
                    strcat(strcpy(command, "G"), map_coord_system(0x00));
                break;                
                break;
             case UNLOCK:  
                disable_lock(state_get());
                break;
             case RESET:                                   // Soft reset controller
                grbl.enqueue_realtime_command(CMD_RESET);
                break;
                                                                                                                                        
             case 'M':                                   // Mist override
                enqueue_accessory_override(CMD_OVERRIDE_COOLANT_MIST_TOGGLE);
                break;
            case 'C':                                   // Coolant override
                enqueue_accessory_override(CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE);
                break;

            case CMD_FEED_HOLD_LEGACY:                  // Feed hold
                grbl.enqueue_realtime_command(CMD_FEED_HOLD);
                break;

            case CMD_CYCLE_START_LEGACY:                // Cycle start
                grbl.enqueue_realtime_command(CMD_CYCLE_START);
                break;

            case CMD_MPG_MODE_TOGGLE:                   // Toggle MPG mode
                if(hal.driver_cap.mpg_mode)
                    stream_mpg_enable(hal.stream.type != StreamType_MPG);
                break;

            case '0':
            case '1':
            case '2':                                   // Set jog mode
                jogMode = (jogmode_t)(keycode - '0');
                break;

            case 'h':                                   // "toggle" jog mode
                jogMode = jogMode == JogMode_Step ? JogMode_Fast : (jogMode == JogMode_Fast ? JogMode_Slow : JogMode_Step);
                if(keypad.on_jogmode_changed)
                    keypad.on_jogmode_changed(jogMode);
                break;

            case 'm':                                   // cycle jog modifier
                jogModify = jogModify == JogModify_001 ? JogModify_1 : (jogModify == JogModify_1 ? JogModify_01 : JogModify_001);
                if(keypad.on_jogmodify_changed)
                    keypad.on_jogmodify_changed(jogModify);
                break;
            case 'H':                                   // Home axes
                strcpy(command, "$H");
                break;

         // Pass most of the top bit set commands trough unmodified

            case CMD_OVERRIDE_FEED_RESET:
            case CMD_OVERRIDE_FEED_COARSE_PLUS:
            case CMD_OVERRIDE_FEED_COARSE_MINUS:
            case CMD_OVERRIDE_FEED_FINE_PLUS:
            case CMD_OVERRIDE_FEED_FINE_MINUS:
            case CMD_OVERRIDE_RAPID_RESET:
            case CMD_OVERRIDE_RAPID_MEDIUM:
            case CMD_OVERRIDE_RAPID_LOW:
                enqueue_feed_override(keycode);
                break;

            case CMD_OVERRIDE_FAN0_TOGGLE:
            case CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE:
            case CMD_OVERRIDE_COOLANT_MIST_TOGGLE:
            case CMD_OVERRIDE_SPINDLE_RESET:
            case CMD_OVERRIDE_SPINDLE_COARSE_PLUS:
            case CMD_OVERRIDE_SPINDLE_COARSE_MINUS:
            case CMD_OVERRIDE_SPINDLE_FINE_PLUS:
            case CMD_OVERRIDE_SPINDLE_FINE_MINUS:
            case CMD_OVERRIDE_SPINDLE_STOP:
                enqueue_accessory_override(keycode);            
                break;

            case CMD_SAFETY_DOOR:
            case CMD_OPTIONAL_STOP_TOGGLE:
            case CMD_SINGLE_BLOCK_TOGGLE:
            case CMD_PROBE_CONNECTED_TOGGLE:
                grbl.enqueue_realtime_command(keycode);
                break;

             case MACRORAISE:                           //  Macro 5
                execute_macro(5); 
                break;

             case MACROLOWER:                           // Macro 6
                execute_macro(6); 
                break; 
               

        }
    }
}
