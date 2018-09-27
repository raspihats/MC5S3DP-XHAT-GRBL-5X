/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2017-2018 Gauthier Briere
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl supports only the
   Arduino Mega2560. */

#ifndef cpu_map_h
#define cpu_map_h

#include "main.h"



#ifdef CPU_MAP_2560_RAMPS_BOARD // (Arduino Mega 2560) with Ramps 1.4 Board
  #include "nuts_bolts.h"


  /************************************************************
   * ********************** STEPPERS **************************
   ************************************************************/

  // Define step pulse output pins.
  #define STEP_PORT_0             X_STP_GPIO_Port
  #define STEP_PORT_1             Y_STP_GPIO_Port
  #define STEP_PORT_2             Z_STP_GPIO_Port
  #if N_AXIS > 3
    #define STEP_PORT_3           A_STP_GPIO_Port // Axis number 4
  #endif
  #if N_AXIS > 4
    #define STEP_PORT_4           B_STP_GPIO_Port // Axis number 5
  #endif
  #if N_AXIS > 5

  #endif
  #define STEP_BIT_0              X_STP_Pin   // X Step
  #define STEP_BIT_1              Y_STP_Pin   // Y Step
  #define STEP_BIT_2              Z_STP_Pin   // Z Step
  #if N_AXIS > 3
    #define STEP_BIT_3            A_STP_Pin   // Axis number 4 Step
  #endif
  #if N_AXIS > 4
    #define STEP_BIT_4            B_STP_Pin   // Axis number 5 Step
  #endif
  #if N_AXIS > 5

  #endif


  // Define direction output pins.
  #define DIRECTION_PORT_0        X_DIR_GPIO_Port
  #define DIRECTION_PORT_1        Y_DIR_GPIO_Port
  #define DIRECTION_PORT_2        Z_DIR_GPIO_Port
  #if N_AXIS > 3
    #define DIRECTION_PORT_3      A_DIR_GPIO_Port // Axis number 4
  #endif
  #if N_AXIS > 4
    #define DIRECTION_PORT_4      B_DIR_GPIO_Port // Axis number 5
  #endif
  #if N_AXIS > 5

  #endif
  #define DIRECTION_BIT_0         X_DIR_Pin // X Dir
  #define DIRECTION_BIT_1         Y_DIR_Pin // Y Dir
  #define DIRECTION_BIT_2         Z_DIR_Pin // Z Dir
  #if N_AXIS > 3
    #define DIRECTION_BIT_3       A_DIR_Pin // Axis number 4 Dir
  #endif
  #if N_AXIS > 4
    #define DIRECTION_BIT_4       B_DIR_Pin // Axis number 5 Dir
  #endif
  #if N_AXIS > 5

  #endif


  // Define enable/disable output pins.
  #define DISABLE_PORT_0          X_EN_GPIO_Port
  #define DISABLE_PORT_1          Y_EN_GPIO_Port
  #define DISABLE_PORT_2          Z_EN_GPIO_Port
  #if N_AXIS > 3
    #define DISABLE_PORT_3        A_EN_GPIO_Port // Axis number 4
  #endif
  #if N_AXIS > 4
    #define DISABLE_PORT_4        B_EN_GPIO_Port // Axis number 5
  #endif
  #if N_AXIS > 5

  #endif
  #define DISABLE_BIT_0           X_EN_Pin // X Enable - Pin D38
  #define DISABLE_BIT_1           Y_EN_Pin // Y Enable - Pin A2
  #define DISABLE_BIT_2           Z_EN_Pin // Z Enable - Pin A8
  #if N_AXIS > 3
    #define DISABLE_BIT_3         A_EN_Pin // Axis number 4
  #endif
  #if N_AXIS > 4
    #define DISABLE_BIT_4         B_EN_Pin // Axis number 5
  #endif
  #if N_AXIS > 5

  #endif


  /************************************************************
   * *********************** LIMITS ***************************
   ************************************************************/
  // Switch inputs pins used for homing/hard limit.

  // Min Limits Ports
  #define MIN_LIMIT_PORT_0        X_MIN_GPIO_Port
  #define MIN_LIMIT_PORT_1        Y_MIN_GPIO_Port
  #define MIN_LIMIT_PORT_2        Z_MIN_GPIO_Port
  #if N_AXIS > 3
    #define MIN_LIMIT_PORT_3      A_MIN_GPIO_Port
  #endif
  #if N_AXIS > 4
    #define MIN_LIMIT_PORT_4      B_MIN_GPIO_Port
  #endif

  // Min Limits Pins
  #define MIN_LIMIT_BIT_0         X_MIN_Pin // X Limit Min pin
  #define MIN_LIMIT_BIT_1         Y_MIN_Pin // Y Limit Min pin
  #define MIN_LIMIT_BIT_2         Z_MIN_Pin // Z Limit Min pin
  #if N_AXIS > 3
    #define MIN_LIMIT_BIT_3       A_MIN_Pin // Axis number 4
  #endif
  #if N_AXIS > 4
    #define MIN_LIMIT_BIT_4       B_MIN_Pin // Axis number 5
  #endif

  // Max Limits Ports
  #define MAX_LIMIT_PORT_0        X_MAX_GPIO_Port
  #define MAX_LIMIT_PORT_1        Y_MAX_GPIO_Port
  #define MAX_LIMIT_PORT_2        Z_MAX_GPIO_Port
  #if N_AXIS > 3
    #define MAX_LIMIT_PORT_3      A_MAX_GPIO_Port
  #endif
  #if N_AXIS > 4
    #define MAX_LIMIT_PORT_4      B_MAX_GPIO_Port
  #endif

  // Max Limits pins
  #define MAX_LIMIT_BIT_0         X_MAX_Pin // X Limit Max
  #define MAX_LIMIT_BIT_1         Y_MAX_Pin // Y Limit Max
  #define MAX_LIMIT_BIT_2         Z_MAX_Pin // Z Limit Max
  #if N_AXIS > 3
    #define MAX_LIMIT_BIT_3       A_MAX_Pin // Axis number 4
  #endif
  #if N_AXIS > 4
    #define MAX_LIMIT_BIT_4       B_MAX_Pin // Axis number 5
  #endif


  //  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  //  #define LIMIT_INT_vect  PCINT0_vect
  //  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  //  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
  #define DISABLE_HW_LIMITS


  /************************************************************
   * ******************* USER CONTROLS ************************
   ************************************************************/
  // Cycle start, reset, feed hold input pins.
  #define CONTROL_RESET_PORT        DI_0_GPIO_Port
  #define CONTROL_RESET_PIN         DI_0_Pin
  #define CONTROL_FEED_HOLD_PORT    DI_1_GPIO_Port
  #define CONTROL_FEED_HOLD_PIN     DI_1_Pin
  #define CONTROL_CYCLE_START_PORT  DI_2_GPIO_Port
  #define CONTROL_CYCLE_START_PIN   DI_2_Pin
  #define CONTROL_SAFETY_DOOR_PORT  DI_3_GPIO_Port
  #define CONTROL_SAFETY_DOOR_PIN   DI_3_Pin
//  #define CONTROL_MASK      ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))


  /************************************************************
   * *********************** PROBE ****************************
   ************************************************************/
  // Define probe switch input pin.
  #define PROBE_PORT                DI_4_GPIO_Port
  #define PROBE_BIT                 DI_4_Pin


  /************************************************************
   * ********************** COOLANT ***************************
   ************************************************************/
  // Define flood and mist coolant enable output pins.
  #define COOLANT_FLOOD_PORT        DQ_0_GPIO_Port
  #define COOLANT_FLOOD_BIT         DQ_0_Pin
  #define COOLANT_MIST_PORT         DQ_1_GPIO_Port
  #define COOLANT_MIST_BIT          DQ_1_Pin


  /************************************************************
   * ********************** SPINDLE ***************************
   ************************************************************/
  // Define spindle output pins.
  #define SPINDLE_PWM_PORT          DQ_2_GPIO_Port
  #define SPINDLE_PWM_BIT           DQ_2_Pin

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_PORT       DQ_3_GPIO_Port
  #define SPINDLE_ENABLE_BIT        DQ_3_Pin
  #define SPINDLE_DIRECTION_PORT    DQ_4_GPIO_Port
  #define SPINDLE_DIRECTION_BIT     DQ_4_Pin

  #define SPINDLE_PWM_MAX_VALUE     4095 // Translates to about 24.4 kHz PWM frequency
  #define SPINDLE_PWM_MIN_VALUE     1   // Must be greater than zero.
  #define SPINDLE_PWM_OFF_VALUE     0
  #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)



#endif


#endif
