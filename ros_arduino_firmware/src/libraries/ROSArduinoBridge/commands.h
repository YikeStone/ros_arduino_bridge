/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define GET_BAUDRATE   'b'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define RESET_ENCODERS 'r'
#define UPDATE_PID_L   'L'
#define UPDATE_PID_R   'R'
#define DISP_PID_P     'z'

#define LEFT            0
#define RIGHT           1

#endif


