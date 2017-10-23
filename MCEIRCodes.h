/***************************************************************************************
 *
 * ArduinoMCERemote - MCEIRCodes.h
 *
 * Microsoft MCE IR Codes
 *
 *****************************************************************************************/
#pragma once

const decode_type_t  irType = RC6;

// MCE scancodes from https://github.com/torvalds/linux/blob/master/drivers/media/rc/keymaps/rc-rc6-mce.c

#define IR_MCE_NUMERIC_0      0x800f0400
#define IR_MCE_NUMERIC_1      0x800f0401
#define IR_MCE_NUMERIC_2      0x800f0402
#define IR_MCE_NUMERIC_3      0x800f0403
#define IR_MCE_NUMERIC_4      0x800f0404
#define IR_MCE_NUMERIC_5      0x800f0405
#define IR_MCE_NUMERIC_6      0x800f0406
#define IR_MCE_NUMERIC_7      0x800f0407
#define IR_MCE_NUMERIC_8      0x800f0408
#define IR_MCE_NUMERIC_9      0x800f0409

#define IR_MCE_DELETE         0x800f040a
#define IR_MCE_ENTER          0x800f040b
#define IR_MCE_SLEEP          0x800f040c      /* Formerly PC Power */
#define IR_MCE_MEDIA          0x800f040d      /* Windows MCE button */
#define IR_MCE_MUTE           0x800f040e
#define IR_MCE_INFO           0x800f040f

#define IR_MCE_VOLUMEUP       0x800f0410
#define IR_MCE_VOLUMEDOWN     0x800f0411
#define IR_MCE_CHANNELUP      0x800f0412
#define IR_MCE_CHANNELDOWN    0x800f0413

#define IR_MCE_FASTFORWARD    0x800f0414
#define IR_MCE_REWIND         0x800f0415
#define IR_MCE_PLAY           0x800f0416
#define IR_MCE_RECORD         0x800f0417
#define IR_MCE_PAUSE          0x800f0418
#define IR_MCE_STOP           0x800f0419
#define IR_MCE_NEXT           0x800f041a
#define IR_MCE_PREVIOUS       0x800f041b
#define IR_MCE_NUMERIC_POUND  0x800f041c
#define IR_MCE_NUMERIC_STAR   0x800f041d

#define IR_MCE_UP             0x800f041e
#define IR_MCE_DOWN           0x800f041f
#define IR_MCE_LEFT           0x800f0420
#define IR_MCE_RIGHT          0x800f0421

#define IR_MCE_OK             0x800f0422
#define IR_MCE_EXIT           0x800f0423
#define IR_MCE_DVD            0x800f0424
#define IR_MCE_TUNER          0x800f0425      /* LiveTV */
#define IR_MCE_EPG            0x800f0426      /* Guide */
#define IR_MCE_ZOOM           0x800f0427      /* Aspect */

#define IR_MCE_MODE           0x800f0432      /* Visualization */
#define IR_MCE_PRESENTATION   0x800f0433      /* Slide Show */
#define IR_MCE_EJECTCD        0x800f0434
#define IR_MCE_BRIGHTNESSUP   0x800f043a

#define IR_MCE_TV             0x800f0446
#define IR_MCE_AUDIO          0x800f0447      /* My Music */
#define IR_MCE_PVR            0x800f0448      /* RecordedTV */
#define IR_MCE_CAMERA         0x800f0449
#define IR_MCE_VIDEO          0x800f044a
#define IR_MCE_LANGUAGE       0x800f044c
#define IR_MCE_TITLE          0x800f044d
#define IR_MCE_PRINT          0x800f044e      /* Print - HP OEM version of remote */

#define IR_MCE_RADIO          0x800f0450

#define IR_MCE_SUBTITLE       0x800f045a      /* Caption/Teletext */
#define IR_MCE_RED            0x800f045b
#define IR_MCE_GREEN          0x800f045c
#define IR_MCE_YELLOW         0x800f045d
#define IR_MCE_BLUE           0x800f045e

#define IR_MCE_POWER2         0x800f0465      /* TV Power */
#define IR_MCE_MESSENGER      0x800f0469
#define IR_MCE_PLAYPAUSE      0x800f046e
#define IR_MCE_PLAYER         0x800f046f      /* Start media application (NEW) */

#define IR_MCE_BRIGHTNESSDOWN 0x800f0480
#define IR_MCE_PLAYPAUSE2     0x800f0481

// additional scancodes of an Intel NUC
#define IR_MCE_POWER_ON       0x800F0429
#define IR_MCE_POWER_OFF      0x800F042A


// Standard Logitech 880 buttons
#define REMOTE_MUTE         IR_MCE_MUTE
#define REMOTE_VOL_UP       IR_MCE_VOLUMEUP
#define REMOTE_VOL_DOWN     IR_MCE_VOLUMEDOWN
/* TODO
#define REMOTE_ARROW_DOWN   IR_MCE_VIDEO1
#define REMOTE_ARROW_UP     IR_MCE_VIDEO2
*/
#define REMOTE_CH_DOWN      IR_MCE_CHANNELDOWN
#define REMOTE_CH_UP        IR_MCE_CHANNELUP
#define REMOTE_PREV         IR_MCE_PREVIOUS
#define REMOTE_UP           IR_MCE_UP
#define REMOTE_DOWN         IR_MCE_DOWN
#define REMOTE_LEFT         IR_MCE_LEFT
#define REMOTE_RIGHT        IR_MCE_RIGHT
#define REMOTE_OK           IR_MCE_OK
// TODO
#define REMOTE_MENU         IR_MCE_MODE
#define REMOTE_EXIT         IR_MCE_EXIT
#define REMOTE_GUIDE        IR_MCE_EPG
#define REMOTE_INFO         IR_MCE_INFO
#define REMOTE_STOP         IR_MCE_STOP

//#define REMOTE_REPLAY       IR_MCE_REPLAY
#define REMOTE_SKIP         IR_MCE_NEXT
#define REMOTE_PLAY         IR_MCE_PLAY
#define REMOTE_REC          IR_MCE_RECORD
#define REMOTE_REW          IR_MCE_REWIND
#define REMOTE_FWD          IR_MCE_FASTFORWARD
#define REMOTE_PAUSE        IR_MCE_PAUSE
#define REMOTE_1            IR_MCE_NUMERIC_1
#define REMOTE_2            IR_MCE_NUMERIC_2
#define REMOTE_3            IR_MCE_NUMERIC_3
#define REMOTE_4            IR_MCE_NUMERIC_4
#define REMOTE_5            IR_MCE_NUMERIC_5
#define REMOTE_6            IR_MCE_NUMERIC_6
#define REMOTE_7            IR_MCE_NUMERIC_7
#define REMOTE_8            IR_MCE_NUMERIC_8
#define REMOTE_9            IR_MCE_NUMERIC_9
#define REMOTE_0            IR_MCE_NUMERIC_0
#define REMOTE_ENTER        IR_MCE_ENTER

#define REMOTE_POWER_TOGGLE IR_MCE_SLEEP
#define REMOTE_POWER_ON     IR_MCE_POWER_ON
#define REMOTE_POWER_OFF    IR_MCE_POWER_OFF

#define REMOTE_CLEAR        IR_MCE_DELETE
#define REMOTE_RED          IR_MCE_RED
#define REMOTE_GREEN        IR_MCE_GREEN
#define REMOTE_YELLOW       IR_MCE_YELLOW
#define REMOTE_BLUE         IR_MCE_BLUE
#define REMOTE_SUBTITLE     IR_MCE_SUBTITLE
#define REMOTE_ASPECT       IR_MCE_MEDIA

/*
#define REMOTE_F1           IR_MCE_AV
#define REMOTE_F2           IR_MCE_HDMI2
#define REMOTE_F3           IR_MCE_HDMI1
#define REMOTE_F4           IR_MCE_HDMI3
#define REMOTE_F6           IR_MCE_ASPECT
*/
#define REMOTE_F5           IR_MCE_TELETEXT
#define REMOTE_F7           IR_MCE_HOME
#define REMOTE_F8           IR_MCE_TV