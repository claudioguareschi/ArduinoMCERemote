/***************************************************************************************
 *
 * ArduinoMCERemote
 *
 * An IR MCE compatible remote receiver with PC on/off hardware switch functionality
 * and IR code debug 
 *
 * Version 1.0 October 2017
 * Copyright 2017 Claudio Guareschi
 *
 * Obtain PC power status by reading 5V from motherboar power LED pin to arduino pin 5
 * Activates hardware power toggle switch from arduino pin 9 when appropriate.
 *
 * Translate MCE remote IR code into appropriate multimedia keyboard commands.
 *
 * Libraries:
 *   IRRemote - https://github.com/z3t0/Arduino-IRremote
 *   HID - https://github.com/NicoHood/HID
 *
 * Uses following Windows Media Center Codes for on/off functions:
 *
 * MCE Remote Power On
 *   800F8429
 *   800F0429
 * MCE Remote Power Off
 *   800F842A
 *   800F042A
 *
 * Connect to USB serial port for debug. Following commands are supported:
 *
 *  Q - Silence serial port output
 *  V - Verbose output
 *  D - Full IR code debug information
 *  ? - PC power status and last IR code received
 *
 *****************************************************************************************/

// Includes
#include <IRremote.h>
#include "HID-Project.h"
#include "MCEIRCodes.h"
#include "stdarg.h"

// global defines
#define VERSION   1.0

#define NONE  0
#define ON    1
#define OFF   2

#define KEYBOARD  0
#define CONSUMER  1

// pin assignment
#define IR_RECV_PIN        10
#define POWER_SENSE_PIN 5
#define POWER_BTN_PIN   9

#define POWER_BTN_HOLD_TIME 250

#define SPRINTF_BUFFER 16



// global flags
bool debug = false;
bool verbose = false;

// global variables
int activeCommand = NONE;
int commandDuration = 1000;   //1 sec
int commandTime = 0;
int powerStatus;

char incomingByte;

// IR code to key mapping

struct CodeMap {
    uint32_t        irCommand;
    uint8_t         hidDevice;
    uint16_t		keyCode;
    String          commandString;
};

const uint8_t KEY_CTRL  = 1;
const uint8_t KEY_ALT   = 2;
const uint8_t KEY_SHIFT = 4;
const uint8_t KEY_GUI   = 8;

// Kodi keyboard controls: http://kodi.wiki/view/Keyboard_controls
const CodeMap irToKeyMap[] = {
  {REMOTE_LEFT    , KEYBOARD, KEY_LEFT_ARROW, "LEFT"},
  {REMOTE_RIGHT   , KEYBOARD, KEY_RIGHT_ARROW, "RIGHT"},
  {REMOTE_UP      , KEYBOARD, KEY_UP_ARROW, "UP"},
  {REMOTE_DOWN    , KEYBOARD, KEY_DOWN_ARROW, "DOWN"},
  {REMOTE_OK      , KEYBOARD, KEY_RETURN, "RETURN"},
  {REMOTE_ENTER   , KEYBOARD, KEY_TAB, "TAB"},           // Fullscreen playback
  {REMOTE_CLEAR   , KEYBOARD, KEY_BACKSPACE, "BACKSPACE"},
  {REMOTE_EXIT    , KEYBOARD, KEY_ESC, "ESC"},
  {REMOTE_GUIDE   , CONSUMER, HID_CONSUMER_MEDIA_SELECT_PROGRAM_GUIDE, "PROGRAM GUIDE"},
  {REMOTE_INFO    , CONSUMER, KEY_I, "KEY_I - INFO"},
  {REMOTE_MENU    , CONSUMER, HID_CONSUMER_MENU, "MENU"},
  {REMOTE_STOP    , CONSUMER, MEDIA_STOP, "STOP"},
  {REMOTE_PLAY    , CONSUMER, MEDIA_PLAY_PAUSE, "PLAY/PAUSE"},
  {REMOTE_PAUSE   , CONSUMER, MEDIA_PAUSE, "PAUSE"},
  {REMOTE_REC     , CONSUMER, MEDIA_RECORD, "RECORD"},
  {REMOTE_REW     , CONSUMER, MEDIA_REWIND, "REW"},
  {REMOTE_FWD     , CONSUMER, MEDIA_FAST_FORWARD, "FWD"},
  {REMOTE_PREV    , CONSUMER, MEDIA_PREVIOUS, "PREVIOUS"},        // FIXME doesn't seem to work with non-us keyboard layout
  {REMOTE_SKIP    , CONSUMER, MEDIA_NEXT, "NEXT"},
// TODO remap for MCE remote
//  {REMOTE_REPLAY  , 0, KEY_COMMA},
  {REMOTE_SUBTITLE, KEYBOARD, KEY_T, "SUBTITLE"},         // toggle subtitles 
  {REMOTE_1       , KEYBOARD, KEY_1, "1"},
  {REMOTE_2       , KEYBOARD, KEY_2, "2"},
  {REMOTE_3       , KEYBOARD, KEY_3, "3"},
  {REMOTE_4       , KEYBOARD, KEY_4, "4"},
  {REMOTE_5       , KEYBOARD, KEY_5, "5"},
  {REMOTE_6       , KEYBOARD, KEY_6, "6"},
  {REMOTE_7       , KEYBOARD, KEY_7, "7"},
  {REMOTE_8       , KEYBOARD, KEY_8, "8"},
  {REMOTE_9       , KEYBOARD, KEY_9, "9"},
  {REMOTE_0       , KEYBOARD, KEY_0, "0"},
  {REMOTE_CH_UP   , CONSUMER, HID_CONSUMER_CHANNEL_INCREMENT, "CHANNEL UP"},       // PgUp / Skip to next queued video or next chapter if no videos are queued. / Increase Rating
  {REMOTE_CH_DOWN , CONSUMER, HID_CONSUMER_CHANNEL_DECREMENT, "CHANNEL DN"},    // PgDown / Skip to previous queued video or previous chapter if no videos are queued. / Decrease Rating
  {REMOTE_MUTE    , CONSUMER, MEDIA_VOLUME_MUTE, "MUTE"},
  {REMOTE_VOL_UP  , CONSUMER, MEDIA_VOLUME_UP, "VOLUME_UP"},
  {REMOTE_VOL_DOWN, CONSUMER, MEDIA_VOLUME_DOWN, "VOLUME_DOWN"},
/* TODO remap for MCE remote
  {REMOTE_F1      , 0, KEY_A},             // Audio delay control
  {REMOTE_F2      , 0, KEY_D},             // Move item down (Playlist editor & Favorites window)
  {REMOTE_F3      , 0, KEY_U},             // Move item up (Playlist editor & Favorites window)  
  {REMOTE_F4      , 0, KEY_Q},             // Queue
  {REMOTE_F5      , 0, KEY_V},             // Teletext / Visualisation settings
  {REMOTE_F6      , 0, KEY_Y},             // Switch/choose player
  {REMOTE_F7      , 0, KEY_HOME},          // Jump to the top of the menu
  TODO remap for MCE remote
  {REMOTE_ARROW_DOWN, KEY_CTRL, KEY_DOWN_ARROW}, // Move subtitles down
  {REMOTE_ARROW_UP  , KEY_CTRL, KEY_UP_ARROW},   // Move subtitles up
*/
//{REMOTE_F8        , KEY_CTRL, KEY_D},          // boot: ChromeOS    TODO test...
//{REMOTE_F9        , KEY_CTRL, KEY_W},          // boot: openELEC    TODO test...
};

const char *decorator = "----------------------------------------------";

const int IR_KEY_MAP_SIZE = sizeof(irToKeyMap) / sizeof(CodeMap);

const int KEY_PRESS_TIME = 150;

const int LOOP_DELAY = KEY_PRESS_TIME / 3;


unsigned long timeKeyDown = 0;  // time of key press initiation
unsigned long timeLastKeyEvent = 0;  // time of last key press event

IRrecv irrecv(IR_RECV_PIN);
decode_results  results, lastIRCode;


//+=============================================================================
// Pin setup and IR Initialization
//
void setup() {

	// setup pins
    digitalWrite(POWER_BTN_PIN, LOW);
    pinMode(POWER_BTN_PIN, OUTPUT); 
    pinMode(POWER_SENSE_PIN, INPUT);

    // initialize control over the keyboard:
    BootKeyboard.begin();
    Consumer.begin();

    // setup serial
    Serial.begin(115200);

    // setup IR Receiver
    // In case the interrupt driver crashes on setup, give a clue
    // to the user what's going on.
    sprintf("%s\n Arduino MCE Remote - Version: %s\nEnabling IR decoder...\n", decorator, VERSION);
    irrecv.enableIRIn(); // Start the receiver
    sprintf(" done.\n %s", decorator);

}

//+=============================================================================
// Main loop
//
void loop() {
    checkSerial();
    checkIR();
    delay(LOOP_DELAY);
}

//+=============================================================================
// sprintf - Serial printf
//
int sprintf(char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[SPRINTF_BUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

  va_list argv;
  va_start(argv, count);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j=0;
      temp[0] = '\0';

      switch(str[++i])
      {
        case 'd': Serial.print(va_arg(argv, int));
                  break;
        case 'l': Serial.print(va_arg(argv, long));
                  break;
        case 'f': Serial.print(va_arg(argv, double));
                  break;
        case 'c': Serial.print((char)va_arg(argv, int));
                  break;
        case 's': Serial.print(va_arg(argv, char *));
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%SPRINTF_BUFFER;
      if(j==0) 
      {
        temp[SPRINTF_BUFFER] = '\0';
        Serial.print(temp);
        temp[0]='\0';
      }
    }
  };
  //Serial.println();
  return count + 1;
}

//+=============================================================================
// Toggle PC hardware switch
//
void powerToggle() {
	if (verbose || debug) {
		sprintf("Activating PC power switch for 250 ms...\n");
	}
    digitalWrite(POWER_BTN_PIN, HIGH);
    delay(POWER_BTN_HOLD_TIME);
    digitalWrite(POWER_BTN_PIN, LOW);
    if (verbose || debug) {
        sprintf("Done.\n\n%s\n\n", decorator);
    }
}

//+=============================================================================
// Release keys
//
void releaseKeys() {
    BootKeyboard.releaseAll();
    Consumer.releaseAll();
    timeKeyDown = 0;
    timeLastKeyEvent = 0;
}

//+=============================================================================
// Check serial port and handle commands
//
void checkSerial() {
	if (Serial.available() > 0) {
		// read the incoming byte:
	    incomingByte = Serial.read();
	    switch (incomingByte) {
	        case 'q':
	        case 'Q':
	            verbose = false;
	            debug = false;
	            break;
	        case 'v':
	        case 'V':
	            verbose = true;
	            debug = false;
	            break;
	        case 'd':
	        case 'D':
	            debug = true;
	            verbose = false;
	            break;
	        case '?':
	        	sprintf("%s\n    PC Power Status:\n%s\n\n", decorator, decorator);
	        	sprintf(". Power is ");
	            if (digitalRead(POWER_SENSE_PIN) == HIGH) {
	                sprintf("ON\n\n");
	            } else {
	                sprintf("OFF\n\n");
	            }
	            sprintf("  Last IR Code Received: ");
	            encoding(&lastIRCode);
	            sprintf(" - 0x");
	            Serial.print(lastIRCode.value, HEX);
	            sprintf("\n\n%s\n\n", decorator);
	            break;
	    }
	}
}

//+=============================================================================
// Check IR code
//
void checkIR() {
    if (irrecv.decode(&results)) {
        if (verbose || debug) {
            sprintf("%s\n\n", decorator);
        }
        if (results.decode_type == irType) {
        	// eliminate RC6 toggle bit
            results.value = results.value & 0xFFFF7FFF;
            if (results.value != lastIRCode.value) {
                // immediately release last key if a different IR value is received. We don't want multiple keys pressed at the same time.
                releaseKeys(); 
            } 
            switch (results.value) {
                case REMOTE_POWER_OFF :
                    if (digitalRead(POWER_SENSE_PIN) == HIGH) {
                    	if (verbose || debug) {
                        	sprintf("PC is ON. Got Power Off code: ");
                		    Serial.print(results.value, HEX);
                			sprintf(". Turning PC off.\n");
            			}
                    	powerToggle();
                  	} else {
                  		if (verbose || debug) {
                        	sprintf("Got Power Off code: ");
                		    Serial.print(results.value, HEX);
                			sprintf(". ignored, device already off.\n");
            			}
                    }
                    break;
            	case REMOTE_POWER_ON :
            		if (digitalRead(POWER_SENSE_PIN) == LOW) {
                    	if (verbose || debug) {
                        	sprintf("PC is OFF. Got Power On code: ");
                		    Serial.print(results.value, HEX);
                			sprintf(". Turning PC on.\n");
            			}
                    	powerToggle();
                  	} else {
                  		if (verbose || debug) {
                        	sprintf("Got Power On code: ");
                		    Serial.print(results.value, HEX);
                			sprintf(". ignored, device already on.\n");
            			}
                    }
                    break;
                // keyboard commands
                default :
                    for (int i = 0; i < IR_KEY_MAP_SIZE; i++) {
                    	String mceCommand = "";
                        if (irToKeyMap[i].irCommand == results.value) {
                          // only press key if not yet pressed
                            if (timeLastKeyEvent == 0) {
                            	mceCommand = irToKeyMap[i].commandString;
                            	if (irToKeyMap[i].hidDevice == KEYBOARD) {
                            		BootKeyboard.press(irToKeyMap[i].keyCode);
                            	} else if (irToKeyMap[i].hidDevice == CONSUMER) {
                                	Consumer.press(irToKeyMap[i].keyCode);
                                }
                                if (verbose || debug) {
                        			sprintf("Received IR code: ");
                		    		Serial.print(results.value, HEX);
                					sprintf(".\n\n Sending ");
                					Serial.print(mceCommand);
                					sprintf(" command.\n\n");
            					}
                            	timeKeyDown = millis();
                        	}
                        	timeLastKeyEvent = millis();
                        	break;
                    	}
                	} // end for
            } // end switch result.value
        } else if (verbose){
            sprintf("  IR Code Received: ");
            encoding(&lastIRCode);
            sprintf(" - 0x");
            Serial.print(lastIRCode.value, HEX);
            sprintf("\n\n%s\n\n", decorator);
            
        }
        if (debug) {
            dumpInfo(&results);           // Output the results
            dumpRaw(&results);            // Output the results in RAW format
            dumpCode(&results);           // Output the results as source code
            sprintf("%s\n\n", decorator);  
        }

        lastIRCode = results;
        irrecv.resume(); // Receive the next value

    } else {
        // check if it's time to release a previously pressed key
        if (timeLastKeyEvent > 0 && (millis() - timeLastKeyEvent >= KEY_PRESS_TIME)) {
            releaseKeys(); 
        }
    }
}

//+=============================================================================
// Display IR code
//
void  ircode (decode_results *results) {
    // Panasonic has an Address
    if (results->decode_type == PANASONIC) {
        Serial.print(results->address, HEX);
        Serial.print(":");
    }

    // Print Code
    Serial.print(results->value, HEX);
}

//+=============================================================================
// Display encoding type
//
void  encoding (decode_results *results) {
    switch (results->decode_type) {
		default:
		case UNKNOWN:      sprintf("UNKNOWN");       break ;
		case NEC:          sprintf("NEC");           break ;
		case SONY:         sprintf("SONY");          break ;
		case RC5:          sprintf("RC5");           break ;
		case RC6:          sprintf("RC6");           break ;
		case DISH:         sprintf("DISH");          break ;
		case SHARP:        sprintf("SHARP");         break ;
		case JVC:          sprintf("JVC");           break ;
		case SANYO:        sprintf("SANYO");         break ;
		case MITSUBISHI:   sprintf("MITSUBISHI");    break ;
		case SAMSUNG:      sprintf("SAMSUNG");       break ;
		case LG:           sprintf("LG");            break ;
		case WHYNTER:      sprintf("WHYNTER");       break ;
		case AIWA_RC_T501: sprintf("AIWA_RC_T501");  break ;
		case PANASONIC:    sprintf("PANASONIC");     break ;
		case DENON:        sprintf("DENON");         break ;
    }
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpInfo (decode_results *results) {
	// Check if the buffer overflowed
	if (results->overflow) {
		sprintf("IR code too long. Edit IRremoteInt.h in IRRemote Library and increase RAWBUF\n");
		return;
	}

	// Show Encoding standard
	sprintf("Encoding  : ");
	encoding(results);
	sprintf("\n");

	// Show Code & length
	sprintf("Code      : ");
	ircode(results);
	sprintf(" (");
	Serial.print(results->bits, DEC);
	sprintf(" bits)\n\n");
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpRaw (decode_results *results) {
  // Print Raw data
	sprintf("Timing[");
	Serial.print(results->rawlen-1, DEC);
	sprintf("]: \n");

	for (int i = 1;  i < results->rawlen;  i++) {
		unsigned long  x = results->rawbuf[i] * USECPERTICK;
		if (!(i & 1)) {  // even
		    sprintf("-");
		    if (x < 1000)  sprintf(" ") ;
		    if (x < 100)   sprintf(" ") ;
		    Serial.print(x, DEC);
	    } else {  // odd
	        sprintf("     ");
	        sprintf("+");
		    if (x < 1000)  sprintf(" ") ;
	        if (x < 100)   sprintf(" ") ;
	  		Serial.print(x, DEC);
	  		if (i < results->rawlen-1) sprintf(", "); //',' not needed for last one
		}
		if (!(i % 8))  sprintf("\n");
	}
	sprintf("\n\n");                    // Newline
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpCode (decode_results *results) {
    // Start declaration
    sprintf("unsigned int  ");          // variable type
    sprintf("rawData[");                // array name
    Serial.print(results->rawlen - 1, DEC);  // array size
    sprintf("] = {");                   // Start declaration

    // Dump data
    for (int i = 1;  i < results->rawlen;  i++) {
        Serial.print(results->rawbuf[i] * USECPERTICK, DEC);
        if ( i < results->rawlen-1 ) sprintf(","); // ',' not needed on last one
        if (!(i & 1))  sprintf(" ");
    }

    // End declaration
    sprintf("};");  // 

    // Comment
    sprintf("  // ");
    encoding(results);
    sprintf(" ");
    ircode(results);

    // Newline
    sprintf("\n\n");

    // Now dump "known" codes
    if (results->decode_type != UNKNOWN) {
        // Some protocols have an address
        if (results->decode_type == PANASONIC) {
            sprintf("unsigned int  addr = 0x");
            Serial.print(results->address, HEX);
            sprintf(";\n\n");
        }

        // All protocols have data
        sprintf("unsigned int  data = 0x");
        Serial.print(results->value, HEX);
        sprintf(";\n\n");
    }
}