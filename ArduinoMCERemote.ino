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


#include <IRremote.h>

#define VERSION   1.0

#define NONE  0
#define ON    1
#define OFF   2

#define IR_POWER_ON0  0x800F0429  
#define IR_POWER_ON1  0x800F8429

#define IR_POWER_OFF0 0x800F042A      
#define IR_POWER_OFF1 0x800F842A


#define RECV_PIN        10
#define PC_POWER_PIN    9
#define PC_LED_PIN      5

bool debug = false;
bool verbose = false;

int activeCommand = NONE;
int commandDuration = 1000;   //1 sec
int commandTime = 0;
int powerStatus;

char incomingByte;

IRrecv irrecv(RECV_PIN);
decode_results  results, lastCode;


//+=============================================================================
// Pin setup and IR Initialization
//
void setup()
{
  digitalWrite(PC_POWER_PIN, LOW);
  pinMode(PC_POWER_PIN, OUTPUT); 
  //digitalWrite(PC_POWER_PIN, LOW);
  pinMode(PC_LED_PIN, INPUT);

  Serial.begin(115200);
  // In case the interrupt driver crashes on setup, give a clue
  // to the user what's going on.
  Serial.print("----------------------------------------------\n");
  Serial.print("IR PC Switch - Version: ");
  Serial.print(VERSION);
  Serial.print("\n");
  Serial.print("Enabling IR decoder...\n");
  irrecv.enableIRIn(); // Start the receiver
  Serial.print("done.\n");
  Serial.print("----------------------------------------------\n\n");

}

//+=============================================================================
// Main loop
//
void loop() {
  powerStatus = digitalRead(PC_LED_PIN);
  checkSerial();
  checkIR();
}

//+=============================================================================
// Toggle PC hardware switch
//
void powerToggle() {
  if (verbose || debug) {
    Serial.print("Activating PC power switch for 250 ms...\n");
  }
  digitalWrite(PC_POWER_PIN, HIGH);
  delay(250);
  digitalWrite(PC_POWER_PIN, LOW);
  if (verbose || debug) {
    Serial.print("Done.\n\n");
    Serial.print("----------------------------------------------\n\n");
  }
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
        Serial.print("----------------------------------------------\n");
        Serial.print("   PC Power Status:\n");
        Serial.print("----------------------------------------------\n\n");
        Serial.print("  Power is ");
        if (powerStatus == HIGH) {
          Serial.print("ON\n\n");
        } else {
          Serial.print("OFF\n\n");
        }
        Serial.print("  Last IR Code Received: ");
        encoding(&lastCode);
        Serial.print(" - 0x");
        Serial.print(lastCode.value, HEX);
        Serial.print("\n\n");
        Serial.print("----------------------------------------------\n\n");
        break;
    }
  }
}

//+=============================================================================
// Check IR code
//
void checkIR() {
  if (irrecv.decode(&results)) {
    lastCode = results;
    if (verbose || debug) {
      Serial.print("----------------------------------------------\n\n");
    }
    if ((results.decode_type == RC6 && (results.value == IR_POWER_ON0 || results.value == IR_POWER_ON1)) && powerStatus == LOW) {
      // received ON IR code and PC is OFF
      if (verbose || debug) {
        Serial.print("PC is OFF. Got Power On code: ");
        Serial.print(lastCode.value, HEX);
        Serial.print("\n");
      }
      if (activeCommand != ON) {
        commandTime = millis();
        // new command not a repeoated remote code
        if (verbose || debug) {
          Serial.print("Turning PC ON...\n");
        }
        powerToggle();
      }
    } else if ((results.decode_type == RC6 && (results.value == IR_POWER_OFF0 || results.value == IR_POWER_OFF1)) && powerStatus == HIGH) {
    // received OFF IR code and PC is ON
      if (verbose || debug) {
        Serial.print("PC is ON. Got Power Off code: ");
        Serial.print(lastCode.value, HEX);
        Serial.print("\n");
      }
      if (activeCommand != OFF) {
        commandTime = millis();
        // new command not a repeoated remote code
        if (verbose || debug) {
          Serial.print("Turning PC OFF...\n");
        }
        powerToggle();
      }
    } else if (verbose){
      Serial.print("  IR Code Received: ");
        encoding(&lastCode);
        Serial.print(" - 0x");
        Serial.print(lastCode.value, HEX);
        Serial.print("\n\n");
        Serial.print("----------------------------------------------\n\n");
    }
    if (debug) {
      dumpInfo(&results);           // Output the results
      dumpRaw(&results);            // Output the results in RAW format
      dumpCode(&results);           // Output the results as source code
      Serial.println("----------------------------------------------\n\n");  
    }
    irrecv.resume(); // Receive the next value
  }
  delay(100);
}

//+=============================================================================
// Display IR code
//
void  ircode (decode_results *results)
{
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
void  encoding (decode_results *results)
{
  switch (results->decode_type) {
    default:
    case UNKNOWN:      Serial.print("UNKNOWN");       break ;
    case NEC:          Serial.print("NEC");           break ;
    case SONY:         Serial.print("SONY");          break ;
    case RC5:          Serial.print("RC5");           break ;
    case RC6:          Serial.print("RC6");           break ;
    case DISH:         Serial.print("DISH");          break ;
    case SHARP:        Serial.print("SHARP");         break ;
    case JVC:          Serial.print("JVC");           break ;
    case SANYO:        Serial.print("SANYO");         break ;
    case MITSUBISHI:   Serial.print("MITSUBISHI");    break ;
    case SAMSUNG:      Serial.print("SAMSUNG");       break ;
    case LG:           Serial.print("LG");            break ;
    case WHYNTER:      Serial.print("WHYNTER");       break ;
    case AIWA_RC_T501: Serial.print("AIWA_RC_T501");  break ;
    case PANASONIC:    Serial.print("PANASONIC");     break ;
    case DENON:        Serial.print("DENON");         break ;
  }
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpInfo (decode_results *results)
{
  // Check if the buffer overflowed
  if (results->overflow) {
    Serial.print("IR code too long. Edit IRremoteInt.h in IRRemote Library and increase RAWBUF\n");
    return;
  }

  // Show Encoding standard
  Serial.print("Encoding  : ");
  encoding(results);
  Serial.print("\n");

  // Show Code & length
  Serial.print("Code      : ");
  ircode(results);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.print(" bits)\n\n");
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpRaw (decode_results *results)
{
  // Print Raw data
  Serial.print("Timing[");
  Serial.print(results->rawlen-1, DEC);
  Serial.print("]: \n");

  for (int i = 1;  i < results->rawlen;  i++) {
    unsigned long  x = results->rawbuf[i] * USECPERTICK;
    if (!(i & 1)) {  // even
      Serial.print("-");
      if (x < 1000)  Serial.print(" ") ;
      if (x < 100)   Serial.print(" ") ;
      Serial.print(x, DEC);
    } else {  // odd
      Serial.print("     ");
      Serial.print("+");
      if (x < 1000)  Serial.print(" ") ;
      if (x < 100)   Serial.print(" ") ;
      Serial.print(x, DEC);
      if (i < results->rawlen-1) Serial.print(", "); //',' not needed for last one
    }
    if (!(i % 8))  Serial.print("\n");
  }
  Serial.print("\n\n");                    // Newline
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpCode (decode_results *results)
{
  // Start declaration
  Serial.print("unsigned int  ");          // variable type
  Serial.print("rawData[");                // array name
  Serial.print(results->rawlen - 1, DEC);  // array size
  Serial.print("] = {");                   // Start declaration

  // Dump data
  for (int i = 1;  i < results->rawlen;  i++) {
    Serial.print(results->rawbuf[i] * USECPERTICK, DEC);
    if ( i < results->rawlen-1 ) Serial.print(","); // ',' not needed on last one
    if (!(i & 1))  Serial.print(" ");
  }

  // End declaration
  Serial.print("};");  // 

  // Comment
  Serial.print("  // ");
  encoding(results);
  Serial.print(" ");
  ircode(results);

  // Newline
  Serial.print("\n\n");

  // Now dump "known" codes
  if (results->decode_type != UNKNOWN) {

    // Some protocols have an address
    if (results->decode_type == PANASONIC) {
      Serial.print("unsigned int  addr = 0x");
      Serial.print(results->address, HEX);
      Serial.print(";\n\n");
    }

    // All protocols have data
    Serial.print("unsigned int  data = 0x");
    Serial.print(results->value, HEX);
    Serial.print(";\n\n");
  }
}